#include <string>
#include <chrono>
#include <termios.h>
#include <math.h>
#include <algorithm>

#include "fbUtil.h"
#include "qpu_program.h"
#include "qpu_info.h"
#include "gcs.h"

#include "interface/mmal/mmal_encodings.h"
#include "bcm_host.h"
#include "user-vcsm.h" // for vcsm_vc_hdl_from_ptr

#define DEFAULT 0		// source and target butter uniforms only
#define FULL_FRAME 1	// uniforms for full frame processing, e.g. blit
#define TILED 2			// uniforms and setup for tiled frame processing, e.g. blob detection

#define RGB32 0			// Normal RGB 32Bit buffer (e.g. framebuffer)
#define BITMSK 1		// Special 1Bit buffer in normal layout
#define BLKMSK 2		// Special 1Bit buffer in 8x32 block layout
#define BILMSK 3		// Special 1Bit buffer in 8x32 block layout, interleaved

#define RUN_CAMERA	// Have the camera supply frames
			// EITHER use emulated buffers with debug content (default)
#define USE_CAMERA	// OR use camera frames directly in QPU program
//#define CPY_CAMERA	// OR copy copy frames into emulated buffers

struct termios terminalSettings;
static void setConsoleRawMode();

int main(int argc, char **argv)
{
	// ---- Read arguments ----

	GCS_CameraParams params = {
		.mmalEnc = MMAL_ENCODING_I420,
		.width = 1280,
		.height = 720,
		.fps = 30,
		.shutterSpeed = 0,
		.iso = 0,
		.disableEXP = false,
		.disableAWB = false,
		.disableISPBlocks = 0 // https://www.raspberrypi.org/forums/viewtopic.php?f=43&t=175711
//			  (1<<2) // Black Level Compensation
//			| (1<<3) // Lens Shading
//			| (1<<5) // White Balance Gain
//			| (1<<7) // Defective Pixel Correction
//			| (1<<9) // Crosstalk
//			| (1<<18) // Gamma
//			| (1<<22) // Sharpening
//			| (1<<24) // Some Color Conversion
	};
	char codeFile[64];
	uint32_t maxNumFrames = -1; // Enough to run for years
	bool drawToFrameBuffer = false;
	int mode = FULL_FRAME;
	int buffer = RGB32;
	bool enableQPU[12] = { 1,1,1,1, 1,1,1,1, 1,1,1,1 };
	int padding = 0; // padding on both sides of the image - set up for 5x5 kernel (2 on each side)
	int blockLength = 16;

	int arg;
	while ((arg = getopt(argc, argv, "c:w:h:f:s:i:m:b:o:t:da:e:q:p:l:")) != -1)
	{
		switch (arg)
		{
			case 'c':
				strncpy(codeFile, optarg, sizeof(codeFile));
				break;
			case 'w':
				params.width = std::stoi(optarg);
				break;
			case 'h':
				params.height = std::stoi(optarg);
				break;
			case 'f':
				params.fps = std::stoi(optarg);
				break;
			case 's':
				params.shutterSpeed = std::stoi(optarg);
				break;
			case 'i':
				params.iso = std::stoi(optarg);
				break;
			case 'm':
				if (strcmp(optarg, "full") == 0) mode = FULL_FRAME;
				else if (strcmp(optarg, "tiled") == 0) mode = TILED;
				else mode = DEFAULT;
				break;
			case 'b':
				if (strcmp(optarg, "RGB") == 0) buffer = RGB32;
				else if (strcmp(optarg, "bitmsk") == 0) buffer = BITMSK;
				else if (strcmp(optarg, "blkmsk") == 0) buffer = BLKMSK;
				else if (strcmp(optarg, "bilmsk") == 0) buffer = BILMSK;
				else buffer = RGB32;
				break;
			case 't':
				maxNumFrames = std::stoi(optarg);
				break;
			case 'd':
				drawToFrameBuffer = true;
				break;
			case 'e':
				params.disableAWB = true;
				break;
			case 'x':
				params.disableEXP = true;
				break;
			case 'q':
				for (int i = 0; i < 12 && i < strlen(optarg); i++)
					enableQPU[i] = optarg[i] == '1';
				break;
			case 'p':
				padding = std::stoi(optarg);
				break;
			case 'l':
				blockLength = std::stoi(optarg);
				break;
			default:
				printf("Usage: %s -c codefile [-w width] [-h height] [-f fps] [-s shutter-speed-ns] [-i iso] [-m mode (full, tiled, bitmsk)] [-d display-to-fb] [-t max-num-frames]\n", argv[0]);
				break;
		}
	}
	if (optind < argc - 1)
		printf("Usage: %s -c codefile [-w width] [-h height] [-f fps] [-s shutter-speed-ns] [-i iso] [-m mode (full, tiled, bitmsk)] [-d display-to-fb] [-t max-num-frames]\n", argv[0]);

	// ---- Init ----

	// Core QPU structures
	QPU_BASE base;
	QPU_PROGRAM program;
	QPU_BUFFER targetBuffer;
	QPU_BUFFER bitmskBuffer;
	// QPU Debugging
	QPU_PerformanceState perfState;
	QPU_HWConfiguration hwConfig;
	QPU_UserProgramInfo upInfo;
	// MMAL Camera
	GCS *gcs;
	// Camera emulation buffers
	const int emulBufCnt = 4;
	QPU_BUFFER camEmulBuf[emulBufCnt];
	// Frame Counter
	auto startTime = std::chrono::high_resolution_clock::now();
	auto lastTime = startTime;
	int lastFrames = 0, numFrames = 0;
	// QPU usage
	int qpusUsed = 0;
	for (int i = 0; i < 12; i++)
		qpusUsed += enableQPU[i]? 1 : 0;

	// Init BCM Host
	bcm_host_init();

	// Init QPU Base (basic information to work with QPU)
	int ret = qpu_initBase(&base);
	if (ret != 0)
	{
		printf("Failed to init qpu base! %d \n", ret);
		return ret;
	}

	// ---- Setup target buffer ----

	uint32_t tgtBufferPtr, tgtStride;
	uint32_t srcStride = params.width;
	uint32_t lineWidth = params.width, lineCount = params.height;
	int fbfd = 0;
	struct fb_var_screeninfo orig_vinfo;
	struct fb_var_screeninfo vinfo;
	struct fb_fix_screeninfo finfo;
	if (drawToFrameBuffer)
	{ // Get frame buffer information
		fbfd = setupFrameBuffer(&orig_vinfo, &vinfo, &finfo, true);
		if (!fbfd) drawToFrameBuffer = false;
		else
		{
			tgtStride = finfo.line_length;
			tgtBufferPtr = finfo.smem_start;
			if ((lineWidth > vinfo.xres || lineCount > vinfo.yres) && buffer == RGB32)
			{
				lineWidth = std::min(lineWidth, vinfo.xres);
				lineCount = std::min(lineCount, vinfo.yres);
				printf("Limiting resolution to screen while -d is enabled! %dx%d \n", lineWidth, lineCount);
			}
		}
	}
	if (!drawToFrameBuffer)
	{ // Allocate buffer to render into
		qpu_allocBuffer(&targetBuffer, &base, params.width*params.height*4, 4096);
		tgtStride = params.width*4;
		tgtBufferPtr = targetBuffer.ptr.vc;
	}
	if (buffer == BITMSK || buffer == BLKMSK || buffer == BILMSK)
	{ // Set up bit target, one bit per pixel
		qpu_allocBuffer(&bitmskBuffer, &base, lineWidth/8*lineCount, 4096);
		// Width and height must be multiple of 32 and 16 respectively
		lineWidth = (uint32_t)std::floor((float)params.width/8/4)*8*4;
		lineCount = (uint32_t)std::floor((float)params.height/16)*16;
		tgtStride = lineWidth/8;
		tgtBufferPtr = bitmskBuffer.ptr.vc;
	}

	// ---- Generate tiling setup ----

	// Split image in columns of 8x16 pixels assigned to one QPU each.
	// Split vertically until most or all QPUs are used
	int numTileCols = (int)floor((lineWidth-padding) / 8.0); // Num of 8px Tiles in a row with padding
	int numTileRows = (int)floor((lineCount-padding) / 8.0); // Num of 8px Tiles in a col with padding
	int numProgCols = (int)floor(numTileCols / 16.0); // Number of instances required (QPU is 16-way)
	int droppedTileCols = numTileCols - numProgCols * 16; // Some are dropped for maximum performance, extra effort is not worth it
	int splitCols = 1;
	while (numProgCols * (splitCols+1) <= qpusUsed)
	{ // Split columns among QPUs to minimize the number of idle QPUs
		splitCols++;
	}
	int numInstances = numProgCols * splitCols;
	if (mode == TILED)
	{
		printf("SETUP: %d instances processing 1/%d columns each, covering %dx%d tiles, plus %d columns dropped\n",
			numInstances, splitCols, numProgCols*16, numTileRows, droppedTileCols);
//		lineWidth = lineWidth - 8*droppedTileCols;
//		if (buffer == BITMSK || buffer == BLKMSK)
//			tgtStride = lineWidth/8;
	}
	else numInstances = 1;

	// ---- Setup program ----

	// Setup program with specified progmem sizes
	QPU_PROGMEM progmemSetup {
		.codeSize = qpu_getCodeSize(codeFile), //4096*4;
		.uniformsSize =
			(mode==FULL_FRAME)? 6 :
			(mode==TILED? (uint32_t)numInstances*6 : 2),
		.messageSize = 0 // 2 if qpu_executeProgramMailbox is used, instead of qpu_executeProgramDirect
	};
	qpu_initProgram(&program, &base, progmemSetup);
	qpu_loadProgramCode(&program, codeFile);

	// ---- Setup progmem ----

	// Set up uniforms of the QPU program
	qpu_lockBuffer(&program.progmem_buffer);
	if (mode == DEFAULT)
	{ // Simple default program with no additional requirements
		program.progmem.uniforms.arm.uptr[0] = 0; // Enter source pointer each frame
		program.progmem.uniforms.arm.uptr[1] = tgtBufferPtr;
	}
	else if (mode == FULL_FRAME)
	{ // Set up one program to handle the full frame
		program.progmem.uniforms.arm.uptr[0] = 0; // Enter source pointer each frame
		program.progmem.uniforms.arm.uptr[1] = tgtBufferPtr;
		program.progmem.uniforms.arm.uptr[2] = params.width;
		program.progmem.uniforms.arm.uptr[3] = tgtStride;
		program.progmem.uniforms.arm.uptr[4] = lineWidth;
		program.progmem.uniforms.arm.uptr[5] = lineCount;
	}
	else if (mode == TILED)
	{ // Set up each program instance with their column
		for (int c = 0; c < numProgCols; c++)
		{
			for (int r = 0; r < splitCols; r++)
			{
				program.progmem.uniforms.arm.uptr[c*splitCols*6 + r*6 + 0] = 0; // Enter source pointer each frame
				program.progmem.uniforms.arm.uptr[c*splitCols*6 + r*6 + 1] = tgtBufferPtr + c*(buffer==RGB32? 4*8*16 : 16*blockLength) + r*lineCount/splitCols*tgtStride;
				program.progmem.uniforms.arm.uptr[c*splitCols*6 + r*6 + 2] = params.width;
				program.progmem.uniforms.arm.uptr[c*splitCols*6 + r*6 + 3] = tgtStride;
				program.progmem.uniforms.arm.uptr[c*splitCols*6 + r*6 + 4] = 8*16; // 16 elements working on 8-pixel columns each
				program.progmem.uniforms.arm.uptr[c*splitCols*6 + r*6 + 5] = lineCount / splitCols;
			}
		}
	}
	qpu_unlockBuffer(&program.progmem_buffer);

	// ---- Setup QPU ----

	// Enable QPU
	if (qpu_enable(base.mb, 1)) {
		printf("QPU enable failed!\n");
		goto error_qpu;
	}
	printf("-- QPU Enabled --\n");

	// Debug QPU Hardware
	qpu_debugHW(&base);
	// VPM memory reservation
	base.peripherals[V3D_VPMBASE] = 16; // times 4 to get number of vectors; Default: 8 (32/4), Max: 16 (64/4)
	qpu_getHWConfiguration(&hwConfig, &base);
	qpu_getUserProgramInfo(&upInfo, &base);
	printf("Reserved %d / %d vectors of VPM memory for user programs!\n", upInfo.VPMURSV_V, hwConfig.VPMSZ_V);
	// QPU scheduler reservation
//	for (int i = 0; i < 12; i++) // Reset all QPUs to be freely sheduled
//		qpu_setReservationSetting(&base, i, 0b0000);
//	for (int i = 0; i < 12; i++) // Disable ALL QPUs
//		qpu_setReservationSetting(&base, i, 0b0001);
	for (int i = 0; i < 12; i++) // Enable only QPUs selected as parameter
		qpu_setReservationSetting(&base, i, enableQPU[i]? 0b1110 : 0b1111);
//	for (int i = numInstances; i < 12; i++) // Exempt unused QPUs from user programs
//		qpu_setReservationSetting(&base, i, 0b0111);
	qpu_logReservationSettings(&base);
	// Setup performance monitoring
	qpu_setupPerformanceCounters(&base, &perfState);
	perfState.qpusUsed = std::min(numInstances, qpusUsed);

	// ---- Setup Camera ----

	// Create GPU camera stream (MMAL camera)
#ifdef RUN_CAMERA
	gcs = gcs_create(&params);
	if (gcs == NULL)
	{
		printf("Failed to greate GCS! \n");
		goto error_gcs;
	}
	gcs_start(gcs);
	printf("-- Camera Stream started --\n");
#endif
#ifndef USE_CAMERA
	for (int i = 0; i < emulBufCnt; i++)
	{
		qpu_allocBuffer(&camEmulBuf[i], &base, params.width*params.height*3, 4096); // Emulating full YUV frame
		qpu_lockBuffer(&camEmulBuf[i]);
		uint8_t *YUVFrameData = (uint8_t*)camEmulBuf[i].ptr.arm.vptr;
		for (int y = 0; y < params.height; y++)
		{
			for (int x = 0; x < params.width; x++)
			{
				// Write test data in Y component (UV are after this, but are not used)
				YUVFrameData[y*params.width + x] = ((x+params.width/(i+1))*255/params.width)%256 + ((y+params.height/(i+1))*255/params.height)%256;
//				YUVFrameData[y*params.width + x] = ((y+params.height/(i+1))*255/params.height)%256;
//				YUVFrameData[y*params.width + x] = ((y)*255/params.height)%256;
			}
		}
		qpu_unlockBuffer(&camEmulBuf[i]);
	}
#endif

	// ---- Start Loop ----

	// For non-blocking input even over ssh
	setConsoleRawMode();

	lastTime = startTime = std::chrono::high_resolution_clock::now();
	while (numFrames < maxNumFrames)
	{
#ifdef RUN_CAMERA
		// Get most recent MMAL buffer from camera
		void *cameraBufferHeader = gcs_requestFrameBuffer(gcs);
		if (!cameraBufferHeader) printf("GCS returned NULL frame! \n");
		else
#else
		// Emulate framerate
		usleep(std::max(0,(int)(1.0f/params.fps*1000*1000)-4000));
#endif
		{
			// ---- Camera Frame Access ----

#ifdef RUN_CAMERA
			// Get buffer data from opaque buffer handle
			void *cameraBuffer = gcs_getFrameBufferData(cameraBufferHeader);
			// Source: https://www.raspberrypi.org/forums/viewtopic.php?f=43&t=167652
			// Get VCSM Handle of frameBuffer (works only if zero-copy is enabled, so buffer is in VCSM)
			uint32_t cameraBufferHandle = vcsm_vc_hdl_from_ptr(cameraBuffer);
	#ifndef USE_CAMERA
			// Lock VCSM buffer to get VC-space address
			mem_lock(base.mb, cameraBufferHandle);

		#ifdef CPY_CAMERA
			int i = (numFrames)%emulBufCnt;
			qpu_lockBuffer(&camEmulBuf[i]);
			uint8_t *srcCameraFrame = (uint8_t*)cameraBuffer;
			uint8_t *YUVFrameData = (uint8_t*)camEmulBuf[i].ptr.arm.vptr;
			for (int y = 0; y < params.height; y++)
			{
				for (int x = 0; x < params.width; x++)
				{
					// Write test data in Y component (UV are after this, but are not used)
					YUVFrameData[y*params.width + x] = srcCameraFrame[y*params.width+x];
				}
			}
			qpu_unlockBuffer(&camEmulBuf[i]);
		#endif

	#endif
#endif
#ifdef USE_CAMERA
			// Lock VCSM buffer to get VC-space address
			uint32_t cameraBufferPtr = mem_lock(base.mb, cameraBufferHandle);
#else
			qpu_lockBuffer(&camEmulBuf[numFrames%emulBufCnt]);
			uint32_t cameraBufferPtr = camEmulBuf[numFrames%emulBufCnt].ptr.vc;
#endif
#ifdef RUN_CAMERA
			// Unlock VCSM buffer (no need to keep locked, VC-space adress won't change)
//			mem_unlock(base.mb, cameraBufferHandle);
			// Return camera buffer to camera
//			gcs_returnFrameBuffer(gcs);
#endif

//			usleep(10000);

			// ---- Uniform preparation ----

			// Set source buffer pointer in progmem uniforms
			qpu_lockBuffer(&program.progmem_buffer);
			if (mode == TILED)
			{ // Set up individual source pointer for each program instance
				for (int c = 0; c < numProgCols; c++)
					for (int r = 0; r < splitCols; r++)
						program.progmem.uniforms.arm.uptr[c*splitCols*6 + r*6 + 0] = cameraBufferPtr + c*8*16 + r*lineCount/splitCols*params.width;
			}
			else
			{
				program.progmem.uniforms.arm.uptr[0] = cameraBufferPtr;
			}
			qpu_unlockBuffer(&program.progmem_buffer);

			// ---- Program execution ----

/*			if (mode == BITMSK || mode == BLKMSK || mode == BILMSK)
				qpu_lockBuffer(&bitmskBuffer);
			else if (!drawToFrameBuffer)
				qpu_lockBuffer(&targetBuffer);
*/
			// Execute programs
			int result;
			if (mode == TILED)
			{ // Execute numInstances programs each with their own set of uniforms
				result = qpu_executeProgramDirect(&program, &base, numInstances, 6, 6, &perfState);
			}
			else
			{ // Execute single program handling full frame
				result = qpu_executeProgramDirect(&program, &base, 1, program.progmem.uniformsSize, 0, &perfState);
			}

			// Log errors occurred during execution
			qpu_logErrors(&base);

#ifndef USE_CAMERA
			qpu_unlockBuffer(&camEmulBuf[numFrames%emulBufCnt]);
#endif
#ifdef RUN_CAMERA
			// Unlock VCSM buffer (no need to keep locked, VC-space adress won't change)
			mem_unlock(base.mb, cameraBufferHandle);
			// Return camera buffer to camera
			gcs_returnFrameBuffer(gcs);
#endif

			// Unlock target buffers
/*			if (mode == BITMSK || mode == BLKMSK || mode == BILMSK)
				qpu_unlockBuffer(&bitmskBuffer);
			else if (!drawToFrameBuffer)
				qpu_unlockBuffer(&targetBuffer);
*/
			if (result != 0)
			{
				printf("Encountered an error after %d frames!\n", numFrames);
				break;
			}

			// ---- Debugging and Statistics ----

			numFrames++;
			if (numFrames % 100 == 0)
			{ // Frames per second
				auto currentTime = std::chrono::high_resolution_clock::now();
				int elapsedMS = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastTime).count();
				float elapsedS = (float)elapsedMS / 1000;
				lastTime = currentTime;
				int frames = (numFrames - lastFrames);
				lastFrames = numFrames;
				float fps = frames / elapsedS;
				printf("%d frames over %.2fs (%.1ffps)! \n", frames, elapsedS, fps);
			}
			if (numFrames % 10 == 0)
			{ // Detailed QPU performance gathering (every 10th frame to handle QPU performance register overflows)
				qpu_updatePerformance(&base, &perfState);
			}
			if (numFrames % 100 == 0 && numFrames <= 500)
			{ // Detailed QPU performance report
				printf("QPU Performance over past 100 frames:\n");
				qpu_logPerformance(&perfState);
			}
			if (numFrames % 500 == 0 && numFrames > 500)
			{ // Detailed QPU performance report
				printf("QPU Performance over past 500 frames:\n");
				qpu_logPerformance(&perfState);
			}

			// ---- Framebuffer debugging ----

			static int ilCnt = 0;
			int ilMax = 1;//2;
			ilCnt = (ilCnt+1)%ilMax;
			if (drawToFrameBuffer && (buffer == BITMSK || buffer == BLKMSK || buffer == BILMSK) && (numFrames % 10) < ilMax)
			{ // Manual access to framebuffer
				void *fbp = lock_fb(fbfd, finfo.smem_len);
				if ((int)fbp == -1)
					printf("Failed to mmap.\n");
				else
				{ // Copy custom bitmap from buffer to screen for debugging
					qpu_lockBuffer(&bitmskBuffer);
					int dV = std::min(lineCount, 480u), dH = std::min(lineWidth, 640u);
					//printf("srcStride: %d, tgtStride: %d, lineWidth: %d, lineCount: %d, width: %d \n", srcStride, tgtStride, lineWidth, lineCount, params.width);

					for (int y = ilCnt; y < dV; y+=ilMax)
					{
						int tgtY = y * lineCount / dV;
						uint8_t *px = (uint8_t*)fbp + y*finfo.line_length;
						uint8_t *maskBase = (uint8_t*)bitmskBuffer.ptr.arm.uptr;
						int blkFactor = 1;
						if (buffer == BLKMSK)
						{ // Address in blocks and not in bits
							int blkLine = tgtY % blockLength;
							blkFactor = blockLength;
							maskBase += (tgtY-blkLine) * tgtStride + blkLine;
						}
						else if (buffer == BILMSK)
						{
							int blkLine = tgtY % blockLength;
							blkFactor = blockLength;
							maskBase += (tgtY-blkLine) * tgtStride;
							int intLine = tgtY%4;
							int blkInt = blkLine - intLine;
							for (int x = 0; x < dH; x++)
							{
								int tgtX = x * lineWidth / dH;
								uint8_t *mask = maskBase + tgtX / 8 * blkFactor + blkInt + tgtX % 4;
								*(uint32_t*)px = ((*mask >> ((tgtX%8)/4+intLine*2)) & 1)? 0xFFFFFFFF : 0xFF000000;
								px += vinfo.bits_per_pixel/8;
							}
							continue;
						}
						else
							maskBase += tgtY * tgtStride;
						for (int x = 0; x < dH; x++)
						{
							int tgtX = x * lineWidth / dH;
							uint8_t *mask = maskBase + tgtX / 8 * blkFactor;
							*(uint32_t*)px = ((*mask >> tgtX%8) & 1)? 0xFFFFFFFF : 0xFF000000;
							px += vinfo.bits_per_pixel/8;
						}
					}
					qpu_unlockBuffer(&bitmskBuffer);
					unlock_fb(fbp, finfo.smem_len);
				}
			}
		}

		// ---- Input ----

//		if (numFrames % 10 == 0)
		{ // Check for keys
			char cin;
			if (read(STDIN_FILENO, &cin, 1) == 1)
			{
				if (iscntrl(cin)) printf("%d", cin);
				else if (cin == 'q') break;
				else printf("%c", cin);
			}
		}
	}

#ifdef RUN_CAMERA
	gcs_stop(gcs);
	gcs_destroy(gcs);
	printf("-- Camera Stream stopped --\n");
#endif
#ifndef USE_CAMERA
	for (int i = 0; i < emulBufCnt; i++)
		qpu_releaseBuffer(&camEmulBuf[i]);
#endif


error_gcs:

	// Disable QPU
	if (qpu_enable(base.mb, 0))
		printf("-- QPU Disable failed --\n");
	else
		printf("-- QPU Disabled --\n");

	for (int i = 0; i < 12; i++) // Reset all QPUs to be freely sheduled
		qpu_setReservationSetting(&base, i, 0b0000);

error_qpu:

	if (buffer == BITMSK || buffer == BLKMSK)
		qpu_releaseBuffer(&bitmskBuffer);

	if (drawToFrameBuffer)
		close(fbfd);
	else
		qpu_releaseBuffer(&targetBuffer);

	qpu_destroyProgram(&program);

	qpu_destroyBase(&base);

	return EXIT_SUCCESS;
}

/* Sets console to raw mode which among others allows for non-blocking input, even over SSH */
static void setConsoleRawMode()
{
	tcgetattr(STDIN_FILENO, &terminalSettings);
	struct termios termSet = terminalSettings;
	atexit([]{ // Reset at exit
		tcsetattr(STDIN_FILENO, TCSAFLUSH, &terminalSettings);
	});
	termSet.c_lflag &= ~ECHO;
	termSet.c_lflag &= ~ICANON;
	termSet.c_cc[VMIN] = 0;
	termSet.c_cc[VTIME] = 0;
	tcsetattr(STDIN_FILENO, TCSAFLUSH, &termSet);
}
