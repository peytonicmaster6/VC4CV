#include <sys/time.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <vector>
#include <chrono>
#include <termios.h>

#include "applog.h"

#include "bcm_host.h"

#include "eglUtil.h"
#include "camGL.h"

#include "defines.hpp"
#include "mesh.hpp"
#include "shader.hpp"
#include "texture.hpp"

#include <math.h>

#include <fcntl.h>
#include <sstream>
#include <iostream>

CamGL *camGL;
CamGL *camGL1;
int dispWidth, dispHeight;
int camWidth = 1280, camHeight = 720, camFPS = 30;
float renderRatioCorrection;

std::string heart_rate;
std::string body_temp;
std::string helmet_temp;

EGL_Setup eglSetup;
Mesh *SSQuad;
ShaderProgram *shaderCamBlitRGB, *shaderCamBlitY, *shaderCamBlitYUV;
int texRGBAdr, texYAdrY, texYUVAdrY, texYUVAdrU, texYUVAdrV;

struct termios terminalSettings;

static void setConsoleRawMode();
static void processCameraFrame(CamGL_Frame *frame);
static void processCameraFrame(CamGL_Frame *frame1);
static void bindExternalTexture(GLuint adr, GLuint tex, int slot);
static void CreateVertexBuffer();

int main(int argc, char **argv)
{
	// ---- Start Serial Port ----
	int fd; /* File descriptor for the port */
	fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1)
	{
		perror("open_port: Unable to open /dev/ACM0 - "); //coult not open the port
	}
	else
		fcntl(fd, F_SETFL, 0);

	
	// ---- Read arguments ----

	CamGL_Params params = {
		.format = CAMGL_YUV,
		.width = (uint16_t)camWidth,
		.height = (uint16_t)camHeight,
		.fps = (uint16_t)camFPS,
		.shutterSpeed = 0,
		.iso = 0,
		.camera_num = 0
	};
	
	int arg;
	while ((arg = getopt(argc, argv, "c:w:h:f:s:i:n:")) != -1)
	{
		switch (arg)
		{
			case 'c':
				if (strcmp(optarg, "YUV") == 0) params.format = CAMGL_YUV;
				else if (strcmp(optarg, "Y") == 0) params.format = CAMGL_Y;
				else if (strcmp(optarg, "RGB") == 0) params.format = CAMGL_RGB;
				break;
			case 's':
				params.shutterSpeed = std::stoi(optarg);
				break;
			case 'i':
				params.iso = std::stoi(optarg);
				break;
			case 'w':
				params.width = camWidth = std::stoi(optarg);
				break;
			case 'h':
				params.height = camHeight = std::stoi(optarg);
				break;
			case 'f':
				params.fps = camFPS = std::stoi(optarg);
				break;
			case 'n':
				params.camera_num = std::stoi(optarg);
				break;
			default:
				printf("Usage: %s [-c (RGB, Y, YUV)] [-w width] [-h height] [-f fps] [-s shutter-speed-ns] [-i iso] [-n camera-num] \n", argv[0]);
				break;
		}
	}
	if (optind < argc - 1)
		printf("Usage: %s [-c (RGB, Y, YUV)] [-w width] [-h height] [-f fps] [-s shutter-speed-ns] [-i iso] [-n camera-num] \n", argv[0]);

	// ---- Init ----

	// Init BCM Host
	bcm_host_init();

	// Create native window (not real GUI window)
	EGL_DISPMANX_WINDOW_T window;
	if (createNativeWindow(&window) != 0)
		return EXIT_FAILURE;
	dispWidth = window.width;
	dispHeight = window.height;
	renderRatioCorrection = (((float)dispHeight / camHeight) * camWidth) / dispWidth;  

	// Setup EGL context
	setupEGL(&eglSetup, (EGLNativeWindowType*)&window);
	glClearColor(0.8f, 0.2f, 0.1f, 1.0f);
	
	std::cout << "Camera Number " << params.camera_num << "\n";

	// ---- Setup GL Resources ----
	std::vector<float> vertices; 
	std::vector<unsigned short> indices;
    
	float N = 100;
        float z = 0;
    
        for (float x = -1, a = 0; x <= 1, a <= 1; x+= 2/N, a += 1/N)
        {
        	for (float y = -1, b = 0; y <= 1, b <= 1; y+= 2/N, b+= 1/N)
        	{
			float theta = atan2(y, x);
			float r = sqrt(x*x + y*y);
			r = r -0.15*pow(r, 3.0) + 0.01*pow(r, 5.0);
			vertices.push_back(r*cos(theta));
			vertices.push_back(r*sin(theta));
			//vertices.push_back(x);
			//vertices.push_back(y);
			vertices.push_back(z);
			vertices.push_back(a);
			vertices.push_back(b);
        	}
        }
	 
	for (int x = 0; x < N; x++)
        {
        	for (int z = 0; z < N; z++)
        	{
			int offset = x * (N+1) + z;
            		indices.push_back((short)(offset+0));
            		indices.push_back((short)(offset+1));
            		indices.push_back((short)(offset+ (N+1) + 1));
            		indices.push_back((short)(offset+0));
            		indices.push_back((short)(offset+ (N+1)));
            		indices.push_back((short)(offset+ (N+1) + 1));
        	}
    	}
    
    	//Debugging info, will print out the indices, vertices, and how many of each there are
   
    	//unsigned int indicesCount = indices.size();
    	//unsigned int verticesCount = vertices.size();
        
    	//for (auto i: indices)
		//std::cout << i << ' ';
    
    	//for (auto i: vertices)
		//std::cout << i << ' ';
		
	//std::cout << '\n' << verticesCount << '\n';
	//std::cout << '\n' << indicesCount << '\n';
	
	SSQuad = new Mesh ({ POS, TEX }, vertices, indices);

	// Load shaders
	shaderCamBlitRGB = new ShaderProgram("../gl_shaders/CamES/vert.glsl", "../gl_shaders/CamES/frag_camRGB.glsl");
	shaderCamBlitY = new ShaderProgram("../gl_shaders/CamES/vert.glsl", "../gl_shaders/CamES/frag_camY.glsl");
	shaderCamBlitYUV = new ShaderProgram("../gl_shaders/CamES/vert.glsl", "../gl_shaders/CamES/frag_camYUV.glsl");
	
	// Load shader uniform adresses
	texRGBAdr = glGetUniformLocation(shaderCamBlitRGB->ID, "image");
	texYAdrY = glGetUniformLocation(shaderCamBlitY->ID, "imageY");
	texYUVAdrY = glGetUniformLocation(shaderCamBlitYUV->ID, "imageY");
	texYUVAdrU = glGetUniformLocation(shaderCamBlitYUV->ID, "imageU");
	texYUVAdrV = glGetUniformLocation(shaderCamBlitYUV->ID, "imageV");

	// ---- Setup Camera ----

	// Init camera GL
	printf("Initializing Camera GL!\n");
	camGL = camGL_create(eglSetup, (const CamGL_Params*)&params);
	params.camera_num = 1;
	camGL1 = camGL_create(eglSetup, (const CamGL_Params*)&params);
	if (camGL == NULL)
	{
		printf("Failed to start Camera GL\n");
		terminateEGL(&eglSetup);
		return EXIT_FAILURE;
	}
	else
	{ // Start CamGL

		printf("Starting Camera GL!\n");
		int status = camGL_startCamera(camGL);
		printf("Starting Camera GL1!\n");
		int status1 = camGL_startCamera(camGL1);
		
		if (status != CAMGL_SUCCESS)
		{
			printf("Failed to start camera GL with code %d!\n", status);
		}
		else if (status1 != CAMGL_SUCCESS)
		{
			printf("Failed to start camera GL1 with code %d!\n", status1);
		}
		else
		{ // Process incoming frames
			
			// For non-blocking input even over ssh
			setConsoleRawMode();

			auto startTime = std::chrono::high_resolution_clock::now();
			auto lastTime = startTime;
			int numFrames = 0, lastFrames = 0;
           
			// Get handle to frame struct, stays the same when frames are updated
			CamGL_Frame *frame = camGL_getFrame(camGL);
			CamGL_Frame *frame1 = camGL_getFrame(camGL1);
			
			while ((status = camGL_nextFrame(camGL)) == CAMGL_SUCCESS)
			{// Frames was available and has been processed
				
				////Read the Serial port
				fcntl(fd, F_SETFL, FNDELAY);
				char buff[64];
				ssize_t rc = read(fd, buff, sizeof(buff));
				buff[rc] = 0;
				
				if (rc > 0){
					std::string str = buff;
					std::vector<std::string> result;
					std::stringstream s_stream(str);
					//std::vector<std::string> strings = {"Heart Rate: ", "Body Temp: ", "Helmet Temp: "};
      
					while(s_stream.good()) {
						std::string substr;
						getline(s_stream, substr, ','); //get first string delimited by comma
						result.push_back(substr);
					}
      
					if (result.size() > 1){
						heart_rate = result[0];
						body_temp = result[1];
						helmet_temp = result[2];
					}
				}
				
				status1 = camGL_nextFrame(camGL1);
				if (status1 != CAMGL_SUCCESS)
				{
					break;
				}

				std::string left = "Heart Rate: " + heart_rate + ", Body Temp: " + body_temp + ", Helmet Temp: " + helmet_temp + "";
				const char* left_eye = left.c_str();
				
				std::string right = " ";
				const char* right_eye = right.c_str();
				
				camGL_update_annotation(camGL, left_eye);
				camGL_update_annotation(camGL1, right_eye);
				ShaderProgram *shader;
				
				//Camera 1
				//if (frame->format == CAMGL_RGB)
				//{
					//shader = shaderCamBlitRGB;
					//shader->use();
					//bindExternalTexture(texRGBAdr, frame->textureRGB, 0);
				//}
				//else if (frame->format == CAMGL_Y)
				//{
					//shader = shaderCamBlitY;
					//shader->use();
					//bindExternalTexture(texYAdrY, frame->textureY, 0);
				//}
				//else if (frame->format == CAMGL_YUV)
				//{
				shader = shaderCamBlitYUV;
				shader->use();
				bindExternalTexture(texYUVAdrY, frame->textureY, 0);
				bindExternalTexture(texYUVAdrU, frame->textureU, 1);
				bindExternalTexture(texYUVAdrV, frame->textureV, 2);
				//}
				
				//glViewport((int)((1-renderRatioCorrection) * dispWidth / 2), 0, (int)(renderRatioCorrection * dispWidth), dispHeight);
				glViewport(0, 0, 960, 1080);
				SSQuad->draw();
				
				//Camera 2
				//if (frame1->format == CAMGL_RGB)
				//{
					//shader = shaderCamBlitRGB;
					//shader->use();
					//bindExternalTexture(texRGBAdr, frame1->textureRGB, 0);
				//}
				//else if (frame1->format == CAMGL_Y)
				//{
					//shader = shaderCamBlitY;
					//shader->use();
					//bindExternalTexture(texYAdrY, frame1->textureY, 0);
				//}
				//else if (frame1->format == CAMGL_YUV)
				//{
				//shader = shaderCamBlitYUV;
				//shader->use();
				bindExternalTexture(texYUVAdrY, frame1->textureY, 0);
				bindExternalTexture(texYUVAdrU, frame1->textureU, 1);
				bindExternalTexture(texYUVAdrV, frame1->textureV, 2);
				//}				
					
				glViewport(960, 0, 960, 1080);
				glBindFramebuffer(GL_FRAMEBUFFER, 0);
				SSQuad->draw();
					
				eglSwapBuffers(eglSetup.display, eglSetup.surface); 


				// ---- Debugging and Statistics ----
		
				numFrames++;
				if (numFrames % 200 == 0)
				{ // Log FPS
					auto currentTime = std::chrono::high_resolution_clock::now();
					int elapsedMS = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastTime).count();
					float elapsedS = (float)elapsedMS / 1000;
					lastTime = currentTime;
					int frames = (numFrames - lastFrames);
					lastFrames = numFrames;
					float fps = frames / elapsedS;
					int droppedFrames = 0;
					printf("%d frames over %.2fs (%.1ffps)! \n", frames, elapsedS, fps);
				}
				if (numFrames % 10 == 0)
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
			if (status != 0)
				printf("Camera GL was interrupted with code %d!\n", status);
			else if(status1 != 0)
				printf("Camera GL was interrupted with code %d!\n", status1);
			else
				camGL_stopCamera(camGL);
				camGL_stopCamera(camGL1);
		}
		camGL_destroy(camGL);
		camGL_destroy(camGL1);
		terminateEGL(&eglSetup);

		return status == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
		return status1 == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
	}
}

/* Sets console to raw mode which among others allows for non-blocking input, even over SSH */
static void setConsoleRawMode()
{
	tcgetattr(STDIN_FILENO, &terminalSettings);
	struct termios termSet = terminalSettings;
	atexit([]{
		tcsetattr(STDIN_FILENO, TCSAFLUSH, &terminalSettings);
		camGL_stopCamera(camGL);
	});
	termSet.c_lflag &= ~ECHO;
	termSet.c_lflag &= ~ICANON;
	termSet.c_cc[VMIN] = 0;
	termSet.c_cc[VTIME] = 0;
	tcsetattr(STDIN_FILENO, TCSAFLUSH, &termSet);
}

/* Bind external EGL tex to adr using specified texture slot */
static void bindExternalTexture (GLuint adr, GLuint tex, int slot)
{
	glUniform1i(adr, slot);
	glActiveTexture(GL_TEXTURE0 + slot);
	glBindTexture(GL_TEXTURE_EXTERNAL_OES, tex);
	CHECK_GL();
}


