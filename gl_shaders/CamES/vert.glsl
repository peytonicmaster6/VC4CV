#version 100

attribute vec3 vPos;
attribute vec2 vTex;

varying vec2 uv;

vec2 Distort(vec2 p)
{
    float r = length(p);
    if (r > 0.0)
    {
        float theta = atan(p.y,p.x);
        r = pow(r, 1.5);
        p.x = r * cos(theta);
        p.y = r * sin(theta);
    }
    return 0.5*(p+1.0);
}

void main()
{
    gl_Position = vec4(vPos, 1.0); //second value is zoom 

    uv = vTex;
    //Apply barrel distortion
    uv = uv*2.0 - 1.0;
    uv = Distort(uv);
}
