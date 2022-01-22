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
    //vec3 v = vPos.xyz;

    //float radius = length(vPos.xyz);
 
    //float theta = atan(vTex.y,vTex.x);

    //radius = pow(radius, 1.0); //changes zoom again

    //v.x = radius * cos(theta);
    //v.y = radius * sin(theta);
    
    //gl_Position = vec4(Distort(vPos), 1.0);
    //gl_Position = vec4(gl_Position.xyz, 1.0);
    //vec3 pos = vPos;
    //pos.y = 1.0*cos(1.0);
    //pos.xy = vTex.xy;
    //pos.xy = pos.xy*0.5 + 0.0;



    gl_Position = vec4(vPos, 1.0); //second value is zoom 

    uv = vTex;
    uv = uv*2.0 - 1.0;
    uv = Distort(uv);
}
