#version 330 core
out vec4 FragColor;

in vec2 TexCoords;
in vec3 Normal;
in vec3 Position;

//uniform sampler2D texture1;
uniform samplerCube skybox;
uniform vec3 cameraPos;


vec4 reflectFun()
{
    vec3 I = normalize(Position-cameraPos);
    vec3 R = reflect(I,Normal);
    vec4 texColor = texture(skybox, R);
    return texColor;
}

vec4 refractFun()
{
    float ratio = 1.00 / 1.52;
    vec3 I = normalize(Position-cameraPos);
    vec3 R = refract(I,Normal,ratio);  
    vec4 texColor = texture(skybox, R);
    return texColor;
}

void main()
{              
    FragColor = mix(reflectFun(),refractFun(),0.2);
}