// this is model loading.fs
#version 330 core
out vec4 FragColor;

in vec2 texCoords;

uniform sampler2D texture_diffuse1;
uniform sampler2D texture_specular1;


float near = 0.1; 
float far  = 20.0; 

float LinearizeDepth(float depth) 
{
    float z = depth * 2.0 - 1.0; // back to NDC 
    return (2.0 * near * far) / (far + near - z * (far - near));    
}

void main()
{             
    float depth = LinearizeDepth(gl_FragCoord.z) / far; // 为了演示除以 far
    FragColor = mix(texture(texture_diffuse1, texCoords),texture(texture_specular1, texCoords),0.5);
}