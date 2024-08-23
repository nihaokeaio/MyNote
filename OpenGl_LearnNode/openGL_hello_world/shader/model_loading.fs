// this is model loading.fs
#version 330 core
out vec4 FragColor;

in vec2 TexCoords;

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
    //FragColor = vec4(vec3(gl_FragCoord.z), 1.0);
    FragColor = mix(texture(texture_diffuse1, TexCoords),texture(texture_specular1, TexCoords),0.5);
}