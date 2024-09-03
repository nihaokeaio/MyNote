#version 330 core
layout (location=0) in vec3 aPos;
layout (location=1) in vec3 aNormal;
layout (location=2) in vec2 aTexCoords;
layout (location=3) in mat4 aModel;

out vec3 Normal;
out vec3 FragPos; 
out vec2 texCoords;

//uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main()
{
	gl_Position=projection*view*aModel*vec4(aPos,1.0);
	FragPos = vec3(aModel * vec4(aPos,1.0));
	//Normal=vec3(model * vec4(aNormal,1.0));
	Normal=mat3(transpose(inverse(aModel)))*aNormal;
	texCoords=aTexCoords;
}