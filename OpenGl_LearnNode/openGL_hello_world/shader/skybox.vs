#version 330 core
layout (location=0) in vec3 aPos;
layout (location=0) in vec3 aNormal;

out vec3 TexCoords;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main()
{
	TexCoords = aPos;
	//gl_Position=projection*view*model*vec4(aPos,1.0);
	vec4 pos = projection * view * model *vec4(aPos, 1.0);
    gl_Position = pos.xyww;
}