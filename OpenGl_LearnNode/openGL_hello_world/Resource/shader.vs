#version 330 core
layout (location=0) in vec3 aPos;
layout (location=1) in vec3 aColor;
layout (location=2) in vec2 aTexCoord;
out vec3 ourColor;
out vec4 ourPos;
out vec2 TexCoord;
uniform float myPos;
void main()
{
	gl_Position=vec4(aPos.x+myPos,aPos.y,aPos.z,1.0);
	ourPos=vec4(aPos.x+myPos,aPos.y,aPos.z,1.0);
	ourColor=aColor;
	TexCoord = aTexCoord;
}