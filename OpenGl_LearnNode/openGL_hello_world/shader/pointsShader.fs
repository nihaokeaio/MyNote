#version 330 core

out vec4 FragColor;
in vec3 fcolor;

void main()
{
	//FragColor=vec4(0.0, 1.0, 0.0, 1.0);
	FragColor=vec4(fcolor, 1.0);
}