#version 100

uniform mat4 mvp_matrix;

attribute vec4 vertex;
attribute vec4 texcoord0;

varying vec4 texcoord[1];

void main()
{
	texcoord[0].st = texcoord0.st;
	gl_Position = mvp_matrix * vertex;
}
