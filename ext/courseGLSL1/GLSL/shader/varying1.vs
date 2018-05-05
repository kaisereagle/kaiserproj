#version 110

attribute vec4 color;

varying vec4 revColor;

void main()
{
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
	revColor = vec4(vec3(1.0, 1.0, 1.0) - color.rgb, 1.0);
}
