#version 110

vec4 myFunc(in vec4 vertex, out vec4 color)
{
	const vec4 red = vec4(1.0, 0.0, 0.0, 1.0);

	color = red;

	return gl_ModelViewProjectionMatrix * vertex;
}

void main()
{
	vec4 color;
	
	gl_Position = myFunc(gl_Vertex, color);
	gl_FrontColor = color;
}
