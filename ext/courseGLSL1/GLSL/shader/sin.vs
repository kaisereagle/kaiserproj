#version 110

uniform float time;

attribute float startTime;

void main()
{
	vec4 vertex = gl_Vertex;

	if (startTime < time) {
		vertex.y = 200.0 * sin((time - startTime) * 3.14159 / 180.0);
	}
	
	gl_Position = gl_ModelViewProjectionMatrix * vertex;
}
