#version 110

uniform float time;

attribute vec3 vector;
attribute float startTime;

void main()
{
	vec4 vertex = gl_Vertex;
	vec3 color = vec3(1.0, 1.0, 1.0);
	
	float t = (time - startTime);

	if (startTime < time) {
		vertex.xyz += vector * t;
		vertex.y -= 4.9 * t * t;
	}

	gl_Position = gl_ModelViewProjectionMatrix * vertex;
	
	color /= (t / 20.0 + 1.0);
	gl_FrontColor = vec4(color, 1.0);
}