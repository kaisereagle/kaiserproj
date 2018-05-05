#version 100

uniform mat4 mvp_matrix;
uniform float time;

attribute vec4 vertex;
attribute vec3 vector;
attribute float startTime;

varying vec4 fragColor;

void main()
{
	vec4 _vertex = vertex;
	vec3 color = vec3(1.0, 1.0, 1.0);

	float t = (time - startTime);

	if (startTime < time) {
		_vertex.xyz += vector * t;
		_vertex.y -= 4.9 * t * t;
	}

	gl_Position = mvp_matrix * _vertex;
	
	color /= (t / 20.0 + 1.0);
	fragColor = vec4(color, 1.0);
}
