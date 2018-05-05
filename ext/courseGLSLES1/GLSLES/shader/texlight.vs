#version 100	

uniform mat4 modelview_matrix;
uniform mat4 mvp_matrix;
uniform mat3 normal_matrix;

attribute vec4 vertex;
attribute vec3 normal;
attribute vec4 texcoord0;

invariant varying vec3 Hvec;
invariant varying vec3 Normal;
invariant varying vec3 Light;
invariant varying vec4 texcoord[1];
invariant gl_Position;

void main()
{
	const vec4 light = vec4(1.0, 1.0, 1.0, 0.0);

	vec4 position = modelview_matrix * vertex;

	vec3 view = normalize(vec3(0.0) - position.xyz);
	Normal = normalize(normal_matrix * normal);
	Light = normalize(light.xyz);

	Hvec = normalize(Light + view);

	texcoord[0] = texcoord0;
	gl_Position = mvp_matrix * vertex;
}
