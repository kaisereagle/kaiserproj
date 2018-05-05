#version 110	

varying vec3 Hvec;
varying vec3 Normal;
varying vec3 Light;

void main()
{
	const vec4 light = vec4(1.0, 1.0, 1.0, 0.0);
	
	vec4 position = gl_ModelViewMatrix * gl_Vertex;

	vec3 view = normalize(vec3(0.0) - position.xyz);
	Normal = normalize(gl_NormalMatrix * gl_Normal);
	Light = normalize(light.xyz);
	
	Hvec = normalize(Light + view);
	
	gl_TexCoord[0] = gl_MultiTexCoord0;
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
}
