#version 110

void main()
{
	const vec4 directional = vec4(1.0, 1.0, 1.0, 0.0);
	const vec3 color = vec3(0.4, 0.4, 1.0);
	const vec3 ca = 0.2 * color;
	const vec3 cd = 0.8 * color;
	const vec3 cs = vec3(1.0);
	const float power = 64.0;
	vec3 spec;
	
	vec4 position = gl_ModelViewMatrix * gl_Vertex;

	vec3 view = normalize(vec3(0.0) - position.xyz);
	vec3 normal = normalize(gl_NormalMatrix * gl_Normal);
	vec3 light = normalize(directional.xyz);
	
	float nl = dot(normal, light);
	
	if (nl <= 0.0) {
		spec = vec3(0.0);
	} else {
		vec3 hvec = normalize(light + view);
		spec = cs * pow(max(dot(normal, hvec), 0.0), power);
	}
	
	vec3 diff = ca + cd * nl;	
	
	gl_FrontColor = vec4(diff + spec, 1.0);
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
}
