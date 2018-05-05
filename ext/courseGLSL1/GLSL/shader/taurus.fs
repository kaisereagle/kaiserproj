#version 110

void main()
{
	float intens = gl_FragCoord.z;
	vec3 color = vec3(0.2, 0.2, 1.0);

	gl_FragColor = vec4(color * intens, 1.0);
}
