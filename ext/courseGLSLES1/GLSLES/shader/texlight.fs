#version 100

precision mediump float;

uniform sampler2D texture0;

invariant varying vec3 Hvec;
invariant varying vec3 Normal;
invariant varying vec3 Light;
invariant varying vec4 texcoord[1];

void main()
{
	const vec3 ca = vec3(0.2);
	const vec3 cd = vec3(0.8);
	const vec3 cs = vec3(1.0);
	const float power = 64.0;
	vec3 spec;


	vec3 tex = texture2D(texture0, texcoord[0].st).rgb;

	vec3 normal = normalize(Normal);
	vec3 light = normalize(Light);

	float nl = dot(normal, light);

	if (nl <= 0.0) {
		spec = vec3(0.0);
	} else {
		vec3 hvec = normalize(Hvec);
		spec = cs * pow(max(dot(normal, hvec), 0.0), power);
	}

	vec3 diff = (ca + cd * nl) * tex;
	
	gl_FragColor = vec4(diff + spec, 1.0);
}
