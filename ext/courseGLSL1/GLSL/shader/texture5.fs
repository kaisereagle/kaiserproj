#version 110

uniform sampler2D texture0;
uniform sampler2D texture1;

void main()
{
	vec3 color;
	vec2 texcoord0 = gl_TexCoord[0].st;
	vec4 mask;
	
	mask = texture2D(texture1, texcoord0.st);
	
	texcoord0.s *= 2.0;
	if (texcoord0.s >= 1.0) {
		texcoord0.t *= 2.0;
	}	
	
	if (texcoord0.s >= 1.0 && texcoord0.t < 1.0) {
		color = texture2D(texture1, texcoord0.st).rgb;
	} else {
		color = texture2D(texture0, texcoord0.st).rgb;
	}
	
	if (texcoord0.s >= 1.0 && texcoord0.t >= 1.0) {
		color = 1.0 - color;		
	}

	color = mix(vec3(1.0, 1.0, 0.0), color, mask.a);

	gl_FragColor = vec4(color, 1.0);
}
