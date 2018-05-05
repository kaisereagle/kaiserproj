#version 110

uniform sampler2D texture0;

void main()
{
	vec3 color;
	vec2 texcoord0 = gl_TexCoord[0].st;
	
	texcoord0.s *= 2.0;
	if (texcoord0.s >= 1.0) {
		texcoord0.t *= 2.0;
	}	
	
	color = texture2D(texture0, texcoord0.st).rgb;
	
	if (texcoord0.s >= 1.0 && texcoord0.t >= 1.0) {
		color = 1.0 - color;		
	}

	gl_FragColor = vec4(color, 1.0);
}
