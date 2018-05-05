#version 100

precision mediump float;

uniform sampler2D texture0;

varying vec4 texcoord[1];

void main()
{
	vec3 color;
	
	color = texture2D(texture0, texcoord[0].st).rgb;
	
	gl_FragColor = vec4(color, 1.0);
}
