#version 110

uniform sampler2D texture0;

void main()
{
	vec3 color;
	
	color = texture2D(texture0, gl_TexCoord[0].st).rgb;
	
	gl_FragColor = vec4(color, 1.0);
}
