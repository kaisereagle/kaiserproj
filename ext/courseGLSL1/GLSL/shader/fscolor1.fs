#version 110

// WINDOW SIZE 640 x 480
#define WIDTH	640		
#define	HEIGHT	480

void main()
{
	vec3 red = vec3(1.0, 0.0, 0.0);
	vec3 green = vec3(0.0, 1.0, 0.0);
	vec3 blue = vec3(0.0, 0.0, 1.0);
	vec3 white = vec3(1.0, 1.0, 1.0);
	vec3 color;
	
	
	if (gl_FragCoord.x < float(WIDTH / 2)) {
		if (gl_FragCoord.y < float(HEIGHT / 2)) {
			color = red;
		} else {
			color = green;
		}	
	} else {
		if (gl_FragCoord.y < float(HEIGHT / 2)) {
			color = blue;
		} else {
			color = white;
		}	
	}

	gl_FragColor = vec4(color, 1.0);
}
