//#include "ap_main.hpp"

// C �����^�C�� �w�b�_�[ �t�@�C��
#include <windows.h>
#include <GL/gl.h>
#include <GL/glut.h>
#include "common.h"
#include "core.h"

//#include <GL/wglext.h>
//#include <GLES2/gl2.h>

#include <string>

void display()
{}
_MAIN()
{
	nammu::core dr;
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
	glutCreateWindow("hello");
	glutDisplayFunc(display);
	dr.init();
	glClearColor(0.0, 1.0, 1.0, 1.0); //�w�i�F

	glutMainLoop();
	__log("hello\n");
	return 0;
}