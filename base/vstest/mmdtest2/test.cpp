#include "windows.h"
#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glut.h>


#include <math.h>
#include <string>
#include <stdio.h>
#include <memory.h>

#include "mmd_model.h"


// �Ƃ肠���������Ƀv���g�^�C�v�����Ă���

// �Ƃ肠����
MMD_File    mmdfile;
Texture     madoka_magic;

int WindowWidth = 512;
int WindowHeight = 512;



void Reshape(int x, int y)
{
	WindowWidth = x;
	WindowHeight = y;
	if (WindowWidth < 1) WindowWidth = 1;
	if (WindowHeight < 1) WindowHeight = 1;
}


void disp(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPushMatrix();

	// mmd���̃R�[�h�������ɍČ����Ă����A
	// ���_���ǂ��ɂ��邩�m�F����
	// --------��������--------
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, (double)WindowWidth / (double)WindowHeight, 0.1, 1000.0);
	glViewport(0, 0, WindowWidth, WindowHeight);

	gluLookAt(
		0.0, 10.0, -32.0,
		0.0, 10.0, 0.0,
		0.0, 1.0, 0.0);

	// �����̐ݒ�
	const GLfloat lightPos[] = { 0 , 0 , -2 , 0 };
	const GLfloat lightCol[] = { 1 , 1 , 1 , 1 };
	const GLfloat ambient[] = { 0.3f, 0.3f, 0.3f, 1.0f };
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightCol);
	glEnable(GL_TEXTURE_2D);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_DEPTH_TEST);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	//    glTranslatef(10.0f, 0.0f, -30.0f);        // �E����W�n�Bz+����ʎ�O����
	//    glRotatef(00, 0.0f, 1.0f, 0.0f);
	// -------- �����܂� --------

	// �p�[�c���Ƃɕ`�悷��
	mmdfile.draw();

	glPopMatrix();
}

void timer(int value)
{
	disp();
}


int main(int argc, char ** argv)
{
	glutInit(&argc, argv);
	glutInitWindowPosition(100, 50);
	glutInitWindowSize(WindowWidth, WindowHeight);
	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGBA | GLUT_DEPTH);

	glutCreateWindow("Window caption");
	/* GLEW������ */
	 glewInit();
		glutDisplayFunc(disp);
	glutReshapeFunc(Reshape);
	glutTimerFunc(100, timer, 0);

	mmdfile.setpath("D:/work/baseproj/resource/madoka/maho/");
	mmdfile.load("md_m.pmd");

	glEnable(GL_NORMALIZE);
	glutMainLoop();

	return 0;
}

