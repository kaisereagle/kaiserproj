#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#if defined(WIN32)
//#  pragma comment(linker, "/subsystem:\"windows\" /entry:\"mainCRTStartup\"")
#  include "glut.h"
#  include "glext.h"
#elif defined(__APPLE__) || defined(MACOSX)
#  include <GLUT/glut.h>
#else
#  define GL_GLEXT_PROTOTYPES
#  include <GL/glut.h>
#endif

/*
** ����
*/
static const GLfloat lightpos[] = { 0.0f, 0.0f, 5.0f, 1.0f }; /* �ʒu�@�@�@ */
static const GLfloat lightcol[] = { 1.0f, 1.0f, 1.0f, 1.0f }; /* ���ڌ����x */
static const GLfloat lightamb[] = { 0.1f, 0.1f, 0.1f, 1.0f }; /* �������x */

/*
** ������
*/
static void init(void)
{
  /* �����ݒ� */
  glClearColor(0.3f, 0.3f, 1.0f, 0.0f);
  glEnable(GL_DEPTH_TEST);
  glDisable(GL_CULL_FACE);

  /* �����̏����ݒ� */
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, lightcol);
  glLightfv(GL_LIGHT0, GL_SPECULAR, lightcol);
  glLightfv(GL_LIGHT0, GL_AMBIENT, lightamb);
  glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
}

/*
** �V�[���̕`��
*/
static void scene(void)
{
  static const GLfloat diffuse[] = { 0.6f, 0.1f, 0.1f, 1.0f };
  static const GLfloat specular[] = { 0.3f, 0.3f, 0.3f, 1.0f };

  /* �ގ��̐ݒ� */
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, diffuse);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
  glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 100.0f);

#if 1
  /* �P���̂S�p�`��`�� */
  glNormal3d(0.0, 0.0, 1.0);
  glBegin(GL_QUADS);
  glVertex3d(-1.0, -1.0,  0.0);
  glVertex3d( 1.0, -1.0,  0.0);
  glVertex3d( 1.0,  1.0,  0.0);
  glVertex3d(-1.0,  1.0,  0.0);
  glEnd();
#else
  glutSolidTeapot(1.0);
#endif
}


/****************************
** GLUT �̃R�[���o�b�N�֐� **
****************************/

/* �g���b�N�{�[�������p�֐��̐錾 */
#include "trackball.h"

static void display(void)
{
  /* ���f���r���[�ϊ��s��̏����� */
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  /* ��ʃN���A */
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  /* �����̈ʒu��ݒ� */
  glLightfv(GL_LIGHT0, GL_POSITION, lightpos);

  /* ���_�̈ړ��i���̂̕������Ɉړ��j*/
  glTranslated(0.0, 0.0, -3.0);

  /* �g���b�N�{�[�������ɂ���] */
  glMultMatrixd(trackballRotation());

  /* �V�[���̕`�� */
  scene();

  /* �_�u���o�b�t�@�����O */
  glutSwapBuffers();
}

static void resize(int w, int h)
{
  /* �g���b�N�{�[������͈� */
  trackballRegion(w, h);

  /* �E�B���h�E�S�̂��r���[�|�[�g�ɂ��� */
  glViewport(0, 0, w, h);

  /* �����ϊ��s��̎w�� */
  glMatrixMode(GL_PROJECTION);

  /* �����ϊ��s��̏����� */
  glLoadIdentity();
  gluPerspective(60.0, (double)w / (double)h, 1.0, 100.0);
}

static void idle(void)
{
  /* ��ʂ̕`���ւ� */
  glutPostRedisplay();
}

static void mouse(int button, int state, int x, int y)
{
  switch (button) {
  case GLUT_LEFT_BUTTON:
    switch (state) {
    case GLUT_DOWN:
      /* �g���b�N�{�[���J�n */
      trackballStart(x, y);
      glutIdleFunc(idle);
      break;
    case GLUT_UP:
      /* �g���b�N�{�[����~ */
      trackballStop(x, y);
      glutIdleFunc(0);
      break;
    default:
      break;
    }
    break;
  default:
    break;
  }
}

static void motion(int x, int y)
{
  /* �g���b�N�{�[���ړ� */
  trackballMotion(x, y);
}

static void keyboard(unsigned char key, int x, int y)
{
  switch (key) {
  case 'q':
  case 'Q':
  case '\033':
    /* ESC �� q �� Q ���^�C�v������I�� */
    exit(0);
  default:
    break;
  }
}

/*
** ���C���v���O����
*/
int main(int argc, char *argv[])
{
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
  glutCreateWindow(argv[0]);
  glutDisplayFunc(display);
  glutReshapeFunc(resize);
  glutMouseFunc(mouse);
  glutMotionFunc(motion);
  glutKeyboardFunc(keyboard);
  init();
  glutMainLoop();
  return 0;
}
