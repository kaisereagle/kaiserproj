/*
 *  GlutTrans.h
 *  glTest
 *
 *
 */
#ifdef WIN32
#include <windows.h>
#include "windowsx.h"
#include <winuser.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include    <stdio.h>
#include    <stdlib.h>
#include    <stdbool.h>
#include    <assert.h>
#include    <string.h>
#define _USE_MATH_DEFINES
#include <math.h>
#else
#endif
#include "GlTrans.h"
#ifdef __APPLE__
#include <OpenGLES/ES1/gl.h>
#include <OpenGLES/ES1/glext.h>
#endif
class GlutTrans {
	public:
	static void glutWireCube( GLfloat dSize);
	static void glutSolidCube( GLfloat dSize);
	static void gluOrtho2D(	GLfloat left , GLfloat right , GLfloat bottom , GLfloat top );

	
};
