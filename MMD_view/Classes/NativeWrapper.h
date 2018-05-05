/*
 *  NativeWrapper.h
 *  glTest
 *
 *  Created by Interlink on 10/09/27.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
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
#include <OpenGLES/ES1/gl.h>
#include <OpenGLES/ES1/glext.h>

#endif
class NativeWrapper {
	public:
	static void nglNormal3f(float nx, float ny, float nz);
};
