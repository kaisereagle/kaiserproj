#include "stdafx.h"
#include "main.h"

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glsl.h>

#include <stdio.h>

#define NumberOfLines(x)	(sizeof(x) / sizeof(x[0]))

/* �o�[�e�b�N�X�V�F�[�_�\�[�X�v���O���� */
const GLchar* VertexShaderSrc[] = {
	"#version 110												\n",
	"															\n",
	"void main()												\n",
	"{															\n",
	"	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;	\n",
	"	gl_FrontColor = vec4(1.0, 0.0, 0.0, 1.0);				\n",
	"}															\n",
};

/* �t���O�����g�V�F�[�_�\�[�X�v���O���� */
const GLchar* FragmentShaderSrc[] = {
	"#version 110												\n",
	"															\n",
	"void main()												\n",
	"{															\n",
	"	gl_FragColor = gl_Color;								\n",
	"}															\n",
};

/* �v���O�����I�u�W�F�N�g */
static GLuint program = 0;

/* �V�F�[�_�I�u�W�F�N�g */
static GLuint vs = 0;
static GLuint fs = 0;

/* �����_�����O�R���e�L�X�g */
static HGLRC wglContext = 0;

/*
 *   ������
 */
int Initialize(int width, int height)
{
	float aspect = (float)width / (float)height;
	int major, minor;


	/* OpenGL �o�[�W�����`�F�b�N */
	char *version = (char *)glGetString(GL_VERSION);

	sscanf(version, "%d.%d", &major, &minor);
	if (major < 2) {
		TRACE("Need OpenGL 2.0 or later.\n");
		return 0;
	}


	/* �r���[�|�[�g�A�J�����ݒ� */
	glViewport(0, 0, width, height);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-aspect, aspect, -1.0f, 1.0f, -1.0f, 1.0f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();


	/* �w�i�F */
	glClearColor(0.8f, 0.8f, 0.8f, 0.0f);


	/*
	 *   �o�[�e�b�N�X�V�F�[�_�̃R���p�C��
	 */

	vs = glCreateShader(GL_VERTEX_SHADER);

	glShaderSource(vs, NumberOfLines(VertexShaderSrc), VertexShaderSrc, NULL);
	glCompileShader(vs);

	/*
	 *   �t���O�����g�V�F�[�_�̃R���p�C��
	 */

	fs = glCreateShader(GL_FRAGMENT_SHADER);

	glShaderSource(fs, NumberOfLines(FragmentShaderSrc), FragmentShaderSrc, NULL);
	glCompileShader(fs);


	/****/

	program = glCreateProgram();

	glAttachShader(program, vs);
	glAttachShader(program, fs);


	/*
	 *   �����N
	 */

	glLinkProgram(program);
	
	glUseProgram(program);
/*	glUseProgram(0);	*/

	return 1;
}

void DrawRectangle()
{
	const GLfloat v[][3] = {
		{ -0.8f, -0.8f, 0.0f, },
		{ -0.8f,  0.8f, 0.0f, },
		{  0.8f, -0.8f, 0.0f, },
		{  0.8f,  0.8f, 0.0f, },
	};


	glEnableClientState(GL_VERTEX_ARRAY);

	glVertexPointer(3, GL_FLOAT, 0, &v[0][0]);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

	glDisableClientState(GL_VERTEX_ARRAY);
}

/*
 *   �V�[���̕`��
 */
void DrawScene(int width, int height)
{
	glClear(GL_COLOR_BUFFER_BIT);

	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	DrawRectangle();

	SwapBuffers(wglGetCurrentDC());
}

/*
 *   WGLCreate: �����_�����O�R���e�L�X�g�̍쐬
 */
int WGLCreate(HWND window)
{
	int value;

	PIXELFORMATDESCRIPTOR pfd = { 
		sizeof(PIXELFORMATDESCRIPTOR),   // size of this pfd 
		1,                     // version number 
		PFD_DRAW_TO_WINDOW |   // support window 
		PFD_SUPPORT_OPENGL |   // support OpenGL 
		PFD_DOUBLEBUFFER,      // double buffered 
		PFD_TYPE_RGBA,         // RGBA type 
		32,                    // 32-bit color depth 
		0, 0, 0, 0, 0, 0,      // color bits ignored 
		0,                     // no alpha buffer 
		0,                     // shift bit ignored 
		0,                     // no accumulation buffer 
		0, 0, 0, 0,            // accum bits ignored 
		32,                    // 32-bit z-buffer 
		0,                     // no stencil buffer 
		0,                     // no auxiliary buffer 
		PFD_MAIN_PLANE,        // main layer 
		0,                     // reserved 
		0, 0, 0                // layer masks ignored 
	};
	int  iPixelFormat; 
	HDC hdc;


	hdc = GetDC(window);

	iPixelFormat = ChoosePixelFormat(hdc, &pfd); 
	SetPixelFormat(hdc, iPixelFormat, &pfd);

	wglContext = wglCreateContext(hdc);

	wglMakeCurrent(hdc, wglContext);

	/*
	 *   �V�F�[�_API�̏���
	 */
	SetupGLSL();

	TRACE("GL_VENDOR                   : %s\n", glGetString(GL_VENDOR));
	TRACE("GL_RENDERER                 : %s\n", glGetString(GL_RENDERER));
	TRACE("GL_VERSION                  : %s\n", glGetString(GL_VERSION));
	TRACE("GL_SHADING_LANGUAGE_VERSION : %s\n", glGetString(GL_SHADING_LANGUAGE_VERSION));
	TRACE("GL_EXTENSIONS               : %s\n", glGetString(GL_EXTENSIONS));

	if (wglGetExtensionsStringARB) {
		TRACE("WGL_EXTENSIONS              : %s\n", wglGetExtensionsStringARB(hdc));
	}


	// gl_MaxLights = 8
	// gl_MaxClipPlanes = 6
	// gl_MaxTextureUnits = 2
	// gl_MaxTextureCoords = 2
	// gl_MaxVertexAttribs = 16
	// gl_MaxVertexUniformComponents = 512
	// gl_MaxVaryingFloats = 32
	// gl_MaxVertexTextureImageUnits = 0
	// gl_MaxTextureImageUnits = 2
	// gl_MaxFragmentUniformComponents = 64
	// gl_MaxCombinedTextureImageUnits = 2

	glGetIntegerv(GL_MAX_LIGHTS, &value);
	TRACE("gl_MaxLights                    (%4d) : %4d\n", 8, value);
	glGetIntegerv(GL_MAX_CLIP_PLANES, &value);
	TRACE("gl_MaxClipPlanes                (%4d) : %4d\n", 6, value);
	glGetIntegerv(GL_MAX_TEXTURE_UNITS, &value);
	TRACE("gl_MaxTextureUnits              (%4d) : %4d\n", 2, value);
	glGetIntegerv(GL_MAX_TEXTURE_COORDS, &value);
	TRACE("gl_MaxTextureCoords             (%4d) : %4d\n", 2, value);
	glGetIntegerv(GL_MAX_VERTEX_ATTRIBS, &value);
	TRACE("gl_MaxVertexAttribs             (%4d) : %4d\n", 16, value);
	glGetIntegerv(GL_MAX_VERTEX_UNIFORM_COMPONENTS, &value);
	TRACE("gl_MaxVertexUniformComponents   (%4d) : %4d\n", 512, value);
	glGetIntegerv(GL_MAX_VARYING_FLOATS, &value);
	TRACE("gl_MaxVaryingFloats             (%4d) : %4d\n", 32, value);
	glGetIntegerv(GL_MAX_VERTEX_TEXTURE_IMAGE_UNITS, &value);
	TRACE("gl_MaxVertexTextureImageUnits   (%4d) : %4d\n", 0, value);
	glGetIntegerv(GL_MAX_TEXTURE_IMAGE_UNITS, &value);
	TRACE("gl_MaxTextureImageUnits         (%4d) : %4d\n", 2, value);
	glGetIntegerv(GL_MAX_FRAGMENT_UNIFORM_COMPONENTS, &value);
	TRACE("gl_MaxFragmentUniformComponents (%4d) : %4d\n", 64, value);
	glGetIntegerv(GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS, &value);
	TRACE("gl_MaxCombinedTextureImageUnits (%4d) : %4d\n", 2, value);

    return 1;
}

/*
 *   WGLDestroy: �����_�����O�R���e�L�X�g�̍폜
 */
void WGLDestroy()
{
	glDetachShader(program, vs);
	glDetachShader(program, fs);

	glDeleteShader(vs);
	glDeleteShader(fs);

	glDeleteProgram(program);

	wglMakeCurrent(NULL, NULL);

	wglDeleteContext(wglContext);
}
