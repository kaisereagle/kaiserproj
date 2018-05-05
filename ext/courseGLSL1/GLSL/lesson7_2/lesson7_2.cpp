#include "stdafx.h"
#include "main.h"

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glsl.h>

#include <stdio.h>

#define VSHFILENAME	"..\\shader\\attrib2.vs"
#define FSHFILENAME	"..\\shader\\attrib2.fs"

/* プログラムオブジェクト */
static GLuint program = 0;

/* シェーダオブジェクト */
static GLuint vs = 0;
static GLuint fs = 0;

/* attribute インデックス */
const GLuint colorIndex = 3;

/* レンダリングコンテキスト */
static HGLRC wglContext = 0;

/*
 *   補助関数: loadShaderFile
 */
static GLuint loadShaderFile(GLenum shaderType, const char *filename)
{
	FILE *fp;
	size_t l;
	GLchar *shaderSrc;
	GLuint shader;


	if ((fp = fopen(filename, "rb")) == NULL) {
		TRACE("ERROR: can not open. : %s\n", filename);
		return 0;
	}

	fseek(fp, 0, SEEK_END);
	size_t len = ftell(fp);
	fseek(fp, 0, SEEK_SET);

	if (!(shaderSrc = (GLchar *)calloc(len + 1, sizeof(char)))) {
		TRACE("ERROR: can not allocate memory.\n");
		fclose(fp);
		return 0;
	}

	if ((l = fread((void *)shaderSrc, sizeof(char), len, fp)) != len) {
		TRACE("ERROR: can not read file. %d %d\n", l, len);
		fclose(fp);
		free((void *)shaderSrc);
		return 0;
	}
	fclose(fp);


	/*
	 *   シェーダのコンパイル
	 */

	shader = glCreateShader(shaderType);

	if (shader) {
		GLint compiled;
		const GLchar *src = shaderSrc;


		glShaderSource(shader, 1, &src, NULL);

		glCompileShader(shader);

		glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);

		if (!compiled) {
			GLint len;
			GLchar *log;

			glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &len);
			log = (GLchar *)malloc(len);
			
			glGetShaderInfoLog(shader, len, &len, log);

			TRACE("*****\n");
			if (shaderType == GL_VERTEX_SHADER) {
				TRACE("VS ");
			} else {
				TRACE("FS ");
			}
			TRACE("Compile\n");
			TRACE("*****\n");
			TRACE("%s\n", log);

			free(log);
		}
	}
	free((void *)shaderSrc);

	return shader;
}

/*
 *   初期化
 */
int Initialize(int width, int height)
{
	float aspect = (float)width / (float)height;
	GLint linked;

	int major, minor;

	/* OpenGL バージョンチェック */
	char *version = (char *)glGetString(GL_VERSION);


	sscanf(version, "%d.%d", &major, &minor);
	if (major < 2) {
		TRACE("Need OpenGL 2.0 or later.\n");
		return 0;
	}

	
	/* ビューポート、カメラ設定 */
	glViewport(0, 0, width, height);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-aspect, aspect, -1.0f, 1.0f, -1.0f, 1.0f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();


	/* 背景色 */
	glClearColor(0.8f, 0.8f, 0.8f, 0.0f);

	/****/

	if ((vs = loadShaderFile(GL_VERTEX_SHADER, VSHFILENAME)) == 0) {
		TRACE("ERROR: can not read shader. : %s\n", VSHFILENAME);
		return 0;
	}

	/****/

	if ((fs = loadShaderFile(GL_FRAGMENT_SHADER, FSHFILENAME)) == 0) {
		TRACE("ERROR: can not read shader. : %s\n", FSHFILENAME);
		return 0;
	}

	program = glCreateProgram();

	glAttachShader(program, vs);
	glAttachShader(program, fs);

	glBindAttribLocation(program, colorIndex, "color");

	/*
	 *   リンク
	 */

	glLinkProgram(program);
	
	glGetProgramiv(program, GL_LINK_STATUS, &linked);

	if (linked) {
		glUseProgram(program);
	} else {
		GLint len;
		GLchar *log;

		glGetProgramiv(program, GL_INFO_LOG_LENGTH, &len);
		log = (GLchar *)malloc(len);
		
		glGetProgramInfoLog(program, len, &len, log);
		TRACE("*****\n");
		TRACE("Link\n");
		TRACE("*****\n");
		TRACE("%s\n", log);

		free(log);
	}

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
	const GLubyte c[][4] = {
		{ 255,  64,  64, 255, },
		{  64, 255,  64, 255, },
		{  64,  64, 255, 255, },
		{ 255, 255, 255, 255, },
	};

	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(colorIndex);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, &v[0][0]);
	glVertexAttribPointer(colorIndex, 4, GL_UNSIGNED_BYTE, GL_TRUE, 0, &c[0][0]);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(colorIndex);
}

/*
 *   シーンの描画
 */
void DrawScene(int width, int height)
{
	glClear(GL_COLOR_BUFFER_BIT);

	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	DrawRectangle();

	SwapBuffers(wglGetCurrentDC());
}

/*
 *   WGLCreate: レンダリングコンテキストの作成
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
	 *   シェーダAPIの準備
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
 *   WGLDestroy: レンダリングコンテキストの削除
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
