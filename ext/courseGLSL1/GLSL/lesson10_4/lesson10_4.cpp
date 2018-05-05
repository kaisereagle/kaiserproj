#include "stdafx.h"
#include "main.h"

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glsl.h>

#include <stdio.h>

#include "TexImage.h"
#include <math.h>

#define VSHFILENAME	"..\\shader\\texture4.vs"
#define FSHFILENAME	"..\\shader\\texture4.fs"

/* プログラムオブジェクト */
static GLuint program = 0;

/* シェーダオブジェクト */
static GLuint vs = 0;
static GLuint fs = 0;

/* テクスチャオブジェクト */
static GLuint tex[2] = { 0, 0 };

/* uniform location */
static GLint texture0_location;
static GLint texture1_location;

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

	CTexImage image;
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

	glGenTextures(2, tex);

	image.ReadFile("..\\images\\4.2.03.tiff");

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, tex[0]);

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image.m_width, image.m_height, 0, 
		GL_RGBA, GL_UNSIGNED_BYTE, image.m_buffer);

	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	/****/

	image.ReadFile("..\\images\\a.bmp");

	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, tex[1]);

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image.m_width, image.m_height, 0, 
		GL_RGBA, GL_UNSIGNED_BYTE, image.m_buffer);

	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

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

    /****/

	texture0_location = glGetUniformLocation(program, "texture0");
	texture1_location = glGetUniformLocation(program, "texture1");

	return 1;
}

void DrawRectangle()
{
	const GLfloat v[][5] = {
		{ -0.8f,  0.8f, 0.0f, 0.0, 0.0, },
		{ -0.8f, -0.8f, 0.0f, 0.0, 1.0, },
		{  0.8f,  0.8f, 0.0f, 1.0, 0.0, },
		{  0.8f, -0.8f, 0.0f, 1.0, 1.0, },
	};


	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);

	glVertexPointer(3, GL_FLOAT, sizeof(float) * 5, &v[0][0]);
	glTexCoordPointer(2, GL_FLOAT, sizeof(float) * 5, &v[0][3]);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_TEXTURE_COORD_ARRAY);
}

/*
 *   シーンの描画
 */
void DrawScene(int width, int height)
{
    glClear(GL_COLOR_BUFFER_BIT);

	glUniform1i(texture0_location, 0);	/* TEXTURE UNIT 0 */
	glUniform1i(texture1_location, 1);	/* TEXTURE UNIT 1 */

	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	DrawRectangle();

	SwapBuffers(wglGetCurrentDC());
}

/*
 *   WGLCreate: レンダリングコンテキストの作成
 */
int WGLCreate(HWND window)
{
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
	glDeleteTextures(2, tex);

	wglMakeCurrent(NULL, NULL);

	wglDeleteContext(wglContext);
}
