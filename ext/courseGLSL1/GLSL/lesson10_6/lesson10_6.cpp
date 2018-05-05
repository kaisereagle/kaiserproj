#include "stdafx.h"
#include "main.h"

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glsl.h>

#include <stdio.h>
#include <math.h>

#define PI 3.14159265358979

#define VSHFILENAME	"..\\shader\\lighting.vs"
#define FSHFILENAME	"..\\shader\\lighting.fs"

/* プログラムオブジェクト */
static GLuint program = 0;

/* シェーダオブジェクト */
static GLuint vs = 0;
static GLuint fs = 0;

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
	gluPerspective(40.0f, aspect, 0.1f, 100.0f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0.0f, 1.5f, 4.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f);


	/* 背景色 */
	glClearColor(0.8f, 0.8f, 0.8f, 0.0f);

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);

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

	return 1;
}

void DrawTaurus()
{
#define BLOCKS	36

	const float r0 = 1.0f;
	const float r1 = 0.3f;
	static GLfloat vertex[(BLOCKS + 1) * (BLOCKS + 1) * 8];
	static GLuint indices[BLOCKS * (BLOCKS + 1) * 2];
	static int flag = 0;
	int i;

	/*
	 *
	 *   setup
	 *
	 */

	if (!flag) {
		int idx;
		float theta, pai;
		int i, j, k;

		idx = 0;
		for (theta = 0.0f; theta <= 360.0f; theta += (360.0f / BLOCKS)) {
			float co0 = (float)cos(theta * PI / 180.0f);
			float si0 = (float)sin(theta * PI / 180.0f);

			for (pai = 0.0f; pai <= 360.0f; pai += (360.0f / BLOCKS)) {
				float co1 = (float)cos(pai * PI / 180.0f);
				float si1 = (float)sin(pai * PI / 180.0f);

				/* vertex */
				vertex[idx++] = (r0 + r1 * co1) * co0;
				vertex[idx++] = r1 * si1;
				vertex[idx++] = - (r0 + r1 * co1) * si0;

				/* normal */
				vertex[idx++] = co1 * co0;
				vertex[idx++] = si1;
				vertex[idx++] = - co1 * si0;

				/* texture */
				vertex[idx++] = theta / 360.0f;
				vertex[idx++] = pai / 360.0f;
			}
		}

		k = 0;
		for (i = 0; i < BLOCKS; i++) {
			for (j = 0; j < BLOCKS + 1; j++) {
				indices[k++] = i * (BLOCKS + 1) + j;
				indices[k++] = (i + 1) * (BLOCKS + 1) + j;
			}
		}

		flag = 1;
	}


	/*
	 *
	 *   rendering
	 *
	 */

	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);

	glVertexPointer(3, GL_FLOAT, sizeof(float) * 8, vertex);
	glNormalPointer(GL_FLOAT, sizeof(float) * 8, vertex + 3);
	glTexCoordPointer(3, GL_FLOAT, sizeof(float) * 8, vertex + 6);

	for (i = 0; i < BLOCKS; i++) {
		glDrawElements(GL_TRIANGLE_STRIP, (BLOCKS + 1) * 2, GL_UNSIGNED_INT, indices + (BLOCKS + 1) * i  * 2);
	}

	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
	glDisableClientState(GL_TEXTURE_COORD_ARRAY);
}

/*
 *   シーンの描画
 */
void DrawScene(int width, int height)
{
	static float angle = 0.0f;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glPushMatrix();
//angle = 297.0;
	glRotatef(angle, 1.0f, 1.0f, -1.0f);

	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	DrawTaurus();

	glPopMatrix();

	SwapBuffers(wglGetCurrentDC());

	angle++;
	if (angle > 360.0f) {
		angle -= 360.0f;
	}
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

	/* 垂直同期 */
	if (wglSwapIntervalEXT) {
		wglSwapIntervalEXT(1);
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

	wglMakeCurrent(NULL, NULL);

	wglDeleteContext(wglContext);
}
