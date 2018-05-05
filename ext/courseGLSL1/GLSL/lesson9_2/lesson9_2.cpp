#include "stdafx.h"
#include "main.h"

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glsl.h>

#include <stdio.h>

#define VSHFILENAME	"..\\shader\\particle.vs"
#define FSHFILENAME	"..\\shader\\particle.fs"

/* プログラムオブジェクト */
static GLuint program = 0;

/* シェーダオブジェクト */
static GLuint vs = 0;
static GLuint fs = 0;

/* attribute インデックス */
const GLuint vectorIndex = 1;
const GLuint startTimeIndex = 2;

/* uniform location */
static GLint time_location;

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
	glOrtho(-width / 2.0f, width / 2.0f, -height / 2.0f, height / 2.0f, -1.0f, 1.0f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();


	/* 背景色 */
	glClearColor(0.2f, 0.2f, 0.2f, 0.0f);

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

	glBindAttribLocation(program, vectorIndex, "vector");
	glBindAttribLocation(program, startTimeIndex, "startTime");

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

	time_location = glGetUniformLocation(program, "time");

	return 1;
}

#define frand()	((rand() % 65536) / 65535.0f)

void
DrawObject()
{
	typedef struct {
		GLfloat vertex[3];
		GLfloat vector[3];
		GLfloat startTime;
	} ATTRIBUTE;

	static ATTRIBUTE attribute[1000];
	int n;

	srand(17);
	for (n = 0; n < 1000; n++) {
		attribute[n].vertex[0] = 200.0f;
		attribute[n].vertex[1] = 0.0f;
		attribute[n].vertex[2] = 0.0f;
		attribute[n].vector[0] = -30.0f + 20.0f * frand();
		attribute[n].vector[1] = 30.0f  + 20.0f * frand();
		attribute[n].vector[2] = 0.0f;
		attribute[n].startTime = n / 100.0f + frand() / 33.0f;
	}

	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(vectorIndex);
	glEnableVertexAttribArray(startTimeIndex);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(ATTRIBUTE), &(attribute[0].vertex[0]));
	glVertexAttribPointer(vectorIndex, 3, GL_FLOAT, GL_FALSE, sizeof(ATTRIBUTE), &(attribute[0].vector[0]));
	glVertexAttribPointer(startTimeIndex, 1, GL_FLOAT, GL_FALSE, sizeof(ATTRIBUTE), &(attribute[0].startTime));
	glDrawArrays(GL_POINTS, 0, n);

	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(vectorIndex);
	glDisableVertexAttribArray(startTimeIndex);
}

/*
 *   経過時間の取得
 */
#include <sys/timeb.h>
#include <time.h>

float GetAppTime(int interval)
{
	float t;
	static int flag = 0;
	static struct _timeb start;
	struct _timeb timebuffer;


	/* 現在時刻を取得 */
	_ftime(&timebuffer);

	if (!flag) {
		/* 最初に GetAppTime をコールした時の時刻をスタート時刻に設定 */
		start = timebuffer;
		flag = 1;
	}

	/*
	 * スタート時刻を変更
	 *     interval が 0 の場合は変更しない
	 */
	if (timebuffer.time - start.time > (float)interval) {
		start.time += interval;
	}

	/* 経過時間(秒)の計算 */
	t = (timebuffer.time - start.time) + (timebuffer.millitm - start.millitm) / 1000.0f;

	return t;
}

/*
 *   シーンの描画
 */
void DrawScene(int width, int height)
{
	float t;


	t = GetAppTime(20);	/* 20秒周期 */

	glClear(GL_COLOR_BUFFER_BIT);

	glUniform1f(time_location, t);

	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	DrawObject();

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
