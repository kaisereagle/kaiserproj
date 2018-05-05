#include "stdafx.h"
#include "main.h"

#include <GL/gl.h>
#include <GL/wglext.h>
#include <GLES2/gl2.h>

#include <stdio.h>

#include "TexImage.h"

#define VSHFILENAME	"..\\shader\\texture1.vs"
#define FSHFILENAME	"..\\shader\\texture1.fs"

/* プログラムオブジェクト */
static GLuint program = 0;

/* シェーダオブジェクト */
static GLuint vs = 0;
static GLuint fs = 0;

/* テクスチャオブジェクト */
static GLuint tex = 0;

static float mvp[16];       /* ModelViewProjection Matrix */

/* attribute インデックス */
const GLuint vertexIndex = 0;
const GLuint texcoord0Index = 1;

/* uniform location */
static GLint mvp_matrix_location;
static GLint texture0_location;

/* レンダリングコンテキスト */
static HGLRC wglContext = 0;


static void 
LoadMatrix(float mat[16], const float a[16])
{
    memcpy(mat, a, sizeof(float) * 16);
}

static void 
LoadIdentity(float mat[16])
{
    const float identity[16] = {
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f,
    };

    LoadMatrix(mat, identity);
}

static void 
MultMatrixf(float mat[16], const float m[16])
{
    float a[16];
    int i;

    LoadMatrix(a, mat);

#define A(row, col) a[(col << 2) + row]
#define M(row, col) m[(col << 2) + row]
#define MAT(row, col) mat[(col << 2) + row]

    for (i = 0; i < 4; i++) {
        MAT(i, 0) = A(i, 0) * M(0, 0) + A(i, 1) * M(1, 0) + A(i, 2) * M(2, 0) + A(i, 3) * M(3, 0);
        MAT(i, 1) = A(i, 0) * M(0, 1) + A(i, 1) * M(1, 1) + A(i, 2) * M(2, 1) + A(i, 3) * M(3, 1);
        MAT(i, 2) = A(i, 0) * M(0, 2) + A(i, 1) * M(1, 2) + A(i, 2) * M(2, 2) + A(i, 3) * M(3, 2);
        MAT(i, 3) = A(i, 0) * M(0, 3) + A(i, 1) * M(1, 3) + A(i, 2) * M(2, 3) + A(i, 3) * M(3, 3);
    }

#undef A
#undef M
#undef MAT
}

// 補助関数Orthof
void 
Orthof(float mat[16], float left, float right, float bottom, float top, float zNear, float zFar)
{
    float m[16];

    float xx = 1.0f / (right - left);
    float yy = 1.0f / (top - bottom);
    float zz = 1.0f / (zFar - zNear);

#define M(row, col) m[(col << 2) + row]

    M(0, 0) = 2.0f * xx;
    M(0, 1) = 0.0f;
    M(0, 2) = 0.0f;
    M(0, 3) = - (right + left) * xx;

    M(1, 0) = 0.0f;
    M(1, 1) = 2.0f * yy;
    M(1, 2) = 0.0f;
    M(1, 3) = -(top + bottom) * yy;

    M(2, 0) = 0.0f;
    M(2, 1) = 0.0f;
    M(2, 2) = -2.0f * zz;
    M(2, 3) = -(zFar + zNear) * zz;

    M(3, 0) = 0.0f;
    M(3, 1) = 0.0f;
    M(3, 2) = 0.0f;
    M(3, 3) = 1.0f;

#undef M

	MultMatrixf(mat, m);
}

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
	
	/* ビューポート、カメラ設定 */
	glViewport(0, 0, width, height);

    LoadIdentity(mvp);
    Orthof(mvp, -aspect, aspect, -1.0f, 1.0f, -1.0f, 1.0f);

	/* 背景色 */
	glClearColor(0.8f, 0.8f, 0.8f, 0.0f);

	/****/

	glGenTextures(1, &tex);

	image.ReadFile("..\\images\\4.2.03.tiff");

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, tex);

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

    /* シェーダコンパイラのリソース解放 */
    glReleaseShaderCompiler();

	/****/

	program = glCreateProgram();

	glAttachShader(program, vs);
	glAttachShader(program, fs);

	/****/

    glBindAttribLocation(program, vertexIndex, "vertex");
    glBindAttribLocation(program, texcoord0Index, "texcoord0");

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

    mvp_matrix_location = glGetUniformLocation(program, "mvp_matrix");
    texture0_location = glGetUniformLocation(program, "texture0");

	return 1;
}

void DrawRectangle()
{
	const GLfloat v[][5] = {
		{ -0.8f,  0.8f, 0.0f, 0.0f, 0.0f, },
		{ -0.8f, -0.8f, 0.0f, 0.0f, 1.0f, },
		{  0.8f,  0.8f, 0.0f, 1.0f, 0.0f, },
		{  0.8f, -0.8f, 0.0f, 1.0f, 1.0f, },
	};


	glEnableVertexAttribArray(vertexIndex);
	glEnableVertexAttribArray(texcoord0Index);

	glVertexAttribPointer(vertexIndex, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 5, &v[0][0]);
	glVertexAttribPointer(texcoord0Index, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 5, &v[0][3]);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

	glDisableVertexAttribArray(vertexIndex);
	glDisableVertexAttribArray(texcoord0Index);
}

/*
 *   シーンの描画
 */
void DrawScene(int width, int height)
{
	glClear(GL_COLOR_BUFFER_BIT);

	glUniformMatrix4fv(mvp_matrix_location, 1, GL_FALSE, mvp);
	glUniform1i(texture0_location, 0);	/* TEXTURE UNIT 0 */

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
    HGLRC rc;


	hdc = GetDC(window);

	iPixelFormat = ChoosePixelFormat(hdc, &pfd); 
	SetPixelFormat(hdc, iPixelFormat, &pfd);

	rc = wglCreateContext(hdc);

	wglMakeCurrent(hdc, rc);

    /*
     *   OpenGL ES APIの準備
     */
    if (!SetupGLSLES100()) {
        ErrorMessage("Not support OpenGL ES 2.0\n");
        wglMakeCurrent(NULL, NULL);
        wglDeleteContext(rc);
        return 0;
    }

    if (dmpIsSupported("WGL_EXT_create_context_es2_profile")) {
        if (wglCreateContextAttribsARB) {
            /* ES2 PROFILE */
            const int attribList[] = {
                WGL_CONTEXT_MAJOR_VERSION_ARB,  2,
                WGL_CONTEXT_MINOR_VERSION_ARB,  0,
                WGL_CONTEXT_PROFILE_MASK_ARB,   WGL_CONTEXT_ES2_PROFILE_BIT_EXT,
                0,
            };

            wglContext = wglCreateContextAttribsARB(hdc, NULL, attribList);
        }

		if (!wglContext) {
			ErrorMessage("Not support OpenGL ES 2.0\n");
			wglMakeCurrent(NULL, NULL);
			wglDeleteContext(rc);
			return 0;
		}
		wglMakeCurrent(hdc, wglContext);
		wglDeleteContext(rc);
    } else {
	    int major, minor;

	    /* OpenGL バージョンチェック */
	    char *version = (char *)glGetString(GL_VERSION);

	    sscanf(version, "%d.%d", &major, &minor);
	    if (major >= 5 || (major == 4 && major >= 1)) {
		    wglContext = rc;
        } else {
			ErrorMessage("Not support OpenGL ES 2.0\n");
			wglMakeCurrent(NULL, NULL);
			wglDeleteContext(rc);
			return 0;
        }
    }


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
