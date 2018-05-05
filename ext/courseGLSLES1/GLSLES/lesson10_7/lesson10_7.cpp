#include "stdafx.h"
#include "main.h"

#include <GL/gl.h>
#include <GL/wglext.h>
#include <GLES2/gl2.h>

#include <stdio.h>
#include <math.h>
#include "TexImage.h"

#define PI 3.14159265358979

#define VSHFILENAME	"..\\shader\\texlight.vs"
#define FSHFILENAME	"..\\shader\\texlight.fs"

/* プログラムオブジェクト */
static GLuint program = 0;

/* シェーダオブジェクト */
static GLuint vs = 0;
static GLuint fs = 0;

/* テクスチャオブジェクト */
static GLuint tex = 0;

static float projection[16];
static float modelview[16];
static float mvp[16];           /* ModelViewProjection Matrix */

/* attribute インデックス */
const GLuint vertexIndex = 0;
const GLuint normalIndex = 1;
const GLuint texcoord0Index = 2;

/* uniform location */
static GLint modelview_matrix_location;
static GLint mvp_matrix_location;
static GLint normal_matrix_location;
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

static void 
Translatef(float mat[16], float x, float y, float z)
{
    mat[12] = mat[0] * x + mat[4] * y + mat[8] * z + mat[12];
    mat[13] = mat[1] * x + mat[5] * y + mat[9] * z + mat[13];
    mat[14] = mat[2] * x + mat[6] * y + mat[10] * z + mat[14];
    mat[15] = mat[3] * x + mat[7] * y + mat[11] * z + mat[15];
}

static void 
Rotatef(float *mat, float angle, float x, float y, float z)
{
    float m[16];
    float theta;
    float s, c, xx, yy, zz, xy, yz, zx, xs, ys, zs, one_c;
    float mag, invMag;

    mag = (float)sqrt(x * x + y * y + z * z);
    if (mag == 0.0f) {
        return;
    }

    theta = (float)(angle * PI / 180.0);
    s = (float)sin(theta);
    c = (float)cos(theta);

    invMag = 1.0f / mag;
    x *= invMag;
    y *= invMag;
    z *= invMag;

    xx = x * x;
    yy = y * y;
    zz = z * z;
    xy = x * y;
    yz = y * z;
    zx = z * x;
    xs = x * s;
    ys = y * s;
    zs = z * s;
    one_c = 1.0f - c;

#define M(row, col) m[(col << 2) + row]

    M(0, 0) = (one_c * xx) + c;
    M(0, 1) = (one_c * xy) - zs;
    M(0, 2) = (one_c * zx) + ys;
    M(0, 3) = 0.0f;

    M(1, 0) = (one_c * xy) + zs;
    M(1, 1) = (one_c * yy) + c;
    M(1, 2) = (one_c * yz) - xs;
    M(1, 3) = 0.0f;

    M(2, 0) = (one_c * zx) - ys;
    M(2, 1) = (one_c * yz) + xs;
    M(2, 2) = (one_c * zz) + c;
    M(2, 3) = 0.0f;

    M(3, 0) = 0.0f;
    M(3, 1) = 0.0f;
    M(3, 2) = 0.0f;
    M(3, 3) = 1.0f;

#undef M

	MultMatrixf(mat, m);
}

static void 
Scalef(float mat[16], float x, float y, float z)
{
    mat[0] *= x; mat[4] *= y; mat[8] *= z;
    mat[1] *= x; mat[5] *= y; mat[9] *= z;
    mat[2] *= x; mat[6] *= y; mat[10] *= z;
    mat[3] *= x; mat[7] *= y; mat[11] *= z;
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

// 補助関数Perspectivef
void
Perspectivef(float mat[16], float fovy, float aspect, float zNear, float zFar)
{
    float m[16];
	float left, right, top, bottom;
    float xx, yy, zz;
    float n2 = zNear + zNear;

	top = (float)tan(fovy / 2.0f * PI / 180.0) * zNear;
	bottom = -top;
	left = aspect * bottom;
	right = -left;

    xx = 1.0f / (right - left);
    yy = 1.0f / (top - bottom);
    zz = 1.0f / (zFar - zNear);

#define M(row, col) m[col * 4 + row]

    M(0, 0) = n2 * xx;
    M(0, 1) = 0.0f;
    M(0, 2) = (right + left) * xx;
    M(0, 3) = 0.0f;
    
    M(1, 0) = 0.0f;
    M(1, 1) = n2 * yy;
    M(1, 2) = (top + bottom) * yy;
    M(1, 3) = 0.0f;

    M(2, 0) = 0.0f;
    M(2, 1) = 0.0f;
    M(2, 2) = -(zFar + zNear) * zz;
    M(2, 3) = -(n2 * zFar) * zz;

    M(3, 0) = 0.0f;
    M(3, 1) = 0.0f;
    M(3, 2) = -1.0f;
    M(3, 3) = 0.0f;

#undef M

	MultMatrixf(mat, m);
}

// 正規化
static void
normalize(float v[3])
{
	float l;

	l = (float)sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
	if (l == 0.0f) {
		return;
	}

	v[0] /= l;
	v[1] /= l;
	v[2] /= l;
}

// 外積
static void
cross(float v1[3], float v2[3], float result[3])
{
	result[0] = v1[1] * v2[2] - v1[2] * v2[1];
	result[1] = v1[2] * v2[0] - v1[0] * v2[2];
	result[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

// 補助関数LookAtf
void
LookAtf(float mat[16], float eyex, float eyey, float eyez, float tarx, float tary, float tarz, float upx, float upy, float upz)
{
	float view[3], up[3], side[3];
	float m[16];

	view[0] = tarx - eyex;
	view[1] = tary - eyey;
	view[2] = tarz - eyez;

	up[0] = upx;
	up[1] = upy;
	up[2] = upz;

	normalize(view);
	normalize(up);

	cross(view, up, side);
	normalize(side);

	cross(side, view, up);

#define M(row, col) m[(col << 2) + row]

	M(0, 0) = side[0];
	M(0, 1) = side[1];
	M(0, 2) = side[2];
	M(0, 3) = 0.0f;

	M(1, 0) = up[0];
	M(1, 1) = up[1];
	M(1, 2) = up[2];
	M(1, 3) = 0.0f;

	M(2, 0) = -view[0];
	M(2, 1) = -view[1];
	M(2, 2) = -view[2];
	M(2, 3) = 0.0f;

	M(3, 0) = 0.0f;
	M(3, 1) = 0.0f;
	M(3, 2) = 0.0f;
	M(3, 3) = 1.0f;

#undef M

	MultMatrixf(mat, m);
    Translatef(mat, -eyex, -eyey, -eyez);
}

static void 
NormalMatrix(float mat[9], float mv[16])
{
#define MV(row, col) mv[(col << 2) + row]
#define M(row, col) mat[col * 3 + row]

    float delta = 
        MV(0, 0) * (MV(1, 1) * MV(2, 2) - MV(1, 2) * MV(2, 1)) +
        MV(1, 0) * (MV(2, 1) * MV(0, 2) - MV(2, 2) * MV(0, 1)) +
        MV(2, 0) * (MV(0, 1) * MV(1, 2) - MV(0, 2) * MV(1, 1));
    float invDelta;

    if (delta == 0.0f) {
        return;
    }

    invDelta = 1.0f / delta;

    M(0, 0) = (MV(1, 1) * MV(2, 2) - MV(1, 2) * MV(2, 1)) * invDelta;
    M(0, 1) = (MV(1, 2) * MV(2, 0) - MV(1, 0) * MV(2, 2)) * invDelta;
    M(0, 2) = (MV(1, 0) * MV(2, 1) - MV(1, 1) * MV(2, 0)) * invDelta;

    M(1, 0) = (MV(2, 1) * MV(0, 2) - MV(2, 2) * MV(0, 1)) * invDelta;
    M(1, 1) = (MV(2, 2) * MV(0, 0) - MV(2, 0) * MV(0, 2)) * invDelta;
    M(1, 2) = (MV(2, 0) * MV(0, 1) - MV(2, 1) * MV(0, 0)) * invDelta;

    M(2, 0) = (MV(0, 1) * MV(1, 2) - MV(0, 2) * MV(1, 1)) * invDelta;
    M(2, 1) = (MV(0, 2) * MV(1, 0) - MV(0, 0) * MV(1, 2)) * invDelta;
    M(2, 2) = (MV(0, 0) * MV(1, 1) - MV(0, 1) * MV(1, 0)) * invDelta;

#undef MV
#undef M
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

    LoadIdentity(projection);
    Perspectivef(projection, 40.0f, aspect, 0.1f, 100.0f);
    LoadIdentity(modelview);
	LookAtf(modelview, 0.0f, 1.5f, 4.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f);

	/* 背景色 */
	glClearColor(0.8f, 0.8f, 0.8f, 0.0f);

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);

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
    glBindAttribLocation(program, normalIndex, "normal");
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

    /***/

	modelview_matrix_location = glGetUniformLocation(program, "modelview_matrix");
	mvp_matrix_location = glGetUniformLocation(program, "mvp_matrix");
	normal_matrix_location = glGetUniformLocation(program, "normal_matrix");
	texture0_location = glGetUniformLocation(program, "texture0");

	return 1;
}

void DrawTaurus()
{
#define BLOCKS	36

	const float r0 = 1.0f;
	const float r1 = 0.3f;
	static GLfloat vertex[(BLOCKS + 1) * (BLOCKS + 1) * 8];
	static GLushort indices[BLOCKS * (BLOCKS + 1) * 2];
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

	glEnableVertexAttribArray(vertexIndex);
	glEnableVertexAttribArray(normalIndex);
	glEnableVertexAttribArray(texcoord0Index);

	glVertexAttribPointer(vertexIndex, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 8, vertex);
	glVertexAttribPointer(normalIndex, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 8, vertex + 3);
	glVertexAttribPointer(texcoord0Index, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 8, vertex + 6);

	for (i = 0; i < BLOCKS; i++) {
		glDrawElements(GL_TRIANGLE_STRIP, (BLOCKS + 1) * 2, GL_UNSIGNED_SHORT, indices + (BLOCKS + 1) * i  * 2);
	}

	glDisableVertexAttribArray(vertexIndex);
	glDisableVertexAttribArray(normalIndex);
	glDisableVertexAttribArray(texcoord0Index);
}

/*
 *   シーンの描画
 */
void DrawScene(int width, int height)
{
	static float angle = 0.0f;
    float matModelView[16];
    float matNormal[9];

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    LoadMatrix(matModelView, modelview);
//angle = 297.0;
    Rotatef(matModelView, angle, 1.0f, 1.0f, -1.0f);

    NormalMatrix(matNormal, matModelView);

    LoadMatrix(mvp, projection);
    MultMatrixf(mvp, matModelView);

    glUniformMatrix4fv(modelview_matrix_location, 1, GL_FALSE, matModelView);
    glUniformMatrix4fv(mvp_matrix_location, 1, GL_FALSE, mvp);
    glUniformMatrix3fv(normal_matrix_location, 1, GL_FALSE, matNormal);
	glUniform1i(texture0_location, 0);	/* TEXTURE UNIT 0 */

	DrawTaurus();

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
	glDeleteTextures(1, &tex);

	wglMakeCurrent(NULL, NULL);

	wglDeleteContext(wglContext);
}
