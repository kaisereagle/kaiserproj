#include <windows.h>
#include "windowsx.h"
#include <winuser.h>
#include <GL/gl.h>
#include <GL/glut.h>

#include <stdlib.h>
#include <malloc.h>
#include <memory.h>
#include <tchar.h>
#include <math.h>
#include <string>
#include <stdio.h>
#include <memory.h>
#include <assert.h>
#include <vector>
#include <map>
#if !defined( _MSC_VER )
#include <alloca.h>
#endif
#include "main.h"

// ���g���G���f�B�A�����ۂ�
#define ISLITTLEENDIAN 1

// �e��f�[�^�^

#define WORD unsigned short
#define DWORD unsigned int
#define LONG int
#define BYTE unsigned char
#define BOOL int
#define FALSE 0
#define TRUE 1



// �s�N�Z���̃f�[�^��ǂނ��߂̃o�b�t�@�̍\����
typedef struct tagBuf
{
	union {
		unsigned int buf : 32;
		unsigned char vbuf[4];
	} BufU;
	int buflen;
} BUF;

#define BITMAPFILEHEADER_SIZE ( sizeof( WORD ) * 3 + sizeof( DWORD ) * 2 )
#define BITMAPINFOHEADER_SIZE ( sizeof( WORD ) * 2 + sizeof( DWORD ) * 5 + sizeof( LONG ) * 4 )


static int PixIdx(int w, int h, const IBMP *pBmp);


///////////////////////////////////////////////////////////////////////////////////
// �\�z�E�j��

// �\�z
IBMP* BmpIO_CreateBitmap(int width, int height, int BitPerPixcel)
{
	IBMP *p = NULL;

	// �r�b�g���̎w����m�F����B
	assert(BitPerPixcel == 24 ||
		BitPerPixcel == 8 ||
		BitPerPixcel == 4 ||
		BitPerPixcel == 1);
	if (BitPerPixcel != 24 && BitPerPixcel != 8 &&
		BitPerPixcel != 4 && BitPerPixcel != 1)
		return NULL;

	p = (IBMP*)malloc(sizeof(IBMP));
	if (NULL == p) return NULL;
	p->pColor = NULL;
	p->pPix = NULL;

	// 24Bit�J���[�̏ꍇ
	if (24 == BitPerPixcel) {
		// p->pPix�͎g�p�����Ap->pColor�Ƀs�N�Z���f�[�^���i�[����B
		p->pColor = (ICOLOR*)calloc(width * height, sizeof(ICOLOR));
		if (NULL == p->pColor) goto ERR_EXIT;
	}
	else {
		// p->pColor�ɂ̓J���[�e�[�u�����i�[����B
		p->pColor = (ICOLOR*)calloc((1 << BitPerPixcel), sizeof(ICOLOR));
		p->pPix = (unsigned char*)calloc(width * height, sizeof(unsigned char));
		if (NULL == p->pColor || NULL == p->pPix) goto ERR_EXIT;
	}

	p->width = width;
	p->height = height;
	p->BitPerPix = BitPerPixcel;

	return p;

	// ���s�����ꍇ
ERR_EXIT:
	free(p->pColor);
	free(p->pPix);
	free(p);
	return NULL;
}

// �j��
void BmpIO_DeleteBitmap(IBMP *pBmp)
{
	if (NULL == pBmp) return;
	if (NULL != pBmp->pPix) free(pBmp->pPix);
	if (NULL != pBmp->pColor) free(pBmp->pColor);
	free(pBmp);
}


int BmpIO_GetWidth(const IBMP *pBmp)
{
	assert(NULL != pBmp);
	return pBmp->width;
}

int BmpIO_GetHeight(const IBMP *pBmp)
{
	assert(NULL != pBmp);
	return pBmp->height;
}

int BmpIO_GetBitPerPixcel(const IBMP *pBmp)
{
	assert(NULL != pBmp);
	return pBmp->BitPerPix;
}

unsigned char BmpIO_GetR(int x, int y, const IBMP *pBmp)
{
	assert(NULL != pBmp && NULL != pBmp->pColor);

	if (pBmp->BitPerPix == 24)
		return pBmp->pColor[PixIdx(x, y, pBmp)].r;
	else
		return BmpIO_GetColorTableR(BmpIO_GetPixcel(x, y, pBmp), pBmp);
}

unsigned char BmpIO_GetG(int x, int y, const IBMP *pBmp)
{
	assert(NULL != pBmp && NULL != pBmp->pColor);

	if (pBmp->BitPerPix == 24)
		return pBmp->pColor[PixIdx(x, y, pBmp)].g;
	else
		return BmpIO_GetColorTableG(BmpIO_GetPixcel(x, y, pBmp), pBmp);
}

unsigned char BmpIO_GetB(int x, int y, const IBMP *pBmp)
{
	assert(NULL != pBmp && NULL != pBmp->pColor);

	if (pBmp->BitPerPix == 24)
		return pBmp->pColor[PixIdx(x, y, pBmp)].b;
	else
		return BmpIO_GetColorTableB(BmpIO_GetPixcel(x, y, pBmp), pBmp);
}


///////////////////////////////////////////////////////////////////////////////////
// 24bit�r�b�g�}�b�v�p

void BmpIO_SetR(int x, int y, IBMP *pBmp, unsigned char v)
{
	assert(NULL != pBmp && NULL != pBmp->pColor && pBmp->BitPerPix == 24);
	pBmp->pColor[PixIdx(x, y, pBmp)].r = v;
}

void BmpIO_SetG(int x, int y, IBMP *pBmp, unsigned char v)
{
	assert(NULL != pBmp && NULL != pBmp->pColor && pBmp->BitPerPix == 24);
	pBmp->pColor[PixIdx(x, y, pBmp)].g = v;
}

void BmpIO_SetB(int x, int y, IBMP *pBmp, unsigned char v)
{
	assert(NULL != pBmp && NULL != pBmp->pColor && pBmp->BitPerPix == 24);
	pBmp->pColor[PixIdx(x, y, pBmp)].b = v;
}


///////////////////////////////////////////////////////////////////////////////////
// 1,4,8bit�r�b�g�}�b�v�p

unsigned char BmpIO_GetColorTableR(int idx, const IBMP *pBmp)
{
	assert(NULL != pBmp && NULL != pBmp->pColor);
	assert(1 == pBmp->BitPerPix ||
		4 == pBmp->BitPerPix ||
		8 == pBmp->BitPerPix);
	assert(idx >= 0 && idx < (1 << pBmp->BitPerPix));
	idx = idx % (1 << pBmp->BitPerPix);
	return pBmp->pColor[idx].r;
}

unsigned char BmpIO_GetColorTableG(int idx, const IBMP *pBmp)
{
	assert(NULL != pBmp && NULL != pBmp->pColor);
	assert(1 == pBmp->BitPerPix ||
		4 == pBmp->BitPerPix ||
		8 == pBmp->BitPerPix);
	assert(idx >= 0 && idx < (1 << pBmp->BitPerPix));
	idx = idx % (1 << pBmp->BitPerPix);
	return pBmp->pColor[idx].g;
}

unsigned char BmpIO_GetColorTableB(int idx, const IBMP *pBmp)
{
	assert(NULL != pBmp && NULL != pBmp->pColor);
	assert(1 == pBmp->BitPerPix ||
		4 == pBmp->BitPerPix ||
		8 == pBmp->BitPerPix);
	assert(idx >= 0 && idx < (1 << pBmp->BitPerPix));
	idx = idx % (1 << pBmp->BitPerPix);
	return pBmp->pColor[idx].b;
}

void BmpIO_SetColorTableR(int idx, const IBMP *pBmp, unsigned char v)
{
	assert(NULL != pBmp && NULL != pBmp->pColor);
	assert(1 == pBmp->BitPerPix ||
		4 == pBmp->BitPerPix ||
		8 == pBmp->BitPerPix);
	assert(idx >= 0 && idx < (1 << pBmp->BitPerPix));
	idx = idx % (1 << pBmp->BitPerPix);
	pBmp->pColor[idx].r = v;
}

void BmpIO_SetColorTableG(int idx, const IBMP *pBmp, unsigned char v)
{
	assert(NULL != pBmp && NULL != pBmp->pColor);
	assert(1 == pBmp->BitPerPix ||
		4 == pBmp->BitPerPix ||
		8 == pBmp->BitPerPix);
	assert(idx >= 0 && idx < (1 << pBmp->BitPerPix));
	idx = idx % (1 << pBmp->BitPerPix);
	pBmp->pColor[idx].g = v;
}

void BmpIO_SetColorTableB(int idx, const IBMP *pBmp, unsigned char v)
{
	assert(NULL != pBmp && NULL != pBmp->pColor);
	assert(1 == pBmp->BitPerPix ||
		4 == pBmp->BitPerPix ||
		8 == pBmp->BitPerPix);
	assert(idx >= 0 && idx < (1 << pBmp->BitPerPix));
	idx = idx % (1 << pBmp->BitPerPix);
	pBmp->pColor[idx].b = v;
}

unsigned char BmpIO_GetPixcel(int x, int y, const IBMP *pBmp)
{
	assert(NULL != pBmp && NULL != pBmp->pColor && NULL != pBmp->pPix);
	assert(1 == pBmp->BitPerPix ||
		4 == pBmp->BitPerPix ||
		8 == pBmp->BitPerPix);
	return pBmp->pPix[PixIdx(x, y, pBmp)];
}

void BmpIO_SetPixcel(int x, int y, const IBMP *pBmp, unsigned char v)
{
	assert(NULL != pBmp && NULL != pBmp->pColor && NULL != pBmp->pPix);
	assert(1 == pBmp->BitPerPix ||
		4 == pBmp->BitPerPix ||
		8 == pBmp->BitPerPix);
	assert(v < (1 << pBmp->BitPerPix));
	v = v % (1 << pBmp->BitPerPix);
	pBmp->pPix[PixIdx(x, y, pBmp)] = v;
}

int BmpIO_TranseTo24BitColor(IBMP *pBmp)
{
	ICOLOR *wpColor;
	int PixCount;	// ���s�N�Z����
	int i;

	assert(NULL != pBmp);

	// 24Bit�J���[�̃f�[�^���n���ꂽ��A�����������ɐ^��Ԃ�
	if (24 == pBmp->BitPerPix) return TRUE;

	assert(NULL != pBmp->pColor && NULL != pBmp->pPix);
	assert(1 == pBmp->BitPerPix ||
		4 == pBmp->BitPerPix ||
		8 == pBmp->BitPerPix);

	PixCount = pBmp->width * pBmp->height;

	// 24Bit�J���[�̉摜���i�[���邽�߂̗̈���m��
	wpColor = (ICOLOR*)calloc(PixCount, sizeof(ICOLOR));
	if (NULL == wpColor) return FALSE;

	// �e�s�N�Z���ɐF��ݒ肷��
	for (i = 0; i < PixCount; i++)
		wpColor[i] = pBmp->pColor[pBmp->pPix[i]];

	// �\���̂̒��g�������ւ���
	free(pBmp->pColor);
	free(pBmp->pPix);
	pBmp->pColor = wpColor;
	pBmp->pPix = NULL;
	pBmp->BitPerPix = 24;

	return TRUE;
}

///////////////////////////////////////////////////////////////////////////////////
// ��������

// �C���f�b�N�X�𐶐�
static int PixIdx(int w, int h, const IBMP *pBmp)
{
	assert(w >= 0 && w < pBmp->width && h >= 0 && h <= pBmp->height);
	w = w % pBmp->width;
	h = h % pBmp->height;
	return h * pBmp->width + w;
}


///////////////////////////////////////////////////////////////////////////////////
// ���͗p���[�`��

static BOOL LoadHeader(
	FILE *infile, unsigned int *ctsize, int *blen, int *pWidth, int *pHeight
);
static BOOL LoadBody1(FILE *infile, int BitLen, IBMP *pBmp);
static BOOL LoadBody24(FILE *infile, IBMP *pBmp);
static void FrushBuf_ipt(BUF *pBuf, FILE *infile);
static int ReadBuf(BUF *pBuf, int len, FILE *infile);
static size_t int_fread(void *buffer, size_t size, size_t count, FILE *stream);

// �C���[�W�ǂݍ��ݎw��
IBMP* BmpIO_Load(FILE *infile)
{
	unsigned int ctsize;	// �J���[�e�[�u���̃G���g����
	int blen;		// �P�s�N�Z��������r�b�g��
	IBMP *pBmp;
	int w, h;
	BOOL r;

	// �t�@�C���w�b�_�̓��́A�J���[�e�[�u�����A
	// pixels�̗̈撷�A�s�N�Z��������̃r�b�g�����擾
	if (!LoadHeader(infile, &ctsize, &blen, &w, &h)) return FALSE;

	// �������̈���m��
	pBmp = BmpIO_CreateBitmap(w, h, blen);
	if (NULL == pBmp) return NULL;

	// �s�N�Z��������r�b�g���ʂɃt�@�C���{�f�B����ǂݍ���
	if (24 != blen)
		r = LoadBody1(infile, blen, pBmp);
	else
		r = LoadBody24(infile, pBmp);

	if (!r) {
		// ���s
		BmpIO_DeleteBitmap(pBmp);
		return NULL;
	}

	return pBmp;
}

// �w�b�_����ǂ�
static BOOL LoadHeader(
	FILE *infile,
	unsigned int *ctsize,
	int *blen,
	int *pWidth,
	int *pHeight
)
{
	BITMAPFILEHEADER bfh;
	BITMAPINFOHEADER bi;

	// �e�\���̂����
	if (int_fread(&(bfh.bfType), sizeof(bfh.bfType), 1, infile) < 1)
		return FALSE;
	if (int_fread(&(bfh.bfSize), sizeof(bfh.bfSize), 1, infile) < 1)
		return FALSE;
	if (int_fread(&(bfh.bfReserved1), sizeof(bfh.bfReserved1), 1, infile) < 1)
		return FALSE;
	if (int_fread(&(bfh.bfReserved2), sizeof(bfh.bfReserved2), 1, infile) < 1)
		return FALSE;
	if (int_fread(&(bfh.bfOffBits), sizeof(bfh.bfOffBits), 1, infile) < 1)
		return FALSE;

	if (int_fread(&(bi.biSize), sizeof(bi.biSize), 1, infile) < 1)
		return FALSE;
	if (int_fread(&(bi.biWidth), sizeof(bi.biWidth), 1, infile) < 1)
		return FALSE;
	if (int_fread(&(bi.biHeight), sizeof(bi.biHeight), 1, infile) < 1)
		return FALSE;
	if (int_fread(&(bi.biPlanes), sizeof(bi.biPlanes), 1, infile) < 1)
		return FALSE;
	if (int_fread(&(bi.biBitCount), sizeof(bi.biBitCount), 1, infile) < 1)
		return FALSE;
	if (int_fread(&(bi.biCompression), sizeof(bi.biCompression), 1, infile) < 1)
		return FALSE;
	if (int_fread(&(bi.biSizeImage), sizeof(bi.biSizeImage), 1, infile) < 1)
		return FALSE;
	if (int_fread(&(bi.biXPelsPerMeter), sizeof(bi.biXPelsPerMeter), 1, infile) < 1)
		return FALSE;
	if (int_fread(&(bi.biYPelsPerMeter), sizeof(bi.biYPelsPerMeter), 1, infile) < 1)
		return FALSE;
	if (int_fread(&(bi.biClrUsed), sizeof(bi.biClrUsed), 1, infile) < 1)
		return FALSE;
	if (int_fread(&(bi.biClrImportant), sizeof(bi.biClrImportant), 1, infile) < 1)
		return FALSE;
	// �t�@�C���^�C�v���m�F
	if (bfh.bfType != 0x4D42) return FALSE;

	// �ǂݍ��񂾏���Ԃ�
	(*blen) = bi.biBitCount;	// �P�s�N�Z������̃r�b�g��
	(*ctsize) = bi.biClrUsed;	// �g�p�����F�̐�
	(*pWidth) = bi.biWidth;		// ��
	(*pHeight) = bi.biHeight;	// ����

	return TRUE;
}

// �J���[�e�[�u�����g�p����r�b�g�}�b�v��ǂݍ���
static BOOL LoadBody1(FILE *infile, int BitLen, IBMP *pBmp)
{
	int i, j;
	BUF buf;	// �ǂݍ��ނ��߂̃o�b�t�@
	int ctsize = (1 << BitLen);	// �J���[�e�[�u���̐�

								// �J���[�e�[�u����ǂ�
	for (i = 0; i < ctsize && !ferror(infile) && !feof(infile); i++) {
		pBmp->pColor[i].b = fgetc(infile);
		pBmp->pColor[i].g = fgetc(infile);
		pBmp->pColor[i].r = fgetc(infile);
		fgetc(infile);	// rgbReserved
	}
	if (ferror(infile) || feof(infile)) return FALSE;

	// �o�b�t�@�̓��e������������
	buf.BufU.buf = 0;
	buf.buflen = 0;
	FrushBuf_ipt(&buf, infile);

	for (i = 0; i < pBmp->height && !feof(infile) && !ferror(infile); i++) {
		for (j = 0; j < pBmp->width && !feof(infile) && !ferror(infile); j++) {
			int wIdx = PixIdx(j, i, pBmp);
			// �w�肳�ꂽ�r�b�g���̃f�[�^���擾
			pBmp->pPix[wIdx] = ReadBuf(&buf, BitLen, infile);
		}
		FrushBuf_ipt(&buf, infile);
	}

	return (i == pBmp->height && j == pBmp->width);
}

// �Q�S�r�b�g�J���[�r�b�g�}�b�v��ǂ�
static BOOL LoadBody24(FILE *infile, IBMP *pBmp)
{
	int PixCnt = pBmp->width * pBmp->height;	// �s�N�Z����
	int i, j;
	BUF buf;

	// �o�b�t�@�̓��e������������
	buf.buflen = 0;
	buf.BufU.buf = 0;
	FrushBuf_ipt(&buf, infile);

	for (i = 0; i < pBmp->height && !feof(infile) && !ferror(infile); i++) {
		for (j = 0; j < pBmp->width && !feof(infile) && !ferror(infile); j++) {
			// �F��BGR�̏��ɋL�^����Ă���
			BmpIO_SetB(j, i, pBmp, (unsigned char)ReadBuf(&buf, 8, infile));
			BmpIO_SetG(j, i, pBmp, (unsigned char)ReadBuf(&buf, 8, infile));
			BmpIO_SetR(j, i, pBmp, (unsigned char)ReadBuf(&buf, 8, infile));
		}
		FrushBuf_ipt(&buf, infile);
	}
	return (i == pBmp->height && j == pBmp->width);
}

// �o�b�t�@�Ƀf�[�^��ǂݍ���
static void FrushBuf_ipt(BUF *pBuf, FILE *infile)
{
	int i;

#if ISLITTLEENDIAN
	for (i = 3; i >= 0; i--)
#else
	for (i = 0; i < 4; i++)
#endif
		pBuf->BufU.vbuf[i] = getc(infile);
	pBuf->buflen = 32;
}

// �o�b�t�@����w�肵���r�b�g�����f�[�^���擾����
static int ReadBuf(BUF *pBuf, int len, FILE *infile)
{
	int r;
	if (pBuf->buflen < len) FrushBuf_ipt(pBuf, infile);
	r = ((((1 << len) - 1) << (32 - len)) & pBuf->BufU.buf) >> (32 - len);
	pBuf->BufU.buf <<= len;
	pBuf->buflen -= len;
	return r;
}

// �G���f�B�A���l�X���z�����āA�t�@�C����ǂݍ���
size_t int_fread(void *buffer, size_t size, size_t count, FILE *stream)
{
#if ISLITTLEENDIAN
	return fread(buffer, size, count, stream);
#else
	size_t i, j;
	size_t r;
	char *cbuf = (char*)buffer;

	// �ǂݍ���
	r = fread(buffer, size, count, stream);
	if (1 == size) return r;

	// ���ڂ��ƂɃo�C�g�I�[�_�[�𔽓]
	for (i = 0; i < count; i++) {
		for (j = 0; j < size / 2; j++) {
			int idx1 = i * size + j;
			int idx2 = i * size + (size - j - 1);
			char c = cbuf[idx1];
			cbuf[idx1] = cbuf[idx2];
			cbuf[idx2] = c;
		}
	}
	return r;
#endif
}

///////////////////////////////////////////////////////////////////////////////////
// �o�͗p���[�`��

static BOOL WriteHeader(FILE *outfile, const IBMP *pBmp);
static BOOL WriteBody1(FILE *outfile, const IBMP *pBmp);
static BOOL WriteBody24(FILE *outfile, const IBMP *pBmp);
static void FrushBuf_opt(BUF *pBuf, FILE *outfile);
static void WriteBuf(BUF *pBuf, int BitLen, unsigned char c, FILE *outfile);
static size_t int_fwrite(const void *buffer, size_t size, size_t count, FILE *stream);

// 1,4,8bit�r�b�g�}�b�v���o�͂���
BOOL WriteBody1(FILE *outfile, const IBMP *pBmp)
{
	int i, j;
	int ctcnt = 1 << pBmp->BitPerPix;	// �J���[�e�[�u���̐�
	BUF buf;

	// �J���[�e�[�u�����o�͂���
	for (i = 0; i < ctcnt && !ferror(outfile); i++) {
		fputc(pBmp->pColor[i].b, outfile);
		fputc(pBmp->pColor[i].g, outfile);
		fputc(pBmp->pColor[i].r, outfile);
		fputc(0, outfile);
	}

	buf.BufU.buf = 0;
	buf.buflen = 0;

	// �s�N�Z���f�[�^���o��
	for (i = 0; i < pBmp->height; i++) {
		for (j = 0; j < pBmp->width; j++) {
			WriteBuf(&buf, pBmp->BitPerPix, BmpIO_GetPixcel(j, i, pBmp), outfile);
		}
		FrushBuf_opt(&buf, outfile);
	}
	return TRUE;
}

// �Q�S�r�b�g�J���[�r�b�g�}�b�v���o�͂���
static BOOL WriteBody24(FILE *outfile, const IBMP *pBmp)
{
	int i, j;
	BUF buf;

	buf.BufU.buf = 0;
	buf.buflen = 0;

	// �R�o�C�g���o�͂���
	for (i = 0; i < pBmp->height; i++) {
		for (j = 0; j < pBmp->width; j++) {
			WriteBuf(&buf, 8, BmpIO_GetB(j, i, pBmp), outfile);
			WriteBuf(&buf, 8, BmpIO_GetG(j, i, pBmp), outfile);
			WriteBuf(&buf, 8, BmpIO_GetR(j, i, pBmp), outfile);
		}
		FrushBuf_opt(&buf, outfile);
	}
	return TRUE;
}

// �o�b�t�@�̃f�[�^��S�ďo�͂���
static void FrushBuf_opt(BUF *pBuf, FILE *outfile)
{
	int i;
	if (0 == pBuf->buflen) return;

#if ISLITTLEENDIAN
	for (i = 3; i >= 0; i--)
#else
	for (i = 0; i < 4; i++)
#endif
		putc(pBuf->BufU.vbuf[i], outfile);

	pBuf->buflen = 0;
	pBuf->BufU.buf = 0;
}

// �o�b�t�@�Ɏw�肵���r�b�g�����f�[�^���o�͂���
static void WriteBuf(BUF *pBuf, int BitLen, unsigned char c, FILE *outfile)
{
	const unsigned long MASK = (0x1 << (BitLen + 1)) - 1;
	pBuf->BufU.buf |= (((unsigned long)c) & MASK) << (32 - (pBuf->buflen + BitLen));
	pBuf->buflen += BitLen;
	if (pBuf->buflen >= 32)
		FrushBuf_opt(pBuf, outfile);
}

// �G���f�B�A���l�X���z�����āA�t�@�C�����o�͂���
size_t int_fwrite(const void *buffer, size_t size, size_t count, FILE *stream)
{
#if ISLITTLEENDIAN
	return fwrite(buffer, size, count, stream);
#else
	size_t i, j;
	char *cbuf = (char*)buffer;
	char *wbuf = (char*)alloca(size);
	size_t r;

	// 1�o�C�g�P�ʂȂ�΁A���]����K�v�͂Ȃ�
	if (1 == size)
		return fwrite(buffer, size, count, stream);

	// ���ڂ��ƂɃo�C�g�I�[�_�[�𔽓]���ďo�͂���
	r = 0;
	for (i = 0; i < count; i++) {
		for (j = 0; j < size; j++) {
			int idx1 = i * size + j;
			int idx2 = i * size + (size - j - 1);
			wbuf[idx2] = cbuf[idx1];
		}
		if (fwrite(wbuf, size, 1, stream) < 1)
			return r;
		r++;
	}
	return r;
#endif
}

MMD_BoneNode       root_bone;


// parent�Ŏw�肳�ꂽ�e�m�[�h��T���A���̎q�Ƃ���chid��ǉ�����
// ���m�[�h��root_bone
void add_children(MMD_BoneNode* node, uint16_t parent_id, uint16_t child_id)
{
	if (child_id == 0) {  // root�ł���
		node->bone_index = 0;
	}

	// �e�ƂȂ�ׂ��m�[�h�����o����
	if (parent_id == node->bone_index) {
		MMD_BoneNode    child_node;
		child_node.bone_index = child_id;
		node->children.push_back(child_node);
	}

	for (int i = 0; i < node->children.size(); i++) {
		// �q�m�[�h�֍~��āA�ړI��parent_id���Ȃ����T��
		MMD_BoneNode*   child = &node->children[i];
		add_children(child, parent_id, child_id);
	}
}


//���Z�q�̃I�[�o�[���[�h Quaternion�̐�
Quaternion & operator *(Quaternion &q1, Quaternion &q2)
{
	Quaternion q0 = {
		q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
		q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
		q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
		q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w,
	};
	q1 = q0;
	return q1;
}

//�N�H�[�^�j�I�������]�s����Z�o
// r = 16*16��openGL���W�n�ɑΉ�������]�s��
void qtor(float *r, Quaternion q)
{
	double xx = q.x * q.x * 2.0;
	double yy = q.y * q.y * 2.0;
	double zz = q.z * q.z * 2.0;
	double xy = q.x * q.y * 2.0;
	double yz = q.y * q.z * 2.0;
	double zx = q.z * q.x * 2.0;
	double xw = q.x * q.w * 2.0;
	double yw = q.y * q.w * 2.0;
	double zw = q.z * q.w * 2.0;
	double r1[16] = { 1.0 - yy - zz, xy + zw, zx - yw, 0.0,
		xy - zw, 1.0 - zz - xx, yz + xw, 0.0,
		zx + yw, yz - xw, 1.0 - xx - yy, 0.0,
		0.0, 0.0, 0.0, 1.0 };
	for (int i = 0; i < 16; i++) {
		r[i] = r1[i];
	}
}


void MMD_Header::read(FILE* fp) {
	// �A���C�����g��������Ă��Ȃ��̂ŁA���O�œǂݍ���
	fread(magic, 1, 3, fp);
	fread(&version, 1, 4, fp);
	fread(model_name, 1, 20, fp);
	fread(comment, 1, 256, fp);
}

MMD_Header::MMD_Header() {
	memset(magic, 3, 0);
	version = 0.0f;
	memset(model_name, 0, 20);
	memset(comment, 0, 256);
}
MMD_Header::~MMD_Header() {}


void MMD_face::read(FILE* fp) {
	fread(&face_vert_count, 1, 4, fp);
	printf("face vert count = %d\n", face_vert_count);

	face_index = new GLuint[face_vert_count];
	for (int i = 0; i < face_vert_count; i++) {
		fread(&face_index[i], 1, 2, fp);
	}
}

MMD_face::MMD_face() {
}

MMD_face::~MMD_face() {
}

void MMD_face::draw()
{
}

MMD_vertex::MMD_vertex() {
}

MMD_vertex::~MMD_vertex() {
}

void MMD_vertex::read(FILE* fp)
{
	fread(&x, 1, 4, fp);
	fread(&y, 1, 4, fp);
	fread(&z, 1, 4, fp);
	fread(&nx, 1, 4, fp);
	fread(&ny, 1, 4, fp);
	fread(&nz, 1, 4, fp);
	fread(&u, 1, 4, fp);
	fread(&v, 1, 4, fp);

	fread(bone_num, 2, 2, fp);
	fread(&bone_weight, 1, 1, fp);
	fread(&edge_flag, 1, 1, fp);
}


MMD_VertexArray::MMD_VertexArray()
{
	p3dVerted = NULL;
	p3dNormal = NULL;
	pTexuv = NULL;
}

MMD_VertexArray::~MMD_VertexArray()
{
	if (p3dVerted) {
		delete[]p3dVerted;
		p3dVerted = NULL;
	}
	if (p3dNormal) {
		delete[]p3dNormal;
		p3dNormal = NULL;
	}
	if (pTexuv) {
		delete[]pTexuv;
		pTexuv = NULL;
	}
};

void MMD_VertexArray::read(FILE* fp)
{
	fread(&count, 4, 1, fp);
	printf("vertex count == %d\n", count);       // �f�o�b�O�p

	pVertex = new MMD_vertex[count];
	for (int i = 0; i < count; i++) {
		pVertex[i].read(fp);
	}

	p3dVerted = new GLfloat[count * 3];
	p3dNormal = new GLfloat[count * 3];
	pTexuv = new GLfloat[count * 2];
}

void MMD_VertexArray::draw()
{
	// ���_, �@��, �e�N�X�`�����W���Ƃ̔z��ɒu��������
	for (int i = 0; i < count; i++) {
		p3dVerted[i * 3 + 0] = pVertex[i].x;
		p3dVerted[i * 3 + 1] = pVertex[i].y;
		p3dVerted[i * 3 + 2] = pVertex[i].z;
		p3dNormal[i * 3 + 0] = pVertex[i].nx;
		p3dNormal[i * 3 + 1] = pVertex[i].ny;
		p3dNormal[i * 3 + 2] = pVertex[i].nz;
		pTexuv[i * 2 + 0] = pVertex[i].u;
		pTexuv[i * 2 + 1] = fabsf(1.0f - pVertex[i].v);
	}
	glVertexPointer(3, GL_FLOAT, 0, p3dVerted);     // ���_�z�� (0�̓X�g���C�h)
	glNormalPointer(GL_FLOAT, 0, p3dNormal);        // �@���z��
	glTexCoordPointer(2, GL_FLOAT, 0, pTexuv);      // �e�N�X�`�����W
}

Texture::Texture(void)
{
}

Texture::~Texture()
{
	if (pBmp)
		glDeleteTextures(1, &texture_id);
};

void Texture::load(const char* filename, GLuint tex_id)
{
	pBmp = NULL;
	FILE* fp = fopen(filename, "rb");
	if (fp != NULL) {
		pBmp = BmpIO_Load(fp);
		if (pBmp != NULL) {
			// OpenGL�p�Ƀe�N�X�`������
			this->texture_id = tex_id;
			printf("load texture_id = %d\n", this->texture_id);
			printf("id: %d = %s\n", this->texture_id, filename);
			glBindTexture(GL_TEXTURE_2D, this->texture_id);
			glTexImage2D(
				GL_TEXTURE_2D, 0, GL_RGB, pBmp->width, pBmp->height,
				0, GL_RGB, GL_UNSIGNED_BYTE, pBmp->pColor);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glFlush();
			glBindTexture(GL_TEXTURE_2D, 0);
			//            glBindTexture( GL_TEXTURE_2D, 10 );   // ����Ȃ�����
		}
		else {
			printf(">>>>>>pBmp == NULL\n");
		}
		fclose(fp);
	}
}

unsigned int Texture::get_gl_texture_id(void)
{
	return this->texture_id;
}


// �}�e���A���f�[�^��\���N���X
Material::Material(void)
{
	tex_index = 0;
}
Material::~Material() {}

uint32_t Material::get_face_vert_count(void)
{
	return face_vert_count;
}

void Material::setpath(std::string pathname)
{
	path = pathname;
}

void Material::read(FILE* fp, GLuint texture_id)
{
	memset(texture_name, 0, 21);
	fread(diffuse_color, 4, 3, fp);
	fread(&alpha, 4, 1, fp);
	fread(&specular, 4, 1, fp);
	fread(specular_color, 4, 3, fp);
	fread(mirror_color, 4, 3, fp);
	fread(&toon_index, 1, 1, fp);
	fread(&edge_flag, 1, 1, fp);
	fread(&face_vert_count, 4, 1, fp);
	fread(texture_name, 1, 20, fp);
	std::string fullpath_texture = path + texture_name;
	texture.load(fullpath_texture.c_str(), texture_id);
}

// �}�e���A���̔z��
MaterialArray::MaterialArray() {};
MaterialArray::~MaterialArray() {};
void MaterialArray::setpath(std::string pathname)
{
	path = pathname;
}

void MaterialArray::read(FILE* fp)
{
	fread(&count, 1, 4, fp);
	mat_array = new Material[count];

	// �e�N�X�`���̏�����������Ŋm�肷��̂ŁA
	// ���̈ʒu�Ńe�N�X�`��ID��ێ�����z�������
	// TODO: �e�N�X�`�����g��Ȃ��}�e���A���̏ꍇ�͖��ʂ��o�Ă��܂��̂ŁA�Ή�������
	textureIds = new GLuint[count];
	glGenTextures(count, textureIds);
	printf("glGenTextures Array = %d\n", count);
	for (int i = 0; i < count; i++) {
		printf("[index:%d == id=%d]\n", i, textureIds[i]);
	}

	// �e�N�X�`�����ꖇ���ǂݍ���ł���
	// TODO: �قȂ�}�e���A���ł������t�@�C�������w���Ă��邱�Ƃ�����̂ŁA
	// �}�e���A���ƃe�N�X�`����1:1�̊֌W�ł͌����������B
	int face_sum = 0;
	for (int i = 0; i < count; i++) {
		mat_array[i].setpath(path);
		mat_array[i].read(fp, textureIds[i]);
		mat_array[i].start_face = face_sum;
		face_sum += mat_array[i].get_face_vert_count();
	}
}


void MaterialArray::draw(void)
{
	for (int i = 0; i < count; i++) {
		mat_array[i].draw();
	}
}

MMD_File    mmdfile;
VMD_File    vmdfile;
Texture     madoka_magic;
void Material::draw()
{
	MMD_face*   face = &mmdfile.m_face;
	int face_count = this->face_vert_count;

	glBindTexture(GL_TEXTURE_2D, texture.get_gl_texture_id());
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	glFrontFace(GL_CCW);        // GL_CW(���v��肪�\), GL_CCW(�����v��肪�\)
	glEnable(GL_CULL_FACE);     //�J�����OOFF

	GLsizei primitive_count = face_count;

	glDrawElements(GL_LINES,                    // �v���~�e�B�u�̎��
		primitive_count,                // �����_�����O�v�f�̐�
		GL_UNSIGNED_INT,                // �C���f�b�N�X�z��̌^
		&face->face_index[start_face]); // �C���f�b�N�X�z��������|�C���^

	glFlush();
}


// �e�N�X�`��������pmd�t�@�C���̔z���ɂ���̂Ŏd���Ȃ��B�B
void MMD_File::setpath(const char* pathname)
{
	path = pathname;
}

MMD_Bone::MMD_Bone()
{
}

MMD_Bone::~MMD_Bone()
{
}

void MMD_Bone::read(FILE* fp)
{
	fread(bone_name, 1, 20, fp);            // �{�[����
	fread(&parent_bone_index, 2, 1, fp);    // �e�{�[���C���f�b�N�X
	fread(&tail_pos_bone_index, 2, 1, fp);  // tail�ʒu�̃{�[���C���f�b�N�X
	fread(&bone_type, 1, 1, fp);             // �{�[���̎��
	fread(&ik_parent_bone_index, 2, 1, fp);   // IK�{�[���ԍ�(�Ȃ��ꍇ��0)
	fread(bone_head_pos, 4, 3, fp);       // �{�[���̃w�b�h�̈ʒu

	printf("bone_name = %s, ", bone_name);
	printf("parent index = %d\n", parent_bone_index);
}

void MMD_Bone::draw()
{
	// ��]�s��̓N�H�[�^�j�I���Ƃ��ė^������̂ŁA��]�s��ɕϊ�����
	qtor(rot_mat, quot);

	glTranslatef(tx, ty, tz);
	glMultMatrixf(rot_mat);

	// glutSolidCone(��ʂ̔��a,�~���̍���,�~�̕�����,�����̕�����)�@
	glutSolidCone(0.5f, 2.2f, 10, 1);
}

void MMD_Bone::set_translate(float itx, float ity, float itz)
{
	tx = itx + bone_head_pos[0];
	ty = ity + bone_head_pos[1];
	tz = itz + bone_head_pos[2];
}

void MMD_Bone::set_rotation(float iqx, float iqy, float iqz, float iqw)
{
	quot.x = iqx;
	quot.y = iqy;
	quot.z = iqz;
	quot.w = iqw;
}

MMD_BoneArray::MMD_BoneArray()
{
}

MMD_BoneArray::~MMD_BoneArray()
{
}

void MMD_BoneArray::read(FILE* fp)
{
	// �{�[���f�[�^��ǂݍ���
	fread(&bone_count, 1, 2, fp);

	bone_array = new MMD_Bone[bone_count];

	// ���Ƃ͌J��Ԃ�
	for (int i = 0; i < bone_count; i++) {
		bone_array[i].read(fp);
		// �{�[���̖؂��\������
		uint16_t    parent_id = bone_array[i].parent_bone_index;
		uint16_t    child_id = i;
		add_children(&root_bone, parent_id, child_id);
	}

	// �{�[�������L�[�ɂ����C���f�b�N�X���쐬
	for (int i = 0; i < bone_count; i++) {
		std::string bone_name_str = bone_array[i].bone_name;
		bone_name_dict[bone_name_str] = i;
	}
}

// �e�K�w���牺���Ă����Ȃ���`����s���֐�
// array�͖{���͕s�v�����A�O����������n���Ă��炤���Ƃɂ���
void DrawRecarsive(MMD_BoneNode* node, MMD_BoneArray* array)
{
	uint16_t bone_index = node->bone_index;
	array->bone_array[bone_index].draw();
	for (int i = 0; i < node->children.size(); i++) {
		MMD_BoneNode* child = &node->children[i];
		// �q�m�[�h�̕`��ɂ͂���O��
		// �q�m�[�h�ւ̒������Ђ��Ă���
		glBegin(GL_LINES);
		MMD_Bone* p = &array->bone_array[node->bone_index];
		MMD_Bone* c = &array->bone_array[child->bone_index];
		glVertex3d(p->tx, p->ty, p->tz);
		glVertex3d(c->tx, c->ty, c->tz);
		glEnd();

		glPushMatrix();
		DrawRecarsive(child, array);
		glPopMatrix();
	}
}

void MMD_BoneArray::draw()
{
	glPushMatrix();
	glTranslatef(10, 0, 0);
	DrawRecarsive(&root_bone, this);
	glPopMatrix();
#if 0
	// �S�Ẵ{�[����`�悵
	for (int i = 0; i < bone_count; i++) {
		glPushMatrix();
		bone_array[i].draw();
		glPopMatrix();
	}
#endif
}

// �����Ŏw�肳�ꂽ�t�@�C�����J���āA���f���f�[�^��ǂݍ���
void MMD_File::load(const char* iFilename)
{
	std::string fullpath = path + iFilename;
	FILE* fp = fopen(fullpath.c_str(), "rb");
	if (fp == NULL) {
		printf("file not open\n");
	}

	m_header.read(fp);
	m_vertics.read(fp);
	m_face.read(fp);
	m_materials.setpath(path);
	m_materials.read(fp);
	m_bones.read(fp);

	fclose(fp);
};

void MMD_File::draw(void) {
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	m_bones.draw();
	m_vertics.draw();
	m_materials.draw();
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
	glDisableClientState(GL_TEXTURE_COORD_ARRAY);
}

VMD_Header::VMD_Header()
{
	memset(VmdHeader, 0, 30);
	memset(VmdModelName, 0, 20);
	motion_count = 0;
}


VMD_Header::~VMD_Header()
{
};

void VMD_Header::read(FILE* fp)
{
	fread(VmdHeader, 1, 30, fp);
	fread(VmdModelName, 1, 20, fp);
	fread(&motion_count, 1, 4, fp);
}

VMD_Motion::VMD_Motion()
{
}

VMD_Motion::~VMD_Motion()
{
}

void VMD_Motion::read(FILE* fp)
{
	// �ȉ��̃��[�V�����f�[�^��ǂݍ��ރR�[�h���L�q����
	fread(BoneName, 1, 15, fp);        // �{�[����
	fread(&FrameNumber, 4, 1, fp);      // �t���[���ԍ�
	fread(&px, 4, 1, fp);               // �ʒu
	fread(&py, 4, 1, fp);
	fread(&pz, 4, 1, fp);
	fread(&rq, 4, 4, fp);               // ��]�p(�N�H�[�^�j�I��)
	fread(Interpolation, 1, 4 * 4 * 4, fp); // �⊮

											//    printf("%d:BoneName = %s\n", FrameNumber, BoneName);
}


VMD_File::VMD_File()
{
	motion_index = 0;
}

VMD_File::~VMD_File()
{
}

void VMD_File::read(const char* filename)
{
	FILE* fp = fopen(filename, "rb");
	vmd_header.read(fp);

	// ���[�V�����̐��ɍ��킹�āA�������̈���m�ۂ���
	uint32_t motion_count = vmd_header.get_motion_count();
	vmd_motion = new VMD_Motion[motion_count];

	// ���ׂẴ��[�V������ǂݍ���
	for (uint32_t i = 0; i < motion_count; i++) {
		vmd_motion[i].read(fp);
	}

}


// mmd���f���ɑ΂��Ďp����ݒ肷��
// TOOD : mmd�t�@�C���S����������Ă���K�v�͂Ȃ��͂��Ȃ̂ŁA
//        ���t�@�N�^�����O���s��
void VMD_File::setMmdMotion(MMD_File* pmd, int frame_number)
{
	// ���ׂĂ̎p����񂩂�A�ړI�̃t���[���̏��݂̂𒊏o���āAmodel�ɐݒ肷��
	for (uint32_t i = 0; i < vmd_header.motion_count; i++) {
		if (vmd_motion[i].FrameNumber == frame_number) {
			float px = vmd_motion[i].px;
			float py = vmd_motion[i].py;
			float pz = vmd_motion[i].pz;
			std::string bone_name = vmd_motion[i].BoneName;
			int index = pmd->m_bones.bone_name_dict[bone_name];

			//            printf("draw motion %d, index =%d\n", i, index);

			glPushMatrix();

			pmd->m_bones.bone_array[index].set_translate(px, py, pz);
			Quaternion  rq = vmd_motion[i].rq;
			pmd->m_bones.bone_array[index].set_rotation(
				rq.x, rq.y, rq.z, rq.w);

			glPopMatrix();
		}
	}
}

//int WindowWidth = 512;

//int WindowHeight = 512;
int WindowWidth = 512;      //��������E�B���h�E�̕�
int WindowHeight = 512;     //��������E�B���h�E�̍���

int  motion_index = 0;


void Reshape(int x, int y)
{
	WindowWidth = x;
	WindowHeight = y;
	if (WindowWidth < 1) WindowWidth = 1;
	if (WindowHeight < 1) WindowHeight = 1;
}

void disp(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPushMatrix();

	// ���_, �r���[�|�[�g�̐ݒ�
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, (double)WindowWidth / (double)WindowHeight, 0.1, 1000.0);
	glViewport(0, 0, WindowWidth, WindowHeight);
	gluLookAt(
		0.0, 10.0, -48.0,
		0.0, 10.0, 0.0,
		0.0, 1.0, 0.0);

	// �����̐ݒ�
	const GLfloat lightPos[] = { 0 , 0 , -2 , 0 };
	const GLfloat lightCol[] = { 1 , 1 , 1 , 1 };
	const GLfloat ambient[] = { 0.3f, 0.3f, 0.3f, 1.0f };
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightCol);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	// �e�N�X�`���̗L����
	glEnable(GL_TEXTURE_2D);

	// Z�o�b�t�@�̗L����
	glEnable(GL_DEPTH_TEST);

	// ���f�����W�n�̐ݒ���J�n���邱�Ƃ�����
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// vmd���[�V������i�߁A�K�p����
	vmdfile.setMmdMotion(&mmdfile, motion_index);
	motion_index++;
	printf("motion frame number = %d\n", motion_index);

	// �p�[�c���Ƃɕ`�悷��
	mmdfile.draw();

	// �����܂ł�MMD���f���̕`�悪����
	// �s��X�^�b�N�����ɖ߂�
	glPopMatrix();

	glutSwapBuffers();
}


void timer(int value)
{
	glutPostRedisplay();
	glutTimerFunc(100, timer, 0);
}

int main(int argc, char ** argv)
{
	glutInit(&argc, argv);
	glutInitWindowPosition(100, 50);
	glutInitWindowSize(WindowWidth, WindowHeight);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);

	glutCreateWindow("Window caption");
	glutDisplayFunc(disp);
	glutReshapeFunc(Reshape);
	timer(100);
	//glutTimerFunc(100, timer, 0);

	// �܂ǂ�
	//    mmdfile.setpath("/home/catalina/workspace/opengl/madoka/");
	//    mmdfile.load("md_m.pmd");

	// GUMI
	mmdfile.setpath("D:/work/baseproj/resource/madoka/maho/");
	mmdfile.load("md_m.pmd");

	vmdfile.read("D:/work/baseproj/resource/vmd1.vmd");
	//    return 0;

	// �قނ�
	// ��: ���@�������̃e�N�X�`���`����tga�̂��߁A�ǂݍ��ݎ��s����B
	// opencv�̉摜�ǂݍ��݊֐����g���Έꔭ�ł�������
	//    mmdfile.setpath("/home/catalina/workspace/opengl/homura/");
	//    mmdfile.load("hm_m.pmd");

	glEnable(GL_NORMALIZE);
	glutMainLoop();

	return 0;
}

//----------------------------------------------------
// �ϐ��̐錾
//----------------------------------------------------
int WindowPositionX = 200;  //��������E�B���h�E�ʒu��X���W
int WindowPositionY = 200;  //��������E�B���h�E�ʒu��Y���W
char WindowTitle[] = "���E�̎n�܂�";  //�E�B���h�E�̃^�C�g��

								//----------------------------------------------------
								// �֐��v���g�^�C�v�i��ɌĂяo���֐����ƈ����̐錾�j
								//----------------------------------------------------
void Initialize(void);   //�����ݒ莞�ɌĂяo���֐�
void Idle(void);         //�A�C�h�����ɌĂяo���֐�
void Display(void);      //��ʕ`�掞�ɌĂяo���֐�
void Ground(void);       //��n�̕`��p�̊֐�
						 //----------------------------------------------------
						 // ���C���֐�
						 //----------------------------------------------------
int _main(int argc, char *argv[]) {
	glutInit(&argc, argv);                                     //���̏�����
	glutInitWindowPosition(WindowPositionX, WindowPositionY);  //�E�B���h�E�̈ʒu�̎w��
	glutInitWindowSize(WindowWidth, WindowHeight);             //�E�B���h�E�T�C�Y�̎w��
	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE); //�f�B�X�v���C���[�h�̎w��
	glutCreateWindow(WindowTitle);                             //�E�B���h�E�̍쐬
	glutIdleFunc(Idle);                                        //�v���O�����A�C�h����Ԏ��ɌĂяo�����֐�
	glutDisplayFunc(Display);                                  //�`�掞�ɌĂяo�����֐����w�肷��i�֐����FDisplay�j
	Initialize();                                              //�����ݒ�̊֐����Ăяo��
	glutMainLoop();
	return 0;
}
//----------------------------------------------------
// �����ݒ�̊֐�
//----------------------------------------------------
void Initialize(void) {
	glClearColor(1.0, 1.0, 1.0, 0.0); //�w�i�F
	glEnable(GL_DEPTH_TEST);//�f�v�X�o�b�t�@���g�p�FglutInitDisplayMode() �� GLUT_DEPTH ���w�肷��

							//�����ϊ��s��̐ݒ�------------------------------
	glMatrixMode(GL_PROJECTION);//�s�񃂁[�h�̐ݒ�iGL_PROJECTION : �����ϊ��s��̐ݒ�AGL_MODELVIEW�F���f���r���[�ϊ��s��j
	glLoadIdentity();//�s��̏�����
	gluPerspective(30.0, (double)WindowWidth / (double)WindowHeight, 0.1, 1000.0); //�������e�@�̎��̐�gluPerspactive(th, w/h, near, far);
																				   //------------------------------------------------
	gluLookAt(
		0.0, -200.0, 50.0, // ���_�̈ʒux,y,z;
		0.0, 0.0, 20.0,   // ���E�̒��S�ʒu�̎Q�Ɠ_���Wx,y,z
		0.0, 0.0, 1.0);  //���E�̏�����̃x�N�g��x,y,z
}
//----------------------------------------------------
// �A�C�h�����ɌĂяo�����֐�
//----------------------------------------------------
void Idle() {
	glutPostRedisplay(); //glutDisplayFunc()���P����s����
}
//----------------------------------------------------
// �`��̊֐�
//----------------------------------------------------
void Display(void) {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); //�o�b�t�@�̏���
														//���f���r���[�ϊ��s��̐ݒ�--------------------------
	glMatrixMode(GL_MODELVIEW);//�s�񃂁[�h�̐ݒ�iGL_PROJECTION : �����ϊ��s��̐ݒ�AGL_MODELVIEW�F���f���r���[�ϊ��s��j
	glLoadIdentity();//�s��̏�����
	glViewport(0, 0, WindowWidth, WindowHeight);
	//----------------------------------------------

	Ground();

	glutSwapBuffers(); //glutInitDisplayMode(GLUT_DOUBLE)�Ń_�u���o�b�t�@�����O�𗘗p��
}
//----------------------------------------------------
// ��n�̕`��
//----------------------------------------------------
void Ground(void) {
	double ground_max_x = 300.0;
	double ground_max_y = 300.0;
	glColor3d(0.8, 0.8, 0.8);  // ��n�̐F
	glBegin(GL_LINES);
	for (double ly = -ground_max_y; ly <= ground_max_y; ly += 20.0) {
		glVertex3d(-ground_max_x, ly, 0);
		glVertex3d(ground_max_x, ly, 0);
	}
	for (double lx = -ground_max_x; lx <= ground_max_x; lx += 20.0) {
		glVertex3d(lx, ground_max_y, 0);
		glVertex3d(lx, -ground_max_y, 0);
	}
	glEnd();
}