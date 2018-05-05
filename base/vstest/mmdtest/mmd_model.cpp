#include "windows.h"
#include "windowsx.h"
#include <winuser.h>
#include <GL/gl.h>
#include <GL/glut.h>
#include <math.h>
#include <string>
#include <stdio.h>
#include <memory.h>

#include <assert.h>  
#include "mmd_model.h"


// リトルエンディアンか否か
#define ISLITTLEENDIAN 1

// 各種データ型

#define WORD unsigned short
#define DWORD unsigned int
#define LONG int
#define BYTE unsigned char
#define BOOL int
#define FALSE 0
#define TRUE 1

typedef struct tagBITMAPFILEHEADER {
	WORD bfType;
	DWORD bfSize;
	WORD bfReserved1;
	WORD bfReserved2;
	DWORD bfOffBits;
} BITMAPFILEHEADER;

typedef struct tagBITMAPINFOHEADER {
	DWORD  biSize;
	LONG   biWidth;
	LONG   biHeight;
	WORD   biPlanes;
	WORD   biBitCount;
	DWORD  biCompression;
	DWORD  biSizeImage;
	LONG   biXPelsPerMeter;
	LONG   biYPelsPerMeter;
	DWORD  biClrUsed;
	DWORD  biClrImportant;
} BITMAPINFOHEADER;


// ピクセルのデータを読むためのバッファの構造体
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
// 構築・破棄

// 構築
IBMP* BmpIO_CreateBitmap(int width, int height, int BitPerPixcel)
{
	IBMP *p = NULL;

	// ビット数の指定を確認する。
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

	// 24Bitカラーの場合
	if (24 == BitPerPixcel) {
		// p->pPixは使用せず、p->pColorにピクセルデータを格納する。
		p->pColor = (ICOLOR*)calloc(width * height, sizeof(ICOLOR));
		if (NULL == p->pColor) goto ERR_EXIT;
	}
	else {
		// p->pColorにはカラーテーブルを格納する。
		p->pColor = (ICOLOR*)calloc((1 << BitPerPixcel), sizeof(ICOLOR));
		p->pPix = (unsigned char*)calloc(width * height, sizeof(unsigned char));
		if (NULL == p->pColor || NULL == p->pPix) goto ERR_EXIT;
	}

	p->width = width;
	p->height = height;
	p->BitPerPix = BitPerPixcel;

	return p;

	// 失敗した場合
ERR_EXIT:
	free(p->pColor);
	free(p->pPix);
	free(p);
	return NULL;
}

// 破棄
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
// 24bitビットマップ用

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
// 1,4,8bitビットマップ用

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
	int PixCount;	// 総ピクセル数
	int i;

	assert(NULL != pBmp);

	// 24Bitカラーのデータが渡されたら、処理をせずに真を返す
	if (24 == pBmp->BitPerPix) return TRUE;

	assert(NULL != pBmp->pColor && NULL != pBmp->pPix);
	assert(1 == pBmp->BitPerPix ||
		4 == pBmp->BitPerPix ||
		8 == pBmp->BitPerPix);

	PixCount = pBmp->width * pBmp->height;

	// 24Bitカラーの画像を格納するための領域を確保
	wpColor = (ICOLOR*)calloc(PixCount, sizeof(ICOLOR));
	if (NULL == wpColor) return FALSE;

	// 各ピクセルに色を設定する
	for (i = 0; i < PixCount; i++)
		wpColor[i] = pBmp->pColor[pBmp->pPix[i]];

	// 構造体の中身を差し替える
	free(pBmp->pColor);
	free(pBmp->pPix);
	pBmp->pColor = wpColor;
	pBmp->pPix = NULL;
	pBmp->BitPerPix = 24;

	return TRUE;
}

///////////////////////////////////////////////////////////////////////////////////
// 内部処理

// インデックスを生成
static int PixIdx(int w, int h, const IBMP *pBmp)
{
	assert(w >= 0 && w < pBmp->width && h >= 0 && h <= pBmp->height);
	w = w % pBmp->width;
	h = h % pBmp->height;
	return h * pBmp->width + w;
}


///////////////////////////////////////////////////////////////////////////////////
// 入力用ルーチン

static BOOL LoadHeader(
	FILE *infile, unsigned int *ctsize, int *blen, int *pWidth, int *pHeight
);
static BOOL LoadBody1(FILE *infile, int BitLen, IBMP *pBmp);
static BOOL LoadBody24(FILE *infile, IBMP *pBmp);
static void FrushBuf_ipt(BUF *pBuf, FILE *infile);
static int ReadBuf(BUF *pBuf, int len, FILE *infile);
static size_t int_fread(void *buffer, size_t size, size_t count, FILE *stream);

// イメージ読み込み指示
IBMP* BmpIO_Load(FILE *infile)
{
	unsigned int ctsize;	// カラーテーブルのエントリ数
	int blen;		// １ピクセルあたりビット長
	IBMP *pBmp;
	int w, h;
	BOOL r;

	// ファイルヘッダの入力、カラーテーブル長、
	// pixelsの領域長、ピクセルあたりのビット長を取得
	if (!LoadHeader(infile, &ctsize, &blen, &w, &h)) return FALSE;

	// メモリ領域を確保
	pBmp = BmpIO_CreateBitmap(w, h, blen);
	if (NULL == pBmp) return NULL;

	// ピクセルあたりビット長別にファイルボディ部を読み込む
	if (24 != blen)
		r = LoadBody1(infile, blen, pBmp);
	else
		r = LoadBody24(infile, pBmp);

	if (!r) {
		// 失敗
		BmpIO_DeleteBitmap(pBmp);
		return NULL;
	}

	return pBmp;
}

// ヘッダ部を読む
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

	// 各構造体を入力
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
	// ファイルタイプを確認
	if (bfh.bfType != 0x4D42) return FALSE;

	// 読み込んだ情報を返す
	(*blen) = bi.biBitCount;	// １ピクセル当りのビット長
	(*ctsize) = bi.biClrUsed;	// 使用される色の数
	(*pWidth) = bi.biWidth;		// 幅
	(*pHeight) = bi.biHeight;	// 高さ

	return TRUE;
}

// カラーテーブルを使用するビットマップを読み込む
static BOOL LoadBody1(FILE *infile, int BitLen, IBMP *pBmp)
{
	int i, j;
	BUF buf;	// 読み込むためのバッファ
	int ctsize = (1 << BitLen);	// カラーテーブルの数

								// カラーテーブルを読む
	for (i = 0; i < ctsize && !ferror(infile) && !feof(infile); i++) {
		pBmp->pColor[i].b = fgetc(infile);
		pBmp->pColor[i].g = fgetc(infile);
		pBmp->pColor[i].r = fgetc(infile);
		fgetc(infile);	// rgbReserved
	}
	if (ferror(infile) || feof(infile)) return FALSE;

	// バッファの内容を初期化する
	buf.BufU.buf = 0;
	buf.buflen = 0;
	FrushBuf_ipt(&buf, infile);

	for (i = 0; i < pBmp->height && !feof(infile) && !ferror(infile); i++) {
		for (j = 0; j < pBmp->width && !feof(infile) && !ferror(infile); j++) {
			int wIdx = PixIdx(j, i, pBmp);
			// 指定されたビット長のデータを取得
			pBmp->pPix[wIdx] = ReadBuf(&buf, BitLen, infile);
		}
		FrushBuf_ipt(&buf, infile);
	}

	return (i == pBmp->height && j == pBmp->width);
}

// ２４ビットカラービットマップを読む
static BOOL LoadBody24(FILE *infile, IBMP *pBmp)
{
	int PixCnt = pBmp->width * pBmp->height;	// ピクセル数
	int i, j;
	BUF buf;

	// バッファの内容を初期化する
	buf.buflen = 0;
	buf.BufU.buf = 0;
	FrushBuf_ipt(&buf, infile);

	for (i = 0; i < pBmp->height && !feof(infile) && !ferror(infile); i++) {
		for (j = 0; j < pBmp->width && !feof(infile) && !ferror(infile); j++) {
			// 色はBGRの順に記録されている
			BmpIO_SetB(j, i, pBmp, (unsigned char)ReadBuf(&buf, 8, infile));
			BmpIO_SetG(j, i, pBmp, (unsigned char)ReadBuf(&buf, 8, infile));
			BmpIO_SetR(j, i, pBmp, (unsigned char)ReadBuf(&buf, 8, infile));
		}
		FrushBuf_ipt(&buf, infile);
	}
	return (i == pBmp->height && j == pBmp->width);
}

// バッファにデータを読み込む
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

// バッファから指定したビット数分データを取得する
static int ReadBuf(BUF *pBuf, int len, FILE *infile)
{
	int r;
	if (pBuf->buflen < len) FrushBuf_ipt(pBuf, infile);
	r = ((((1 << len) - 1) << (32 - len)) & pBuf->BufU.buf) >> (32 - len);
	pBuf->BufU.buf <<= len;
	pBuf->buflen -= len;
	return r;
}

// エンディアンネスを吸収して、ファイルを読み込む
size_t int_fread(void *buffer, size_t size, size_t count, FILE *stream)
{
#if ISLITTLEENDIAN
	return fread(buffer, size, count, stream);
#else
	size_t i, j;
	size_t r;
	char *cbuf = (char*)buffer;

	// 読み込む
	r = fread(buffer, size, count, stream);
	if (1 == size) return r;

	// 項目ごとにバイトオーダーを反転
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
// 出力用ルーチン

static BOOL WriteHeader(FILE *outfile, const IBMP *pBmp);
static BOOL WriteBody1(FILE *outfile, const IBMP *pBmp);
static BOOL WriteBody24(FILE *outfile, const IBMP *pBmp);
static void FrushBuf_opt(BUF *pBuf, FILE *outfile);
static void WriteBuf(BUF *pBuf, int BitLen, unsigned char c, FILE *outfile);
static size_t int_fwrite(const void *buffer, size_t size, size_t count, FILE *stream);

// イメージを出力する
BOOL BmpIO_Save(FILE *outfile, const IBMP *pBmp)
{
	// ヘッダを出力する
	if (!WriteHeader(outfile, pBmp)) return FALSE;

	// イメージのデータを出力する
	if (24 == pBmp->BitPerPix)
		return WriteBody24(outfile, pBmp);
	else
		return WriteBody1(outfile, pBmp);
}

// ヘッダ部を出力する
static BOOL WriteHeader(FILE *outfile, const IBMP *pBmp)
{
	BITMAPFILEHEADER bfh;
	BITMAPINFOHEADER bi;

	// 構造体に値を設定
	bfh.bfType = 0x4D42;
	bfh.bfSize = 0;
	bfh.bfReserved1 = 0;
	bfh.bfReserved2 = 0;
	bfh.bfOffBits = BITMAPFILEHEADER_SIZE + BITMAPINFOHEADER_SIZE;
	if (pBmp->BitPerPix <= 8)
		bfh.bfOffBits += 4 * (1 << pBmp->BitPerPix);
	bi.biSize = BITMAPINFOHEADER_SIZE;
	bi.biWidth = pBmp->width;
	bi.biHeight = pBmp->height;
	bi.biPlanes = 1;
	bi.biBitCount = pBmp->BitPerPix;
	bi.biCompression = 0L;
	bi.biSizeImage = 0;
	bi.biXPelsPerMeter = 1;
	bi.biYPelsPerMeter = 1;
	bi.biClrUsed = 0;
	bi.biClrImportant = 0;

	// 各構造体を出力
	if (int_fwrite(&(bfh.bfType), sizeof(bfh.bfType), 1, outfile) < 1)
		return FALSE;
	if (int_fwrite(&(bfh.bfSize), sizeof(bfh.bfSize), 1, outfile) < 1)
		return FALSE;
	if (int_fwrite(&(bfh.bfReserved1), sizeof(bfh.bfReserved1), 1, outfile) < 1)
		return FALSE;
	if (int_fwrite(&(bfh.bfReserved2), sizeof(bfh.bfReserved2), 1, outfile) < 1)
		return FALSE;
	if (int_fwrite(&(bfh.bfOffBits), sizeof(bfh.bfOffBits), 1, outfile) < 1)
		return FALSE;

	if (int_fwrite(&(bi.biSize), sizeof(bi.biSize), 1, outfile) < 1)
		return FALSE;
	if (int_fwrite(&(bi.biWidth), sizeof(bi.biWidth), 1, outfile) < 1)
		return FALSE;
	if (int_fwrite(&(bi.biHeight), sizeof(bi.biHeight), 1, outfile) < 1)
		return FALSE;
	if (int_fwrite(&(bi.biPlanes), sizeof(bi.biPlanes), 1, outfile) < 1)
		return FALSE;
	if (int_fwrite(&(bi.biBitCount), sizeof(bi.biBitCount), 1, outfile) < 1)
		return FALSE;
	if (int_fwrite(&(bi.biCompression), sizeof(bi.biCompression), 1, outfile) < 1)
		return FALSE;
	if (int_fwrite(&(bi.biSizeImage), sizeof(bi.biSizeImage), 1, outfile) < 1)
		return FALSE;
	if (int_fwrite(&(bi.biXPelsPerMeter), sizeof(bi.biXPelsPerMeter), 1, outfile) < 1)
		return FALSE;
	if (int_fwrite(&(bi.biYPelsPerMeter), sizeof(bi.biYPelsPerMeter), 1, outfile) < 1)
		return FALSE;
	if (int_fwrite(&(bi.biClrUsed), sizeof(bi.biClrUsed), 1, outfile) < 1)
		return FALSE;
	if (int_fwrite(&(bi.biClrImportant), sizeof(bi.biClrImportant), 1, outfile) < 1)
		return FALSE;

	return TRUE;
}

// 1,4,8bitビットマップを出力する
BOOL WriteBody1(FILE *outfile, const IBMP *pBmp)
{
	int i, j;
	int ctcnt = 1 << pBmp->BitPerPix;	// カラーテーブルの数
	BUF buf;

	// カラーテーブルを出力する
	for (i = 0; i < ctcnt && !ferror(outfile); i++) {
		fputc(pBmp->pColor[i].b, outfile);
		fputc(pBmp->pColor[i].g, outfile);
		fputc(pBmp->pColor[i].r, outfile);
		fputc(0, outfile);
	}

	buf.BufU.buf = 0;
	buf.buflen = 0;

	// ピクセルデータを出力
	for (i = 0; i < pBmp->height; i++) {
		for (j = 0; j < pBmp->width; j++) {
			WriteBuf(&buf, pBmp->BitPerPix, BmpIO_GetPixcel(j, i, pBmp), outfile);
		}
		FrushBuf_opt(&buf, outfile);
	}
	return TRUE;
}

// ２４ビットカラービットマップを出力する
static BOOL WriteBody24(FILE *outfile, const IBMP *pBmp)
{
	int i, j;
	BUF buf;

	buf.BufU.buf = 0;
	buf.buflen = 0;

	// ３バイトずつ出力する
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

// バッファのデータを全て出力する
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

// バッファに指定したビット数分データを出力する
static void WriteBuf(BUF *pBuf, int BitLen, unsigned char c, FILE *outfile)
{
	const unsigned long MASK = (0x1 << (BitLen + 1)) - 1;
	pBuf->BufU.buf |= (((unsigned long)c) & MASK) << (32 - (pBuf->buflen + BitLen));
	pBuf->buflen += BitLen;
	if (pBuf->buflen >= 32)
		FrushBuf_opt(pBuf, outfile);
}

// エンディアンネスを吸収して、ファイルを出力する
size_t int_fwrite(const void *buffer, size_t size, size_t count, FILE *stream)
{
#if ISLITTLEENDIAN
	return fwrite(buffer, size, count, stream);
#else
	size_t i, j;
	char *cbuf = (char*)buffer;
	char *wbuf = (char*)alloca(size);
	size_t r;

	// 1バイト単位ならば、反転する必要はない
	if (1 == size)
		return fwrite(buffer, size, count, stream);

	// 項目ごとにバイトオーダーを反転して出力する
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
// TODO : 整理する。これは殴り書きしたときの残骸
// クラス構造が整理できたら消えるはず
extern MMD_File    mmdfile;
extern Texture     madoka_magic;
MMD_BoneNode       root_bone;


// parentで指定された親ノードを探し、その子としてchidを追加する
// 根ノードはroot_bone
void add_children(MMD_BoneNode* node, uint16_t parent_id, uint16_t child_id)
{
    if(child_id == 0){  // rootである
        node->bone_index = 0;
    }

    // 親となるべきノードを検出した
    if(parent_id == node->bone_index){
        MMD_BoneNode    child_node;
        child_node.bone_index = child_id;
        node->children.push_back(child_node);
    }

    for(int i = 0; i < node->children.size(); i++){
        // 子ノードへ降りて、目的のparent_idがないか探す
        MMD_BoneNode*   child = &node->children[i];
        add_children(child, parent_id, child_id);
    }
}


//演算子のオーバーロード Quaternionの積
Quaternion & operator *( Quaternion &q1, Quaternion &q2 )
{
    Quaternion q0={
        q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
        q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
        q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
        q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w,
    };
    q1=q0;
    return q1;
}

//クォータニオンから回転行列を算出
// r = 16*16のopenGL座標系に対応した回転行列
void qtor(float *r , Quaternion q)
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
    double r1[16]={ 1.0 - yy - zz, xy + zw, zx - yw, 0.0,
        xy - zw, 1.0 - zz - xx, yz + xw, 0.0,
        zx + yw, yz - xw, 1.0 - xx - yy, 0.0,
        0.0, 0.0, 0.0, 1.0};
    for (int i = 0;i < 16;i++) {
        r[i]=r1[i];
    }
}


void MMD_Header::read(FILE* fp){
    // アライメントがそろっていないので、自前で読み込む
    fread(magic, 1, 3, fp);
    fread(&version, 1, 4, fp);
    fread(model_name, 1, 20, fp);
    fread(comment, 1, 256, fp);
}

MMD_Header::MMD_Header(){
    memset(magic, 3, 0);
    version = 0.0f;
    memset(model_name, 0, 20);
    memset(comment, 0, 256);
}
MMD_Header::~MMD_Header(){}


void MMD_face::read(FILE* fp){
    fread(&face_vert_count, 1, 4, fp);
    printf("face vert count = %d\n", face_vert_count);

    face_index = new GLuint[face_vert_count];
    for(int i = 0; i < face_vert_count; i++){
        fread(&face_index[i], 1, 2, fp);
    }
}

MMD_face::MMD_face(){
}

MMD_face::~MMD_face(){
}

void MMD_face::draw()
{
}

MMD_vertex::MMD_vertex(){
}

MMD_vertex::~MMD_vertex(){
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
    p3dVerted=NULL;
    p3dNormal=NULL;
    pTexuv=NULL;
}

MMD_VertexArray::~MMD_VertexArray()
{
    if(p3dVerted){
        delete []p3dVerted;
        p3dVerted = NULL;
    }
    if(p3dNormal){
        delete []p3dNormal;
        p3dNormal = NULL;
    }
    if(pTexuv){
        delete []pTexuv;
        pTexuv = NULL;
    }
};

void MMD_VertexArray::read(FILE* fp)
{
    fread(&count, 4, 1, fp);
    printf("vertex count == %d\n", count);       // デバッグ用

    pVertex = new MMD_vertex[count];
    for(int i = 0;i < count; i++){
        pVertex[i].read(fp);
    }

    p3dVerted = new GLfloat[count*3]; 
    p3dNormal = new GLfloat[count*3]; 
    pTexuv = new GLfloat[count*2];
}

void MMD_VertexArray::draw()
{
    // 頂点, 法線, テクスチャ座標ごとの配列に置き換える
    for(int i = 0; i < count; i++ ){
        p3dVerted[i*3 + 0] = pVertex[i].x;
        p3dVerted[i*3 + 1] = pVertex[i].y;
        p3dVerted[i*3 + 2] = pVertex[i].z;
        p3dNormal[i*3 + 0] = pVertex[i].nx;
        p3dNormal[i*3 + 1] = pVertex[i].ny;
        p3dNormal[i*3 + 2] = pVertex[i].nz;
        pTexuv[i*2+0] = pVertex[i].u;
        pTexuv[i*2+1] = fabsf(1.0f - pVertex[i].v);
    }
    glVertexPointer(3, GL_FLOAT, 0, p3dVerted);     // 頂点配列 (0はストライド)
    glNormalPointer(GL_FLOAT, 0, p3dNormal);        // 法線配列
    glTexCoordPointer(2, GL_FLOAT, 0, pTexuv);      // テクスチャ座標
}

Texture::Texture(void)
{
}

Texture::~Texture()
{
    if(pBmp)
        glDeleteTextures(1, &texture_id);
};

void Texture::load(const char* filename, GLuint tex_id)
{
    pBmp = NULL;
    FILE* fp = fopen(filename, "rb");
    if(fp != NULL){
        pBmp = BmpIO_Load(fp);
        if(pBmp != NULL){
            // OpenGL用にテクスチャつくる
            this->texture_id = tex_id;
            printf("load texture_id = %d\n", this->texture_id);
            printf("id: %d = %s\n", this->texture_id, filename);
            glBindTexture(GL_TEXTURE_2D, this->texture_id);
            glTexImage2D(
                GL_TEXTURE_2D ,0 ,GL_RGB ,pBmp->width ,pBmp->height,
                0 ,GL_RGB ,GL_UNSIGNED_BYTE, pBmp->pColor );
            glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
            glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
            glFlush();
            glBindTexture(GL_TEXTURE_2D, 0 );
//            glBindTexture( GL_TEXTURE_2D, 10 );   // いらないかも
        }else{
            printf(">>>>>>pBmp == NULL\n");
        }
        fclose(fp);
    }
}

unsigned int Texture::get_gl_texture_id(void)
{
    return this->texture_id;
}


// マテリアルデータを表すクラス
Material::Material(void)
{
    tex_index = 0;
}
Material::~Material(){}

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
    texture.load( fullpath_texture.c_str(), texture_id );
}

// マテリアルの配列
MaterialArray::MaterialArray(){};
MaterialArray::~MaterialArray(){};
void MaterialArray::setpath(std::string pathname)
{
    path = pathname;
}

void MaterialArray::read(FILE* fp)
{
    fread(&count, 1, 4, fp);
    mat_array = new Material[count];

    // テクスチャの上限数がここで確定するので、
    // この位置でテクスチャIDを保持する配列をつくる
    // TODO: テクスチャを使わないマテリアルの場合は無駄が出てしまうので、対応したい
    textureIds = new GLuint[count];
    glGenTextures(count, textureIds);
    printf("glGenTextures Array = %d\n", count);
    for(int i = 0;i < count; i++){
        printf("[index:%d == id=%d]\n", i, textureIds[i] );
    }

    // テクスチャを一枚ずつ読み込んでいく
    // TODO: 異なるマテリアルでも同じファイル名を指していることもあるので、
    // マテリアルとテクスチャを1:1の関係では効率悪そう。
    int face_sum = 0;
    for(int i = 0;i < count; i++){
        mat_array[i].setpath(path);
        mat_array[i].read(fp, textureIds[i] );
        mat_array[i].start_face = face_sum;
        face_sum += mat_array[i].get_face_vert_count();
    }
}


void MaterialArray::draw(void)
{
    for(int i = 0; i < count; i++){
        mat_array[i].draw();
    }
}


void Material::draw()
{
    MMD_face*   face = &mmdfile.m_face;
    int face_count = this->face_vert_count;

    glBindTexture(GL_TEXTURE_2D, texture.get_gl_texture_id() );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );

    glFrontFace(GL_CCW);        // GL_CW(時計回りが表), GL_CCW(反時計回りが表)
    glEnable(GL_CULL_FACE);     //カリングOFF

    GLsizei primitive_count = face_count;

    glDrawElements(GL_TRIANGLES,                    // プリミティブの種類
                    primitive_count,                // レンダリング要素の数
                    GL_UNSIGNED_INT,                // インデックス配列の型
                    &face->face_index[start_face]); // インデックス配列をさすポインタ

    glFlush();
}


// テクスチャがこのpmdファイルの配下にあるので仕方なく。。
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
    fread(bone_name, 1, 20, fp);            // ボーン名
    fread(&parent_bone_index, 2, 1, fp);    // 親ボーンインデックス
    fread(&tail_pos_bone_index, 2, 1, fp);  // tail位置のボーンインデックス
    fread(&bone_type,1, 1, fp);             // ボーンの種類
    fread(&ik_parent_bone_index, 2, 1, fp);   // IKボーン番号(ない場合は0)
    fread(bone_head_pos, 4, 3, fp);       // ボーンのヘッドの位置

    printf("bone_name = %s, ", bone_name);
    printf("parent index = %d\n", parent_bone_index);
}

void MMD_Bone::draw()
{
    // 回転行列はクォータニオンとして与えられるので、回転行列に変換する
    qtor(rot_mat, quot);

    glTranslatef(tx, ty, tz);
    glMultMatrixf(rot_mat);

    // glutSolidCone(底面の半径,円錐の高さ,円の分割数,高さの分割数)　
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
    // ボーンデータを読み込む
    fread(&bone_count, 1, 2, fp);

    bone_array = new MMD_Bone[bone_count];

    // あとは繰り返し
    for(int i = 0;i < bone_count; i++){
        bone_array[i].read(fp);
        // ボーンの木を構成する
        uint16_t    parent_id = bone_array[i].parent_bone_index;
        uint16_t    child_id = i;
        add_children(&root_bone, parent_id, child_id);
    }

    // ボーン名をキーにしたインデックスを作成
    for(int i = 0; i < bone_count; i++){
        std::string bone_name_str = bone_array[i].bone_name;
        bone_name_dict[bone_name_str] = i;
    }
}

// 親階層から下っていきながら描画を行う関数
// arrayは本来は不要だが、外部から引き渡してもらうことにする
void DrawRecarsive(MMD_BoneNode* node, MMD_BoneArray* array)
{
    uint16_t bone_index = node->bone_index;
    array->bone_array[bone_index].draw();
    for(int i = 0; i < node->children.size(); i++){
        MMD_BoneNode* child = &node->children[i];
        // 子ノードの描画にはいる前に
        // 子ノードへの直線をひいておく
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
    // 全てのボーンを描画し
    for(int i = 0;i < bone_count; i++){
        glPushMatrix();
        bone_array[i].draw();
        glPopMatrix();
    }
#endif
}

// 引数で指定されたファイルを開いて、モデルデータを読み込む
void MMD_File::load(const char* iFilename)
{
    std::string fullpath = path + iFilename;
    FILE* fp = fopen(fullpath.c_str(), "rb");
    if(fp == NULL){
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

void MMD_File::draw(void){
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


