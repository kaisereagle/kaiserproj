//**********************
// �e�N�X�`�����X�g�Ǘ�
//**********************

#include	<windows.h>
#include	<GL/gl.h>
#include	<GL/glu.h>

#include	<stdio.h>
#include	<malloc.h>
#include	<string.h>
#include	<math.h>
//#include	<gl/glut.h>
#include	"TextureList.h"

#ifdef _WIN32
#include <gdiplus.h>
#endif

cTextureList	g_clsTextureList;

//================
// �R���X�g���N�^
//================
cTextureList::cTextureList( void ) : m_pTextureList( NULL )
{
}

//==============
// �f�X�g���N�^
//==============
cTextureList::~cTextureList( void )
{
	TextureData	*pTemp = m_pTextureList,
				*pNextTemp;

	while( pTemp )
	{
		pNextTemp = pTemp->pNext;

		glDeleteTextures( 1, &pTemp->uiTexID );
		delete pTemp;

		pTemp = pNextTemp;
	}

	m_pTextureList = NULL;
}

//====================
// �e�N�X�`��ID���擾
//====================
unsigned int cTextureList::getTexture( const char *szFileName )
{
	unsigned int	uiTexID = 0xFFFFFFFF;

	// �܂��͂��łɓǂݍ��܂�Ă��邩�ǂ�������
	if( findTexture( szFileName, &uiTexID ) )
	{
		return uiTexID;
	}

	// �Ȃ���΃t�@�C����ǂݍ���Ńe�N�X�`���쐬
	if( createTexture( szFileName, &uiTexID ) )
	{
		// �V�������X�g�m�[�h���쐬���擪�֐ڑ�
		TextureData	*pNew = new TextureData;

		strcpy( pNew->szFileName, szFileName );
		pNew->uiTexID = uiTexID;
		pNew->uiRefCount = 1;

		pNew->pNext = m_pTextureList;
		m_pTextureList = pNew;

		return uiTexID;
	}

	return 0xFFFFFFFF;	// �e�N�X�`���Ǎ����s�܂��͍쐬���s
}

//----------------------------------
// �ǂݍ��ݍς݂̃e�N�X�`�����猟��
//----------------------------------
bool cTextureList::findTexture( const char *szFileName, unsigned int *puiTexID )
{
	TextureData	*pTemp = m_pTextureList;

	while( pTemp )
	{
		if( strcmp( pTemp->szFileName, szFileName ) == 0 )
		{
			// �ǂݍ��ݍς݂̃e�N�X�`���𔭌�
			*puiTexID = pTemp->uiTexID;
			pTemp->uiRefCount++;

			return true;
		}

		pTemp = pTemp->pNext;
	}

	return false;	// �����ł���
}

//--------------------------------------
// �t�@�C����ǂݍ���Ńe�N�X�`�����쐬
//--------------------------------------
bool cTextureList::createTexture( const char *szFileName, unsigned int *puiTexID )
{
	FILE	*pFile;
	fpos_t	fposFileSize;
	unsigned char
			*pData;

	int		iLen = strlen( szFileName );
	bool	bRet = false;

	// 2009/07/14 Ru--en	������OS�ł������悤���݂�
#ifdef _WIN32
	wchar_t wszFileName[256];
	int		iWLen;
	ZeroMemory( &wszFileName, 256 );
	// �����R�[�h�ϊ��B932(Shift-JIS)���烏�C�h������
	iWLen = MultiByteToWideChar( 932, 0, szFileName, iLen, wszFileName, 256 );
	if ( iWLen == 0 )	return false;	// �ϊ����s

	if (_wfopen_s( &pFile, wszFileName, L"rb" )) {
		return false;	// �t�@�C�����J���Ȃ�
	}
#else
	pFile = fopen( szFileName, "rb" );
	if( !pFile )	return false;	// �t�@�C�����J���Ȃ�
#endif

	// �t�@�C���T�C�Y�擾
	fseek( pFile, 0, SEEK_END );
	fgetpos( pFile, &fposFileSize );

	// �������m��
	pData = (unsigned char *)malloc( (size_t)fposFileSize );

	// �ǂݍ���
	fseek( pFile, 0, SEEK_SET );
	fread( pData, 1, (size_t)fposFileSize, pFile );

	fclose( pFile );

	if( 	(szFileName[iLen - 3] == 'b' || szFileName[iLen - 3] == 'B') &&
			(szFileName[iLen - 2] == 'm' || szFileName[iLen - 2] == 'M') &&
			(szFileName[iLen - 1] == 'p' || szFileName[iLen - 1] == 'P')		)
	{
		bRet = createFromBMP( pData, puiTexID );
	}
	else if((szFileName[iLen - 3] == 't' || szFileName[iLen - 3] == 'T') &&
			(szFileName[iLen - 2] == 'g' || szFileName[iLen - 2] == 'G') &&
			(szFileName[iLen - 1] == 'a' || szFileName[iLen - 1] == 'A')		)
	{
		bRet = createFromTGA( pData, puiTexID );
	}
	else if((szFileName[iLen - 3] == 's' || szFileName[iLen - 3] == 'S') &&
			(szFileName[iLen - 2] == 'p' || szFileName[iLen - 2] == 'P') &&
			(szFileName[iLen - 1] == 'h' || szFileName[iLen - 1] == 'H')		)
	{
		bRet = createFromBMP( pData, puiTexID );
	}
	else if((szFileName[iLen - 3] == 's' || szFileName[iLen - 3] == 'S') &&
			(szFileName[iLen - 2] == 'p' || szFileName[iLen - 2] == 'P') &&
			(szFileName[iLen - 1] == 'a' || szFileName[iLen - 1] == 'A')		)
	{
		bRet = createFromBMP( pData, puiTexID );
#ifdef _WIN32
	} else {
		// GDI+���g�����e�N�X�`���Ǎ�
		bRet = createUsingGdi( wszFileName, puiTexID );
#endif
	}

	free( pData );

	return bRet;
}

//-------------------------------
// BMP�t�@�C������e�N�X�`���쐬
//-------------------------------
bool cTextureList::createFromBMP( const unsigned char *pData, unsigned int *puiTexID )
{
#pragma pack( push, 1 )
	// BMP�t�@�C���w�b�_�\����
	struct BMPFileHeader
	{
		unsigned short	bfType;			// �t�@�C���^�C�v
		unsigned int	bfSize;			// �t�@�C���T�C�Y
		unsigned short	bfReserved1;
		unsigned short	bfReserved2;
		unsigned int	bfOffBits;		// �t�@�C���擪����摜�f�[�^�܂ł̃I�t�Z�b�g
	};

	// BMP���w�b�_�\����
	struct BMPInfoHeader
	{
		unsigned int	biSize;			// ���w�b�_�[�̃T�C�Y
		int				biWidth;		// ��
		int				biHeight;		// ����(���Ȃ�Ή������A���Ȃ�Ώォ�牺)
		unsigned short	biPlanes;		// �v���[����(���1)
		unsigned short	biBitCount;		// 1��f������̃r�b�g��
		unsigned int	biCompression;
		unsigned int	biSizeImage;
		int				biXPelsPerMeter;
		int				biYPelsPerMeter;
		unsigned int	biClrUsed;		// �p���b�g�̐F��
		unsigned int	biClrImportant;
	};

	// �p���b�g�f�[�^
	struct RGBQuad
	{
		unsigned char	rgbBlue;
		unsigned char	rgbGreen;
		unsigned char	rgbRed;
		unsigned char	rgbReserved;
	};
#pragma pack( pop )

	// BMP�t�@�C���w�b�_
	BMPFileHeader	*pBMPFileHeader = (BMPFileHeader *)pData;

	if( pBMPFileHeader->bfType != ('B' | ('M' << 8)) )
	{
		return false;	// �t�@�C���^�C�v���Ⴄ
	}

	// BMP���w�b�_
	BMPInfoHeader	*pBMPInfoHeader = (BMPInfoHeader *)(pData + sizeof(BMPFileHeader));

	if( pBMPInfoHeader->biBitCount == 1 || pBMPInfoHeader->biCompression != 0 )
	{
		return false;	// 1�r�b�g�J���[�ƈ��k�`���ɂ͖��Ή�
	}

	// �J���[�p���b�g
	RGBQuad			*pPalette = NULL;

	if( pBMPInfoHeader->biBitCount < 24 )
	{
		pPalette = (RGBQuad *)(pData + sizeof(BMPFileHeader) + sizeof(BMPInfoHeader));
	}

	// �摜�f�[�^�̐擪��
	pData += pBMPFileHeader->bfOffBits;

	// �摜�f�[�^��1���C���̃o�C�g��
	unsigned int	uiLineByte = ((pBMPInfoHeader->biWidth * pBMPInfoHeader->biBitCount + 0x1F) & (~0x1F)) / 8;

	// �e�N�X�`���C���[�W�̍쐬
	unsigned char	*pTexelData = (unsigned char *)malloc( pBMPInfoHeader->biWidth * pBMPInfoHeader->biHeight * 4 ),
					*pTexelDataTemp = pTexelData;

	if( pBMPInfoHeader->biBitCount == 4 )
	{
		// 4Bit�J���[
		for( int h = pBMPInfoHeader->biHeight - 1 ; h >= 0 ; h-- )
		{
			const unsigned char *pLineTop = &pData[uiLineByte * h];

			for( int w = 0 ; w < (pBMPInfoHeader->biWidth >> 1) ; w++ )
			{
				*pTexelDataTemp = pPalette[(pLineTop[w] >> 4) & 0x0F].rgbRed;	pTexelDataTemp++;
				*pTexelDataTemp = pPalette[(pLineTop[w] >> 4) & 0x0F].rgbGreen;	pTexelDataTemp++;
				*pTexelDataTemp = pPalette[(pLineTop[w] >> 4) & 0x0F].rgbBlue;	pTexelDataTemp++;
				*pTexelDataTemp = 255;											pTexelDataTemp++;

				*pTexelDataTemp = pPalette[(pLineTop[w]     ) & 0x0F].rgbRed;	pTexelDataTemp++;
				*pTexelDataTemp = pPalette[(pLineTop[w]     ) & 0x0F].rgbGreen;	pTexelDataTemp++;
				*pTexelDataTemp = pPalette[(pLineTop[w]     ) & 0x0F].rgbBlue;	pTexelDataTemp++;
				*pTexelDataTemp = 255;											pTexelDataTemp++;
			}
		}
	}
	else if( pBMPInfoHeader->biBitCount == 8 )
	{
		// 8Bit�J���[
		for( int h = pBMPInfoHeader->biHeight - 1 ; h >= 0 ; h-- )
		{
			const unsigned char *pLineTop = &pData[uiLineByte * h];

			for( int w = 0 ; w < pBMPInfoHeader->biWidth ; w++ )
			{
				*pTexelDataTemp = pPalette[pLineTop[w]].rgbRed;		pTexelDataTemp++;
				*pTexelDataTemp = pPalette[pLineTop[w]].rgbGreen;	pTexelDataTemp++;
				*pTexelDataTemp = pPalette[pLineTop[w]].rgbBlue;	pTexelDataTemp++;
				*pTexelDataTemp = 255;								pTexelDataTemp++;
			}
		}
	}
	else if( pBMPInfoHeader->biBitCount == 24 )
	{
		// 24Bit�J���[
		for( int h = pBMPInfoHeader->biHeight - 1 ; h >= 0 ; h-- )
		{
			const unsigned char *pLineTop = &pData[uiLineByte * h];

			for( int w = 0 ; w < pBMPInfoHeader->biWidth ; w++ )
			{
				*pTexelDataTemp = pLineTop[w * 3 + 2];	pTexelDataTemp++;
				*pTexelDataTemp = pLineTop[w * 3 + 1];	pTexelDataTemp++;
				*pTexelDataTemp = pLineTop[w * 3    ];	pTexelDataTemp++;
				*pTexelDataTemp = 255;					pTexelDataTemp++;
			}
		}
	}
	else if( pBMPInfoHeader->biBitCount == 32 )
	{
		// 32Bit�J���[
		for( int h = pBMPInfoHeader->biHeight - 1 ; h >= 0 ; h-- )
		{
			const unsigned char *pLineTop = &pData[uiLineByte * h];

			for( int w = 0 ; w < pBMPInfoHeader->biWidth ; w++ )
			{
				*pTexelDataTemp = pLineTop[w * 4 + 2];	pTexelDataTemp++;
				*pTexelDataTemp = pLineTop[w * 4 + 1];	pTexelDataTemp++;
				*pTexelDataTemp = pLineTop[w * 4    ];	pTexelDataTemp++;
				*pTexelDataTemp = 255;					pTexelDataTemp++;
			}
		}
	}

	// 2009/07/09 Ru--en	2^n�T�C�Y�ɋ����I�ɍ��킹��
	int width = pBMPInfoHeader->biWidth;
	int height = pBMPInfoHeader->biHeight;
	fitTextureSize(&pTexelData, &width, &height);

	// �e�N�X�`���̍쐬
	glGenTextures( 1, puiTexID );

	glBindTexture( GL_TEXTURE_2D, *puiTexID );
	glPixelStorei( GL_UNPACK_ALIGNMENT, 4 );

	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );

	glTexImage2D(	GL_TEXTURE_2D, 0, GL_RGBA, 
					width, height, //pBMPInfoHeader->biWidth, pBMPInfoHeader->biHeight,
					0, GL_RGBA, GL_UNSIGNED_BYTE,
					pTexelData );

	float	fPrioritie = 1.0f;
	glPrioritizeTextures( 1, puiTexID, &fPrioritie );

	free( pTexelData );

	return true;
}

//-------------------------------
// TGA�t�@�C������e�N�X�`���쐬
//-------------------------------
bool cTextureList::createFromTGA( const unsigned char *pData, unsigned int *puiTexID )
{
#pragma pack( push, 1 )
	struct TGAFileHeader
	{
		unsigned char	tfIdFieldLength;
		unsigned char	tfColorMapType;
		unsigned char	tfImageType;
		unsigned short	tfColorMapIndex;
		unsigned short	tfColorMapLength;
		unsigned char	tfColorMapSize;
		unsigned short	tfImageOriginX;
		unsigned short	tfImageOriginY;
		unsigned short	tfImageWidth;
		unsigned short	tfImageHeight;
        unsigned char	tfBitPerPixel;
        unsigned char	tfDiscripter;
	};
#pragma pack( pop )

	TGAFileHeader	*pTgaFileHeader = (TGAFileHeader *)pData;

/*
0	�C���[�W�Ȃ�
1	�C���f�b�N�X�J���[�i256�F�j
2	�t���J���[
3	����
9	�C���f�b�N�X�J���[�BRLE���k
A	�t���J���[�BRLE���k
B	�����BRLE���k
*/
	if( pTgaFileHeader->tfImageType != 0x02 && pTgaFileHeader->tfImageType != 0x0A )
	{
		// ��Ή��t�H�[�}�b�g
		return false;
	}

	pData += sizeof( TGAFileHeader );

	unsigned char	*pTexelData = (unsigned char *)malloc( pTgaFileHeader->tfImageWidth * pTgaFileHeader->tfImageHeight * 4 ),
					*pTexelDataTemp = pTexelData;

	if( pTgaFileHeader->tfImageType == 0x02 && pTgaFileHeader->tfBitPerPixel == 24 )
	{
		// �񈳏k24Bit�J���[
		if( pTgaFileHeader->tfDiscripter & 0x20 )
		{
			// �ォ�牺��
			for( int h = 0 ; h < pTgaFileHeader->tfImageHeight ; h++ )
			{
				const unsigned char *pLineTop = &pData[(pTgaFileHeader->tfImageWidth * 3) * h];

				for( int w = 0 ; w < pTgaFileHeader->tfImageWidth ; w++ )
				{
					*pTexelDataTemp = pLineTop[w * 3 + 2];	pTexelDataTemp++;
					*pTexelDataTemp = pLineTop[w * 3 + 1];	pTexelDataTemp++;
					*pTexelDataTemp = pLineTop[w * 3    ];	pTexelDataTemp++;
					*pTexelDataTemp = 255;					pTexelDataTemp++;
				}
			}
		}
		else
		{
			// ��������
			for( int h = pTgaFileHeader->tfImageHeight - 1 ; h >= 0 ; h-- )
			{
				const unsigned char *pLineTop = &pData[(pTgaFileHeader->tfImageWidth * 3) * h];

				for( int w = 0 ; w < pTgaFileHeader->tfImageWidth ; w++ )
				{
					*pTexelDataTemp = pLineTop[w * 3 + 2];	pTexelDataTemp++;
					*pTexelDataTemp = pLineTop[w * 3 + 1];	pTexelDataTemp++;
					*pTexelDataTemp = pLineTop[w * 3    ];	pTexelDataTemp++;
					*pTexelDataTemp = 255;					pTexelDataTemp++;
				}
			}
		}
	}
	else if( pTgaFileHeader->tfImageType == 0x02 && pTgaFileHeader->tfBitPerPixel == 32 )
	{
		// �񈳏k32Bit�J���[
		if( pTgaFileHeader->tfDiscripter & 0x20 )
		{
			// �ォ�牺��
			for( int h = 0 ; h < pTgaFileHeader->tfImageHeight ; h++ )
			{
				const unsigned char *pLineTop = &pData[(pTgaFileHeader->tfImageWidth * 4) * h];

				for( int w = 0 ; w < pTgaFileHeader->tfImageWidth ; w++ )
				{
					*pTexelDataTemp = pLineTop[w * 4 + 2];	pTexelDataTemp++;
					*pTexelDataTemp = pLineTop[w * 4 + 1];	pTexelDataTemp++;
					*pTexelDataTemp = pLineTop[w * 4    ];	pTexelDataTemp++;
					*pTexelDataTemp = pLineTop[w * 4 + 3];	pTexelDataTemp++;
				}
			}
		}
		else
		{
			// ��������
			for( int h = pTgaFileHeader->tfImageHeight - 1 ; h >= 0 ; h-- )
			{
				const unsigned char *pLineTop = &pData[(pTgaFileHeader->tfImageWidth * 4) * h];

				for( int w = 0 ; w < pTgaFileHeader->tfImageWidth ; w++ )
				{
					*pTexelDataTemp = pLineTop[w * 4 + 2];	pTexelDataTemp++;
					*pTexelDataTemp = pLineTop[w * 4 + 1];	pTexelDataTemp++;
					*pTexelDataTemp = pLineTop[w * 4    ];	pTexelDataTemp++;
					*pTexelDataTemp = pLineTop[w * 4 + 3];	pTexelDataTemp++;
				}
			}
		}
	}
	else if( pTgaFileHeader->tfImageType == 0x0A )
	{
		// ���k24/32Bit�J���[
		if( pTgaFileHeader->tfDiscripter & 0x20 )
		{
			// �ォ�牺��
			short	nPosX = 0,
					nPosY = 0;

            while( nPosY < pTgaFileHeader->tfImageHeight )
            {
                bool	bCompress =	((*pData) & 0x80) == 0x80;
                short	nLength = ((*pData) & 0x7F) + 1;

				pData++;

                if( bCompress )
                {
                    for( short i = 0 ; i < nLength ; i++ )
                    {
						*pTexelDataTemp = pData[2];	pTexelDataTemp++;
						*pTexelDataTemp = pData[1];	pTexelDataTemp++;
						*pTexelDataTemp = pData[0];	pTexelDataTemp++;

						if( pTgaFileHeader->tfBitPerPixel == 32 )	*pTexelDataTemp = pData[3];
						else										*pTexelDataTemp = 255;
						pTexelDataTemp++;

						nPosX++;
						if( pTgaFileHeader->tfImageWidth <= nPosX )
						{
							nPosX = 0;
							nPosY++;
						}
                    }

					if( pTgaFileHeader->tfBitPerPixel == 32 )	pData += 4;
					else										pData += 3;
                }
                else
                {
                    for( short i = 0 ; i < nLength ; i++ )
                    {
						*pTexelDataTemp = pData[2];	pTexelDataTemp++;
						*pTexelDataTemp = pData[1];	pTexelDataTemp++;
						*pTexelDataTemp = pData[0];	pTexelDataTemp++;

						if( pTgaFileHeader->tfBitPerPixel == 32 )	*pTexelDataTemp = pData[3];
						else										*pTexelDataTemp = 255;
						pTexelDataTemp++;

						if( pTgaFileHeader->tfBitPerPixel == 32 )	pData += 4;
						else										pData += 3;

                        nPosX++;
                        if( pTgaFileHeader->tfImageWidth <= nPosX )
                        {
                            nPosX = 0;
                            nPosY++;
                        }
                    }
                }
            }
		}
		else
		{
			// ��������
			short	nPosX = 0,
					nPosY = pTgaFileHeader->tfImageHeight - 1;

            while( 0 <= nPosY )
            {
                bool	bCompress =	((*pData) & 0x80) == 0x80;
                short	nLength = ((*pData) & 0x7F) + 1;

				pData++;

                if( bCompress )
                {
                    for( short i = 0 ; i < nLength ; i++ )
                    {
						pTexelDataTemp = &pTexelData[(nPosX + nPosY * pTgaFileHeader->tfImageWidth) * 4];

						*pTexelDataTemp = pData[2];	pTexelDataTemp++;
						*pTexelDataTemp = pData[1];	pTexelDataTemp++;
						*pTexelDataTemp = pData[0];	pTexelDataTemp++;

						if( pTgaFileHeader->tfBitPerPixel == 32 )	*pTexelDataTemp = pData[3];
						else										*pTexelDataTemp = 255;

						nPosX++;
						if( pTgaFileHeader->tfImageWidth <= nPosX )
						{
							nPosX = 0;
							nPosY--;
						}
                    }

					if( pTgaFileHeader->tfBitPerPixel == 32 )	pData += 4;
					else										pData += 3;
                }
                else
                {
                    for( short i = 0 ; i < nLength ; i++ )
                    {
						pTexelDataTemp = &pTexelData[(nPosX + nPosY * pTgaFileHeader->tfImageWidth) * 4];

						*pTexelDataTemp = pData[2];	pTexelDataTemp++;
						*pTexelDataTemp = pData[1];	pTexelDataTemp++;
						*pTexelDataTemp = pData[0];	pTexelDataTemp++;

						if( pTgaFileHeader->tfBitPerPixel == 32 )	*pTexelDataTemp = pData[3];
						else										*pTexelDataTemp = 255;

						if( pTgaFileHeader->tfBitPerPixel == 32 )	pData += 4;
						else										pData += 3;

                        nPosX++;
                        if( pTgaFileHeader->tfImageWidth <= nPosX )
                        {
                            nPosX = 0;
                            nPosY--;
                        }
                    }
                }
            }
		}
	}

	// 2009/07/09 Ru--en	2^n�T�C�Y�ɋ����I�ɍ��킹��
	int width = pTgaFileHeader->tfImageWidth;
	int height =  pTgaFileHeader->tfImageHeight;
	fitTextureSize(&pTexelData, &width, &height);

	// �e�N�X�`���̍쐬
	glGenTextures( 1, puiTexID );

	glBindTexture( GL_TEXTURE_2D, *puiTexID );
	glPixelStorei( GL_UNPACK_ALIGNMENT, 4 );

	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );

	glTexImage2D(	GL_TEXTURE_2D, 0, GL_RGBA, 
					width, height,	//pTgaFileHeader->tfImageWidth, pTgaFileHeader->tfImageHeight,
					0, GL_RGBA, GL_UNSIGNED_BYTE,
					pTexelData );

	float	fPrioritie = 1.0f;
	glPrioritizeTextures( 1, puiTexID, &fPrioritie );

	free( pTexelData );

	return true;
}

// PNG,GIF,JPEG,TIFF������e�N�X�`���쐬	2011/04/30 Ru--en
//-------------------------------
#ifdef _WIN32
bool cTextureList::createUsingGdi( const WCHAR *wszFileName, unsigned int *puiTexID )
{
	// GDI+ �Ńt�@�C������Ǎ�
	Gdiplus::Bitmap bmp(wszFileName);
	Gdiplus::BitmapData bmpData;
	//bmp.LockBits( 0, Gdiplus::ImageLockModeRead, bmp.GetPixelFormat(), &bmpData );
	bmp.LockBits( 0, Gdiplus::ImageLockModeRead, PixelFormat32bppARGB, &bmpData );

	// �摜�̕������쐬
	int width = bmpData.Width;
	int height = bmpData.Height;
	int size =  bmpData.Stride * bmpData.Height;
	unsigned char	*pTexelData = (unsigned char *)malloc( size );
	memcpy( pTexelData, bmpData.Scan0, size );
	
	bmp.UnlockBits( &bmpData );


	// 2^n�T�C�Y�ɋ����I�ɍ��킹��
	fitTextureSize(&pTexelData, &width, &height);

	// �e�N�X�`���̍쐬
	glGenTextures( 1, puiTexID );

	glBindTexture( GL_TEXTURE_2D, *puiTexID );
	glPixelStorei( GL_UNPACK_ALIGNMENT, 4 );

	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
	glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );

	glTexImage2D(	GL_TEXTURE_2D, 0, GL_RGBA, 
					width, height,
					0, GL_BGRA_EXT, GL_UNSIGNED_BYTE,
					pTexelData );

	float	fPrioritie = 1.0f;
	glPrioritizeTextures( 1, puiTexID, &fPrioritie );

	free( pTexelData );

	return true;
}
#endif


//==============
// �f�o�b�O�\��
//==============
void cTextureList::debugDraw( void )
{
	glDisable( GL_DEPTH_TEST );
	glDisable( GL_LIGHTING );

	glMatrixMode( GL_PROJECTION );
	glPushMatrix();
	glLoadIdentity();
	//gluOrtho2D( 0.0f, (float)glutGet( GLUT_WINDOW_WIDTH ), 0.0f, (float)glutGet( GLUT_WINDOW_HEIGHT ) );
	gluOrtho2D( 0.0f, 400.0f, 0.0f, 400.0f );

	glMatrixMode( GL_MODELVIEW );
	glPushMatrix();
	glLoadIdentity();

	glColor4f( 1.0f, 1.0f, 1.0f, 1.0f );

	glEnable( GL_TEXTURE_2D );

	TextureData	*pTemp = m_pTextureList;
	float		fPosX = 0.0f,
				fPosY = 400.0f;

	#define		DISP_SIZE	64.0f

	while( pTemp )
	{
		glBindTexture( GL_TEXTURE_2D, pTemp->uiTexID );

		glBegin( GL_TRIANGLE_FAN );
			glTexCoord2f( 1.0f, 1.0f );	glVertex2f( fPosX,             fPosY - DISP_SIZE );
			glTexCoord2f( 1.0f, 0.0f );	glVertex2f( fPosX + DISP_SIZE, fPosY - DISP_SIZE );
			glTexCoord2f( 0.0f, 0.0f );	glVertex2f( fPosX + DISP_SIZE, fPosY             );
			glTexCoord2f( 0.0f, 1.0f );	glVertex2f( fPosX,             fPosY             );
		glEnd();

		fPosX += DISP_SIZE + 2.0f;
		if( fPosX >= 400.0f - DISP_SIZE )
		{
			fPosX  =             0.0f;
			fPosY -= DISP_SIZE + 2.0f;
		}

		pTemp = pTemp->pNext;
	}

	glMatrixMode( GL_PROJECTION );
	glPopMatrix();
	glMatrixMode( GL_MODELVIEW );
	glPopMatrix();

	glEnable( GL_DEPTH_TEST );
	glEnable( GL_LIGHTING );
}

//================
// �e�N�X�`�����
//================
void cTextureList::releaseTexture( unsigned int uiTexID )
{
	TextureData	*pTemp = m_pTextureList,
				*pPrevTemp = NULL;

	while( pTemp )
	{
		if( pTemp->uiTexID == uiTexID )
		{
			pTemp->uiRefCount--;
			if( pTemp->uiRefCount <= 0 )
			{
				glDeleteTextures( 1, &pTemp->uiTexID );

				if( pPrevTemp )	pPrevTemp->pNext = pTemp->pNext;
				else			m_pTextureList = pTemp->pNext;

				delete pTemp;
			}

			return;
		}

		pPrevTemp = pTemp;
		pTemp = pTemp->pNext;
	}
}

//=========================================
// �e�N�X�`���̉摜�T�C�Y�� 2^n �ɍ��킹��
//=========================================
void cTextureList::fitTextureSize(unsigned char **pData, int *pWidth, int *pHeight)
{
	int width2 = ceilToPowerOfTwo(*pWidth);
	int height2 = ceilToPowerOfTwo(*pHeight);

	// ���݂̃T�C�Y�Ŗ��Ȃ���Ή������Ȃ�
	if (width2 == *pWidth && height2 == *pHeight) return;

	// �摜���T�C�Y
	unsigned char *pNewData = (unsigned char *)malloc( width2 * height2 * 4 );
	gluScaleImage(
		GL_RGBA,
		*pWidth,
		*pHeight,
		GL_UNSIGNED_BYTE,
		*pData,
		width2,
		height2,
		GL_UNSIGNED_BYTE,
		pNewData
		);
	free( *pData );		// �O�̃f�[�^�͏��� 

	// �f�[�^����ւ�
	*pData = pNewData;

	// �T�C�Y����ւ�
	*pWidth = width2;
	*pHeight = height2;
}

//=======================================
// ���鐳�̐�����2^n�ɐ؂�グ������Ԃ�
//
//  2009/01/06 1024�ȓ��ɂȂ�悤�ύX
//=======================================
int cTextureList::ceilToPowerOfTwo(int number)
{
	int result = 0;
	int n = 0;
	if (number < 0)		return 0;		// ���̐��ɂ͑Ή����Ȃ�
	if (number > 1024)	return 1024;	// �ő�l�I�[�o�[

	for (int n = 10; n >= 0; n--) { // �ő�e�N�X�`���T�C�Y 2^(9+1)==1024
		int val = 1 << n;			// 2^n
		if (number & val) {
			if (number & ~val) {
				result = val << 1;
			} else {
				result = val;
			}
			break;
		}
	}
	if (result < 1) result = 1;
	return result;
}
