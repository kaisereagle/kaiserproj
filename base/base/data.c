//
//  data.c
//  base
//
//  Created by kaiser on 2018/01/29.
//  Copyright © 2018年 kaiser. All rights reserved.
//

#define _CRT_SECURE_NO_WARNINGS
#define GL_VERSION_1_3
//VS2010までない
//#include "windows.h"
#include <stdint.h>
#ifdef __APPLE__
#import     <OpenGLES/ES2/gl.h>
#import     <OpenGLES/ES2/glext.h>
#include <fcntl.h> // for open
#include <unistd.h> // for close

#endif

#ifdef _WINDOWS
#include <windows.h>
#include "windowsx.h"
#include <winuser.h>
#include <GL/gl.h>
#include <GL/glu.h>
//#include <GL/glsl.h>
//#include <GL/gl.h>
//#include <GL/glu.h>
//#include <GL/glsl.h>
#include "GL/glext.h"
//#include "corecrt_math_defines.h"
#endif
#include    <stdio.h>
#include    <stdlib.h>
#include    <stdbool.h>
#include    <assert.h>
#include    <string.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include "data.h"

#ifdef _WINDOWS
#include <sys\stat.h>
#else
#include <sys/stat.h>
#include <sys/xattr.h>
#endif
#include "lodepng.h"
#ifdef _WINDOWS


/**
* SJIS文字列をUTF8に変換する
*/
void ES20_sjis2utf8(GLchar *str, const int array_length) {
	const int str_len = (int)strlen(str);
	if (!str_len) {
		return;
	}
	const int nSize = MultiByteToWideChar(CP_ACP, 0, (LPCSTR)str, -1, NULL, 0);

	char* buffUtf16 = (char *)calloc((nSize * 2 + 2), sizeof(char));
	MultiByteToWideChar(CP_ACP, 0, (LPCSTR)str, -1, (LPWSTR)buffUtf16, nSize);
	const int nSizeUtf8 = WideCharToMultiByte(CP_UTF8, 0, (LPCWSTR)buffUtf16, -1, NULL, 0, NULL, NULL);
	char* buffUtf8 = (char *)calloc((nSizeUtf8 * 2), sizeof(char));
	ZeroMemory(buffUtf8, nSizeUtf8 * 2);
	WideCharToMultiByte(CP_UTF8, 0, (LPCWSTR)buffUtf16, -1, (LPSTR)	buffUtf8, nSizeUtf8, NULL, NULL);

	int *pSize = lstrlen((char*)buffUtf8);
	memcpy(str, buffUtf8, *pSize);

	memcpy(str, buffUtf8, str_len);
	str[str_len] = '\0';
}
#endif
/**
 * PVRTCテクスチャヘッダ。
 */
typedef struct PVRTexHeader {
    uint32_t headerLength;
    uint32_t height;
    uint32_t width;
    uint32_t numMipmaps;
    uint32_t flags;
    uint32_t dataLength;
    uint32_t bpp;
    uint32_t bitmaskRed;
    uint32_t bitmaskGreen;
    uint32_t bitmaskBlue;
    uint32_t bitmaskAlpha;
    uint8_t pvrTag[4];
    uint32_t numSurfs;
} PVRTexHeader;

#define PVR_TEXTURE_FLAG_TYPE_MASK  0xff
#define 	CC_GL_DEPTH24_STENCIL8   GL_DEPTH24_STENCIL8
//todo ios用
#define 	GL_COMPRESSED_RGB_PVRTC_4BPPV1_IMG   0x8C00
#define 	GL_COMPRESSED_RGB_PVRTC_2BPPV1_IMG   0x8C01
#define 	GL_COMPRESSED_RGBA_PVRTC_4BPPV1_IMG   0x8C02
#define 	GL_COMPRESSED_RGBA_PVRTC_2BPPV1_IMG   0x8C03
/**
 * PVRTC圧縮画像を読み込む
 * 読み込んだ画像はes20_freePvrtcImage()で解放する
 */
PvrtcImage* PvrtcImage_load(GLApplication *app, const char *file_name) {
    
    enum {
        kPVRTextureFlagTypePVRTC_2 = 24,
        kPVRTextureFlagTypePVRTC_4
    };
    
    RawData *raw = RawData_loadFile(app, file_name);
    
    if (!raw) {
        // fail...
        return NULL;
    }
    
    PVRTexHeader *header = (PVRTexHeader*) RawData_getReadHeader(raw);
    
    // check tag
    if (header->pvrTag[0] != 'P' || header->pvrTag[1] != 'V' || header->pvrTag[2] != 'R' || header->pvrTag[3] != '!') {
        RawData_freeFile(app, raw);
       // __logf("texture format error(%s)", file_name);
        return NULL;
    }
    
   // __logf("pvrtc mipmaps(%d) surfs(%d)", header->numMipmaps, header->numSurfs);
    
    PvrtcImage *result = (PvrtcImage*) malloc(sizeof(PvrtcImage));
    
    // データコピー
    result->raw = raw;
    result->width = header->width;
    result->height = header->height;
    {
        // 詳細フォーマットのチェック
        switch (header->flags & PVR_TEXTURE_FLAG_TYPE_MASK) {
            case kPVRTextureFlagTypePVRTC_2:
                result->format = GL_COMPRESSED_RGBA_PVRTC_2BPPV1_IMG;
                result->bits_per_pixel = 2;
                break;
            case kPVRTextureFlagTypePVRTC_4:
                result->format = GL_COMPRESSED_RGBA_PVRTC_4BPPV1_IMG;
                result->bits_per_pixel = 4;
                break;
            default:
                // format error
                assert(0);
                break;
        }
    }
    
    // 各圧縮画像へのポインタを計算する
    // 画像は固定ビットレートで格納されるため、ポインタ位置を計算可能。
    {
        // mipmap対応チェック
        // 等倍テクスチャ+mipmap数を保持するため、例えば等倍テクスチャであればnumMpmapは0になる。
        // +1を行うことでfor文で回すことができる。
        result->mipmaps = header->numMipmaps + 1;
        result->image_table = (void**) malloc(sizeof(void*) * result->mipmaps);
        result->image_length_table = (int*) malloc(sizeof(void*) * result->mipmaps);
        
        int miplevel = 0;
        int texWidth = result->width;
        int texHeight = result->height;
        RawData_setHeaderPosition(raw, header->headerLength);
        uint8_t *image_header = RawData_getReadHeader(raw);
        for (miplevel = 0; miplevel < result->mipmaps; ++miplevel) {
            
            // 1ブロックのサイズは圧縮時オプションで変動する
            const int blockSize = header->flags == kPVRTextureFlagTypePVRTC_4 ? (4 * 4) : (8 * 4);
            int widthBlocks = texWidth / (header->flags == kPVRTextureFlagTypePVRTC_4 ? 4 : 8);
            int heightBlocks = texHeight / 4;
            const int bpp = header->flags == kPVRTextureFlagTypePVRTC_4 ? 4 : 2;
            
            // 最低限のブロック数は持たなければならない
            if (widthBlocks < 2) {
                widthBlocks = 2;
            }
            if (heightBlocks < 2) {
                heightBlocks = 2;
            }
            
           // __logf("decode blockSize(%d) Xblock(%d) Yblocks(%d) bit par pix(%d)", blockSize, widthBlocks, heightBlocks, bpp);
            
            const int dataSize = widthBlocks * heightBlocks * ((blockSize * bpp) / 8);
            
            // 次のmipmapを読む
            texWidth /= 2;
            texHeight /= 2;
            
            // テクスチャデータを保存する
            result->image_length_table[miplevel] = dataSize;
            result->image_table[miplevel] = image_header;
            
            image_header += dataSize;
        }
    }
    
    assert(result->width == result->height);
    assert(Texture_checkPowerOfTwo(result->width));
    assert(Texture_checkPowerOfTwo(result->height));
    
   // __logf("PVRTC mipmaps(%d) tex size(%d x %d) format(%s)", result->mipmaps, result->width, result->height, result->format == GL_COMPRESSED_RGBA_PVRTC_2BPPV1_IMG ? "GL_COMPRESSED_RGBA_PVRTC_2BPPV1_IMG" : "GL_COMPRESSED_RGBA_PVRTC_4BPPV1_IMG");
    return result;
}

/**
 * PVRTC圧縮画像を解放する
 */
void PvrtcImage_free(GLApplication *app, PvrtcImage *pvrtc) {
    if (pvrtc) {
        RawData_freeFile(app, pvrtc->raw);
        
        free(pvrtc->image_table);
        free(pvrtc->image_length_table);
        
        free(pvrtc);
    }
}


/**
 * 画像をテクスチャとして読み込む。
 * 読み込んだ画像はes20_freeTexture()で解放する
 */
Texture* PvrtcImage_loadTexture(GLApplication *app, const char* file_name) {
	return NULL;
#ifndef _WINDOWS

    PvrtcImage *pvrtc = PvrtcImage_load(app, file_name);
    
    // error images
    if (!pvrtc) {
        return NULL;
    }
    
    Texture *texture = (Texture*) malloc(sizeof(Texture));
    
    {
        // 元画像から必要情報をコピーする
        texture->width = pvrtc->width;
        texture->height = pvrtc->height;
    }
    
    {
        // 領域確保
        glGenTextures(1, &texture->id);
        assert(texture->id > 0);
        assert(glGetError() == GL_NO_ERROR);
    }
    
    glBindTexture(GL_TEXTURE_2D, texture->id);
    
    {
        // VRAMへピクセル情報をコピーする
        int width = pvrtc->width;
        int height = pvrtc->height;
        int miplevel = 0;
        
        for (miplevel = 0; miplevel < pvrtc->mipmaps; ++miplevel) {
            glCompressedTexImage2D(GL_TEXTURE_2D, miplevel, pvrtc->format, width, height, 0, pvrtc->image_length_table[miplevel], pvrtc->image_table[miplevel]);
            
            width /= 2;
            height /= 2;
            assert(glGetError() == GL_NO_ERROR);
        }
    }
    
    {
        // wrapの初期設定
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        assert(glGetError() == GL_NO_ERROR);
    }
    
    {
        // filterの初期設定
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        if (pvrtc->mipmaps > 1) {
            // mipmapを保持している場合
            // texturetoolへ -m のオプションを与えると生成できる
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
        } else {
            // mipmapを保持していない場合
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        }
        assert(glGetError() == GL_NO_ERROR);
    }
    
    // unbindする
    glBindTexture(GL_TEXTURE_2D, 0);
    assert(glGetError() == GL_NO_ERROR);
    
    // 元画像を解放
    PvrtcImage_free(app, pvrtc);
    return texture;
#endif
}

/**
 * 圧縮画像を読み込む。
 * 読み込んだ画像はes20_freePkmImage()で解放する
 */
PkmImage* PkmImage_load(GLApplication *app, const char* file_name) {
    RawData *raw = RawData_loadFile(app, file_name);
    
    if (!raw) {
        // fail...
        return NULL;
    }
    
    PkmImage *image = (PkmImage*) malloc(sizeof(PkmImage));
    
    image->raw = raw;
    image->image_bytes = RawData_getLength(raw);
    
    {
        // check header
        char magic[4] = { 0 };
        magic[0] = RawData_read8(raw);
        magic[1] = RawData_read8(raw);
        magic[2] = RawData_read8(raw);
        magic[3] = RawData_read8(raw);
        
        assert(magic[0] == 'P');
        assert(magic[1] == 'K');
        assert(magic[2] == 'M');
    }
    {
        // check version
        char version[2] = { 0 };
        version[0] = RawData_read8(raw);
        version[1] = RawData_read8(raw);
        
        assert(version[0] == '1');
        assert(version[1] == '0');
    }
    // read data type
    image->data_type = RawData_readBE16(raw);
    // 圧縮後の幅と高さを読み込む
    image->width = RawData_readBE16(raw);
    image->height = RawData_readBE16(raw);
    // 圧縮前の幅と高さを読み込む
    image->origin_width = RawData_readBE16(raw);
    image->origin_height = RawData_readBE16(raw);
    
    image->image = RawData_getReadHeader(raw);
    image->image_bytes = RawData_getAvailableBytes(raw);
    
    // texture size check
    assert(image->width >= image->origin_width);
    assert(image->height >= image->origin_height);
    
   // __logf("ETC1 bytes(%d) tex size(%d x %d) origin size(%d x %d)", image->image_bytes, image->width, image->height, image->origin_width, image->origin_height);
    return image;
}


/**
 * 画像をテクスチャとして読み込む。
 * 読み込んだ画像はes20_freeTexture()で解放する
 */
Texture* PkmImage_loadTexture(GLApplication *app, const char* file_name) {
    // GL_OES_compressed_ETC1_RGB8_textureがサポートされていないプラットフォームではifdefで切る
#ifndef GL_OES_compressed_ETC1_RGB8_texture
  //  __log("サポート外のテクスチャ形式(GL_OES_compressed_ETC1_RGB8_texture)");
    return NULL;
#else
    PkmImage *pkm = PkmImage_load(app, file_name);
    
    // error images
    if (!pkm) {
        return NULL;
    }
    
    Texture *texture = (Texture*) malloc(sizeof(Texture));
    
    {
        // 元画像から必要情報をコピーする
        texture->width = pkm->width;
        texture->height = pkm->height;
    }
    
    {
        // 領域確保
        glGenTextures(1, &texture->id);
        assert(texture->id > 0);
        assert(glGetError() == GL_NO_ERROR);
    }
    
    glBindTexture(GL_TEXTURE_2D, texture->id);
    
    {
        // VRAMへピクセル情報をコピーする
        glCompressedTexImage2D(GL_TEXTURE_2D, 0, GL_ETC1_RGB8_OES, pkm->width, pkm->height, 0, pkm->image_bytes, pkm->image);
        assert(glGetError() == GL_NO_ERROR);
    }
    
    {
        // wrapの初期設定
        // 互換性のため、初期は常にGL_CLAMP_TO_EDGE
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        assert(glGetError() == GL_NO_ERROR);
    }
    
    {
        // filterの初期設定
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        assert(glGetError() == GL_NO_ERROR);
    }
    
    // unbindする
    glBindTexture(GL_TEXTURE_2D, 0);
    assert(glGetError() == GL_NO_ERROR);
    
    // 元画像を解放
    PkmImage_free(app, pkm);
    return texture;
#endif
}

/**
 * RGB888のポインタをdst_pixelsへピクセル情報をコピーする。
 */
void RawPixelImage_convertColorRGB(const void *rgb888_pixels, const int pixel_format, void *dst_pixels, const int pixel_num) {
    // 残ピクセル数
    int pixels = pixel_num;
    unsigned char *src_rgb888 = (unsigned char *) rgb888_pixels;
    
    switch (pixel_format) {
        case TEXTURE_RAW_RGB565: {
            unsigned short *p = (unsigned short*) dst_pixels;
            while (pixels) {
                
                const int r = src_rgb888[0] & 0xff;
                const int g = src_rgb888[1] & 0xff;
                const int b = src_rgb888[2] & 0xff;
                
                (*p) = ((r >> 3) << 11) | ((g >> 2) << 5) | ((b >> 3));
                src_rgb888 += 3;
                ++p;
                --pixels;
            }
        }
            break;
        case TEXTURE_RAW_RGBA5551: {
            unsigned short *p = (unsigned short*) dst_pixels;
            while (pixels) {
                
                const int r = src_rgb888[0] & 0xff;
                const int g = src_rgb888[1] & 0xff;
                const int b = src_rgb888[2] & 0xff;
                const int a = 1;
                (*p) = ((r >> 3) << 11) | ((g >> 3) << 6) | ((b >> 3) << 1) | a;
                src_rgb888 += 3;
                ++p;
                --pixels;
            }
        }
            break;
        case TEXTURE_RAW_RGB8: {
            memcpy(dst_pixels, src_rgb888, pixels * 3);
        }
            break;
        case TEXTURE_RAW_RGBA8: {
            unsigned char *dst = (unsigned char*) dst_pixels;
            while (pixels) {
                
                dst[0] = src_rgb888[0];
                dst[1] = src_rgb888[1];
                dst[2] = src_rgb888[2];
                dst[3] = 0xFF;
                
                src_rgb888 += 3;
                dst += 4;
                --pixels;
            }
        }
            break;
    }
}
/**
 * RGBA8888のポインタをdst_pixelsへピクセル情報をコピーする。
 */
void RawPixelImage_convertColorRGBA(const void *rgba8888_pixels, const int pixel_format, void *dst_pixels, const int pixel_num) {
    // 残ピクセル数
    int pixels = pixel_num;
    unsigned char *src_rgba8888 = (unsigned char *) rgba8888_pixels;
    
    switch (pixel_format) {
        case TEXTURE_RAW_RGB565: {
            unsigned short *p = (unsigned short*) dst_pixels;
            while (pixels) {
                
                const int r = src_rgba8888[0] & 0xff;
                const int g = src_rgba8888[1] & 0xff;
                const int b = src_rgba8888[2] & 0xff;
                
                (*p) = ((r >> 3) << 11) | ((g >> 2) << 5) | ((b >> 3));
                src_rgba8888 += 4;
                ++p;
                --pixels;
            }
        }
            break;
        case TEXTURE_RAW_RGBA5551: {
            unsigned short *p = (unsigned short*) dst_pixels;
            while (pixels) {
                
                const int r = src_rgba8888[0] & 0xff;
                const int g = src_rgba8888[1] & 0xff;
                const int b = src_rgba8888[2] & 0xff;
                const int a = (src_rgba8888[3] & 0xff) > 0 ? 1 : 0;
                (*p) = ((r >> 3) << 11) | ((g >> 3) << 6) | ((b >> 3) << 1) | a;
                src_rgba8888 += 4;
                ++p;
                --pixels;
            }
        }
            break;
        case TEXTURE_RAW_RGB8: {
            unsigned char *dst = (unsigned char*) dst_pixels;
            while (pixels) {
                
                dst[0] = src_rgba8888[0];
                dst[1] = src_rgba8888[1];
                dst[2] = src_rgba8888[2];
                
                src_rgba8888 += 4;
                dst += 3;
                --pixels;
            }
        }
            break;
        case TEXTURE_RAW_RGBA8: {
            memcpy(dst_pixels, src_rgba8888, pixels * 4);
        }
            break;
    }
}




/**
* 画像を読み込む
*/
RawPixelImage*    RawPixelImage_load(GLApplication *app, const char* file_name, const int pixel_format) {

	int pixelsize = 0;
	unsigned char* buffer;
	unsigned char* image;
	size_t buffersize, imagesize;
//	LodePNG_Decoder decoder;
	//LodePNG_Decoder_init(&decoder);
	/**
	* 1ピクセルの深度を指定する
	*/
	switch (pixel_format) {
	case TEXTURE_RAW_RGBA8:
		pixelsize = 4;
		break;
	case TEXTURE_RAW_RGB8:
		pixelsize = 3;
		break;
	case TEXTURE_RAW_RGB565:
	case TEXTURE_RAW_RGBA5551:
		pixelsize = 2;
		break;
	}
	RawPixelImage *result = (RawPixelImage*)malloc(sizeof(RawPixelImage));
	//LodePNG_loadFile(&buffer, &buffersize, file_name);
	//LodePNG_decode(&decoder, &image, &imagesize, buffer, buffersize);
	int width;
	int height;
	lodepng_load_file(&buffer, &buffersize, file_name);
	lodepng_decode32(&image, &width, &height, buffer, buffersize);
	// resultを書き込む
	{
		result->format = pixel_format;
		result->pixel_data = (void*)image;
		result->width = width;
		result->height = height;
	}
	return result;
}



/**
 * 画像用に確保したメモリを解放する
 */
void RawPixelImage_free(GLApplication *app, RawPixelImage *image) {
    if (image) {
        if (image->pixel_data) {
            free(image->pixel_data);
            image->pixel_data = NULL;
        }
        free(image);
    }
}



/**
 * 画像をテクスチャとして読み込む。
 * 読み込んだ画像はes20_freeTexture()で解放する
 */
Texture* RawPixelImage_loadTexture(GLApplication *app, const char* file_name, const int pixel_fotmat) {
    RawPixelImage *image = RawPixelImage_load(app, file_name, pixel_fotmat);
    
    // error images
    if (!image) {
        return NULL;
    }
    
    Texture *texture = (Texture*) malloc(sizeof(Texture));
    
    {
        // 元画像から必要情報をコピーする
        texture->width = image->width;
        texture->height = image->height;
    }
    
    {
        // 領域確保
        glGenTextures(1, &texture->id);
        assert(texture->id > 0);
        assert(glGetError() == GL_NO_ERROR);
    }
    
    glBindTexture(GL_TEXTURE_2D, texture->id);
    
    {
        // VRAMへピクセル情報をコピーする
        static const GLenum FORMAT[] = { GL_RGBA, GL_RGB, GL_RGBA, GL_RGB };
        static const GLenum TYPE[] = { GL_UNSIGNED_BYTE, GL_UNSIGNED_BYTE, GL_UNSIGNED_SHORT_5_5_5_1, GL_UNSIGNED_SHORT_5_6_5 };
        glTexImage2D(GL_TEXTURE_2D, 0, FORMAT[pixel_fotmat], image->width, image->height, 0, FORMAT[pixel_fotmat], TYPE[pixel_fotmat], image->pixel_data);
        
        assert(glGetError() == GL_NO_ERROR);
    }
    
    {
        // wrapの初期設定
        // 互換性のため、デフォルトはGL_CLAMP_TO_EDGE
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        assert(glGetError() == GL_NO_ERROR);
    }
    
    {
        // filterの初期設定
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        assert(glGetError() == GL_NO_ERROR);
    }
    
    // unbindする
    glBindTexture(GL_TEXTURE_2D, 0);
    assert(glGetError() == GL_NO_ERROR);
    
    // 元画像を廃棄する
    RawPixelImage_free(app, image);
    return texture;
}


/**
 * 2次元ベクトルを生成する
 */
vec2 vec2_create(const GLfloat x, const GLfloat y) {
    vec2 v = { x, y };
    return v;
}

/**
 * 3次元ベクトルを生成する
 */
vec3 vec3_create(const GLfloat x, const GLfloat y, const GLfloat z) {
    vec3 v = { x, y, z };
    return v;
}

/**
 * 3次元ベクトルの長さを取得する
 */
GLfloat vec3_length(const vec3 v) {
    return (GLfloat) sqrt(((double) v.x * (double) v.x) + ((double) v.y * (double) v.y) + ((double) v.z * (double) v.z));
}

/**
 * 3次元ベクトルを正規化する
 */
vec3 vec3_normalize(const vec3 v) {
    const GLfloat len = vec3_length(v);
    return vec3_create(v.x / len, v.y / len, v.z / len);
}

/**
 * 正規化した3次元ベクトルを生成する
 */
vec3 vec3_createNormalized(const GLfloat x, const GLfloat y, const GLfloat z) {
    return vec3_normalize(vec3_create(x, y, z));
}

/**
 * 3次元ベクトルの内積を計算する
 */
GLfloat vec3_dot(const vec3 v0, const vec3 v1) {
    return (v0.x * v1.x) + (v0.y * v1.y) + (v0.z * v1.z);
}

/**
 * 3次元ベクトルの外積を計算する
 */
vec3 vec3_cross(const vec3 v0, const vec3 v1) {
    return vec3_create((v0.y * v1.z) - (v0.z * v1.y), (v0.z * v1.x) - (v0.x * v1.z), (v0.x * v1.y) - (v0.y * v1.x));
}

/**
 * 単位行列を生成する
 */
mat4 mat4_identity() {
    mat4 result;
    
    int column = 0;
    int row = 0;
    
    for (column = 0; column < 4; ++column) {
        for (row = 0; row < 4; ++row) {
            if (column == row) {
                result.m[column][row] = 1.0f;
            } else {
                result.m[column][row] = 0.0f;
            }
        }
    }
    
    return result;
}

/**
 * 移動行列を作成する
 */
mat4 mat4_translate(const GLfloat x, const GLfloat y, const GLfloat z) {
    mat4 result = mat4_identity();
    
    result.m[3][0] = x;
    result.m[3][1] = y;
    result.m[3][2] = z;
    
    return result;
}

/**
 * 拡縮行列を作成する
 */
mat4 mat4_scale(const GLfloat x, const GLfloat y, const GLfloat z) {
    mat4 result = mat4_identity();
    
    result.m[0][0] = x;
    result.m[1][1] = y;
    result.m[2][2] = z;
    
    return result;
}

/**
 * 回転行列を生成する
 */
mat4 mat4_rotate(const vec3 axis, const GLfloat rotate) {
    mat4 result;
    
    const GLfloat x = axis.x;
    const GLfloat y = axis.y;
    const GLfloat z = axis.z;
    
    const GLfloat c = cos( degree2radian( rotate ));
    const GLfloat s = sin( degree2radian( rotate ) );
    {
        result.m[0][0] = (x * x) * (1.0f - c) + c;
        result.m[0][1] = (x * y) * (1.0f - c) - z * s;
        result.m[0][2] = (x * z) * (1.0f - c) + y * s;
        result.m[0][3] = 0;
    }
    {
        result.m[1][0] = (y * x) * (1.0f - c) + z * s;
        result.m[1][1] = (y * y) * (1.0f - c) + c;
        result.m[1][2] = (y * z) * (1.0f - c) - x * s;
        result.m[1][3] = 0;
    }
    {
        result.m[2][0] = (z * x) * (1.0f - c) - y * s;
        result.m[2][1] = (z * y) * (1.0f - c) + x * s;
        result.m[2][2] = (z * z) * (1.0f - c) + c;
        result.m[2][3] = 0;
    }
    {
        result.m[3][0] = 0;
        result.m[3][1] = 0;
        result.m[3][2] = 0;
        result.m[3][3] = 1;
    }
    
    return result;
}

/**
 * 行列A×行列Bを行う。
 * 頂点に対し、行列B→行列Aの順番で適用することになる。
 */
mat4 mat4_multiply(const mat4 a, const mat4 b) {
    mat4 result;
    
    int i = 0;
    for (i = 0; i < 4; ++i) {
        result.m[i][0] = a.m[0][0] * b.m[i][0] + a.m[1][0] * b.m[i][1] + a.m[2][0] * b.m[i][2] + a.m[3][0] * b.m[i][3];
        result.m[i][1] = a.m[0][1] * b.m[i][0] + a.m[1][1] * b.m[i][1] + a.m[2][1] * b.m[i][2] + a.m[3][1] * b.m[i][3];
        result.m[i][2] = a.m[0][2] * b.m[i][0] + a.m[1][2] * b.m[i][1] + a.m[2][2] * b.m[i][2] + a.m[3][2] * b.m[i][3];
        result.m[i][3] = a.m[0][3] * b.m[i][0] + a.m[1][3] * b.m[i][1] + a.m[2][3] * b.m[i][2] + a.m[3][3] * b.m[i][3];
    }
    
    return result;
}

/**
 * 視点変換行列を生成する
 */
mat4 mat4_lookAt(const vec3 eye, const vec3 look, const vec3 up) {
    
    mat4 result;
    
    vec3 f = vec3_normalize(vec3_create(look.x - eye.x, look.y - eye.y, look.z - eye.z));
    vec3 u = vec3_normalize(up);
    vec3 s = vec3_normalize(vec3_cross(f, u));
    u = vec3_cross(s, f);
    
    result.m[0][0] = s.x;
    result.m[1][0] = s.y;
    result.m[2][0] = s.z;
    result.m[0][1] = u.x;
    result.m[1][1] = u.y;
    result.m[2][1] = u.z;
    result.m[0][2] = -f.x;
    result.m[1][2] = -f.y;
    result.m[2][2] = -f.z;
    result.m[3][0] = -vec3_dot(s, eye);
    result.m[3][1] = -vec3_dot(u, eye);
    result.m[3][2] = vec3_dot(f, eye);
    result.m[0][3] = 0;
    result.m[1][3] = 0;
    result.m[2][3] = 0;
    result.m[3][3] = 1;
    
    return result;
}

/**
 * 射影行列を生成する
 */
mat4 mat4_perspective(const GLfloat _near, const GLfloat _far, const GLfloat fovY_degree, const GLfloat aspect) {
    mat4 result;
    memset(result.m, 0x00, sizeof(mat4));
    
    const GLfloat f = (GLfloat) (1.0 / (tan( degree2radian(fovY_degree) ) / 2.0)); // 1/tan(x) == cot(x)
    
    result.m[0][0] = f / aspect;
    result.m[1][1] = f;
    result.m[2][2] = (_far + _near) / (_near - _far);
    result.m[2][3] = -1;
    result.m[3][2] = (2.0f * _far * _near) / (_near - _far);
    
    return result;
}

extern Texture* PkmImage_loadTexture(GLApplication *app, const char* file_name);
extern Texture* PvrtcImage_loadTexture(GLApplication *app, const char* file_name);
extern Texture* KtxImage_loadTexture(GLApplication *app, const char* file_name);
extern Texture* RawPixelImage_loadTexture(GLApplication *app, const char* file_name, const int pixel_fotmat);

/**
 * サイズがpotならTEXTURE_POTを返す。npotなら、TEXTURE_NPOTを返す。
 */
bool Texture_checkPowerOfTwo(const int size) {
    int bit = 1;
    int i = 0;
    const int BIT_SIZE = sizeof(int) * 8;
    
    if (!size) {
        return false;
    }
    
    for (i = 0; i < BIT_SIZE; ++i) {
        if (size == bit) {
            return true;
        }
        bit <<= 1;
    }
    
    return false;
}

/**
 * 引数sizeがpotならTEXTURE_POTを返す。npotなら、TEXTURE_NPOTを返す。
 */
bool Texture_checkPowerOfTwoWH(const int width, const int height) {
    return Texture_checkPowerOfTwo(width) && Texture_checkPowerOfTwo(height);
}


/**
 * Qualcomm Texture Toolを利用して作成したKTXファイルを読み込む
 */
KtxImage* KtxImage_load(GLApplication *app, const char* file_name) {
    RawData *rawData = RawData_loadFile(app, file_name);
    
    if (rawData == NULL) {
        return NULL;
    }
    // ファイルの識別子を確認する
    // サンプルでは詳細なエラー処理を行わない。
    {
        const uint8_t KTXFileIdentifier[12] = { 0xAB, 0x4B, 0x54, 0x58, 0x20, 0x31, 0x31, 0xBB, 0x0D, 0x0A, 0x1A, 0x0A };
        
        const uint8_t *header = (uint8_t*) RawData_getReadHeader(rawData);
        
        int index = 0;
        for (index = 0; index < 12; ++index) {
            if (header[index] != KTXFileIdentifier[index]) {
               // __logf("header error(%s)", file_name);
                RawData_freeFile(app, rawData);
                return NULL;
            }
        }
        
        RawData_offsetHeader(rawData, 12);
    }
    
    //
    {
        // エンディアンのチェックを行う
        // サンプルではビッグエンディアンのみに対応する
        int32_t check_endian = RawData_readLE32(rawData);
        
        // エンディアンチェックの値は0x04030201でり、それ以外であればエンディアン変換を行う必要がある
        // サンプルのため、エンディアンは固定であると想定する。
        if (check_endian == 0x01020304) {
            // エンディアンが想定と違うため、読み込まない
            //__logf("endian error(%s)", file_name);
            RawData_freeFile(app, rawData);
            return NULL;
        }
    }
    
    // 読み込みできるデータだった
    
    KtxImage *result = (KtxImage*) malloc(sizeof(KtxImage));
    result->raw = rawData;
    
    // ヘッダからデータを読み取る
    typedef struct KTXImageHeader {
        uint32_t glType;
        uint32_t glTypeSize;
        uint32_t glFormat;
        uint32_t glInternalFormat;
        uint32_t glBaseInternalFormat;
        uint32_t pixelWidth;
        uint32_t pixelHeight;
        uint32_t pixelDepth;
        uint32_t numberOfArrayElements;
        uint32_t numberOfFaces;
        uint32_t numberOfMipmapLevels;
        uint32_t bytesOfKeyValueData;
    } KTXImageHeader;
    
    // ヘッダとしてチェックする
    KTXImageHeader *pImageHeader = (KTXImageHeader*) RawData_getReadHeader(rawData);
    
    //__logf("image format(%x)", pImageHeader->glBaseInternalFormat);
   // __logf("image size(%d x %d) depth(%d)", pImageHeader->pixelWidth, pImageHeader->pixelHeight, pImageHeader->pixelDepth);
   // __logf("image mipmaps(%d)", pImageHeader->numberOfMipmapLevels);
   // __logf("image key value data (%d bytes)", pImageHeader->bytesOfKeyValueData);
    
    {
        result->format = pImageHeader->glBaseInternalFormat;
        result->width = pImageHeader->pixelWidth;
        result->height = pImageHeader->pixelHeight;
        result->mipmaps = pImageHeader->numberOfMipmapLevels;
        
        result->image_length_table = (int*) malloc(sizeof(int) * pImageHeader->numberOfMipmapLevels);
        result->image_table = (void**) malloc(sizeof(void*) * pImageHeader->numberOfMipmapLevels);
    }
    
    // ヘッダ位置を動かす
    RawData_offsetHeader(rawData, sizeof(KTXImageHeader));
    // Key Value Dataは無視する
    RawData_offsetHeader(rawData, pImageHeader->bytesOfKeyValueData);
    
    {
        // mipmapごとの画像データを読み込む
        int mip_level = 0;
        for (mip_level = 0; mip_level < pImageHeader->numberOfMipmapLevels; ++mip_level) {
            uint32_t image_size = RawData_readLE32(rawData);
            //__logf("level(%d) image size(%d bytes)", mip_level, image_size);
            result->image_length_table[mip_level] = image_size;
            result->image_table[mip_level] = RawData_getReadHeader(rawData);
            
            RawData_offsetHeader(rawData, image_size);
        }
    }
    
    return result;
}

void KtxImage_free(GLApplication *app, KtxImage *image) {
    if (image) {
        RawData_freeFile(app, image->raw);
        free(image->image_table);
        free(image->image_length_table);
        free(image);
    }
}



/**
 * 画像をテクスチャとして読み込む。
 * 読み込んだ画像はes20_freeTexture()で解放する
 */
Texture* KtxImage_loadTexture(GLApplication *app, const char* file_name) {
	return NULL;
#ifndef _WINDOWS
	KtxImage *ktx = KtxImage_load(app, file_name);
    
    // error images
    if (!ktx) {
        return NULL;
    }
    
    Texture *texture = (Texture*) malloc(sizeof(Texture));
    
    {
        // 元画像から必要情報をコピーする
        texture->width = ktx->width;
        texture->height = ktx->height;
    }
    
    {
        // 領域確保
        glGenTextures(1, &texture->id);
        assert(texture->id > 0);
        assert(glGetError() == GL_NO_ERROR);
    }
    
    glBindTexture(GL_TEXTURE_2D, texture->id);
    
    {
        // VRAMへピクセル情報をコピーする
        int width = ktx->width;
        int height = ktx->height;
        int miplevel = 0;
        
        for (miplevel = 0; miplevel < ktx->mipmaps; ++miplevel) {
            glCompressedTexImage2D(GL_TEXTURE_2D, miplevel, ktx->format, width, height, 0, ktx->image_length_table[miplevel], ktx->image_table[miplevel]);
            
            width /= 2;
            height /= 2;
            assert(glGetError() == GL_NO_ERROR);
        }
    }
    
    {
        // wrapの初期設定
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        assert(glGetError() == GL_NO_ERROR);
    }
    
    {
        // filterの初期設定
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        if (ktx->mipmaps > 1) {
            // mipmapを保持している場合
            // texturetoolへ -m のオプションを与えると生成できる
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
        } else {
            // mipmapを保持していない場合
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        }
        assert(glGetError() == GL_NO_ERROR);
    }
    
    // unbindする
    glBindTexture(GL_TEXTURE_2D, 0);
    assert(glGetError() == GL_NO_ERROR);
    
    // 元画像を解放
    KtxImage_free(app, ktx);
    return texture;
#endif
}


/**
 * 画像をテクスチャとして読み込む。
 * 読み込んだ画像はes20_freeTexture()で解放する
 */
Texture* Texture_load(GLApplication *app, const char* file_name, const int pixel_fotmat) {
    if (pixel_fotmat == TEXTURE_COMPRESS_ETC1) {
        return PkmImage_loadTexture(app, file_name);
    } else if (pixel_fotmat == TEXTURE_COMPRESS_PVRTC) {
        return PvrtcImage_loadTexture(app, file_name);
    } else if (pixel_fotmat == TEXTURE_COMPRESS_KTX) {
        return KtxImage_loadTexture(app, file_name);
    } else {
        return RawPixelImage_loadTexture(app, file_name, pixel_fotmat);
    }
}

/**
 * テクスチャの縦横が2のn乗であればtrueを返す
 */
bool Texture_isPowerOfTwo(Texture *texture) {
    return Texture_checkPowerOfTwo(texture->width) && Texture_checkPowerOfTwo(texture->height);
}

/**
 * テクスチャを解放する。
 */
void Texture_free(Texture *texture) {
    glDeleteTextures(1, &texture->id);
    free((void*) texture);
}


/**
 * 読み込んだファイルを解放する
 */
void RawData_freeFile(GLApplication *app, RawData *rawData) {
    free(rawData->head);
    free(rawData);
}

/**
 * データの長さを取得する
 */
int RawData_getLength(RawData *rawData) {
    return rawData->length;
}

/**
 * 現在の読み取りポインタ位置を取得する
 */
void* RawData_getReadHeader(RawData *rawData) {
    return (void*) rawData->read_head;
}

/**
 * 指定バイト数の情報を読み込む
 */
void RawData_readBytes(RawData* rawData, void* result, int bytes) {
    memcpy(result, rawData->read_head, bytes);
    rawData->read_head += bytes;
}

/**
 * 8bit整数を読み込む
 */
int8_t RawData_read8(RawData* rawData) {
    int8_t result = *rawData->read_head;
    ++rawData->read_head;
    return result;
}

/**
 * 読み取りポインタを指定バイト数移動させる
 */
void RawData_offsetHeader(RawData *rawData, int offsetBytes) {
    rawData->read_head += offsetBytes;
}

/**
 * 読み込みヘッダの位置を指定位置に移動させる
 */
void RawData_setHeaderPosition(RawData *rawData, int position) {
    assert(position >= 0);
    rawData->read_head = ((uint8_t*) rawData->head) + position;
}

/**
 * 読み込める残量を取得する
 */
int RawData_getAvailableBytes(RawData *rawData) {
    return rawData->length - ((int) (rawData->read_head) - (int) ((uint8_t*) rawData->head));
}

/**
 * Big Endian格納の16bit整数を読み込む
 */
int16_t RawData_readBE16(RawData* rawData) {
    int16_t w0 = (int16_t) (rawData->read_head)[0] & 0xFF;
    int16_t w1 = (int16_t) (rawData->read_head)[1] & 0xFF;
    
    rawData->read_head += 2;
    
    return (w0 << 8) | w1;
}

/**
 * Big Endian格納の32bit整数を読み込む
 */
int32_t RawData_readBE32(RawData* rawData) {
    
    int32_t w0 = (int16_t) (rawData->read_head)[0] & 0xFF;
    int32_t w1 = (int16_t) (rawData->read_head)[1] & 0xFF;
    int32_t w2 = (int16_t) (rawData->read_head)[2] & 0xFF;
    int32_t w3 = (int16_t) (rawData->read_head)[3] & 0xFF;
    
    rawData->read_head += 4;
    return (w0 << 24) | (w1 << 16) | (w2 << 8) | w3;
}

/**
 * Big Endian格納の16bit整数を読み込む
 */
int16_t RawData_readLE16(RawData* rawData) {
    
    int16_t w0 = (int16_t) (rawData->read_head)[0] & 0xFF;
    int16_t w1 = (int16_t) (rawData->read_head)[1] & 0xFF;
    
    rawData->read_head += 2;
    return (w1 << 8) | w0;
}

/**
 * Little Endian格納の32bit整数を読み込む
 */
int32_t RawData_readLE32(RawData* rawData) {
    int32_t w0 = (int16_t) (rawData->read_head)[0] & 0xFF;
    int32_t w1 = (int16_t) (rawData->read_head)[1] & 0xFF;
    int32_t w2 = (int16_t) (rawData->read_head)[2] & 0xFF;
    int32_t w3 = (int16_t) (rawData->read_head)[3] & 0xFF;
    
    rawData->read_head += 4;
    return (w3 << 24) | (w2 << 16) | (w1 << 8) | w0;
}


/**
 * PMDヘッダ
 * モデル名
 */
#define PMDFILE_HEADER_MODELNAME_LENGTH 20

/**
 * PMDヘッダ
 * コメント
 */
#define PMDFILE_HEADER_COMMENT_LENGTH 256

/**
 * マテリアル
 * テクスチャ名
 */
#define PMDFILE_MATERIAL_TEXTURENAME_LENGTH 20

/**
 * ボーン名
 */
#define PMDFILE_BONE_NAME_LENGTH 20

/**
 * ヘッダファイルを読み込む
 */
static bool PmdFile_loadHeader(PmdHeader *result, RawData *data) {
    
    // マジックナンバーをチェックする
    {
        GLbyte magic[3] = "";
        RawData_readBytes(data, magic, sizeof(magic));
        if (memcmp("Pmd", magic, sizeof(magic))) {
            //__logf("Magic Error %c%c%c", magic[0], magic[1], magic[2]);
            return false;
        }
    }
    
    // version check
    RawData_readBytes(data, &result->version, sizeof(GLfloat));
    if (result->version != 1.0f) {
        //__logf("File Version Error(%f)", result->version);
        return false;
    }
    
    // モデル名
    RawData_readBytes(data, result->name, PMDFILE_HEADER_MODELNAME_LENGTH);
    
    // コメント
    RawData_readBytes(data, result->comment, PMDFILE_HEADER_COMMENT_LENGTH);
    
    // SJISで文字列が格納されているため、UTF-8に変換をかける
    ES20_sjis2utf8(result->name, sizeof(result->name));
    ES20_sjis2utf8(result->comment, sizeof(result->comment));
    
    //__logf("Name(%s)", result->name);
  //  __logf("Comment(%s)", result->comment);
   //
    return true;
}

/**
 * 頂点情報を取得する
 */
static void PmdFile_loadVertices(PmdFile *result, RawData *data) {
    // 頂点数取得
    const GLuint numVertices = RawData_readLE32(data);
//    __logf("vertices[%d]", numVertices);
    
    // 頂点領域を確保
    result->vertices = malloc(sizeof(PmdVertex) * numVertices);
    result->vertices_num = numVertices;
    int vsize=0;
    // 頂点数分読み込む
    int i = 0;
    for (i = 0; i < numVertices; ++i) {
        PmdVertex *v = &result->vertices[i];
        
        // 頂点情報ロード
        RawData_readBytes(data, &v->position, sizeof(vec3)); // 位置
        RawData_readBytes(data, &v->normal, sizeof(vec3)); // 法線
        RawData_readBytes(data, &v->uv, sizeof(vec2)); // UV
        RawData_readBytes(data, &v->extra.bone_num, sizeof(GLushort) * 2); // ボーン設定
        RawData_readBytes(data, &v->extra.bone_weight, sizeof(GLbyte)); // ボーン重み
        RawData_readBytes(data, &v->extra.edge_flag, sizeof(GLbyte)); // 輪郭フラグ
        vsize +=sizeof(vec3)+sizeof(vec3)+sizeof(vec2)+(sizeof(GLushort) * 2)+sizeof(GLbyte)+sizeof(GLbyte);
        //        __logf("v[%d] p(%f, %f, %f), u(%f, %f)", i, v->position.x, v->position.y, v->position.z, v->uv.x, v->uv.y);
    }
    int32_t w0 = (int16_t) (data->read_head)[0] & 0xFF;
}

/**
 * インデックス情報を取得する
 */
static void PmdFile_loadIndices(PmdFile *result, RawData *data) {
    // インデックス数取得
    const GLuint numIndices = RawData_readLE32(data);
   // __logf("indices[%d]", numIndices);
    
    // インデックス領域を確保
    result->indices = malloc(sizeof(GLuint) * numIndices);
    result->indices_num = numIndices;
    
    // インデックス読み込み
    RawData_readBytes(data, result->indices, sizeof(GLushort) * numIndices);
    
    // 整合性チェック
    {
        int i = 0;
        for (i = 0; i < numIndices; ++i) {
            // インデックスの指す値は頂点数を下回らなければならない
            assert(result->indices[i] < result->vertices_num);
        }
    }
}

/**
 * 材質情報を取得する
 */
static void PmdFile_loadMaterial(PmdFile *result, RawData *data) {
    const GLuint numMaterials = RawData_readLE32(data);
    
    // マテリアル領域を確保
    result->materials = malloc(sizeof(PmdMaterial) * numMaterials);
    result->materials_num = numMaterials;
    
  //  __logf("materials[%d]", numMaterials);
    int i = 0;
    
    int sumVert = 0;
    for (i = 0; i < numMaterials; ++i) {
        PmdMaterial *m = &result->materials[i];
        
        RawData_readBytes(data, &m->diffuse, sizeof(vec4));
        RawData_readBytes(data, &m->extra.shininess, sizeof(GLfloat));
        RawData_readBytes(data, &m->extra.specular_color, sizeof(vec3));
        RawData_readBytes(data, &m->extra.ambient_color, sizeof(vec3));
        RawData_readBytes(data, &m->extra.toon_index, sizeof(GLubyte));
        RawData_readBytes(data, &m->extra.edge_flag, sizeof(GLubyte));
        RawData_readBytes(data, &m->indices_num, sizeof(GLuint));
        
        // テクスチャ名を読み込む
        {
            RawData_readBytes(data, m->diffuse_texture_name, PMDFILE_MATERIAL_TEXTURENAME_LENGTH);
            
            // エフェクトテクスチャが含まれていれば文字列を分離する
            // diffuse.png*effect.spaのように"*"で区切られている
            GLchar *p = strchr(m->diffuse_texture_name, '*');
            if (p) {
                // diffuseは"*"を無効化することで切る
                *p = '\0';
                // エフェクトは文字列コピーする
                strcpy(m->extra.effect_texture_name, p + 1);
            } else {
                // エフェクトが無いなら最初の文字でターミネートする
                m->extra.effect_texture_name[0] = '\0';
            }
            
            // SJISで文字列が格納されているため、UTF-8に変換をかける
            ES20_sjis2utf8(m->diffuse_texture_name, sizeof(m->diffuse_texture_name));
            ES20_sjis2utf8(m->extra.effect_texture_name, sizeof(m->extra.effect_texture_name));
            
        }
        
        //__logf("material[%d] tex(%s) effect(%s) vert(%d) face(%d) toon(%d)", i, m->diffuse_texture_name, m->extra.effect_texture_name, m->indices_num, m->indices_num / 3, (int )m->extra.toon_index);
        
        sumVert += m->indices_num;
    }
    
   // __logf("sum vert(%d) -> num(%d)", sumVert, result->indices_num);
    assert(sumVert == result->indices_num);
}

static void PmdFile_loadBone(PmdFile *result, RawData *data) {
    const GLuint numBones = RawData_readLE16(data);
    
    // ボーン領域を確保
    result->bones = malloc(sizeof(PmdBone) * numBones);
    result->bones_num = numBones;
  //  __logf("bones[%d]", numBones);
    
    // ボーンを読み込む
    int i;
    for (i = 0; i < numBones; ++i) {
        PmdBone *bone = &result->bones[i];
        
        RawData_readBytes(data, bone->name, PMDFILE_BONE_NAME_LENGTH);
        // SJISで文字列が格納されているため、UTF-8に変換をかける
        ES20_sjis2utf8(bone->name, sizeof(bone->name));
        
        RawData_readBytes(data, &bone->parent_bone_index, sizeof(GLshort));
        RawData_readBytes(data, &bone->extra.tail_pos_bone_index, sizeof(GLshort));
        RawData_readBytes(data, &bone->extra.type, sizeof(GLbyte));
        RawData_readBytes(data, &bone->extra.ik_parent_bone_index, sizeof(GLshort));
        RawData_readBytes(data, &bone->position, sizeof(vec3));
        
        //        __logf("bone[%d] name(%s)", i, bone->name);
    }
}

/**
 * PMDファイルを生成する
 */
PmdFile* PmdFile_create(RawData *data) {
    PmdFile *result = calloc(1, sizeof(PmdFile));
    
    // ファイルヘッダを読み込む
    if (!PmdFile_loadHeader(&result->header, data)) {
        // 読み込み失敗
        PmdFile_free(result);
        return NULL;
    }
    
    // 頂点データを読み込み
    PmdFile_loadVertices(result, data);
    // インデックスデータを読み込み
    PmdFile_loadIndices(result, data);
    // 材質情報を読み込み
    PmdFile_loadMaterial(result, data);
    // ボーン情報を読み込み
    PmdFile_loadBone(result, data);
    
    return result;
}
/**
* @brief ファイルサイズ取得. Windows / Linux 両対応, のはず.
* @param[in] in_fname ファイル名の文字列
* @retval ファイルサイズ
*/

static long get_read_file_size(const char *in_fname)
{
	FILE *fp;
	long file_size;
	struct stat stbuf;
	int fd;

	fd = open(in_fname, "rb");
	if (fd == -1)
		printf("cant open file : %s.\n", in_fname);

	fp = fdopen(fd, "rb");
	if (fp == NULL)
		printf("cant open file : %s.\n", in_fname);

	if (fstat(fd, &stbuf) == -1)
		printf("cant get file state : %s.\n", in_fname);

	file_size = stbuf.st_size;

	if (fclose(fp) != 0)
		printf("cant close file : %s.\n", in_fname);

	return file_size;
}
/**
* assets配下からファイルを読み込む
*/
RawData* RawData_loadFile(GLApplication *app, const char* file_name) {

	FILE *fp;
	int  i, size;
	size = get_read_file_size(file_name);
	unsigned char *data = NULL;
	data = (char)malloc(size);

	if (fp == NULL) {
		printf("%sファイルが開けません¥n", file_name);
		return -1;
	}
	if (fp = fopen(file_name, "rb") == NULL){
	//エラー処理
	}
	else {
		int i = 0;

		while (!feof(fp)) {
			fread(&data[i++], sizeof(char), 1, fp);
		}
		fclose(fp);
	}

	RawData *raw = (RawData*)malloc(sizeof(RawData));
	raw->length = size;
	raw->head = malloc(raw->length);
	raw->read_head = (uint8_t*)raw->head;
	memcpy(raw->head, 1, size);
	return data;

}
/**
 * PMDファイルをロードする
 */
PmdFile* PmdFile_load(GLApplication *app, const char* file_name) {
    RawData *data = RawData_loadFile(app, file_name);
    if (!data) {
        return NULL;
    }
    
    PmdFile* result = PmdFile_create(data);
    
    RawData_freeFile(app, data);
    
    return result;
}

/**
 * PMDファイルを解放する
 */
void PmdFile_free(PmdFile *pmd) {
    if (!pmd) {
        return;
    }
    
    free(pmd->vertices);
    free(pmd->indices);
    free(pmd->materials);
    free(pmd->bones);
    free(pmd);
}

/**
 * 最小最大地点を求める
 */
void PmdFile_calcAABB(PmdFile *pmd, vec3 *minPoint, vec3 *maxPoint) {
    *minPoint = vec3_create(10000, 10000, 10000);
    *maxPoint = vec3_create(-10000, -10000, -10000);
    
    if (!pmd) {
        // PMDが無いため、適当な位置を指定する
        *minPoint = vec3_create(-10, -10, -10);
        *maxPoint = vec3_create(10, 10, 10);
        return;
    }
    
    int i = 0;
    for (i = 0; i < pmd->vertices_num; ++i) {
        minPoint->x = (GLfloat) fmin(minPoint->x, pmd->vertices[i].position.x);
        minPoint->y = (GLfloat) fmin(minPoint->y, pmd->vertices[i].position.y);
        minPoint->z = (GLfloat) fmin(minPoint->z, pmd->vertices[i].position.z);
        
        maxPoint->x = (GLfloat) fmax(maxPoint->x, pmd->vertices[i].position.x);
        maxPoint->y = (GLfloat) fmax(maxPoint->y, pmd->vertices[i].position.y);
        maxPoint->z = (GLfloat) fmax(maxPoint->z, pmd->vertices[i].position.z);
    }
}

/**
 * PMDファイル内のテクスチャを列挙する
 */
PmdTextureList* PmdFile_createTextureList(GLApplication *app, PmdFile *pmd) {
    PmdTextureList *result = calloc(1, sizeof(PmdTextureList));
    
    // 読み込み時の一時ファイル名
    GLchar load_name[32] = {"" };
    
    // マテリアル数だけチェックする
    int i;
    for (i = 0; i < pmd->materials_num; ++i) {
        
        PmdMaterial *material = &pmd->materials[i];
        
        // テクスチャ名が設定されている
        if (strlen(material->diffuse_texture_name)) {
            // diffuseチェック
            Texture *t = PmdFile_getTexture(result, material->diffuse_texture_name);
            
            if (!t) {
                // テクスチャがまだ読み込まれていない
                sprintf(load_name, "%s.png", material->diffuse_texture_name);
                // テクスチャを読み込む
                t = Texture_load(app, load_name, TEXTURE_RAW_RGBA8);
                
                // テクスチャの読み込みに成功したら末尾へ登録する
                if (t) {
                    // メモリを確保する
                    if (result->textures) {
                        result->textures = realloc(result->textures, sizeof(Texture*) * result->textures_num + 1);
                        result->texture_names = realloc(result->texture_names, sizeof(GLchar*) * result->textures_num + 1);
                    } else {
                        result->textures = malloc(sizeof(Texture*));
                        result->texture_names = malloc(sizeof(GLchar*));
                    }
                    
                    const int index = result->textures_num;
                    result->textures_num++;
                    
                    result->textures[index] = t;
                    result->texture_names[index] = malloc(strlen(material->diffuse_texture_name) + 1);
                    // ファイル名をコピーする
                    strcpy(result->texture_names[index], material->diffuse_texture_name);
                }else {
                  //  __logf("Texture load fail(%s)", material->diffuse_texture_name);
                }
            }
        }
    }
    
    return result;
}

/**
 * 指定した名前のテクスチャを取得する
 */
Texture* PmdFile_getTexture(PmdTextureList *texList, const GLchar *name) {
    if (!name[0] || !texList) {
        // 空文字の場合はNULLを返す
        return NULL;
    }
    
    int i = 0;
    for (i = 0; i < texList->textures_num; ++i) {
        // 名前が一致したらそれを返す
        if (strcmp(texList->texture_names[i], name) == 0) {
            return texList->textures[i];
        }
    }
    
    return NULL;
}

/**
 * 管理しているテクスチャを解放する
 */
void PmdFile_freeTextureList(PmdTextureList* texList) {
    if (!texList) {
        return;
    }
    
    // 各々のテクスチャを解放する
    {
        int i = 0;
        for (i = 0; i < texList->textures_num; ++i) {
            free(texList->texture_names[i]);
            Texture_free(texList->textures[i]);
        }
        
        // 配列をクリアする
        free(texList->texture_names);
        free(texList->textures);
    }
    free(texList);
}

