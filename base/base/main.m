//
//  main.m
//  base
//
//  Created by kaiser on 2018/01/23.
//  Copyright © 2018年 kaiser. All rights reserved.
//
#define CSTR2NSSTRING(c_str)       [[NSString alloc] initWithUTF8String:c_str]
#import <UIKit/UIKit.h>
#import "AppDelegate.h"
#include "data.h"
#include "common.h"
int main(int argc, char * argv[]) {
    @autoreleasepool {
        return UIApplicationMain(argc, argv, nil, NSStringFromClass([AppDelegate class]));
    }
}
/**
 * assets配下からファイルを読み込む
 */
RawData* RawData_loadFile_(GLApplication *app, const char* file_name) {
    NSString *fileName = [[NSString alloc ] initWithFormat:@"/%@", CSTR2NSSTRING(file_name)];
    NSBundle *bundle  = [NSBundle mainBundle];
    fileName = [bundle pathForResource:fileName ofType:@""];
    NSData *data = [NSData dataWithContentsOfMappedFile:fileName];
    
    if(!data) {
        __logf("file not found(%s)", file_name);
        return NULL;
    }
    
    RawData *raw = (RawData*)malloc(sizeof(RawData));
    raw->length = (int)data.length;
    raw->head = malloc(raw->length);
    raw->read_head = (uint8_t*)raw->head;
    
    memcpy(raw->head, data.bytes, raw->length);
    return raw;
}


/**
 * 画像を読み込む
 */
RawPixelImage*    RawPixelImage_load_(GLApplication *app, const char* file_name, const int pixel_format) {
    int pixelsize = 0;
    
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
    
    assert(pixelsize > 0);
    
    NSString *fileName = [[NSString alloc ] initWithFormat:@"%@", CSTR2NSSTRING(file_name)];
    UIImage *image = [UIImage imageNamed:fileName];
    if(!image) {
        __logf("image(%@) is nil...", fileName);
        return NULL;
    }
    
    CGImageRef raw_image = image.CGImage;
    
    RawPixelImage *result = (RawPixelImage*)malloc(sizeof(RawPixelImage));
    const int image_width = (int)CGImageGetWidth(raw_image);
    const int image_height = (int)CGImageGetHeight(raw_image);
    const int pixel_bytes = (int)CGImageGetBitsPerPixel(raw_image) / 8;
    // 戻り値にするRGBA情報
    UInt8 *rgba = (unsigned char*)malloc(image_width * image_height * 4);
    
    // resultを書き込む
    {
        result->format = pixel_format;
        result->pixel_data = (void*)rgba;
        result->width = image_width;
        result->height = image_height;
    }
    
    // 画像幅と高さを出力と、NPOTチェック
   // __logf("image(%d bit) size(%d x %d) = %s", pixel_bytes * 8, image_width, image_height, Texture_checkPowerOfTwoWH(image_width, image_height) ? "power of two" : "non power of two");
    
    assert(pixel_bytes == 3 || pixel_bytes == 4);
    
    // ref情報を取り出す
    CGDataProviderRef provider = CGImageGetDataProvider(raw_image);
    CFDataRef data = CGDataProviderCopyData(provider);
    
    // RGBAピクセル
    UInt8 *raw_pixels = (UInt8*)CFDataGetBytePtr(data);
    
    switch(pixel_bytes) {
        case 3:
            RawPixelImage_convertColorRGB(raw_pixels, pixel_format, result->pixel_data, result->width * result->height);
            break;
        case 4:
            RawPixelImage_convertColorRGBA(raw_pixels, pixel_format, result->pixel_data, result->width * result->height);
            break;
    }
    
    
    // tempを解放
    CFRelease(data);
    image = nil;
    return result;
}

/**
 * SJIS文字列をUTF8に変換する
 */
void ES20_sjis2utf8(GLchar *str, const int array_length) {
    const int str_len = (int)strlen(str);
    if (!str_len) {
        return;
    }
    
    NSString *converted = [[NSString alloc] initWithBytes:str length:str_len encoding:NSShiftJISStringEncoding];
    const char* utf8 = [converted UTF8String];
    int converted_length = (int)strlen(utf8);
    if(converted_length >= array_length) {
        converted_length = array_length - 1;
    }
    memcpy(str, utf8, converted_length);
    str[converted_length] = '\0';
}
