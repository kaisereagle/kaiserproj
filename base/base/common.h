//
//  common.h
//  base
//
//  Created by kaiser on 2018/01/23.
//  Copyright © 2018年 kaiser. All rights reserved.
//

#ifndef common_h
#define common_h

    #ifdef __APPLE__
		#ifdef __OBJC__
			#define CSTR2NSSTRING(c_str)       [[NSString alloc] initWithUTF8String:c_str]

			// Objective-C source
			#define     __log(message)        NSLog(@message)
			#define     __logf(fmt, ...)      NSLog(@fmt, __VA_ARGS__)
			#else
			// C Source
			#define     __log(message)        printf("%s\n", message)
			#define     __logf(fmt, ...)      printf(fmt "\n", __VA_ARGS__)

		#endif
	#ifdef ANDROID
			#include    <android/log.h>
			#define __LOG_TAG   "GLES20"
			#define __log(msg)       __android_log_print(ANDROID_LOG_DEBUG, __LOG_TAG, "%s", msg)
			#define __logf(...)      __android_log_print(ANDROID_LOG_DEBUG, __LOG_TAG, __VA_ARGS__)

		#endif
	#else
		#ifdef _WIN32
		#ifdef _CONSOLE
			#define __log(msg) printf("%s",msg)
			#define     __logf(fmt, ...)      printf(fmt "\n", __VA_ARGS__)
		#else
			#include <Windows.h>
			#define __log(msg) OutputDebugString(msg)
			#ifdef _DEBUG
			#   define MyOutputDebugString( str, ... ) \
				  { \
					TCHAR c[256]; \
					_stprintf( c, str, __VA_ARGS__ ); \
					OutputDebugString( c ); \
				  }
			#else
			#    define MyOutputDebugString( str, ... ) // 空実装
			#define __logf(fmt, ...)  MyOutputDebugString(fmt, ...)    

			#endif
		#endif


		#endif
		#ifdef _WIN64
		#define __log(msg) OutputDebugString(msg)
		#ifdef _DEBUG
		#   define MyOutputDebugString( str, ... ) \
								  				  						  { \
							TCHAR c[256]; \
							_stprintf( c, str, __VA_ARGS__ ); \
							OutputDebugString( c ); \
								  				  						  }
		#else
		#    define MyOutputDebugString( str, ... ) // 空実装
		#define __logf(fmt, ...)  MyOutputDebugString(fmt, ...)    

		#endif

		#endif
	#endif

#endif /* common_h */
