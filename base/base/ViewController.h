//
//  ViewController.h
//  base
//
//  Created by kaiser on 2018/01/23.
//  Copyright © 2018年 kaiser. All rights reserved.
//

#import <UIKit/UIKit.h>
#import <GLKit/GLKit.h>
@interface ViewController : GLKViewController 
{
    // レンダリング用のContext
    EAGLContext *renderingContext;
    
    // 深度＆ステンシルバッファ
    GLuint   buffer_depth_stencil;
    // カラーバッファ
    GLuint   buffer_color;
    // フレームバッファ
    GLuint   buffer_frame;
    
    // レンダリングサイズ（幅）
    GLint   rendering_width;
    // レンダリングサイズ（高さ）
    GLint   rendering_height;
    
    // 初期化が完了している場合YES
    BOOL    initialized;
}
@property (readonly) GLint rendering_width;
@property (readonly) GLint rendering_height;
@property (strong, nonatomic) GLKBaseEffect *baseEffect;
- (EAGLContext*) renderingContext;
// OpenGL ES 2.0の専有を開始する
- (void) bind;

// OpenGL ES 2.0の専有を終了する
- (void) unbind;

// レンダリング内容を画面へ反映する
- (void) postFrontBuffer;
@end

