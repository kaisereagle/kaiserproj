//
//  eaglview.h
//  ac.kaiser.saba
//
//  Created by sakya-suzuki on 2018/05/02.
//  Copyright © 2018年 sakya-suzuki. All rights reserved.
//

#import <UIKit/UIKit.h>

#import <OpenGLES/ES1/gl.h>
#import <OpenGLES/ES1/glext.h>
#import <OpenGLES/ES2/gl.h>
#import <OpenGLES/ES2/glext.h>
@interface eaglview : UIView
{
@private
    //レンダリングコンテキスト
    EAGLContext *context;
    
    // フレームバッファの幅
    GLint framebufferWidth;
    //フレームバッファの高さ
    GLint framebufferHeight;
    
    // defaultFramebuffer    フレームバッファ
    //colorRenderbuffer    色レンダバッファ
    GLuint defaultFramebuffer, colorRenderbuffer;
}
@end
