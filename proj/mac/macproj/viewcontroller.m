
#import "viewcontroller.h"


@interface OpenGLView : UIView
@end

@implementation OpenGLView
+ (Class)layerClass {
    return [CAEAGLLayer class];
}
@end


@implementation viewcontroller {
    EAGLContext* _ctx;
    OpenGLView* _outputView;
    CADisplayLink* _displayLink;
}



- (void)viewWillLayoutSubviews {
    [EAGLContext setCurrentContext:_ctx];
}

- (void)viewDidLoad {
    self.title = @"OpenGLES 2.0";
    
    _ctx = [[EAGLContext alloc] initWithAPI:kEAGLRenderingAPIOpenGLES2];
    [EAGLContext setCurrentContext:_ctx];
    
    
    self.view.backgroundColor = [UIColor blackColor];
    [self.view addSubview:_outputView];
}

- (void)viewWillAppear:(BOOL)animated {
    [_displayLink addToRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];
}

- (void)viewWillDisappear:(BOOL)animated {
    [_displayLink removeFromRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];
}

- (void)render {
    //  [EAGLContext setCurrentContext:_ctx];
}
@end

