//
//  ES1Renderer.h
//  MMD_View
//
//

#import "ESRenderer.h"

#import <AVFoundation/AVFoundation.h>
#import <OpenGLES/ES1/gl.h>
#import <OpenGLES/ES1/glext.h>


@interface ES1Renderer : NSObject <ESRenderer>
{
@private
	EAGLContext *context;
	AVAudioPlayer *player;
	float priviousTime;
	float anglex;
	float angley;
	float distance;
	
	
	// The pixel dimensions of the CAEAGLLayer
	GLint backingWidth;
	GLint backingHeight;
	
	// The OpenGL names for the framebuffer and renderbuffer used to render to this view
	GLuint defaultFramebuffer, colorRenderbuffer;
	float g_matPlanarProjection[4][4];
}

- (void) render;
- (BOOL) resizeFromLayer:(CAEAGLLayer *)layer;
- (void)drawScene:(GLfloat)screenWidth with:(GLfloat) screenHeight anglex:(float) anglex angley:(float)angley distance:(float) distance;
- (void) startMovie;
- (void) stopMovie;
@property (retain) AVAudioPlayer *player;
@property float anglex;
@property float angley;
@property float distance;

@end
