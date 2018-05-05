//
//  ES1Renderer.m
//  MMD_View
//
// 
//

#import "ES1Renderer.h"
#include "GlTrans.h"
#include "GlutTrans.h"

#include "PMDModel.h"
#include "VMDMotion.h"
#include <unistd.h> // chdir()
#include <sys/param.h> // MAXPATHLEN
#include "dirent.h"


// 頂点のデータ構造を定義する構造体
typedef struct _Vertex {
    GLfloat x, y, z;
    GLfloat nx, ny, nz;
    GLfloat u, v;
} Vertex;


// 深度バッファのハンドルを格納するグローバル関数
static GLuint depthBuffer = 0;

static cPMDModel g_clPMDModel;
static cVMDMotion g_clVMDMotion;

static void createDepthBuffer(GLuint screenWidth, GLuint screenHeight) {
    if(depthBuffer) {
        glDeleteRenderbuffersOES(1, &depthBuffer);
        depthBuffer = 0;
    }
    glGenRenderbuffersOES(1, &depthBuffer);
    glBindRenderbufferOES(GL_RENDERBUFFER_OES, depthBuffer);
    glRenderbufferStorageOES(GL_RENDERBUFFER_OES,
                             GL_DEPTH_COMPONENT16_OES,
                             screenWidth, screenHeight);
    glFramebufferRenderbufferOES(GL_FRAMEBUFFER_OES,
                                 GL_DEPTH_ATTACHMENT_OES,
                                 GL_RENDERBUFFER_OES,
                                 depthBuffer);
    glEnable(GL_DEPTH_TEST);
}





static void matrixPlanarProjection( float matOut[4][4], const Vector4 *pvec4Plane, const Vector3 *pvec3LightPos )
{
	float	fDot =	pvec4Plane->x * pvec3LightPos->x +
	pvec4Plane->y * pvec3LightPos->y +
	pvec4Plane->z * pvec3LightPos->z + 
	pvec4Plane->w;
	
	matOut[0][0] = fDot - pvec3LightPos->x * pvec4Plane->x;
	matOut[1][0] =      - pvec3LightPos->x * pvec4Plane->y;
	matOut[2][0] =      - pvec3LightPos->x * pvec4Plane->z;
	matOut[3][0] =      - pvec3LightPos->x * pvec4Plane->w;
	
	matOut[0][1] =      - pvec3LightPos->y * pvec4Plane->x;
	matOut[1][1] = fDot - pvec3LightPos->y * pvec4Plane->y;
	matOut[2][1] =      - pvec3LightPos->y * pvec4Plane->z;
	matOut[3][1] =      - pvec3LightPos->y * pvec4Plane->w;
	
	matOut[0][2] =      - pvec3LightPos->z * pvec4Plane->x;
	matOut[1][2] =      - pvec3LightPos->z * pvec4Plane->y;
	matOut[2][2] = fDot - pvec3LightPos->z * pvec4Plane->z;
	matOut[3][2] =      - pvec3LightPos->z * pvec4Plane->w;
	
	matOut[0][3] =      - pvec4Plane->x;
	matOut[1][3] =      - pvec4Plane->y;
	matOut[2][3] =      - pvec4Plane->z;
	matOut[3][3] = fDot - pvec4Plane->w;
}



@implementation ES1Renderer
@synthesize player;
@synthesize anglex;
@synthesize angley;
@synthesize distance;


// Create an ES 1.1 context
- (id) init
{
	if (self = [super init])
	{
		context = [[EAGLContext alloc] initWithAPI:kEAGLRenderingAPIOpenGLES1];
        
        if (!context || ![EAGLContext setCurrentContext:context])
		{
            [self release];
            return nil;
        }
		
		// Create default framebuffer object. The backing will be allocated for the current layer in -resizeFromLayer
		glGenFramebuffersOES(1, &defaultFramebuffer);
		glGenRenderbuffersOES(1, &colorRenderbuffer);
		glBindFramebufferOES(GL_FRAMEBUFFER_OES, defaultFramebuffer);
		glBindRenderbufferOES(GL_RENDERBUFFER_OES, colorRenderbuffer);
		glFramebufferRenderbufferOES(GL_FRAMEBUFFER_OES, GL_COLOR_ATTACHMENT0_OES, GL_RENDERBUFFER_OES, colorRenderbuffer);
		//loadTexture();
		//
		// prepare audio
		NSString *path2;	
		NSError *error;
		path2 = [[NSBundle mainBundle] pathForResource:@"VOCALOID" ofType:@"mp3"];
		
		//self.player = [[AVAudioPlayer alloc] initWithContentsOfURL:[NSURL fileURLWithPath:path2] error:&error];
		//if (!self.player)
	//	{
	//		NSLog(@"Error: %@", [error localizedDescription]);
	//		return nil;
	//	}
		
	//	[self.player prepareToPlay];
		//
		// Override point for customization after app launch    
		
		// Change working directory to resources directory inside app bundle,
		// so that camera parameters, models etc. can be loaded using relative paths.
		CFURLRef pathCFURLRef;
		char *path;
		pathCFURLRef = CFBundleCopyResourcesDirectoryURL(CFBundleGetMainBundle()); // Get relative path to resources directory.
		if (pathCFURLRef) {
			path = (char *)calloc(MAXPATHLEN, sizeof(char)); //getcwd(path, MAXPATHLEN);
			if (path) {
				if (CFURLGetFileSystemRepresentation(pathCFURLRef, true, (UInt8*)path, MAXPATHLEN)) { // true in param 2 resolves against base.
					if (chdir(path) != 0) {
						fprintf(stderr, "Error: Unable to change working directory to %s.\n", path);
						perror(NULL);
					} else {
						fprintf(stderr,"Debug: Working directory is %s\n", path);
					}
				}
				free(path);
			}
			CFRelease(pathCFURLRef);
		}
		
		// set
		
		g_clPMDModel.load("miku1.pmd");
		g_clVMDMotion.load("miku.vmd");
		
		g_clPMDModel.setMotion( &g_clVMDMotion, false);
		glEnable(GL_CULL_FACE);
		glCullFace(GL_FRONT);
		g_clPMDModel.updateMotion(0.0f);
		priviousTime = 0.0f;
		g_clPMDModel.updateSkinning();
		
		// 陰付け処理
		Vector4		vec4Plane = { 0.0f, 1.0f, 0.0f, 0.0f };		// { a, b, c, d } ax + by + cz + d = 0
		Vector3		vec4LightPos = { 10.0f, 70.0f, -20.0f };	// 光源
		
		matrixPlanarProjection( g_matPlanarProjection, &vec4Plane, &vec4LightPos );
		
	}
	
	return self;
}


- (void)drawScene:(GLfloat)screenWidth with:(GLfloat) screenHeight anglex:(float) anglex angley:(float)angley distance:(float) distance {
    // 画面をクリア
    glViewport(0, 0, screenWidth, screenHeight);
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//glEnable(GL_CULL_FACE);
	//glCullFace(GL_FRONT);
	
	// ライトとマテリアルの設定
    //const GLfloat lightPos[]     = { 1.0f, 1.0f, 1.0f, 0.0f };
    //const GLfloat lightColor[]   = { 1.0f, 1.0f, 1.0f, 1.0f };
    //const GLfloat lightAmbient[] = { 0.0f, 0.0f, 0.0f, 1.0f };
    //const GLfloat diffuse[]      = { 0.7f, 0.7f, 0.7f, 1.0f };
    //const GLfloat ambient[]      = { 0.3f, 0.3f, 0.3f, 1.0f };
	
    //glMatrixMode(GL_MODELVIEW);
    //glLoadIdentity();
    //glEnable(GL_LIGHTING);
    //glEnable(GL_LIGHT0);
    //glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
    //glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor);
    //glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
    //glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse);
    //glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambient);
		
    // シーンの射影行列を設定
    glMatrixMode(GL_PROJECTION);
    const GLfloat near  = 0.2f, far = 1000.0f;
    const GLfloat aspect = screenWidth / screenHeight;
    const GLfloat width = near * tanf(M_PI * 60.0f / 180.0f / 2.0f);
    glLoadIdentity();
    glFrustumf(-width, width, -width / aspect, width / aspect, near, far);
	
    // 球体の変換行列を設定
    glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glPushMatrix();
	
	glTranslatef(0.0, -10.0, -self.distance);
	
	glScalef(1.0, 1.0, -1.0);
	glRotatef(self.anglex, 0.0, 1.0, 0.0);
	glRotatef(self.angley, 1.0f, 0.0f, 0.0f);

	glColor4f(1.0,1.0,1.0,1.0);
	float temp = self.player.currentTime;
	g_clPMDModel.updateMotion((temp-priviousTime)*30.0);
	priviousTime = temp;
	//GlTrans te;
	//te.glBegin(NGL_QUADS);
	//te.glNormal3f(0.0, 0.0, 1.0);
	//te.glTexCoord2f(0.0, 0.0); 
	//te.glVertex3f(-5.0,-5.0, 0.0);
	//te.glTexCoord2f(0.0, 1.0); 
	//te.glVertex3f(-5.0, 5.0, 0.0);
	//te.glTexCoord2f(1.0, 1.0); 
	//te.glVertex3f( 5.0, 5.0, 0.0);
	//te.glTexCoord2f(1.0, 0.0); 
	//te.glVertex3f( 5.0,-5.0, 0.0);
	//te.glEnd();
	g_clPMDModel.updateSkinning();
	g_clPMDModel.render();
	//
	glDisable( GL_CULL_FACE );
	glDisable( GL_TEXTURE_2D );
	glDisable( GL_LIGHTING );
	
	glEnable( GL_STENCIL_TEST );
	glStencilFunc( GL_ALWAYS, 1, ~0 );
	glStencilOp( GL_REPLACE, GL_KEEP, GL_REPLACE );

	glMultMatrixf( (GLfloat*)g_matPlanarProjection );
	//
	glColor4f(0.2,0.2,0.2,0.4);
	g_clPMDModel.renderForShadow();

	
	glPopMatrix();
}

- (void) render
{
    [EAGLContext setCurrentContext:context];
    glBindFramebufferOES(GL_FRAMEBUFFER_OES, defaultFramebuffer);
	//float temp = self.player.currentTime;
    [self drawScene:backingWidth with: backingHeight anglex: anglex angley: angley distance:distance];
	//priviousTime = temp;
    glBindRenderbufferOES(GL_RENDERBUFFER_OES, colorRenderbuffer);
    [context presentRenderbuffer:GL_RENDERBUFFER_OES];
}

- (BOOL) resizeFromLayer:(CAEAGLLayer *)layer
{
    // Allocate color buffer backing based on the current layer size
    glBindRenderbufferOES(GL_RENDERBUFFER_OES, colorRenderbuffer);
    [context renderbufferStorage:GL_RENDERBUFFER_OES fromDrawable:layer];
    glGetRenderbufferParameterivOES(GL_RENDERBUFFER_OES, GL_RENDERBUFFER_WIDTH_OES, &backingWidth);
    glGetRenderbufferParameterivOES(GL_RENDERBUFFER_OES, GL_RENDERBUFFER_HEIGHT_OES, &backingHeight);

    // 深度バッファを(再)作成する
    createDepthBuffer(backingWidth, backingHeight);

    if (glCheckFramebufferStatusOES(GL_FRAMEBUFFER_OES) != GL_FRAMEBUFFER_COMPLETE_OES)
    {
        NSLog(@"Failed to make complete framebuffer object %x", glCheckFramebufferStatusOES(GL_FRAMEBUFFER_OES));
        return NO;
    }

    return YES;
}

//
//
//
- (void) dealloc
{
	// Tear down GL
	if (defaultFramebuffer)
	{
		glDeleteFramebuffersOES(1, &defaultFramebuffer);
		defaultFramebuffer = 0;
	}

	if (colorRenderbuffer)
	{
		glDeleteRenderbuffersOES(1, &colorRenderbuffer);
		colorRenderbuffer = 0;
	}
	
	// Tear down context
	if ([EAGLContext currentContext] == context)
        [EAGLContext setCurrentContext:nil];
	
	[context release];
	context = nil;
	
	[super dealloc];
}

//
// movie stop
//
- (void) stopMovie{
	[self.player stop];
}


//
// movie start off
//
- (void) startMovie
{
	g_clPMDModel.load("DIVA.pmd");
	
	g_clPMDModel.setMotion( &g_clVMDMotion, false);
	g_clPMDModel.updateMotion(0.0f);
	g_clPMDModel.updateSkinning();
	
	self.player.currentTime =0.0;
	[self.player play];
	priviousTime = 0.0f;
	
}

@end
