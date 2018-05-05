//
//  EAGLView.m
//  MMD_View
//
//

#import "EAGLView.h"

#import "ES1Renderer.h"

@implementation EAGLView

@synthesize touchCount;
@synthesize animating;
@dynamic animationFrameInterval;

// You must implement this method
+ (Class) layerClass
{
    return [CAEAGLLayer class];
}

//The GL view is stored in the nib file. When it's unarchived it's sent -initWithCoder:
- (id) initWithCoder:(NSCoder*)coder
{    
    if ((self = [super initWithCoder:coder]))
	{
		
		self.userInteractionEnabled = YES;
		self.multipleTouchEnabled = YES;
        // Get the layer
        CAEAGLLayer *eaglLayer = (CAEAGLLayer *)self.layer;
        
        eaglLayer.opaque = NO;
        eaglLayer.drawableProperties = [NSDictionary dictionaryWithObjectsAndKeys:
                                        [NSNumber numberWithBool:FALSE], kEAGLDrawablePropertyRetainedBacking, kEAGLColorFormatRGBA8, kEAGLDrawablePropertyColorFormat, nil];
		
		//renderer = [[ES2Renderer alloc] init];
		
		if (!renderer)
		{
			renderer = [[ES1Renderer alloc] init];
			renderer.anglex = 0.0;
			renderer.angley = 0.0;
			renderer.distance = 30.0;
			
			if (!renderer)
			{
				[self release];
				return nil;
			}
		}
        
		animating = FALSE;
		displayLinkSupported = FALSE;
		animationFrameInterval = 1;
		displayLink = nil;
		animationTimer = nil;
		
		// A system version of 3.1 or greater is required to use CADisplayLink. The NSTimer
		// class is used as fallback when it isn't available.
		NSString *reqSysVer = @"3.1";
		NSString *currSysVer = [[UIDevice currentDevice] systemVersion];
		if ([currSysVer compare:reqSysVer options:NSNumericSearch] != NSOrderedAscending)
			displayLinkSupported = TRUE;
    }
	
    return self;
}

- (void) touchesBegan:(NSSet*) touches withEvent:( UIEvent*) event
{ 
	int i;
	self.touchCount = 0;
}


- (void) touchesMoved:(NSSet *) touches withEvent: (UIEvent *) event
{
	int i;
	if( self.touchCount == 0){
		self.touchCount = [[touches allObjects] count];
		for (i = 0; i < touchCount; i++) {
			CGPoint pt = [[[touches allObjects] objectAtIndex:i] locationInView:self];
			startPoints[i] = pt;
		}		
	} else if([[touches allObjects] count] == 2 && touchCount == 1){
		self.touchCount = [[touches allObjects] count];
		for (i = 0; i < touchCount; i++) {
			CGPoint pt = [[[touches allObjects] objectAtIndex:i] locationInView:self];
			startPoints[i] = pt;
		}
 	}else {
		if(!(self.touchCount == 2 && [[touches allObjects] count] == 1)){
			for (i = 0; i < [[touches allObjects] count]; i++) {
				CGPoint pt = [[[touches allObjects] objectAtIndex:i] locationInView:self];
				endPoints[i] = pt;
			}
			
			if(touchCount==1){
				CGFloat diffx = endPoints[0].x- startPoints[0].x;
				CGFloat diffy = endPoints[0].y- startPoints[0].y;
				if( diffx*diffx > diffy*diffy){
					renderer.anglex = renderer.anglex- 90.0/360.0*diffx;
				}else {
					renderer.angley = renderer.angley - 90.0/480.0*diffy;
				}
				
				
			}else {
				renderer.distance = renderer.distance+(sqrt(pow((startPoints[0].x- startPoints[1].x),2)+
														   pow((startPoints[0].y -startPoints[1].y),2))-
				sqrt(pow((endPoints[0].x- endPoints[1].x),2)+
					 pow((endPoints[0].y -endPoints[1].y),2)))*0.5;
			}
			startPoints[0] = endPoints[0];
			startPoints[1] = endPoints[1];

			
		}
	}

}

- (void)touchesEnded:(NSSet *)touches withEvent:(UIEvent *)event
{
	int i;
	NSLog(@"touchCount %d", self.touchCount);
	for (i = 0; i < touchCount; i++) {
		NSLog(@"startPos x,y=%@", NSStringFromCGPoint(startPoints[i]));
	}
	for (i = 0; i < touchCount; i++) {
		NSLog(@"endPos x,y=%@", NSStringFromCGPoint(endPoints[i]));
	}
	
}

- (void) drawView:(id)sender
{
    [renderer render];
}

- (void) layoutSubviews
{
	[renderer resizeFromLayer:(CAEAGLLayer*)self.layer];
    [self drawView:nil];
}

- (NSInteger) animationFrameInterval
{
	return animationFrameInterval;
}

- (void) setAnimationFrameInterval:(NSInteger)frameInterval
{
	// Frame interval defines how many display frames must pass between each time the
	// display link fires. The display link will only fire 30 times a second when the
	// frame internal is two on a display that refreshes 60 times a second. The default
	// frame interval setting of one will fire 60 times a second when the display refreshes
	// at 60 times a second. A frame interval setting of less than one results in undefined
	// behavior.
	if (frameInterval >= 1)
	{
		animationFrameInterval = frameInterval;
		
		if (animating)
		{
			[self stopAnimation];
			[self startAnimation];
		}
	}
}

- (void) startAnimation
{
	if (!animating)
	{
		if (displayLinkSupported)
		{
			// CADisplayLink is API new to iPhone SDK 3.1. Compiling against earlier versions will result in a warning, but can be dismissed
			// if the system version runtime check for CADisplayLink exists in -initWithCoder:. The runtime check ensures this code will
			// not be called in system versions earlier than 3.1.

			displayLink = [NSClassFromString(@"CADisplayLink") displayLinkWithTarget:self selector:@selector(drawView:)];
			[displayLink setFrameInterval:animationFrameInterval];
			[displayLink addToRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];
		}
		else
			animationTimer = [NSTimer scheduledTimerWithTimeInterval:(NSTimeInterval)((1.0 / 60.0) * animationFrameInterval) target:self selector:@selector(drawView:) userInfo:nil repeats:TRUE];
		
		animating = TRUE;
	}
	[renderer startMovie];
}

- (void)stopAnimation
{
	if (animating)
	{
		if (displayLinkSupported)
		{
			[displayLink invalidate];
			displayLink = nil;
		}
		else
		{
			[animationTimer invalidate];
			animationTimer = nil;
		}
		
		animating = FALSE;
	}
	[renderer stopMovie];
}

- (void) dealloc
{
    [renderer release];
	
    [super dealloc];
}

@end
