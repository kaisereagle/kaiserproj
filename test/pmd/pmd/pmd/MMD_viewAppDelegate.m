//
//  MMD_viewAppDelegate.m
//  MMD_view
//

#import "MMD_viewAppDelegate.h"
#import "MMD_viewViewController.h"
#import "EAGLView.h"

@implementation MMD_viewAppDelegate

@synthesize window;
@synthesize viewController;


#pragma mark -
#pragma mark Application lifecycle

- (BOOL)application:(UIApplication *)application didFinishLaunchingWithOptions:(NSDictionary *)launchOptions {    
    
    // Override point for customization after application launch.

    // Add the view controller's view to the window and display.
    [window addSubview:viewController.view];
	[window makeKeyAndVisible];
	
    // カメラが使用可能かどうかチェックする
    if ([UIImagePickerController isSourceTypeAvailable:UIImagePickerControllerSourceTypeCamera]) {
		// イメージピッカーを作る
		pickerController = [[UIImagePickerController alloc] init];
		pickerController.sourceType = UIImagePickerControllerSourceTypeCamera;
		pickerController.cameraViewTransform = CGAffineTransformScale(pickerController.cameraViewTransform,
																		   1.18f,
																		   1.18f);

		// カメラコントローラを隠す
		pickerController.showsCameraControls = NO;
	 
		// カメラオーバーレイビューを追加する
		pickerController.cameraOverlayView = arView;
	 
		// イメージピッカーを表示する
		[viewController presentModalViewController:pickerController animated:NO];
	 }

    return YES;
}



- (void)applicationWillResignActive:(UIApplication *)application {
    /*
     Sent when the application is about to move from active to inactive state. This can occur for certain types of temporary interruptions (such as an incoming phone call or SMS message) or when the user quits the application and it begins the transition to the background state.
     Use this method to pause ongoing tasks, disable timers, and throttle down OpenGL ES frame rates. Games should use this method to pause the game.
     */
}


- (void)applicationDidEnterBackground:(UIApplication *)application {
    /*
     Use this method to release shared resources, save user data, invalidate timers, and store enough application state information to restore your application to its current state in case it is terminated later. 
     If your application supports background execution, called instead of applicationWillTerminate: when the user quits.
     */
	[arView stopAnimation];	
}


- (void)applicationWillEnterForeground:(UIApplication *)application {
    /*
     Called as part of  transition from the background to the inactive state: here you can undo many of the changes made on entering the background.
     */
	[arView startAnimation];
}


- (void)applicationDidBecomeActive:(UIApplication *)application {
    /*
     Restart any tasks that were paused (or not yet started) while the application was inactive. If the application was previously in the background, optionally refresh the user interface.
     */
	[arView startAnimation];
}


- (void)applicationWillTerminate:(UIApplication *)application {
    /*
     Called when the application is about to terminate.
     See also applicationDidEnterBackground:.
     */
	[arView stopAnimation];
}


#pragma mark -
#pragma mark Memory management

- (void)applicationDidReceiveMemoryWarning:(UIApplication *)application {
    /*
     Free up as much memory as possible by purging cached data objects that can be recreated (or reloaded from disk) later.
     */
}


- (void)dealloc {
    [viewController release];
    [window release];
    [super dealloc];
}


@end
