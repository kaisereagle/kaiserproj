//
//  MMD_viewAppDelegate.h
//  MMD_view
//
//

#import <UIKit/UIKit.h>

@class MMD_viewViewController;
@class EAGLView;

@interface MMD_viewAppDelegate : NSObject <UIApplicationDelegate> {
	UIImagePickerController*    pickerController;
    UIWindow *window;
    MMD_viewViewController *viewController;
	IBOutlet EAGLView *arView;
}

@property (nonatomic, retain) IBOutlet UIWindow *window;
@property (nonatomic, retain) IBOutlet MMD_viewViewController *viewController;

@end

