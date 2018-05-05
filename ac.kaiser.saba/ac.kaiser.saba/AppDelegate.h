//
//  AppDelegate.h
//  ac.kaiser.saba
//
//  Created by sakya-suzuki on 2018/05/02.
//  Copyright © 2018年 sakya-suzuki. All rights reserved.
//

#import <UIKit/UIKit.h>
@class eaglview;
@class ViewController;

@interface AppDelegate : NSObject <UIApplicationDelegate>
{
    IBOutlet eaglview *eaview;
    UIWindow *window;
    ViewController *controller;
}
@property (nonatomic, retain) IBOutlet UIWindow *window;
@property (nonatomic, retain) IBOutlet ViewController *controller;


@end

