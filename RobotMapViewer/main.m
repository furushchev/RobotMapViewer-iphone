//
//  main.m
//  RobotMapViewer
//
//  Created by FurutaYuki on 1/6/15.
//  Copyright (c) 2015 JSK Lab. All rights reserved.
//

#import <UIKit/UIKit.h>
#import "AppDelegate.h"
#import <sys/types.h>
#import <signal.h>


int main(int argc, char * argv[]) {

    signal(SIGPIPE, SIG_IGN);
    
    @autoreleasepool {
        @try {
            return UIApplicationMain(argc, argv, nil, NSStringFromClass([AppDelegate class]));
        }
        @catch (NSException *exception) {
            NSLog(@"caught error: %@", exception);
        }
        @finally {
            return 0;
        }

    }
}
