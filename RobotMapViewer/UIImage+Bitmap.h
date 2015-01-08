//
//  UIImage+Bitmap.h
//  RobotMapViewer
//
//  Created by FurutaYuki on 1/8/15.
//  Copyright (c) 2015 JSK Lab. All rights reserved.
//

#import <UIKit/UIKit.h>

@interface UIImage (GrayScale)

+ (UIImage *)imageWithBitmapData:(uint8_t *)data withChannel:(int)channel withWidth:(size_t)width withHeight:(size_t)height;
+ (UIImage *)imageWithGrayScaleBitmapData:(uint8_t *)data withWidth:(size_t)width withHeight:(size_t)height;
+ (UIImage *)imageWithRGBBitmapData:(uint8_t *)data withWidth:(size_t)width withHeight:(size_t)height;
+ (UIImage *)imageWithRGBABitmapData:(uint8_t *)data withWidth:(size_t)width withHeight:(size_t)height;

@end
