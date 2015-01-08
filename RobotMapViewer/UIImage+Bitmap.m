//
//  UIImage+Bitmap.m
//  RobotMapViewer
//
//  Created by FurutaYuki on 1/8/15.
//  Copyright (c) 2015 JSK Lab. All rights reserved.
//

#import "UIImage+Bitmap.h"

@implementation UIImage (GrayScale)

+ (UIImage *)imageWithBitmapData:(uint8_t *)data withChannel:(int)channel withWidth:(size_t)width withHeight:(size_t)height
{
    size_t bufferLength = width * height * channel;
    CGDataProviderRef provider = CGDataProviderCreateWithData(NULL, data, bufferLength, NULL);
    size_t bitsPerComponent = 8;
    size_t bitsPerPixel = bitsPerComponent * channel;
    size_t bytesPerRow = width * channel;
    
    CGColorSpaceRef colorSpaceRef = NULL;
    CGBitmapInfo bitmapInfo = kCGBitmapByteOrderDefault;
    if (channel == 1) {
        colorSpaceRef = CGColorSpaceCreateDeviceGray();
        bitmapInfo |= kCGImageAlphaNone;
    } else {
        colorSpaceRef = CGColorSpaceCreateDeviceRGB();
        bitmapInfo |= kCGImageAlphaPremultipliedLast;
    }
    if(colorSpaceRef == NULL) {
        NSLog(@"Error allocating color space");
        CGDataProviderRelease(provider);
        return nil;
    }

    CGColorRenderingIntent renderingIntent = kCGRenderingIntentDefault;
    
    CGImageRef iref = CGImageCreate(width,
                                    height,
                                    bitsPerComponent,
                                    bitsPerPixel,
                                    bytesPerRow,
                                    colorSpaceRef,
                                    bitmapInfo,
                                    provider,	// data provider
                                    NULL,		// decode
                                    YES,			// should interpolate
                                    renderingIntent);
    
    uint32_t* pixels = (uint32_t*)malloc(bufferLength);
    
    if(pixels == NULL) {
        NSLog(@"Error: Memory not allocated for bitmap");
        CGDataProviderRelease(provider);
        CGColorSpaceRelease(colorSpaceRef);
        CGImageRelease(iref);
        return nil;
    }
    
    CGContextRef context = CGBitmapContextCreate(pixels,
                                                 width,
                                                 height,
                                                 bitsPerComponent,
                                                 bytesPerRow,
                                                 colorSpaceRef,
                                                 bitmapInfo);
    
    if(context == NULL) {
        NSLog(@"Error context not created");
        free(pixels);
    }
    
    UIImage *image = nil;
    if(context) {
        
        CGContextDrawImage(context, CGRectMake(0.0f, 0.0f, width, height), iref);
        
        CGImageRef imageRef = CGBitmapContextCreateImage(context);
        
        // Support both iPad 3.2 and iPhone 4 Retina displays with the correct scale
        if([UIImage respondsToSelector:@selector(imageWithCGImage:scale:orientation:)]) {
            float scale = [[UIScreen mainScreen] scale];
            image = [UIImage imageWithCGImage:imageRef scale:scale orientation:UIImageOrientationUp];
        } else {
            image = [UIImage imageWithCGImage:imageRef];
        }
        
        CGImageRelease(imageRef);
        CGContextRelease(context);
    }
    
    CGColorSpaceRelease(colorSpaceRef);
    CGImageRelease(iref);
    CGDataProviderRelease(provider);
    
    if(pixels) {
        free(pixels);
    }
    return image;
}

+ (UIImage *)imageWithGrayScaleBitmapData:(uint8_t *)data withWidth:(size_t)width withHeight:(size_t)height
{
    return [UIImage imageWithBitmapData:data withChannel:1 withWidth:width withHeight:height];
}

+ (UIImage *)imageWithRGBBitmapData:(uint8_t *)data withWidth:(size_t)width withHeight:(size_t)height
{
    return [UIImage imageWithBitmapData:data withChannel:3 withWidth:width withHeight:height];
}

+ (UIImage *)imageWithRGBABitmapData:(uint8_t *)data withWidth:(size_t)width withHeight:(size_t)height
{
    return [UIImage imageWithBitmapData:data withChannel:4 withWidth:width withHeight:height];
}

@end
