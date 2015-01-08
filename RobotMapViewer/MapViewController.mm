//
//  MapViewController.mm
//  RobotMapViewer
//
//  Created by FurutaYuki on 1/6/15.
//  Copyright (c) 2015 JSK Lab. All rights reserved.
//

#import "MapViewController.h"
#import "RosMapManager.h"
#import "UIImage+Bitmap.h"

@interface MapViewController ()
{
    RosMapManager *ros_controller_;
}

@property (weak, nonatomic) IBOutlet UIScrollView *mapScrollView;

@end

@implementation MapViewController

- (void)viewDidLoad {
    [super viewDidLoad];
    ros_controller_ = new RosMapManager();
    ros_controller_->view_controller_ = self;
    while (!ros_controller_->isNewMapAvailable()) {
        NSLog(@"waiting for map...");
        sleep(1);
    }
    [self drawMap];
}

- (void)viewDidDisappear:(BOOL)animated
{
    delete ros_controller_;
    [super viewDidDisappear:animated];
}

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
}

- (void)drawMap
{
    uint8_t *mapData = (uint8_t*)ros_controller_->getMapData();
    size_t mapWidth = ros_controller_->getMapWidth();
    size_t mapHeight = ros_controller_->getMapHeight();
    NSLog(@"get map %ldx%ld", mapWidth, mapHeight);
    UIImage *mapImage = [UIImage imageWithGrayScaleBitmapData:mapData withWidth:mapWidth withHeight:mapHeight];
    _mapScrollView.contentSize = mapImage.size;
    UIImageView *iv = [[UIImageView alloc] initWithImage:mapImage];
    iv.frame = CGRectMake(0, 0, mapImage.size.width, mapImage.size.height);
    [_mapScrollView addSubview:iv];
}

@end
