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
#import <QuartzCore/QuartzCore.h>
#import "MRoundedButton.h"

@interface MapViewController ()
{
    RosMapManager *ros_controller_;
    NSTimer *timer_;
}

@property (weak, nonatomic) IBOutlet UIScrollView *mapScrollView;
@property (nonatomic, strong) UIImageView *mapImageView;
@property (nonatomic, strong) NSMutableDictionary *robotImageViews;
@property (strong, nonatomic) MRoundedButton *callButton;

@end

@implementation MapViewController

- (void)viewDidLoad {
    [super viewDidLoad];
    
    self.robotImageViews = [NSMutableDictionary dictionary];
    
    self.mapScrollView.maximumZoomScale = 5.0f;
    self.mapScrollView.delegate = self;
    
    auto screenSize = [[UIScreen mainScreen] bounds].size;
    auto center = CGPointMake(screenSize.width / 2.0f, screenSize.height - 80);
    
    MRHollowBackgroundView *backgroundView = [[MRHollowBackgroundView alloc] initWithFrame:CGRectMake(0, 0, 81, 81)];
    backgroundView.center = center;
    backgroundView.userInteractionEnabled = YES;
    backgroundView.foregroundColor = [UIColor clearColor];

    [self.view addSubview:backgroundView];

    self.callButton = [[MRoundedButton alloc] initWithFrame:CGRectMake(0, 0, 80, 80)
                                                buttonStyle:MRoundedButtonSubtitle];
    self.callButton.cornerRadius = MRoundedButtonMaxValue;
    self.callButton.userInteractionEnabled = YES;
    self.callButton.borderWidth = 2;
    self.callButton.borderColor = [[UIColor alloc] initWithWhite:1.0 alpha:0.7];
    self.callButton.foregroundColor = [[UIColor alloc] initWithWhite:1.0 alpha:0.7];
    self.callButton.backgroundColor = [UIColor clearColor];
    self.callButton.textLabel.text = @"!";
    self.callButton.textLabel.font = [UIFont boldSystemFontOfSize:50];
    self.callButton.detailTextLabel.text = @"C A L L";
    self.callButton.detailTextLabel.font = [UIFont systemFontOfSize:10];
    [self.callButton addTarget:self action:@selector(callButtonPressed) forControlEvents:UIControlEventTouchUpInside];
    [backgroundView addSubview:self.callButton];

    ros_controller_ = new RosMapManager();
    ros_controller_->view_controller_ = self;
    while (!ros_controller_->isNewMapAvailable()) {
        NSLog(@"waiting for map...");
        sleep(1);
    }
    [self drawMap];
    
    timer_ = [NSTimer scheduledTimerWithTimeInterval:1.0f target:self selector:@selector(drawRobots) userInfo:nil repeats:YES];
}

- (void)viewWillAppear:(BOOL)animated
{
    [super viewWillAppear:animated];
}

- (void)viewDidDisappear:(BOOL)animated
{
    [timer_ invalidate];
    delete ros_controller_;
    [super viewDidDisappear:animated];
}

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
}

- (void)drawMap
{
    uint8_t *mapData = (uint8_t*)ros_controller_->newMapImageData();
    size_t mapWidth = ros_controller_->getMapWidth();
    size_t mapHeight = ros_controller_->getMapHeight();
    NSLog(@"get map %ldx%ld", mapWidth, mapHeight);
    UIImage *mapImage = [UIImage imageWithGrayScaleBitmapData:mapData withWidth:mapWidth withHeight:mapHeight];
    
    _mapScrollView.contentSize = mapImage.size;
    
    _mapImageView = [[UIImageView alloc] initWithImage:mapImage];
    _mapImageView.frame = CGRectMake(0, 0, mapImage.size.width, mapImage.size.height);
    [_mapScrollView addSubview:_mapImageView];
}

- (void)drawRobots
{
    ros_controller_->updateRobotsPose();
    RobotPoseDictionary dict = ros_controller_->getRobotsPose();
    for (auto &pair : dict) {
        NSString *frame_id = [NSString stringWithUTF8String:pair.first.c_str()];
        tf::StampedTransform trans = pair.second;
        UIImageView *iv = [self.robotImageViews objectForKey:frame_id];
        if (!iv) {
            UIImage *robotImage = [UIImage imageNamed:@"PR2"];
            if ([frame_id rangeOfString:@"turtlebot"].location != NSNotFound ||
                [frame_id rangeOfString:@"kobuki"].location != NSNotFound) {
                robotImage = [UIImage imageNamed:@"turtlebot"];
            }
            float scale = 0.1;
            iv = [[UIImageView alloc] initWithFrame:CGRectMake(0, 0, robotImage.size.width * scale, robotImage.size.height * scale)];
            iv.image = robotImage;
            [iv setBackgroundColor:[UIColor clearColor]];
            [self.mapImageView addSubview:iv];
            self.robotImageViews[frame_id] = iv;
            NSLog(@"draw new robot: %@", NSStringFromCGRect(iv.frame));
        }
        
        CGPoint p = drawPointOfRobot(trans, ros_controller_);
        NSLog(@"draw point: %@", NSStringFromCGPoint(p));
        NSLog(@"offset: %@", NSStringFromCGPoint(self.mapScrollView.contentOffset));
        NSLog(@"scale factor: %f", self.mapScrollView.zoomScale);
        
//        CGFloat newWidth = iv.image.size.width * self.mapScrollView.zoomScale * 0.1;
//        CGFloat newHeight = iv.image.size.height * self.mapScrollView.zoomScale * 0.1;
//        iv.frame = CGRectMake(0, 0, newWidth, newHeight);
//        iv.frame = CGRectMake(iv.frame.origin.x, iv.frame.origin.y, newWidth, newHeight);
        iv.center = CGPointMake(p.x, p.y);
//        iv.bounds = CGRectMake(p.x - newWidth / 2.f, p.y - newHeight / 2.f, newWidth, newHeight);
        NSLog(@"iv frame: %@", NSStringFromCGRect(iv.frame));
/*
        layer.position = CGPointZero;
        CGAffineTransform afTrans = CGAffineTransformMakeTranslation(p.x - self.mapScrollView.contentOffset.x - layer.position.x, p.y - self.mapScrollView.contentOffset.y - layer.position.y);
        CGAffineTransformScale(afTrans, self.mapScrollView.zoomScale, self.mapScrollView.zoomScale);
        [layer setAffineTransform:afTrans];

        layer.position = CGPointMake(p.x * self.mapScrollView.contentScaleFactor, p.y / self.mapScrollView.contentScaleFactor);
        
        if (isNewLayer) {
            UIImage *robotImage = [UIImage imageNamed:@"Turtlebot_small"];
            NSLog(@"created new robot layer %@", NSStringFromCGSize(robotImage.size));
            layer.contents = (id)robotImage.CGImage;
            layer.backgroundColor = [UIColor clearColor].CGColor;
            layer.frame = CGRectMake(0, 0, robotImage.size.width, robotImage.size.height);
            layer.anchorPoint = CGPointZero;
            [self.mapScrollView.layer addSublayer:layer];
            self.robotsLayer[frame_id] = layer;
        }
 */
    }
}

CGPoint drawPointOfRobot(tf::StampedTransform trans, RosMapManager *ros_controller_)
{
    float x_org = trans.getOrigin().x() - ros_controller_->getMapOriginX();
    float y_org = trans.getOrigin().y() - ros_controller_->getMapOriginY();
    float resolution = ros_controller_->getMapResolution();
//    return CGPointMake(x_org / resolution, y_org / resolution);
    return CGPointMake(y_org / resolution, x_org / resolution);
}

#pragma mark - UIScrollViewDelegate

- (UIView *)viewForZoomingInScrollView:(UIScrollView *)scrollView
{
    return [scrollView subviews][0];
}

- (void)scrollViewDidZoom:(UIScrollView *)scrollView
{

}

#pragma mark - CallButton

- (void)callButtonPressed {
    NSLog(@"%s", __PRETTY_FUNCTION__);
    ros_controller_->call();
}


@end
