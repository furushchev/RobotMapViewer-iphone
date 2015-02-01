//
//  RosMapManager.h
//  RobotMapViewer
//
//  Created by FurutaYuki on 1/6/15.
//  Copyright (c) 2015 JSK Lab. All rights reserved.
//

#import <Foundation/Foundation.h>
#import <cstdint>
#import <vector>
#import <map>
#import <string>
#import <ros/ros.h>

#import <boost/thread/thread.hpp>
#import <boost/thread/mutex.hpp>
#import <std_msgs/String.h>
#import <nav_msgs/OccupancyGrid.h>
#import <geometry_msgs/Pose.h>
#import <tf/transform_listener.h>

@class MapViewController;

typedef std::map<std::string, tf::StampedTransform> RobotPoseDictionary;

class RosMapManager {
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    ros::Publisher call_pub_;
    boost::thread *thread_;
    boost::mutex mutex_;
    tf::TransformListener tf_;
    
    ros::Timer timer_;
    
    nav_msgs::OccupancyGrid map_;
    uint8_t *mapImage_;
    bool new_map_;
    RobotPoseDictionary robots_pose_;
public:
    MapViewController __weak * view_controller_;
    
    RosMapManager();
    ~RosMapManager();
    
    void rosSpin();
    void rosSpinOnce();
    void mapCB(const nav_msgs::OccupancyGridConstPtr &map);
    
    uint32_t getMapWidth();
    uint32_t getMapHeight();
    float getMapOriginX();
    float getMapOriginY();
    float getMapOriginRot();
    float getMapResolution();
    int8_t * getMapRawData();
    void updateRobotsPose();
    RobotPoseDictionary getRobotsPose();
    
    uint8_t* newMapImageData();
    void releaseMapImageData();
    
    bool isNewMapAvailable();
    
    void call();
};