//
//  RosMapManager.h
//  RobotMapViewer
//
//  Created by FurutaYuki on 1/6/15.
//  Copyright (c) 2015 JSK Lab. All rights reserved.
//

#import <Foundation/Foundation.h>
#import <cstdint>
#import <ros/ros.h>
#import <boost/thread/thread.hpp>
#import <boost/thread/mutex.hpp>
#import <nav_msgs/OccupancyGrid.h>
#import <geometry_msgs/Pose.h>

@class MapViewController;

class RosMapManager {
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    boost::thread *thread_;
    boost::mutex mutex_;
    
    nav_msgs::OccupancyGrid map_;
    bool new_map_;
public:
    MapViewController __weak * view_controller_;
    
    RosMapManager();
    ~RosMapManager();
    void rosSpin();
    void mapCB(const nav_msgs::OccupancyGridConstPtr &map);
    uint32_t getMapWidth();
    uint32_t getMapHeight();
    int8_t * getMapData();
    bool isNewMapAvailable();
};