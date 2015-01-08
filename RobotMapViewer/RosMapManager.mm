//
//  RosMapManager.mm
//  RobotMapViewer
//
//  Created by FurutaYuki on 1/6/15.
//  Copyright (c) 2015 JSK Lab. All rights reserved.
//

#import "RosMapManager.h"

RosMapManager::RosMapManager()
{
    map_sub_ = nh_.subscribe("/map", 10, &RosMapManager::mapCB, this);
    thread_ = new boost::thread(&RosMapManager::rosSpin, this);
    new_map_ = false;
}

RosMapManager::~RosMapManager()
{
    ros::shutdown();
    thread_->join();
    delete thread_;
}

void RosMapManager::rosSpin()
{
    ros::spin();
}

void RosMapManager::mapCB(const nav_msgs::OccupancyGridConstPtr &map)
{
    ROS_INFO("map received");
    map_ = *map;
    new_map_ = true;
}

uint32_t RosMapManager::getMapWidth()
{
    if (&map_) return map_.info.width;
    else return 0;
}

uint32_t RosMapManager::getMapHeight()
{
    if (&map_) return map_.info.height;
    else return 0;
}

int8_t* RosMapManager::getMapData()
{
    if (!new_map_) {
        return nullptr;
    }
    new_map_ = false;
    return &(map_.data[0]);
}

bool RosMapManager::isNewMapAvailable()
{
    return new_map_;
}