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
    call_pub_ = nh_.advertise<std_msgs::String>("/call", 1);
    thread_ = new boost::thread(&RosMapManager::rosSpin, this);
    new_map_ = false;
    mapImage_ = NULL;
    robots_pose_ = RobotPoseDictionary();
    
//    timer_ = nh_.createTimer(ros::Duration(1), &RosMapManager::timerCB, this);
}

RosMapManager::~RosMapManager()
{
    this->releaseMapImageData();
    ros::shutdown();
    thread_->join();
    delete thread_;
}

void RosMapManager::rosSpin()
{
    ros::spin();
}

void RosMapManager::rosSpinOnce()
{
    ros::spinOnce();
}

void RosMapManager::mapCB(const nav_msgs::OccupancyGridConstPtr &map)
{
    ROS_INFO("map received");
    map_ = *map;
    new_map_ = true;
}

#pragma mark - robot pose

void RosMapManager::updateRobotsPose()
{
    if (!new_map_) return;
    
    std::vector<std::string> frame_ids;
    robots_pose_.clear();
    try {
        tf_.getFrameStrings(frame_ids);
    } catch (tf::TransformException e) {
        ROS_ERROR_THROTTLE(1.0,
                           "failed to get frame_ids"
                           "Reason: %s",
                           e.what());
        return;
    }
    
    try {
        for (auto & frame_id : frame_ids)
        {
            if (frame_id.find("base_link") == std::string::npos) continue;
            ROS_INFO_THROTTLE(1.0, "looking up %s -> %s", "/map", frame_id.c_str());
            tf::StampedTransform trans;
            tf_.lookupTransform("/map",
                                frame_id,
                                ros::Time(0),
                                trans);
            robots_pose_[frame_id] = trans;
        }
    } catch (tf::TransformException e) {
        ROS_ERROR_THROTTLE(1.0,
                           "failed to lookup transform "
                           "Reason: %s",
                           e.what());
        return;
    }
}

RobotPoseDictionary RosMapManager::getRobotsPose()
{
    return robots_pose_;
}

#pragma mark - map

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

float RosMapManager::getMapOriginX()
{
    if (&map_) return map_.info.origin.position.x;
    else return 0;
}

float RosMapManager::getMapOriginY()
{
    if (&map_) return map_.info.origin.position.y;
    else return 0;
}

float RosMapManager::getMapResolution()
{
    if (&map_) return map_.info.resolution;
    else return 0;
}

int8_t* RosMapManager::getMapRawData()
{
    if (!new_map_) {
        return nullptr;
    }
    new_map_ = false;
    
    return &(map_.data[0]);
}

uint8_t* RosMapManager::newMapImageData()
{
    if (!isNewMapAvailable()) {
        return NULL;
    }
    
    this->releaseMapImageData();
    
    int mapsize = map_.info.width * map_.info.height;
    this->mapImage_ = new uint8_t[mapsize];
    for (int x = 0; x < map_.info.width; ++x) {
        for (int y = 0; y < map_.info.height; ++y) {
            if (map_.data[x + y * map_.info.width] < 0) this->mapImage_[x * map_.info.height + y] = 127;
            else this->mapImage_[x * map_.info.height + y] = 255 - map_.data[x + y * map_.info.width];

        }
    }

    return this->mapImage_;
}

void RosMapManager::releaseMapImageData()
{
    if (this->mapImage_ != NULL){
        delete this->mapImage_;
        this->mapImage_ = NULL;
    }
}

bool RosMapManager::isNewMapAvailable()
{
    return new_map_;
}

#pragma mark - Call

void RosMapManager::call()
{
    std_msgs::String call_msg = std_msgs::String();
    call_msg.data = "turtlebot";
    call_pub_.publish(call_msg);
}