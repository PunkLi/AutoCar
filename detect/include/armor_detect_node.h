/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2019, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#ifndef _ARMOR_DETECT_NODE_H_
#define _ARMOR_DETECT_NODE_H_

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <armor_info.h>
#include <armor_detect.h>

#include <ros/ros.h>
#include <serial/gimbal_info.h>
#include <detect/armor_goal.h>
#include <detect/enemy_info.h>

namespace detect_mul
{
class armor_detect_node
{
public:
    armor_detect_node()
    {
        ros::NodeHandle n;
        image_transport::ImageTransport it(n);
        sub_img_   = it.subscribe("camera_info", 1, 
                        boost::bind(&armor_detect_node::armor_callback, this, _1));

        sub_yaw_   = n.subscribe<serial::gimbal_info>("gimbal_info", 5, 
                        boost::bind(&armor_detect_node::gimbal_callback, this, _1));

        pub_armor_ = n.advertise<detect::armor_goal>("armor_info", 1000);

        pub_enemy_ = n.advertise<detect::enemy_info>("enemy_info", 1000);
    }

    void armor_callback(const sensor_msgs::ImageConstPtr& msg);
    void gimbal_callback(const serial::gimbal_info::ConstPtr &gimbal_info);

private:
    image_transport::Subscriber sub_img_;
    ros::Subscriber sub_yaw_;
    ros::Publisher  pub_armor_;
    ros::Publisher  pub_enemy_;

    //ArmorDetector armor_detect_;
    //std::vector<armor_info> multi_armors_;
};

} // namespace detect_mul

#endif
