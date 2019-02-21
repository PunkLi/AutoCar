/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2019, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#include <armor_detect_node.h>

namespace detect_mul
{

void armor_detect_node::car_callback(const serial::car_info::ConstPtr &info)
{
    double gimbal_yaw   = info->yaw;
    double gimbal_pitch = info->yaw;
}

void armor_detect_node::armor_callback(const sensor_msgs::ImageConstPtr& msg)
{
    try{

        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        
        cv::imshow("img", frame);
        cv::waitKey(1);

        //multi_armors_.clear();
        //armor_detect_.detect(frame, multi_armors_);

        //if(multi_armors_.size())
        //{
            // pnp
            // detect::armor_goal armor_pos = what?
            // pub_armor_.publish(armor_pos);
        //}

        // armor detect

        std::vector<geometry_msgs::Point32> multi_goal;
        geometry_msgs::Point32 v1,v2;
        multi_goal.push_back(v1);
        multi_goal.push_back(v2);

        // pnp solver

        
        // DO NOT EDIT!
        armor_info.stamp     = ros::Time::now();
        armor_info.detected  = detected;
        armor_info.multigoal = multi_goal;
        pub_armor_.publish(armor_info);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

} // namespace detect_mul

int main(int argc, char **argv)
{
    ros::init(argc, argv, "armor_detect");

    detect_mul::armor_detect_node node;

    ros::MultiThreadedSpinner spinner(2);
    ros::spin();

    return 0;
}