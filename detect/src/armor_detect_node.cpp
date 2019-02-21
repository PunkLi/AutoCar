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

void armor_detect_node::gimbal_callback(const serial::car_info::ConstPtr &gimbal_info)
{
    double gimbal_yaw_   = gimbal_info->yaw;
    double gimbal_pitch_ = gimbal_info->yaw;
}

void armor_detect_node::armor_callback(const sensor_msgs::ImageConstPtr& msg)
{
    try{
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
            
        cv::imshow("img", frame);
        cv::waitKey(1);

        //multi_armors_.clear();
        //armor_detect_.detect(frame, multi_armors_);
        // Todo: 加预测....
        //if(multi_armors_.size())
        //{
            // pnp
            // detect::armor_goal armor_pos = what?
            // pub_armor_.publish(armor_pos);
        //}
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