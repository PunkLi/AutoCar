/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2019, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#include <ros/ros.h>
#include "serial_write_proto.h"
#include <detect/armor_goal.h>
#include <decision/shoot_info.h>

detect::armor_goal vision_data;

void vision_callback(const detect::armor_goal::ConstPtr & armor_data)
{
    std::vector<geometry_msgs::Point32>::const_iterator it = armor_data->multigoal.begin();

    cout <<"armor_data->multigoal.size(): "<< armor_data->multigoal.size() << endl;

    for (int i = 0; it != armor_data->multigoal.end(); ++it,++i)
    {
        double yaw   = it->x; // yaw
        double pitch = it->y; // pitch
        double z     = it->z; // distance

        cout << yaw << "\t" << pitch << "\t" << z << "\t" << i << endl;
    }
}

void shoot_callback(const decision::shoot_info & data)
{
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "serial_Write");

    ros::NodeHandle nh;
    ros::Subscriber sub_armor = nh.subscribe("armor_info", 5, vision_callback); // update data
    ros::Subscriber sub_shoot = nh.subscribe("shoot_info", 5, shoot_callback); // update data
    ros::Rate loopRate(100);

    // init serial
    serial_mul::serial_write serial;
    
    while(nh.ok())
    {
        ros::spinOnce();
        serial.write_To_serial(vision_data); // pub data
        loopRate.sleep();
    }

  return 0;
}
