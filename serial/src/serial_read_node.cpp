/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2019, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#include <ros/ros.h>
#include "serial_read_proto.h"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "serial_Read");
    ros::NodeHandle nh;
    ros::Publisher pub_chassis;
    ros::Publisher pub_gimbal;

    pub_chassis = nh.advertise<serial::chassis_info>("chassis_info", 100);
    pub_gimbal  = nh.advertise<serial::gimbal_info>("gimbal_info", 100);

    ros::Rate loopRate(50);

    // init serial
    serial_mul::serial_read serial;
  
    while(nh.ok())
    {
        ros::spinOnce();
        serial.read_from_serial();

        pub_chassis.publish(serial.chassis);
        pub_gimbal.publish(serial.gimbal);
        
        loopRate.sleep();
    }
    return 0;
}
