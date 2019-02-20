#pragma once
#include <stdint.h>
#include <ros/ros.h>

struct RobotMsg
{
    uint8_t frame_head;
    uint8_t frame_tail;
    uint8_t car_id;
    int udp_lens;
    // todo: add time stamp
    double yaw;
    double pitch;
    uint8_t distance;
    
    RobotMsg()
    {
        frame_head = 0xFF;
        frame_tail = 0xFE;
        int temp_car_id;
        ros::param::get("car_id",temp_car_id);
        ros::param::get("udp_lens",udp_lens);
        car_id = static_cast<char>(temp_car_id)+'0';
    }
};

bool udp_encode(RobotMsg& msg, uint8_t send_bytes[])
{
    short* data_ptr = (short *)(send_bytes + 2); // 16位指针指向第一个数据

    // msg protocol
    send_bytes[0] = msg.frame_head;               
    send_bytes[1] = msg.car_id;
    data_ptr[0]   = static_cast<short>(msg.yaw   * 100); // send_bytes[2][3]
    data_ptr[1]   = static_cast<short>(msg.pitch * 100); // send_bytes[4][5]
    send_bytes[6] = msg.distance;
    send_bytes[7] = msg.frame_tail; 
}

bool udp_decode(RobotMsg& msg, uint8_t send_bytes[], const int bytes_transferred)
{
    if (msg.car_id     != send_bytes[1]      &&  // check if self-robot
        msg.udp_lens   == bytes_transferred  &&  // check length
        msg.frame_head == send_bytes[0]      &&
        msg.frame_tail == send_bytes[bytes_transferred - 1] )
    {
        // ok! decode.
        msg.car_id   = send_bytes[1];
        msg.yaw      =  ( (send_bytes[3]<<8) + send_bytes[2] ) / 100.0;
        msg.pitch    =  ( (send_bytes[5]<<8) + send_bytes[4] ) / 100.0;
        msg.distance = send_bytes[6];

        return true;
    }
    else {
        // todo: add info
        return false;
    }
}