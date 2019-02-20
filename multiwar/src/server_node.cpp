#include <ros/ros.h>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <thread>
#include <iostream>

#include "udp_broadcast.h"
#include <multiwar/share_info.h>

using namespace std;

multiwar::share_info field_info;

void battle_callback(const multiwar::share_info& info)
{
    ;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "udp_publisher");
    ros::NodeHandle nh;
    ros::Subscriber sub_vision = nh.subscribe("my_info", 5, battle_callback); // update data

    int udp_port = 8888;
    int udp_rate = 10;
    ros::param::get("udp_port",udp_port);
    ros::param::get("udp_rate",udp_rate);

    ros::Rate loop_rate(udp_rate);

    namespace ip = boost::asio::ip;
    boost::asio::io_service io_service;
  
    // Server binds to any address and any port.
    ip::udp::socket socket(io_service, ip::udp::endpoint(ip::udp::v4(), 0));
    socket.set_option(boost::asio::socket_base::broadcast(true));
  
    // Broadcast will go to port 8888.
    ip::udp::endpoint broadcast_endpoint(ip::address_v4::broadcast(), udp_port);
  
    // Broadcast data.
    RobotMsg msg;
    msg.yaw = 3.14;
    msg.pitch = 4.16231311312;
    msg.distance = 48;

    uint8_t send_bytes[8];

    udp_encode(msg, send_bytes);

    while(ros::ok())
    {
        socket.send_to(boost::asio::buffer(send_bytes), broadcast_endpoint);
        //ros::spinOnce();
        loop_rate.sleep();
    }
    io_service.run();

    return EXIT_SUCCESS;
}