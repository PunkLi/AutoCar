#include <ros/ros.h>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include "udp_broadcast.h"

#include <multiwar/share_info.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "udp_listener");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<multiwar::share_info>("team_info", 5); // update data

    int udp_port = 8888;
    ros::param::get("udp_port",udp_port);

    namespace ip = boost::asio::ip;
    boost::asio::io_service io_service;
  
    // Client binds to any address on port 8888 (the same port on which
    // broadcast data is sent from server).
    ip::udp::socket socket(io_service, ip::udp::endpoint(ip::udp::v4(), udp_port ));
  
    ip::udp::endpoint sender_endpoint;
  
    // Receive data.
    
    uint8_t send_bytes[8];
    
start:
    std::size_t bytes_transferred = socket.receive_from(boost::asio::buffer(send_bytes), sender_endpoint);

    RobotMsg msg;
    if (udp_decode(msg, send_bytes, bytes_transferred))
    {
        std::cout << "remote host ip : " << sender_endpoint.address().to_string() <<"\t";
        std::cout << "got " << bytes_transferred << " bytes. " 
            << "      car_id: "<< msg.car_id 
            << "      yaw: "<< msg.yaw 
            << "      pitch: "<< msg.pitch 
            << "      car_distance: "<< msg.distance 
            << "\n";
    }
goto start;

    return EXIT_SUCCESS;
}