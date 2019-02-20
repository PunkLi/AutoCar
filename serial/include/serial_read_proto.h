/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2019, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#ifndef _SERIAL_READ_PROTO_H_
#define _SERIAL_READ_PROTO_H_

#include <ros/ros.h>
#include <serial/gimbal_info.h>

#include <stdint.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

using namespace std;
using namespace boost::asio;

namespace serial_mul
{

struct gimbal_pose {
    uint8_t  sof;
    uint16_t current_yaw;
    uint16_t current_pitch;
    uint8_t  end;
}__attribute__((packed));



class serial_read
{
    io_service m_ios;				// io_service Object
	serial_port *pSerialPort;		// Serial port Object
	std::string port_id;	    	// For save com name
	boost::system::error_code ec;	// Serial_port function exception
    
    uint8_t buf[10];                // buffer

public:
    serial::gimbal_info pubData;

public:
    serial_read(): port_id("/dev/ttyUSB0")
    {
        pSerialPort = new serial_port(m_ios);

        if (pSerialPort){
            ros::param::get("serial_port",port_id);
			if (init_port( port_id, 8 ))
                cout << "init serial [ " << port_id << " ] success! read data... \n";
		}
    }
    ~serial_read() { if(pSerialPort) delete pSerialPort; }

public:
	void read_from_serial()
    {
        async_read( *pSerialPort, buffer(buf), 
            boost::bind( &serial_read::read_callback, this, boost::asio::placeholders::error) );
    }

	void read_callback(const boost::system::error_code & ec )
    {
        if(!ec)
        {
            if (buf[0] == 0xDA && buf[6] == 0xDB)  // 这里要加个循环
            {
                pubData.stamp = ros::Time::now();
                pubData.yaw   = (buf[1]<<8) + buf[2];
                pubData.pitch = (buf[3]<<8) + buf[4];
            }
        }
        else
        {
            ; // error
        }
    }

private:
	bool init_port( const std::string port, const unsigned int char_size = 8)
    {
        if (!pSerialPort) return false;

        pSerialPort->open( port, ec );
        
        pSerialPort->set_option( serial_port::baud_rate( 115200 ), ec );
        pSerialPort->set_option( serial_port::flow_control( serial_port::flow_control::none ), ec );
        pSerialPort->set_option( serial_port::parity( serial_port::parity::none ), ec );
        pSerialPort->set_option( serial_port::stop_bits( serial_port::stop_bits::one ), ec);
        pSerialPort->set_option( serial_port::character_size( char_size ), ec);
    
        return true;
    }
};

} // namespace serial_mul

#endif