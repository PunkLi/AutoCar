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
#include <serial/car_info.h>

#include <stdint.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

using namespace std;
using namespace boost::asio;

namespace serial_mul
{

struct buffer_read 
{
    uint8_t  sof;
    
    int16_t  uwb_x;
    int16_t  uwb_y;
    uint16_t uwb_angle;

    int16_t  angle;
    int16_t  v_x;
    int16_t  v_y;
    int16_t  v_r;

    int16_t  yaw;
    int16_t  pitch;

    uint8_t  end;

}__attribute__((packed));



class serial_read
{
    buffer_read data;
    
    int     data_len;
    int16_t init_yaw;

public:
    serial::car_info info;

public:
    serial_read(): port_id("/dev/ttyUSB0")
    {
        pSerialPort = new serial_port(m_ios);

        if (pSerialPort){
            ros::param::get("serial_port",port_id);
			if (init_port( port_id, 8 ))
                cout << "init serial [ " << port_id << " ] success! read data... \n";
		}
        data_len = sizeof(data);

        // uint8_t buff[data_len];
        // read(*pSerialPort, buffer(buff));

        if (data.sof == 0xDA && data.end == 0xDB) init_yaw = data.angle;
        
    }
    ~serial_read() { if(pSerialPort) delete pSerialPort; }

public:
	void read_from_serial()
    {
        //async_read( *pSerialPort, buffer(data), 
        //    boost::bind( &serial_read::read_callback, this, boost::asio::placeholders::error) );
        // read(...)
    }

	void read_callback(const boost::system::error_code & ec )
    {
        if(!ec)
        {
            if (data.sof == 0xDA && data.end == 0xDB)
            {
                info.stamp = ros::Time::now();
                info.angle = (data.angle - init_yaw) * (M_PI/180.);
                info.v_x   = data.v_x / 1000.;
                info.v_y   = data.v_y / 1000.;
                info.v_r   = data.v_r * (M_PI/180.);

                info.stamp = ros::Time::now();
                info.yaw   = data.yaw;
                info.pitch = data.pitch;
            }
        }
        else
        {
            ; // Todo: error
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

    io_service m_ios;				// io_service Object
	serial_port *pSerialPort;		// Serial port Object
	std::string port_id;	    	// For save com name
	boost::system::error_code ec;	// Serial_port function exception
};

} // namespace serial_mul

#endif