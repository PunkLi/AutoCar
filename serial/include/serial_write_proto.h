/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2019, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#ifndef _SERIAL_WRITE_H_
#define _SERIAL_WRITE_H_

#include <ros/ros.h>


#include <detect/armor_goal.h>

#include <stdint.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

using namespace std;
using namespace boost::asio;

namespace serial_mul
{

struct vision_goal 
{
    uint8_t  sof;

    uint16_t chassis_angle;
    uint16_t chassis_v_w;    
    uint16_t chassis_v_x;
    uint16_t chassis_v_y; 

    uint16_t gimbal_yaw;
    uint16_t gimbal_pitch;

    uint8_t  end;

}__attribute__((packed));


class serial_write
{
    io_service m_ios;				// io_service Object
	serial_port *pSerialPort;		// Serial port Object
	std::string port_id;	    	// For save com name
	boost::system::error_code ec;	// Serial_port function exception

public:
    serial_write(): port_id("/dev/ttyUSB0")
    {
        pSerialPort = new serial_port(m_ios);

        if (pSerialPort){
            ros::param::get("serial_port",port_id);
			if (init_port( port_id, 8 ))
                cout << "init serial [ " << port_id << " ] success! write data... \n";
		}
    }

    ~serial_write() { if(pSerialPort) delete pSerialPort; }

public:
    void write_To_serial(const detect::armor_goal& vision_data)
    {
        uint8_t data[10];

        size_t len = write( *pSerialPort, buffer( data ), ec );
        //cout << "send length: " << len << "\tbuf: " << data << "\n";
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