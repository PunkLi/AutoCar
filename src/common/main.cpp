/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2018, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#include <thread> 
#include <unistd.h>
#include "opencv2/opencv.hpp"

#include "common/ImageConsProd.hpp"

class scoped_thread
{
    std::thread t;
public:
    explicit scoped_thread(std::thread t_)
        : t(std::move(t_))
        {
            if(!t.joinable())
                throw std::logic_error("No thread");
        }
    ~scoped_thread()
    {
        t.join();
    }
    scoped_thread(scoped_thread const&) = delete;
    scoped_thread& operator=(scoped_thread const&) = delete;
};

int main(int argc, char * argv[])
{
    autocar::vision_mul::ImageConsProd camera1("../calibration-param/480P_120_RH.xml");

    std::thread task0(&autocar::vision_mul::ImageConsProd::ImageConsumer, camera1);
    std::thread task1(&autocar::serial_mul::listen2car);

    task0.join();
    task1.join();
    
	return EXIT_SUCCESS;
}
