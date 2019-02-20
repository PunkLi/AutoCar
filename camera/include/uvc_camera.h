/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2018, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#ifndef CAMERA_DRIVER_H
#define CAMERA_DRIVER_H

#include <opencv2/opencv.hpp>

namespace driver_mul
{

class uvc_camera
{
public:
    bool open_camera(cv::VideoCapture&, std::string& ); 

public:
    int exposure_time;
    std::string usb_cam_id;
};

} // namespace driver_mul

#endif // VISION_PARAM_H
