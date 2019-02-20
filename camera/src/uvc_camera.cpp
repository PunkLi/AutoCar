/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2018, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#include "uvc_camera.h"

namespace driver_mul
{

bool uvc_camera::open_camera(cv::VideoCapture & uvc_camera , std::string& param_file) // 以后在拓展其他的吧
{
    cv::FileStorage fs(param_file, cv::FileStorage::READ);
    if(!fs.isOpened())
        std::cout << "Cannot open [" << param_file <<", please check if the file is exist." << std::endl;
    cv::FileNode root = fs.root();
    
    cv::FileNode driver_mul = root["driver_mul"];

    driver_mul["cam_id"] >> usb_cam_id;
    driver_mul["exposure_time"] >> exposure_time;

    uvc_camera.set(CV_CAP_PROP_FPS,120);
    uvc_camera.open(usb_cam_id); // open camera of ptz
    if(!uvc_camera.isOpened())
    {
        std::cout<<"Cannot open uvc_camera!"<<std::endl;
        return false;
    }
    uvc_camera.set(CV_CAP_PROP_FOURCC,CV_FOURCC('M','J','P','G'));
    uvc_camera.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    uvc_camera.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    // set_camera_exposure(usb_cam_id, exposure_time);

    return true;
}

} // namespace driver_mul