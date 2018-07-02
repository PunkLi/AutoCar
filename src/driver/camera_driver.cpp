/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2018, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#include "utility/debug_utility.hpp"
#include "driver/camera_driver.h"

namespace autocar
{
namespace driver
{

bool camera_driver::open_camera(cv::VideoCapture & capture_camera_forward , std::string& param_file) // 以后在拓展其他的吧
{
    camera_mat_file = param_file;

    cv::FileStorage fs(param_file, cv::FileStorage::READ);
    if(!fs.isOpened())
        std::cout << "Cannot open [" << param_file <<", please check if the file is exist." << std::endl;
    cv::FileNode root = fs.root();
    
    cv::FileNode driver_mul = root["driver_mul"];

    driver_mul["cam_id"] >> usb_cam_id;
    driver_mul["exposure_time"] >> exposure_time;

    capture_camera_forward.set(CV_CAP_PROP_FPS,120);
    capture_camera_forward.open(usb_cam_id); // open camera of ptz
    if(!capture_camera_forward.isOpened())
    {
        std::cout<<"Cannot open capture_camera_forward!"<<std::endl;
        return false;
    }
    capture_camera_forward.set(CV_CAP_PROP_FOURCC,CV_FOURCC('M','J','P','G'));
    capture_camera_forward.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    capture_camera_forward.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    set_camera_exposure(usb_cam_id, exposure_time);

    return true;
}

void camera_driver::set_camera_pose()
{
    cv::FileStorage fs(camera_mat_file, cv::FileStorage::READ);
    if(!fs.isOpened())
        std::cout << "Cannot open [" << camera_mat_file << "], please check if the file is exist." << std::endl;
    cv::FileNode root = fs.root();
    
    // driver_mul
    cv::FileNode driver_mul = root["driver_mul"];
    driver_mul["Camera_Matrix"] >> camera_matrix;
    driver_mul["Distortion_Coefficients"] >> dist_coeffs;
    driver_mul["ptz_camera_x"] >> ptz_camera_x;
    driver_mul["ptz_camera_y"] >> ptz_camera_y;
    driver_mul["ptz_camera_z"] >> ptz_camera_z;
    driver_mul["z_scale"] >> z_scale;
    
    driver_mul["barrel_ptz_offset_y"] >> barrel_ptz_offset_y;
    std::cout << "Camera_Matrix Size: " << camera_matrix.size() << std::endl;
    std::cout << "Distortion_Coefficients Size: " << dist_coeffs.size() << std::endl;

    // double barrel_ptz_offset_y = 0;
    const double overlap_dist = 100000.0;
    double theta = -atan((ptz_camera_y + barrel_ptz_offset_y)/overlap_dist);
    double r_data[] = {1,0,0,0,cos(theta),-sin(theta), 0, sin(theta), cos(theta)};
    double t_data[] = {0, ptz_camera_y, ptz_camera_z}; // ptz org position in camera coodinate system
    
    cv::Mat t(3,1, CV_64FC1, t_data);
    cv::Mat r(3,3, CV_64FC1, r_data); // Mat::eye(3, 3, CV_64FC1);
    t.copyTo(t_camera_ptz);
    r.copyTo(r_camera_ptz);
}

} // namespace driver
} // namespace autocar

    

