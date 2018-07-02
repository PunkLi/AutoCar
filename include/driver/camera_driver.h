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

namespace autocar
{
namespace driver
{
/**
 * @brief: 摄像头配置类型
 */
class camera_driver
{
public:
    /**
     * @brief: 读入配置文件
     */
    camera_driver(std::string path)
    {
        camera_mat_file = path;
    }
    /**
     * @brief Read vision paramaters, such as camera matrix, Distortion matrix and so on.
    */
    bool open_camera(cv::VideoCapture&, std::string& ); 
    /**
     * @brief 矫正摄像头与云台的打击点
    */
    void set_camera_pose();
public:
    // 相机参数
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    
    std::string camera_mat_file;
    int exposure_time;
    std::string usb_cam_id;
    // 相机安装
    double ptz_camera_x;
    double ptz_camera_y;
    double ptz_camera_z;

    cv::Mat t_camera_ptz;
    cv::Mat r_camera_ptz;
    double barrel_ptz_offset_y;
    double z_scale;
};

} // namespace driver
} // namespace autocar

#endif // VISION_PARAM_H
