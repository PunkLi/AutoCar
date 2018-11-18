/*******************************************************************************************************************
Copyright 2017 Dajiang Innovations Technology Co., Ltd (DJI)

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
documentation files(the "Software"), to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense, and / or sell copies of the Software, and 
to permit persons to whom the Software is furnished to do so, subject to the following conditions : 

The above copyright notice and this permission notice shall be included in all copies or substantial portions of
the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.
*******************************************************************************************************************/

/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2018, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#ifndef IMAGE_CONS_PROD_H
#define IMAGE_CONS_PROD_H

#include "opencv2/opencv.hpp"
#include "detect_factory/armor_detect.hpp"
#include "driver/camera_driver.h"
#include "common/common_serial.h"

namespace autocar
{
namespace vision_mul
{

class ImageConsProd {
public:
    /**
     * @brief: 接受一个摄像头参数配置文件 
     */
    ImageConsProd(std::string path) : param_file(path)
    {
        cv::FileStorage fs("../config/armor_params.xml", cv::FileStorage::READ);
        if(!fs.isOpened())
            std::cout << "ImageConsProd 构造函数: Cannot open armor_params.xml, please check if the file is exist." << std::endl;
        cv::FileNode root = fs.root();
        
        //vision_mul
        cv::FileNode vision_mul = root["vision_mul"];
        vision_mul["show_image"] >> show_image;
        vision_mul["save_result"] >> save_result;
        vision_mul["video_id"] >> video_name;
     
        #ifdef SHOW_DEBUG
        std::cout << "show_image: " << show_image:<< std::endl;
        std::cout << "save_result: " << save_result << std::endl;
        #endif
    }
    void ImageProducer();
    void ImageConsumer();
    void ImageConsumerCircle();
    
public:
    armor_pos armor_pos_;     // 装甲板信息
private: 
    bool show_image;          // 调试选项
    bool save_result;         // 调试选项
    std::string param_file;   // 配置文件
    std::string video_name;   // 打开视频路径
    std::chrono::steady_clock::time_point speed_test_start_begin_time; // 记时
    std::list<double> history_yaw;
};

} // namespace vision_mul
} // namespace autocar

#endif