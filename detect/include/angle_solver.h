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

#pragma once

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

namespace detect_mul
{
class AngleSolver {
    cv::Mat cam_matrix;
    cv::Mat dist_coeffs;
    
    cv::Mat t_camera_ptz;
    cv::Mat r_camera_ptz;
    double ptz_camera_x;
    double ptz_camera_y;
    double ptz_camera_z;
    double offset_y_barrel_ptz;
   
    double z_scale;
    double min_distance;
    double max_distance;

    double target_width;
    double target_height;
    std::vector<cv::Point3f> object3d;

    cv::Mat position_in_camera;
    cv::Mat position_in_ptz;
    
public:
    AngleSolver(const std::string config_xml, bool isSmall)
    {
        if(isSmall)
        {
            target_width  = 12.4;
            target_height = 5.4;
        }
        else{
            target_width  = 21.6;
            target_height = 5.4;
        }
        
        double half_x = target_width  / 2.0;
        double half_y = target_height / 2.0;

        object3d.push_back(cv::Point3f(-half_x, -half_y, 0));
        object3d.push_back(cv::Point3f(half_x, -half_y, 0));
        object3d.push_back(cv::Point3f(half_x, half_y, 0));
        object3d.push_back(cv::Point3f(-half_x, half_y, 0));

        cv::FileStorage fs(config_xml, cv::FileStorage::READ);
        if(!fs.isOpened())
            std::cout << "Cannot open [" << config_xml << "], please check if the file is exist." << std::endl;
        
        cv::FileNode root = fs.root();
        cv::FileNode detect_mul = root["detect_mul"];
        detect_mul["Camera_Matrix"]           >> cam_matrix;
        detect_mul["Distortion_Coefficients"] >> dist_coeffs;
        detect_mul["ptz_camera_x"]            >> ptz_camera_x;
        detect_mul["ptz_camera_y"]            >> ptz_camera_y;
        detect_mul["ptz_camera_z"]            >> ptz_camera_z;
        detect_mul["z_scale"]                 >> z_scale;
        detect_mul["min_distance"]            >> min_distance;
        detect_mul["max_distance"]            >> max_distance;
        detect_mul["offset_y_barrel_ptz"]     >> offset_y_barrel_ptz;

        std::cout << "Camera_Matrix Size: " << cam_matrix.size() << std::endl;
        std::cout << "Distortion_Coefficients Size: " << dist_coeffs.size() << std::endl;

        const double overlap_dist = 100000.0;
        double theta = -atan((ptz_camera_y + offset_y_barrel_ptz)/overlap_dist);
        double r_data[] = {1,0,0,0,cos(theta),-sin(theta), 0, sin(theta), cos(theta)};
        double t_data[] = {0, ptz_camera_y, ptz_camera_z}; // ptz org position in camera coodinate system
        
        cv::Mat t(3,1, CV_64FC1, t_data);
        cv::Mat r(3,3, CV_64FC1, r_data); // Mat::eye(3, 3, CV_64FC1);
        t.copyTo(t_camera_ptz);
        r.copyTo(r_camera_ptz);
    }

    bool getAngle(const cv::RotatedRect & rect, 
                  double & angle_x, double & angle_y, 
                  double bullet_speed, const cv::Point2f & offset = cv::Point2f());

private:    
    void getTarget2dPoinstion(const cv::RotatedRect & rect, std::vector<cv::Point2f> & target2d, const cv::Point2f & offset);

    void adjustPTZ2Barrel(const cv::Mat & pos_in_ptz, double & angle_x, double & angle_y, const double & bullet_speed);

};

}// namespace detect_mul
