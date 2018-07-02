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

#include "slover/angle_slover.hpp"
#include "opencv2/opencv.hpp"

void RectPnPSolver::solvePnP4Points(const std::vector<cv::Point2f> & points2d, cv::Mat & rot, cv::Mat & trans){
    if(width_target < 10e-5 || height_target < 10e-5){
        rot = cv::Mat::eye(3,3,CV_64FC1);
        trans = cv::Mat::zeros(3,1,CV_64FC1);
        return;
    }
    std::vector<cv::Point3f> point3d;
    double half_x = width_target / 2.0;
    double half_y = height_target / 2.0;

    point3d.push_back(cv::Point3f(-half_x, -half_y, 0));
    point3d.push_back(cv::Point3f(half_x, -half_y, 0));
    point3d.push_back(cv::Point3f(half_x, half_y, 0));
    point3d.push_back(cv::Point3f(-half_x, half_y, 0));

    cv::Mat r;
    cv::solvePnP(point3d, points2d, cam_matrix, distortion_coeff, r, trans);
    Rodrigues(r, rot);
}



void AngleSolver::setRelationPoseCameraPTZ(const cv::Mat & rot_camera_ptz, const cv::Mat & trans_camera_ptz, double y_offset_barrel_ptz) {
    rot_camera_ptz.copyTo(rot_camera2ptz);
    trans_camera_ptz.copyTo(trans_camera2ptz);
    offset_y_barrel_ptz = y_offset_barrel_ptz;
}


bool AngleSolver::getAngle(const cv::RotatedRect & rect, double & angle_x, double & angle_y, double bullet_speed, const cv::Point2f & offset){
    if (rect.size.height < 1)
        return false;

// get 3D positon in camera coordinate
//    double wh_ratio = width_target/height_target;
//    RotatedRect adj_rect(rect.center, Size2f(rect.size.width, rect.size.width/wh_ratio), rect.angle);
//    vector<Point2f> target2d;
//    getTarget2dPoinstion(adj_rect, target2d, offset);

    std::vector<cv::Point2f> target2d;
    getTarget2dPoinstion(rect, target2d, offset);

    cv::Mat r;
    solvePnP4Points(target2d, r, position_in_camera);
    //position_in_camera.at<double>(2, 0) = 1.4596 * position_in_camera.at<double>(2, 0);  // for camera-2 calibration (unfix center)
    //position_in_camera.at<double>(2, 0) = 1.5348 * position_in_camera.at<double>(2, 0);  // for camera-MH calibration (unfix center)
    position_in_camera.at<double>(2, 0) = scale_z * position_in_camera.at<double>(2, 0);

    _distance = /*scale*/ position_in_camera.at<double>(2, 0);

    if (_distance < min_distance || max_distance < _distance) {
        std::cout << "out of range: [" << min_distance << ", " << max_distance << "] distance: " << _distance << std::endl;
        return false;
    }

    // translate camera coordinate to PTZ coordinate
    tranformationCamera2PTZ(position_in_camera, position_in_ptz);
    // cout << "position_in_camera: " << position_in_camera.t() << endl;
    // cout << "position_in_ptz: " << position_in_ptz.t() << endl;
    // calculte angles to turn, so that make barrel aim at target
    adjustPTZ2Barrel(position_in_ptz, angle_x, angle_y, bullet_speed);

    return true;
}

bool AngleSolver::getAngle_rect(const cv::Rect & rect, double & angle_x, double & angle_y, double bullet_speed, const cv::Point2f & offset){
    if (rect.height < 1)
        return false;

// get 3D positon in camera coordinate
//    double wh_ratio = width_target/height_target;
//    RotatedRect adj_rect(rect.center, Size2f(rect.size.width, rect.size.width/wh_ratio), rect.angle);
//    vector<Point2f> target2d;
//    getTarget2dPoinstion(adj_rect, target2d, offset);

    std::vector<cv::Point2f> target2d;
    getTarget2dPoinstion_rect(rect, target2d, offset);

    cv::Mat r;
    solvePnP4Points(target2d, r, position_in_camera);
    //position_in_camera.at<double>(2, 0) = 1.4596 * position_in_camera.at<double>(2, 0);  // for camera-2 calibration (unfix center)
    //position_in_camera.at<double>(2, 0) = 1.5348 * position_in_camera.at<double>(2, 0);  // for camera-MH calibration (unfix center)
    position_in_camera.at<double>(2, 0) = scale_z * position_in_camera.at<double>(2, 0);
    if (position_in_camera.at<double>(2, 0) < min_distance || position_in_camera.at<double>(2, 0) > max_distance){
        std::cout << "out of range: [" << min_distance << ", " << max_distance << "] distance: " << _distance << std::endl;
        return false;
    }

    // translate camera coordinate to PTZ coordinate
    tranformationCamera2PTZ(position_in_camera, position_in_ptz);
//    cout << "position_in_camera: " << position_in_camera.t() << endl;
//    cout << "position_in_ptz: " << position_in_ptz.t() << endl;
    // calculte angles to turn, so that make barrel aim at target
    adjustPTZ2Barrel(position_in_ptz, angle_x, angle_y, bullet_speed);

    return true;
}

void AngleSolver::tranformationCamera2PTZ(const cv::Mat & pos, cv::Mat & transed_pos){
    transed_pos = rot_camera2ptz * pos - trans_camera2ptz;
}

void AngleSolver::adjustPTZ2Barrel(const cv::Mat & pos_in_ptz, double & angle_x, double & angle_y, double bullet_speed){
    const double *_xyz = (const double *)pos_in_ptz.data;
    double down_t = 0.0;
    if (bullet_speed > 10e-3)
        down_t = _xyz[2] / 100.0 / bullet_speed;
    double offset_gravity = 0.5 * 9.8 * down_t * down_t * 100;
    double xyz[3] = {_xyz[0], _xyz[1] - offset_gravity, _xyz[2]};
    double alpha = 0.0, theta = 0.0;

    alpha = asin(offset_y_barrel_ptz/sqrt(xyz[1]*xyz[1] + xyz[2]*xyz[2]));
    if(xyz[1] < 0){
        theta = atan(-xyz[1]/xyz[2]);
        angle_y = -(alpha+theta);  // camera coordinate
    }
    else if (xyz[1] < offset_y_barrel_ptz){
        theta = atan(xyz[1]/xyz[2]);
        angle_y = -(alpha-theta);  // camera coordinate
    }
    else{
        theta = atan(xyz[1]/xyz[2]);
        angle_y = (theta-alpha);   // camera coordinate
    }
    angle_x = atan2(xyz[0], xyz[2]);
    //cout << "angle_x: " << angle_x << "\tangle_y: " << angle_y <<  "\talpha: " << alpha << "\ttheta: " << theta << endl;
    angle_x = angle_x * 180 / 3.1415926;
    angle_y = angle_y * 180 / 3.1415926;
}

void AngleSolver::getTarget2dPoinstion(const cv::RotatedRect & rect, std::vector<cv::Point2f> & target2d, const cv::Point2f & offset){
    cv::Point2f vertices[4];
    rect.points(vertices);
    cv::Point2f lu, ld, ru, rd;
    std::sort(vertices, vertices + 4, [](const cv::Point2f & p1, const cv::Point2f & p2) { return p1.x < p2.x; });
    if (vertices[0].y < vertices[1].y) {
        lu = vertices[0];
        ld = vertices[1];
    }
    else{
        lu = vertices[1];
        ld = vertices[0];
    }
    if (vertices[2].y < vertices[3].y) {
        ru = vertices[2];
        rd = vertices[3];
    }
    else {
        ru = vertices[3];
        rd = vertices[2];
    }

    target2d.clear();
    target2d.push_back(lu + offset);
    target2d.push_back(ru + offset);
    target2d.push_back(rd + offset);
    target2d.push_back(ld + offset);
}

void AngleSolver::getTarget2dPoinstion_rect(const cv::Rect & rect, std::vector<cv::Point2f> & target2d, const cv::Point2f & offset){

    cv::Point2f lu, ld, ru, rd;

    lu = cv::Point2f(rect.x, rect.y);
    ld = cv::Point2f(rect.x, rect.y + rect.height);
    ru = cv::Point2f(rect.x + rect.width, rect.y);
    rd = cv::Point2f(rect.x + rect.width, rect.y + rect.y);

    target2d.clear();
    target2d.push_back(lu + offset);
    target2d.push_back(ru + offset);
    target2d.push_back(rd + offset);
    target2d.push_back(ld + offset);
}


void AngleSolverFactory::setTargetSize(double width, double height, TargetType type){
    if(type == TARGET_RUNE){
        rune_width = width;
        rune_height = height;
    }
    else if(type == TARGET_ARMOR){
        armor_width = width;
        armor_height = height;
    }
    else if(type == TARGET_SAMLL_ARMOR){
        small_armor_width = width;
        small_armor_height = height;
    }
}

bool AngleSolverFactory::getAngle(const cv::RotatedRect & rect, TargetType type, double & angle_x, double & angle_y, double bullet_speed, const cv::Point2f & offset){
    if(slover == NULL){
        std::cerr << "slover not set\n";
        return false;
    }

    double width = 0.0, height = 0.0;
    if(type == TARGET_RUNE){
        width = rune_width;
        height = rune_height;
    }
    else if(type == TARGET_ARMOR){
        width = armor_width;
        height = armor_height;
    }
    else if(type == TARGET_SAMLL_ARMOR){
        width = small_armor_width;
        height = small_armor_height;
    }

    cv::RotatedRect rect_rectifid = rect;
    AngleSolverFactory::adjustRect2FixedRatio(rect_rectifid, width/height);
    slover->setTargetSize(width, height);
    return slover->getAngle(rect_rectifid, angle_x, angle_y, bullet_speed, offset);
}

bool AngleSolverFactory::getAngle_rect(const cv::Rect & rect, TargetType type, double & angle_x, double & angle_y, double bullet_speed, const cv::Point2f & offset){
    if(slover == NULL){
        std::cerr << "slover not set\n";
        return false;
    }

    double width = 0.0, height = 0.0;
    if(type == TARGET_RUNE){
        width = rune_width;
        height = rune_height;
    }
    else if(type == TARGET_ARMOR){
        width = armor_width;
        height = armor_height;
    }
    else if(type == TARGET_SAMLL_ARMOR){
        width = small_armor_width;
        height = small_armor_height;
    }
    cv::Rect rect_rectifid = rect;
    AngleSolverFactory::adjustRect2FixedRatio_rect(rect_rectifid, width/height);
    slover->setTargetSize(width, height);
    return slover->getAngle_rect(rect_rectifid, angle_x, angle_y, bullet_speed, offset);
}
