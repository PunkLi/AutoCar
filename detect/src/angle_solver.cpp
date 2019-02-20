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

#include <angle_solver.h>

namespace detect_mul
{

bool AngleSolver::getAngle(const cv::RotatedRect & rect, 
                           double & angle_x, double & angle_y, 
                           double bullet_speed, const cv::Point2f & offset)
{
    if (rect.size.height < 1) return false;

    double wh_ratio = target_width / target_height;
    RotatedRect adj_rect(rect.center, Size2f(rect.size.width, rect.size.width/wh_ratio), rect.angle);
    vector<Point2f> target2d;
    getTarget2dPoinstion(adj_rect, target2d, offset);

    cv::Mat r;
    cv::solvePnP(object3d, target2d, cam_matrix, dist_coeffs, r, position_in_camera);
    // Rodrigues(r, rot);

    position_in_camera.at<double>(2, 0) = z_scale * position_in_camera.at<double>(2, 0);
    double vel = position_in_camera.at<double>(2, 0);

    if (vel < min_distance || max_distance < vel) {
        std::cout << "out of distance range: [" << min_distance << ", " << max_distance << "] distance: " << vel << std::endl;
        return false;
    }
    position_in_ptz = r_camera_ptz * position_in_camera - t_camera_ptz;
   
    adjustPTZ2Barrel(position_in_ptz, angle_x, angle_y, bullet_speed);

    return true;
}

void AngleSolver::adjustPTZ2Barrel(const cv::Mat & pos_in_ptz, 
                                   double & angle_x, double & angle_y, 
                                   const double & bullet_speed)
{
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
    angle_x = angle_x * 180 / 3.1415926;
    angle_y = angle_y * 180 / 3.1415926;
}

void AngleSolver::getTarget2dPoinstion(const cv::RotatedRect & rect, 
                                       std::vector<cv::Point2f> & target2d, 
                                       const cv::Point2f & offset)
{
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

} // namespace detect_mul
