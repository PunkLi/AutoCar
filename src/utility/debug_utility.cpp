#include "utility/debug_utility.hpp"

#include <unistd.h>     /*Unix标准函数定义*/
#include <sys/types.h>  /**/
#include <sys/stat.h>   /**/
#include <fcntl.h>      /*文件控制定义*/
#include <termios.h>    /*PPSIX终端控制定义*/
#include <errno.h>      /*错误号定义*/
#include <sys/ioctl.h>
#include <iostream>

int set_camera_exposure(std::string id, int val)
{
    // 通过v4l2配置摄像头参数
    int cam_fd;
    if ((cam_fd = open(id.c_str(), O_RDWR)) == -1) {
        std::cerr << "Camera open error" << std::endl;
        return -1;
    }

    // 手动曝光
    struct v4l2_control control_s;
    control_s.id = V4L2_CID_EXPOSURE_AUTO;
    control_s.value = V4L2_EXPOSURE_MANUAL;
    ioctl(cam_fd, VIDIOC_S_CTRL, &control_s);

    // 设置曝光值
    control_s.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    control_s.value = val;
    ioctl(cam_fd, VIDIOC_S_CTRL, &control_s);
    close(cam_fd);

    return 0;
}

void draw_rotated_rect(const cv::Mat &img, const cv::RotatedRect &rect, const cv::Scalar &color, int thickness)
{
    cv::Point2f vertex[4];

    rect.points(vertex);
    for (int i=0; i<4; i++)
        cv::line(img, vertex[i], vertex[(i+1)%4], color, thickness);
}
