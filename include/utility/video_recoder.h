#ifndef VIDEO_RECODER_H
#define VIDEO_RECODER_H

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>

namespace autocar
{
namespace vision_mul
{
    class video_recoder {
    public:
        video_recoder(std::string video_dir_="/home/dji/Videos", int cols_=1280, int rows_=720);
        ~video_recoder();
        void save_frame(const cv::Mat &image);

    private:
        cv::VideoWriter writer;
        int rows;
        int cols;
    };
}
}

#endif // VIDEO_RECODER_H

