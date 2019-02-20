#ifndef _VIDEO_RECODER_H_
#define _VIDEO_RECODER_H_

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>

namespace recoder_mul
{

class video_recoder {
public:
    video_recoder(std::string video_dir_, int cols_, int rows_)
    {
        cols = cols_;
        rows = rows_;

        char filename[128];
        time_t rawtime;
        struct tm *timeinfo;
        time(&rawtime);
        timeinfo = localtime(&rawtime);
        sprintf(filename, "%s/armor_%04d%02d%02d_%02d%02d%02d.avi", 
                video_dir_.c_str(),
                timeinfo->tm_year+1900, 
                timeinfo->tm_mon+1, 
                timeinfo->tm_mday,
                timeinfo->tm_hour, 
                timeinfo->tm_min, 
                timeinfo->tm_sec);

        writer.open(filename, CV_FOURCC('M', 'J', 'P', 'G'), 60.0, cv::Size(cols, rows));
    }

    void save_frame(const cv::Mat &image)
    {
        if (image.rows!=rows || image.cols!=cols) {
            cv::Mat img;
            cv::resize(image, img, cv::Size(cols, rows));
            writer.write(img);
        } 
        else {
            writer.write(image);
        }
    }

private:
    cv::VideoWriter writer;
    int rows;
    int cols;
};

} // namespace recoder_mul

#endif // _VIDEO_RECODER_H_
