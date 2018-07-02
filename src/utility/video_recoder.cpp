#include "utility/video_recoder.h"
#include <stdio.h>
#include <time.h>
namespace autocar
{
namespace vision_mul
{
video_recoder::video_recoder(std::string video_dir_, int cols_, int rows_)
{
    cols = cols_;
    rows = rows_;

    char filename[128];
    time_t rawtime;
    struct tm *timeinfo;
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    sprintf(filename, "%s/armor_%04d%02d%02d_%02d%02d%02d.avi", video_dir_.c_str(),
            timeinfo->tm_year+1900, timeinfo->tm_mon+1, timeinfo->tm_mday,
            timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);

    writer.open(filename, CV_FOURCC('M', 'J', 'P', 'G'), 60.0, cv::Size(cols, rows));
}

video_recoder::~video_recoder()
{
}

void video_recoder::save_frame(const cv::Mat &image)
{
    if (image.rows!=rows || image.cols!=cols) {
        cv::Mat img;
        cv::resize(image, img, cv::Size(cols, rows));
        writer.write(img);
    } else {
        writer.write(image);
    }
}
} // namespace vision_mul
} // namespace autocar
