
#pragma once

#include <opencv2/opencv.hpp>
#include <linux/videodev2.h>
#include <string>
#include <random>
#include <chrono>
#include <iostream>
#include <stdio.h>

// 通过v4l2配置摄像头参数
int set_camera_exposure(std::string id, int val);

void draw_rotated_rect(const cv::Mat &img, const cv::RotatedRect &rect, const cv::Scalar &color, int thickness=1);
void draw_rotated_rects(const cv::Mat &img, const std::vector<cv::RotatedRect> &rects, const cv::Scalar &color, int thickness=1, bool tab=false, const cv::Scalar &text_color=cv::Scalar(100));

#define speed_test_begin() speed_test_start_begin_time = std::chrono::steady_clock::now()
#define speed_test_end(info, unit) std::cout << info << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - speed_test_start_begin_time).count() / 1000.0 << " " << unit << std::endl;
#define speed_test_reset() speed_test_start_begin_time = std::chrono::steady_clock::now()

#define record_time std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() -  armor_recorder.recorder_time).count() / 1000.0
#define console_log(args...) printf(args)
