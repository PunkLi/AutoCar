/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2018, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#ifndef ARMOR_DETECT_H_
#define ARMOR_DETECT_H_

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <ctype.h>
#include "info.h"
#include <chrono>

using namespace cv;

enum EnemyColor { RED = 0, BLUE = 1};
/**
 * @brief: 装甲识别器
 */
class armor_sample
{
public:
    armor_sample() { };
    void setPara(const armor_param & para) {
        _para = para;
    };
	short detect(cv::Mat & src, int);

private:
	void DrawRotatedRect(cv::Mat &img, const cv::RotatedRect &rect, const cv::Scalar &color, int thickness) {    
		cv::Point2f vertex[4];
		rect.points(vertex);
		for (int i = 0; i < 4; i++)
			cv::line(img, vertex[i], vertex[(i + 1) % 4], color, thickness);
	}

	std::vector<std::vector<cv::Point>> FindContours(const cv::Mat &binary_img) {
		std::vector<std::vector<cv::Point>> contours;
		const auto mode = CV_RETR_EXTERNAL;
		const auto method = CV_CHAIN_APPROX_SIMPLE;
		cv::findContours(binary_img, contours, mode, method);
		return contours;
	 }
	cv::Mat DistillationColor(const cv::Mat &src_img, unsigned int color) {
		std::vector<cv::Mat> bgr_channel;
		cv::split(src_img, bgr_channel);
		if (color == RED) {
			cv::Mat result_img;
			cv::subtract(bgr_channel[2], bgr_channel[1], result_img);
			return result_img;
		} 
		else{
			cv::Mat result_img;
			cv::subtract(bgr_channel[0], bgr_channel[1], result_img);
			return result_img;
		}
	}
	void choose_target_from_lights(std::vector<cv::RotatedRect> &lights, std::vector<armor_info> &armor_vector);
	armor_info SlectFinalArmor(std::vector<armor_info> &armors);
	
	void FilterArmors(std::vector<armor_info> &armors, int);
	//void FilterArmors(cv::Mat & src, std::vector<armor_info> &armors);
    void DetectLights(const cv::Mat &src, std::vector<cv::RotatedRect> &lights);  //, double yaw_diff = 0);
	  
	void FilterLights(std::vector<cv::RotatedRect> &lights);   //, double yaw_diff = 0);

    cv::RotatedRect boundingRRect(const cv::RotatedRect & left, const cv::RotatedRect & right);
	cv::RotatedRect boundingRRectFast(const cv::RotatedRect & left, const cv::RotatedRect & right);
	cv::RotatedRect boundingRRectSlow(const cv::RotatedRect & left, const cv::RotatedRect & right);

	void which_armor(const cv::RotatedRect& rect, double& m, double& stddev);
private:
    armor_param _para;	// 装甲板的参数
	cv::Mat src_img_;
	cv::Mat gray_img_;
	
	bool last_detect;
	
	std::vector<cv::RotatedRect> light_rects;
	std::vector<armor_info> filte_rects;
};

void draw_rotated_rect(const cv::Mat &img, const cv::RotatedRect &rect, const cv::Scalar &color, int thickness);

#endif
