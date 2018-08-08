/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2018, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#ifndef ARMOR_DETECT_H_
#define ARMOR_DETECT_H_
#include <iostream>
#include <chrono>
#include <ctype.h>
#include <opencv2/ml.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "detect_factory/armor_info.h"
#include "slover/armor_recorder.hpp"
#include "common/common_serial.h"

using namespace cv;
using namespace cv::ml;

#define POINT_DIST(p1,p2) std::sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y))

namespace autocar
{
namespace vision_mul
{
enum EnemyColor { RED = 0, BLUE = 1};

class ArmorDetector
{
public:
    ArmorDetector() {
		init_armorHist("../config/armor2ev0.jpg", "../config/armor2ev-3.jpg"); // load big armor
		svm_big = StatModel::load<SVM>("../config/big_armor_model.yml");
		svm_small = StatModel::load<SVM>("../config/armor_model.yml");
	};
    void setPara(const param_mul::armor_param & para) {
        _para = para;
    };
	
	void init_armorHist(const std::string& small_armor_pic, 
						const std::string& big_armor_pic)
	{
		cv::Mat armor_Hist = cv::imread(big_armor_pic, cv::IMREAD_GRAYSCALE);
		cv::Mat bigarmor_Hist = cv::imread(small_armor_pic, cv::IMREAD_GRAYSCALE);
		if(bigarmor_Hist.empty())
		{
			std::cout << small_armor_pic << "can not open! check path!" << std::endl; 
		}
		if(armor_Hist.empty())
		{
			std::cout << big_armor_pic << "can not open! check path!" << std::endl; 
		}
		int histSize = 256;  
    	float range[] = { 0, 256 };
    	const float* histRange = { range };
    	int channels[] = {0}; 
    	bool uniform = true; bool accumulate = false; 
		cv::calcHist(&armor_Hist, 1, channels, cv::Mat(), armor_1_hist, 1,&histSize, &histRange, uniform, accumulate); // use 0.7ms
	  	cv::calcHist(&bigarmor_Hist, 1, channels, cv::Mat(), armor_2_hist, 1,&histSize, &histRange, uniform, accumulate); // use 0.7ms
	}
	/**
 	 * @brief: 检测API
	 * @param: camera_src 
	 * @param: vector<armor_info> candidate
	 * @param: Armor_recorder
 	 */
	bool detect(cv::Mat & src, std::vector<armor_info> & armors_candidate);

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
	
	void FilterArmors(std::vector<armor_info> &armors);
	//void FilterArmors(cv::Mat & src, std::vector<armor_info> &armors);
    void DetectLights(const cv::Mat &src, std::vector<cv::RotatedRect> &lights);  //, double yaw_diff = 0);
	  
	void FilterLights(std::vector<cv::RotatedRect> &lights);   //, double yaw_diff = 0);

	/**
	 * @brief: 根据装甲板不同的移动情况框选装甲板
	 */
    cv::RotatedRect boundingRRect(const cv::RotatedRect & left, const cv::RotatedRect & right);
	cv::RotatedRect boundingRRectFast(const cv::RotatedRect & left, const cv::RotatedRect & right);
	cv::RotatedRect boundingRRectSlow(const cv::RotatedRect & left, const cv::RotatedRect & right);
	/**
	 * @brief: filterarmor方法1: 计算灰度和均值
	 * @param: cv::Mat& mask
	 * @param: const cv::RotatedRect& rect
	 * @param: double& mean
	 * @param: double& stddev
	 * no-return;
	 */
	void calu_meanstdev(cv::Mat& mask, const cv::RotatedRect& rect, double& m, double& stddev);
	/**
	 * @brief: filterarmor方法2: 直方图匹配
	 * @param: cv::Mat mask
	 * @param: cv::RotateRect rect
	 * @param: bool visual
	 * @return: 相关度 0 ~ 1
	 */
	double armor_hist_diff(cv::Mat& mask, const cv::RotatedRect& rect, bool Visual);
	/**
	 * @brief: support vector machince for armor
	 * @param: 英雄 wsize(100,25)
	 * @param: 步兵 wsize(60,25)
	 * @return: 预测值 -1 ~ +1
	 */
	float armor_svm(Mat& img_roi);
	float small_armor_svm(Mat& img_roi);

	float get_armor_roi(cv::RotatedRect& rect, bool visual = 0);
private:
    param_mul::armor_param _para;	// 装甲板的参数
	cv::Mat src_img_;
	cv::Mat gray_img_;
	cv::Mat show_lights_before_filter_;
    cv::Mat show_lights_after_filter_;
    cv::Mat show_armors_befor_filter_;
    cv::Mat show_armors_after_filter_;
	std::chrono::steady_clock::time_point speed_test_start_begin_time;
	
	bool last_detect;
	
	std::vector<cv::RotatedRect> light_rects;
	std::vector<armor_info> filte_rects;
	
	// 尝试过对1号步兵和2号英雄进行Hist的匹配分类
	cv::Mat armor_1_hist;  // 1号步兵的 Hist
	cv::Mat armor_2_hist;  // 2号英雄的 Hist

	// SVM效果会好很多，将来可以按照数字贴纸更细化的分类
	Ptr<SVM> svm_big;
	Ptr<SVM> svm_small;
};

int armorToarmorTest(const cv::RotatedRect & _rect1, const cv::RotatedRect & _rect2);

} // namespace vision_mul
} // namespace autocar

#endif
