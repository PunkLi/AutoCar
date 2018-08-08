/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2018, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#include <iostream>
#include <queue>
#include <math.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "detect_factory/armor_detect.hpp"
#include "utility/debug_utility.hpp"
#include "slover/angle_slover.hpp"

// 方便输出一些调试信息
#define LINE(str) // std::cout << "Code Line:" << __LINE__ << "\t" << str << std::endl;
#define LINE_INFO(str,str2) //std::cout << "Code Line:" << __LINE__ << "\t" << str <<":\t"<< str2 << std::endl;

//#define SHOW_DEBUG_IMG  //define了之后就会显示识别处理过程中的中间图像 

namespace autocar
{
namespace vision_mul
{
// 记录时间
std::chrono::steady_clock::time_point speed_test_start_begin_time;

int armorToarmorTest(const cv::RotatedRect & _rect1, const cv::RotatedRect & _rect2)
{
	cv::Point2f center1 = _rect1.center;
	cv::Point2f center2 = _rect2.center;
	cv::Rect rect1 = _rect1.boundingRect();
	cv::Rect rect2 = _rect2.boundingRect();

	if (rect1.x < center2.x && center2.x < rect1.x + rect1.width  && 
		rect2.x < center1.x && center1.x < rect2.x + rect2.width  && 
		rect1.y < center2.y && center2.y < rect1.y + rect1.height &&
		rect2.y < center1.y && center1.y < rect2.y + rect2.height )
		{
			
			if(_rect1.size.area() > _rect2.size.area()) return 1;
			else return 2;
		}
	return -1;
}

bool makeRectSafe(cv::Rect & rect, cv::Size size){
    if (rect.x < 0)
        rect.x = 0;
    if (rect.x + rect.width > size.width)
        rect.width = size.width - rect.x;
    if (rect.y < 0)
        rect.y = 0;
    if (rect.y + rect.height > size.height)
        rect.height = size.height - rect.y;
    if (rect.width <= 0 || rect.height <= 0)
        return false;
    return true;
}

void adjustRect(cv:: RotatedRect &rect)
{
	if(rect.size.width > rect.size.height)
	{
		auto temp = rect.size.height;
		rect.size.height = rect.size.width;
		rect.size.width = temp;
		rect.angle += 90;
		if(rect.angle > 180)
			rect.angle -= 180;
	}
    
	if(rect.angle > 90)
        rect.angle -= 90;
    else if(rect.angle < -90)
        rect.angle += 90;   // 左灯条角度为负, 右灯条角度为正
}
/**
 * @brief: 针对平凡的情况
 */
cv::RotatedRect ArmorDetector::boundingRRect(const cv::RotatedRect & left, const cv::RotatedRect & right){
	const Point & pl = left.center, & pr = right.center;
	Point2f center; 
	center.x = (pl.x + pr.x) / 2.0;
	center.y = (pl.y + pr.y) / 2.0;
	cv::Size2f wh_l = left.size;
	cv::Size2f wh_r = right.size;
	float width = POINT_DIST(pl, pr) - (wh_l.width + wh_r.width) / 2.0;
	float height = std::max(wh_l.height, wh_r.height);
	float angle = std::atan2(right.center.y - left.center.y, right.center.x - left.center.x);
	return RotatedRect(center, Size2f(width, height), angle * 180 / CV_PI);
}
/**
 * @brief: 针对快速平移的情况
 */
cv::RotatedRect ArmorDetector::boundingRRectFast(const cv::RotatedRect & left, const cv::RotatedRect & right){
	const Point & pl = left.center, & pr = right.center;
	Point2f center; 
	center.x = (pl.x + pr.x) / 2.0;
	center.y = (pl.y + pr.y) / 2.0;
	cv::Size2f wh_l = left.size;
	cv::Size2f wh_r = right.size;
	float width = POINT_DIST(pl, pr);// - (wh_l.width + wh_r.width) / 2.0;
	float height = std::max(wh_l.width, wh_r.width);
	float angle = std::atan2(right.center.y - left.center.y, right.center.x - left.center.x);
	return RotatedRect(center, Size2f(width, height), angle * 180 / CV_PI);
}
/**
 * @brief: 针对慢速平移的情况
 */
cv::RotatedRect ArmorDetector::boundingRRectSlow(const cv::RotatedRect & left, const cv::RotatedRect & right){
	const Point & pl = left.center, & pr = right.center;
	Point2f center; 
	center.x = (pl.x + pr.x) / 2.0;
	center.y = (pl.y + pr.y) / 2.0;
	cv::Size2f wh_l = left.size;
	cv::Size2f wh_r = right.size;
	float width = POINT_DIST(pl, pr);// - (wh_l.width + wh_r.width) / 2.0;
	float height = std::max(wh_l.height, wh_r.height);
	float angle = std::atan2(right.center.y - left.center.y, right.center.x - left.center.x);
	return RotatedRect(center, Size2f(width, height), angle * 180 / CV_PI);
}

void ArmorDetector::DetectLights(const cv::Mat &src, std::vector<cv::RotatedRect> &lights) {
	// speed_test_reset();
#ifdef SHOW_DEBUG_IMG	
  	show_lights_before_filter_ = cv::Mat::zeros(src.size(), CV_8UC3);
  	show_lights_after_filter_ = cv::Mat::zeros(src.size(), CV_8UC3);
  	show_armors_befor_filter_ = src.clone();
  	show_armors_after_filter_ = src.clone();
#endif
  	// auto color_light = DistillationColor(src, _para.enemy_color);
	cv::Mat color_light;
	std::vector<cv::Mat> bgr_channel;
	cv::split(src, bgr_channel);
	
	if (_para.enemy_color == RED)
		cv::subtract(bgr_channel[2], bgr_channel[1], color_light);
	else
		cv::subtract(bgr_channel[0], bgr_channel[1], color_light);

  	cv::Mat binary_brightness_img; // 亮度二值化
  	cv::Mat binary_color_img;      // 颜色二值化
  	cv::Mat binary_light_img;      // &
  	
  	cv::cvtColor(src, gray_img_, cv::ColorConversionCodes::COLOR_BGR2GRAY);
  	// TODO(noah.guo): param
	// cv::adaptiveThreshold(gray_img_, binary_brightness_img, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY,7,5);
    // 环境亮度是显著因素之一
  	cv::threshold(gray_img_, binary_brightness_img, _para.light_threshold_val, 255, CV_THRESH_BINARY);  //200
  	//TODO(noah.guo): param
  	float thresh;
 	if (_para.enemy_color == BLUE) // 这里对快速移动依然有影响
   	 	thresh = _para.blue_color_diff;
  	else
    	thresh = _para.red_color_diff;  //50
	//  cv::adaptiveThreshold(color_light, binary_color_img, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 7, 5);
  	cv::threshold(color_light, binary_color_img, thresh, 255, CV_THRESH_BINARY);
    // 这里的形态学需要考虑一下,尤其是装甲板快速移动时
  	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  	cv::dilate(binary_color_img, binary_color_img, element, cv::Point(-1, -1), 1);
    //cv::morphologyEx(binary_color_img,binary_color_img, MORPH_OPEN, element);
  	binary_light_img = binary_color_img & binary_brightness_img;

  	// auto contours_light = FindContours(binary_light_img);
	std::vector<std::vector<cv::Point>> contours_light;
	cv::findContours(binary_light_img, contours_light, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	// auto contours_brightness = FindContours(binary_brightness_img);
	std::vector<std::vector<cv::Point>> contours_brightness;
	cv::findContours(binary_brightness_img, contours_brightness, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

  	lights.reserve(contours_brightness.size());
  	// TODO: To be optimized
  	std::vector<int> is_processes(contours_brightness.size());
  	for (unsigned int i = 0; i < contours_light.size(); ++i) {
    	for (unsigned int j = 0; j < contours_brightness.size(); ++j) {
      		if (!is_processes[j]) {
        		if (cv::pointPolygonTest(contours_brightness[j], contours_light[i][0], true) >= 0.0) {
          			cv::RotatedRect single_light = cv::minAreaRect(contours_brightness[j]);
          			lights.push_back(single_light);
#ifdef SHOW_DEBUG_IMG
            		DrawRotatedRect(show_lights_before_filter_, single_light, cv::Scalar(0,255,0), 1);
#endif
          			is_processes[j] = true;
          			break;
        		}
      		}
    	} // for j loop
  	} // for i loop
#ifdef SHOW_DEBUG_IMG
	cv::imshow("gray_img", gray_img_);
    cv::imshow("binary_color_img", binary_color_img);
    cv::imshow("binary_light_img", binary_light_img);
	cv::imshow("binary_brightness_img", binary_brightness_img);
    cv::imshow("lights_before_filter", show_lights_before_filter_);
#endif
	// speed_test_end("DetectLights 用时 = ", "ms");
}

void ArmorDetector::FilterLights(std::vector<cv::RotatedRect> &lights) 
{
	// speed_test_reset();
	light_rects.clear();
#pragma omp parallel for 
  	for(uchar i = 0; i < lights.size(); i++ ){
	  	adjustRect(lights[i]);
  	}
	
  	for (const auto &armor_light : lights)
	{
        auto rect = std::minmax(armor_light.size.width, armor_light.size.height);
    	auto light_aspect_ratio = rect.second / rect.first;
        auto angle = armor_light.angle;
       
		if(//80 <= abs(angle) && abs(angle) <= 90   // 高速水平移动的灯条,带有拖影  // 特殊情况,无论横竖, 旧版本有这一行代码
		    light_aspect_ratio <= 2.5
		   && armor_light.size.area() >= _para.light_min_area // 1.0
		   && armor_light.size.area() < 100000)  //_para.light_max_area * src_img_.size().height * src_img_.size().width) // 0.04
		{
			light_rects.push_back(armor_light); // 高速水平移动的灯条
#ifdef SHOW_DEBUG_IMG
			DrawRotatedRect(show_lights_after_filter_, armor_light, cv::Scalar(255,0,0), 1);
#endif
		}
        // 针对灯条细小的情况, 没有最大比例的判断, 较为理想的灯条
		else if(armor_light.size.area() >= _para.light_min_area // 1.0
		   		&& armor_light.size.area() < 100000  //_para.light_max_area * src_img_.size().height * src_img_.size().width // 0.04
		   		&& abs(angle) < _para.light_max_angle) // 与垂直的偏角17.5 , 这里是可以取消/2的,进一步细化
		{
			light_rects.push_back(armor_light); // 接近于垂直的灯条, 由于阈值不够合理, 细小的灯条
#ifdef SHOW_DEBUG_IMG
			DrawRotatedRect(show_lights_after_filter_, armor_light, cv::Scalar(0,255,0), 1);
#endif
	    }
        // 检测最为平凡的情况
    	else if (//light_aspect_ratio < _para.light_max_aspect_ratio  // 6.8
                 armor_light.size.area() >= _para.light_min_area // 1.0
			     && armor_light.size.area() < _para.light_max_area * src_img_.size().height * src_img_.size().width // 0.04
			     && abs(angle) < _para.light_max_angle) // 与垂直的偏角35 
        {
      		light_rects.push_back(armor_light);
#ifdef SHOW_DEBUG_IMG
			DrawRotatedRect(show_lights_after_filter_, armor_light, cv::Scalar(0,0,255), 1);
#endif
		}
  	}
#ifdef SHOW_DEBUG_IMG
		cv::imshow("lights_after_filter", show_lights_after_filter_);
#endif
		lights = light_rects;
		// speed_test_end("FilterLights 用时 = ", "ms");
}

/**
 * @brief: 多装甲板检测
 */
void ArmorDetector::choose_target_from_lights(std::vector<cv::RotatedRect> &lights, std::vector<armor_info> &armor_vector)
{
	// speed_test_reset();

	for (int i = 0; i < lights.size(); ++i)
	{
    	for (int j = i; j < lights.size(); ++j) 
		{
			auto rect1 = std::minmax(lights[i].size.width, lights[i].size.height);
    		auto light_aspect_ratio1 = rect1.second / rect1.first;
			auto rect2 = std::minmax(lights[j].size.width, lights[j].size.height);
    		auto light_aspect_ratio2 = rect2.second / rect2.first;

			auto angle_diff = abs(lights[i].angle - lights[j].angle);
			auto height_diff = abs(lights[i].size.height - lights[j].size.height) / std::max(lights[i].size.height, lights[j].size.height);
			auto width_diff = abs(lights[i].size.width - lights[j].size.width) / std::max(lights[i].size.width, lights[j].size.width);

// 快速平移的情况 Fast Move
	// 2个严格平行
			if (1 < light_aspect_ratio1 && light_aspect_ratio1 <= 2.5
			    && 1 < light_aspect_ratio2 && light_aspect_ratio2 <= 2.5
			    && abs(lights[i].angle) == 90 && abs(lights[j].angle) == 90     // 角度为0
			    && static_cast<int>(abs(angle_diff)) % 180 == 0
			    && height_diff < 0.5 && width_diff < 0.5)	
			{
				cv::RotatedRect possible_rect;
				if (lights[i].center.x < lights[j].center.x)
					possible_rect = boundingRRectFast(lights[i],lights[j]);
				else
					possible_rect = boundingRRectFast(lights[j],lights[i]);
				
				auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
				auto armor_angle = possible_rect.angle;
				auto armor_area = possible_rect.size.area();
				// auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

				if (armor_area > _para.armor_min_area 
					&& armor_ratio < 4.5                // 经验参数
					&& abs(armor_angle) < 20 )          // 经验参数
				{	
					Armor_Twist armor_twist = FAST_MOVE;
					LINE("[快速移动] armor")
					armor_info armor(possible_rect, armor_twist);
					armor_vector.push_back(armor);
				} // get_armor
			}// 2个严格平行
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// 2个当中有一个略不平行
			else if (1 < light_aspect_ratio1 && light_aspect_ratio1 <= 2.5
			     	 && 1 < light_aspect_ratio2 && light_aspect_ratio2 <= 2.5
			    	 && ( (abs(lights[i].angle) == 90 && abs(lights[j].angle) > 80) || (abs(lights[i].angle) > 80 && abs(lights[j].angle) == 90))
			    	 && height_diff < 0.5 && width_diff < 0.5)		
			{
				cv::RotatedRect possible_rect;
				if (lights[i].center.x < lights[j].center.x)
					possible_rect = boundingRRectFast(lights[i],lights[j]);
				else
					possible_rect = boundingRRectFast(lights[j],lights[i]);
				
				auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
				auto armor_angle = possible_rect.angle;
				auto armor_area = possible_rect.size.area();
				
				// auto armor_light_angle_diff = abs(lights[i].angle) == 90 ? 
				//						 	     abs(armor_angle) + abs(armor_angle - lights[j].angle - 90); // 左右灯条的积累差

				if (armor_area > _para.armor_min_area
				    && armor_ratio < 4.5                // 经验参数
					&& abs(armor_angle) < 20 )          // 经验参数
				{	
					Armor_Twist armor_twist = FAST_MOVE;
					LINE("[快速移动] armor")
					armor_info armor(possible_rect, armor_twist);
					armor_vector.push_back(armor);
				} // get_armor
			}// 2个当中有一个略不平行 
// 快速平移的情况 Fast Move
// 中速平移的情况 Mid Move
		// 2个严格平行
			else if ( ( (light_aspect_ratio1 == 1 && light_aspect_ratio2 <= 1.5) 
					 || (light_aspect_ratio1 <= 1.5 && light_aspect_ratio2 == 1) )   // 其中一个为正方形
					 && static_cast<int>(abs(angle_diff)) % 90 == 0               	 // 角度差为0
					 && static_cast<int>(abs(lights[i].angle)) % 90 == 0          	 // 角度为0 或 90
					 && static_cast<int>(abs(lights[j].angle)) % 90 == 0          	 // 角度为0 或 90
					 && height_diff < 0.5 && width_diff < 0.5)               	     // 形状差异
			{
				cv::RotatedRect possible_rect;
				if (lights[i].center.x < lights[j].center.x)
					possible_rect = boundingRRectFast(lights[i], lights[j]);
				else
					possible_rect = boundingRRectFast(lights[j], lights[i]);

				auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
				auto armor_angle = possible_rect.angle;
				auto armor_area = possible_rect.size.area();
				// auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

				if (armor_area > _para.armor_min_area	    // 经验参数
					&& armor_ratio < 4                      // 经验参数 （步兵应该只有3, 英雄可能会到5）
					&& abs(armor_angle) <  20 )             // 经验参数
					{
						Armor_Twist armor_twist = MID_MOVE;
						LINE("[中等速度] armor")
						armor_info armor(possible_rect, armor_twist);
						armor_vector.push_back(armor);
					} // get_armor
			}
			// 2个严格平行

			// 1个竖着 1个横着
			else if (1 < light_aspect_ratio1 && light_aspect_ratio1 < 1.3
					 && 1 < light_aspect_ratio2 && light_aspect_ratio2 < 1.3
					 && static_cast<int>(abs(angle_diff)) % 180 == 90            // 角度差为0
					 && ((abs(lights[i].angle) == 0 && abs(lights[j].angle) == 90) || (abs(lights[i].angle) == 90 && abs(lights[j].angle) == 0))  // 角度1个为0 1个为90
					 && height_diff < 0.5 && width_diff < 0.5)               	  // 形状差异
			{
				cv::RotatedRect possible_rect;
				if (lights[i].center.x < lights[j].center.x)
					possible_rect = boundingRRectFast(lights[i], lights[j]);
				else
					possible_rect = boundingRRectFast(lights[j], lights[i]);

				auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
				auto armor_angle = possible_rect.angle;
				auto armor_area = possible_rect.size.area();
				// auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

				if (armor_area > _para.armor_min_area			// 经验参数
					&& armor_ratio < 4                          // 经验参数 （步兵应该只有3, 英雄可能会到5）
					&& abs(armor_angle) <  20 )                 // 经验参数
					{
						Armor_Twist armor_twist = MID_MOVE;
						LINE("[中等速度] armor")
						armor_info armor(possible_rect, armor_twist);
						armor_vector.push_back(armor);
					} // get_armor
			}// 1个竖着 1个横着
// 中速平移的情况 Mid Move
// 慢速移动的情况 Low Move
		// 都是竖着的
			else if (1 < light_aspect_ratio1 && light_aspect_ratio1 < 1.5
					 && 1 < light_aspect_ratio2 && light_aspect_ratio2 < 1.5
					 && static_cast<int>(abs(angle_diff)) % 180 == 0                 // 角度差为0
					 && abs(lights[i].angle) == 0 && abs(lights[j].angle) == 0       // 角度为0 或 180
					 && height_diff < 0.5 && width_diff < 0.5)                  	 // 形状差异
			{
				cv::RotatedRect possible_rect;
				if (lights[i].center.x < lights[j].center.x)
					possible_rect = boundingRRectSlow(lights[i], lights[j]);
				else
					possible_rect = boundingRRectSlow(lights[j], lights[i]);

				auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
				auto armor_angle = possible_rect.angle;
				auto armor_area = possible_rect.size.area();
				// auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

				if (armor_area > _para.armor_min_area	 // 经验参数
					&& armor_ratio < 4                   // 经验参数 （步兵应该只有3, 英雄可能会到5）
					&& abs(armor_angle) <  20 )          // 经验参数
					{
						Armor_Twist armor_twist = LOW_MOVE;
						LINE("[缓慢移动] armor")
						armor_info armor(possible_rect, armor_twist);
						armor_vector.push_back(armor);
					} // get_armor
			}
		// 都是竖着的

		// 其中一块略有倾斜
			else if (1 < light_aspect_ratio1 && light_aspect_ratio1 < 1.5
					 && 1 < light_aspect_ratio2 && light_aspect_ratio2 < 1.5
					 &&	((abs(lights[i].angle) == 0  && abs(lights[j].angle) < 10) || (abs(lights[i].angle) < 10  && abs(lights[j].angle) == 0)) // 角度为0 或 180
					 && height_diff < 0.5 && width_diff < 0.5)                  	 // 形状差异
			{
				cv::RotatedRect possible_rect;
				if (lights[i].center.x < lights[j].center.x)
					possible_rect = boundingRRectFast(lights[i], lights[j]);
				else
					possible_rect = boundingRRectFast(lights[j], lights[i]);

				auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
				auto armor_angle = possible_rect.angle;
				auto armor_area = possible_rect.size.area();
				auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

				if (armor_area > _para.armor_min_area			// 经验参数
					&& armor_ratio < 4                          // 经验参数 （步兵应该只有3, 英雄可能会到5）
					&& abs(armor_angle) <  20                   // 经验参数
					&& armor_light_angle_diff < 20 )
					{
						Armor_Twist armor_twist = LOW_MOVE;
						LINE("[缓慢移动] armor")
						armor_info armor(possible_rect, armor_twist);
						armor_vector.push_back(armor);
					} // get_armor
			}
		// 其中一块略有倾斜

// 慢速移动的情况 Low Move

// 平凡的情况
	// 灯条近乎平行,至少在同一侧
			else if (lights[i].angle * lights[j].angle >= 0              // 灯条近乎同侧 , 或者有一个为0
				     && abs(angle_diff) < 30                             //  _para.light_max_angle_diff   // 20   // 18   这些都要换成相对值
					// && height_diff < _para.light_max_height_diff      // 20  不需要宽度
					)
			{
				cv::RotatedRect possible_rect;
				// 2灯条近乎平行 中速移动
				if (1 < light_aspect_ratio1 && light_aspect_ratio1 <= 1.5
					&& 1 < light_aspect_ratio2 && light_aspect_ratio2 <= 1.5
					&& abs(light_aspect_ratio1 - light_aspect_ratio2) < 0.5 )
				{
					// 2灯条近乎平行 中速移动
					if (abs(lights[i].angle) > 60 && abs(lights[j].angle) > 60)
					{
						if (lights[i].center.x < lights[j].center.x)
							possible_rect = boundingRRectFast(lights[i], lights[j]);
						else
				    		possible_rect = boundingRRectFast(lights[j], lights[i]);

						auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
						auto armor_angle = possible_rect.angle;
						auto armor_area = possible_rect.size.area();
						auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

						if (armor_area > _para.armor_min_area
							&& armor_ratio > 1                                       // _para.small_armor_min_ratio   // 1.5
				   			&& armor_ratio < _para.armor_max_ratio                   // 3.0
				   			&& abs(armor_angle) < _para.armor_max_angle
				   			&& armor_light_angle_diff < _para.armor_light_angle_diff // 应该要更为严格
						){
							Armor_Twist armor_twist = MID_MOVE;
							LINE("armor同侧,这是中速移动的armor,1-1.5,平躺")
							armor_info armor(possible_rect, armor_twist);
							armor_vector.push_back(armor);
						}
					}// 2灯条近乎平行 中速移动

					// 2灯条近乎平行 慢速移动
					else if (abs(lights[i].angle) < 30 && abs(lights[j].angle) < 30)
					{
						if (lights[i].center.x < lights[j].center.x)
							possible_rect = boundingRRectSlow(lights[i], lights[j]);
						else
				    		possible_rect = boundingRRectSlow(lights[j], lights[i]);

						auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
						auto armor_angle = possible_rect.angle;
						auto armor_area = possible_rect.size.area();
						auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

						if (armor_area > _para.armor_min_area
							&& armor_ratio > 1                                       // _para.small_armor_min_ratio   // 1.5
				   			&& armor_ratio < _para.armor_max_ratio                   // 3.0
				   			&& abs(armor_angle) < _para.armor_max_angle
				   			&& armor_light_angle_diff < _para.armor_light_angle_diff // 应该要更为严格
						){
							Armor_Twist armor_twist = LOW_MOVE;
							LINE("[缓慢移动] armor")
							armor_info armor(possible_rect, armor_twist);
							armor_vector.push_back(armor);
						}
					}// 2灯条近乎平行 慢速移动
				}// 2灯条近乎平行 中速移动

				// 2灯条近乎平行 快速移动
				else if (1 < light_aspect_ratio1 && light_aspect_ratio1 < 2.5
						 && 1 < light_aspect_ratio2 && light_aspect_ratio2 < 2.5
						 && abs(light_aspect_ratio1 - light_aspect_ratio2) < 1)
				{
					// 2灯条近乎平行 快速移动
					if (abs(lights[i].angle) > 60 && abs(lights[j].angle) > 60)
					{
						if (lights[i].center.x < lights[j].center.x)
							possible_rect = boundingRRectFast(lights[i], lights[j]);
						else
				    		possible_rect = boundingRRectFast(lights[j], lights[i]);

						auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
						auto armor_angle = possible_rect.angle;
						auto armor_area = possible_rect.size.area();
						
						if (armor_area > _para.armor_min_area
							&& armor_ratio > 1                       // _para.small_armor_min_ratio   // 1.5
				   			&& armor_ratio < _para.armor_max_ratio   // 3.0
				   			&& abs(armor_angle) < _para.armor_max_angle
						){
							Armor_Twist armor_twist = FAST_MOVE;
							LINE("[快速移动] armor")
							armor_info armor(possible_rect, armor_twist);
							armor_vector.push_back(armor);
						}
					} // 2灯条近乎平行 快速移动

					// 2灯条近乎平行 慢速移动
					else if (abs(lights[i].angle) < 30 && abs(lights[j].angle) < 30)
					{
						if (lights[i].center.x < lights[j].center.x)
							possible_rect = boundingRRectSlow(lights[i], lights[j]);
						else
				    		possible_rect = boundingRRectSlow(lights[j], lights[i]);

						auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
						auto armor_angle = possible_rect.angle;
						auto armor_area = possible_rect.size.area();
						auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

						if (armor_area > _para.armor_min_area
							&& armor_ratio > 1                                       // _para.small_armor_min_ratio   // 1.5
				   			&& armor_ratio < _para.armor_max_ratio                   // 3.0
				   			&& abs(armor_angle) < _para.armor_max_angle
				   			&& armor_light_angle_diff < _para.armor_light_angle_diff // 应该要更为严格
						){
							Armor_Twist armor_twist = LOW_MOVE;
							LINE("[缓慢移动] armor")
							armor_info armor(possible_rect, armor_twist);
							armor_vector.push_back(armor);
						}
					}// 2灯条近乎平行 慢速移动
				}// 2灯条近乎平行 快速移动

				else if (_para.light_min_aspect_ratio < light_aspect_ratio1    // && light_aspect_ratio1 < _para.light_max_aspect_ratio
						 && _para.light_min_aspect_ratio < light_aspect_ratio2 // && light_aspect_ratio2 < _para.light_max_aspect_ratio
						 && (lights[i].center.y + lights[i].size.height / 2) > (lights[j].center.y - lights[j].size.height / 2)
						 && (lights[j].center.y + lights[j].size.height / 2) > (lights[i].center.y - lights[i].size.height / 2)
						 && abs(lights[i].angle) < 30 && abs(lights[j].angle) < 30)
				{
					if (lights[i].center.x < lights[j].center.x)
						possible_rect = boundingRRect(lights[i], lights[j]);
					else
				    	possible_rect = boundingRRect(lights[j], lights[i]);

					auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
					auto armor_angle = possible_rect.angle;
					auto armor_area = possible_rect.size.area();
					auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

					if (armor_area > _para.armor_min_area
						&& armor_ratio > 1                                         // _para.small_armor_min_ratio   // 1.5
				   		&& armor_ratio < 4.5                                       // _para.armor_max_ratio   // 3.0
				   		&& abs(armor_angle) < _para.armor_max_angle
				   		&& armor_light_angle_diff < _para.armor_light_angle_diff ) // 应该要更为严格
					{
						LINE_INFO("angle_1",lights[i].angle)
						LINE_INFO("angle_2",lights[j].angle)
						LINE_INFO("angle_diff",angle_diff)
						LINE_INFO("height_diff", height_diff)
						LINE_INFO("width_diff",width_diff)
						LINE_INFO("armor_ratio",armor_ratio)
						LINE_INFO("armor_angle",armor_angle)
						LINE_INFO("armor_light_angle_diff",armor_light_angle_diff)
						
						Armor_Twist armor_twist = STILL;
						LINE("[几乎静止] armor")
						armor_info armor(possible_rect, armor_twist);
						armor_vector.push_back(armor);
					}
				}
			} // 灯条严格平行
			
// 灯条(误差) 并不同侧
			else if (abs(angle_diff) < _para.light_max_angle_diff )     // 40
			{
				cv::RotatedRect possible_rect;
				// 2灯条 中速移动
				if (1 < light_aspect_ratio1 && light_aspect_ratio1 <= 1.5
					&& 1 < light_aspect_ratio2 && light_aspect_ratio2 <= 1.5
					&& abs(light_aspect_ratio1 - light_aspect_ratio2) < 0.5 )
				{
					// 2灯条不平行 慢速移动
					if (abs(lights[i].angle) < 30 && abs(lights[j].angle) < 30)
					{
						if (lights[i].center.x < lights[j].center.x)
							possible_rect = boundingRRectSlow(lights[i], lights[j]);
						else
				    		possible_rect = boundingRRectSlow(lights[j], lights[i]);

						auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
						auto armor_angle = possible_rect.angle;
						auto armor_area = possible_rect.size.area();
						auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

						if (armor_area > _para.armor_min_area
							&& armor_ratio > 1  // _para.small_armor_min_ratio   // 1.5
				   			&& armor_ratio < _para.armor_max_ratio   // 3.0
				   			&& abs(armor_angle) < _para.armor_max_angle
				   			&& armor_light_angle_diff < _para.armor_light_angle_diff // 应该要更为严格
						)
						{
							Armor_Twist armor_twist = MID_MOVE;
							LINE("armor不同侧, 中速的armor, 竖直")
							armor_info armor(possible_rect, armor_twist);
							armor_vector.push_back(armor);
						}
					}// 2灯条不平行 慢速移动
				}// 2灯条 中速移动

				// 2灯条近乎平行 快速移动
				else if (1 < light_aspect_ratio1 && light_aspect_ratio1 < 2.5
						 && 1 < light_aspect_ratio2 && light_aspect_ratio2 < 2.5
						 && abs(light_aspect_ratio1 - light_aspect_ratio2) < 1)
				{
					if (abs(lights[i].angle) < 30 && abs(lights[j].angle) < 30)
					{
						if (lights[i].center.x < lights[j].center.x)
							possible_rect = boundingRRectSlow(lights[i], lights[j]);
						else
				    		possible_rect = boundingRRectSlow(lights[j], lights[i]);

						auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
						auto armor_angle = possible_rect.angle;
						auto armor_area = possible_rect.size.area();
						auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

						if (armor_area > _para.armor_min_area
							&& armor_ratio > 1 // _para.small_armor_min_ratio   // 1.5
				   			&& armor_ratio < _para.armor_max_ratio   // 3.0
				   			&& abs(armor_angle) < _para.armor_max_angle
				   			&& armor_light_angle_diff < _para.armor_light_angle_diff )// 应该要更为严格
						{
							Armor_Twist armor_twist = LOW_MOVE;
							LINE("[缓慢移动] armor")
							armor_info armor(possible_rect, armor_twist);
							armor_vector.push_back(armor);
						}
					}// 2灯条近乎平行 慢速移动
				}// 2灯条近乎平行 快速移动

				else if (_para.light_min_aspect_ratio < light_aspect_ratio1 //&& light_aspect_ratio1 < _para.light_max_aspect_ratio
						 && _para.light_min_aspect_ratio < light_aspect_ratio2 //&& light_aspect_ratio2 < _para.light_max_aspect_ratio
						 && (lights[i].center.y + lights[i].size.height / 2) > (lights[j].center.y - lights[j].size.height / 2)
						 && (lights[j].center.y + lights[j].size.height / 2) > (lights[i].center.y - lights[i].size.height / 2))
				{
					if (lights[i].center.x < lights[j].center.x)
						possible_rect = boundingRRect(lights[i], lights[j]);
					else
				    	possible_rect = boundingRRect(lights[j], lights[i]);

					auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
					auto armor_angle = possible_rect.angle;
					auto armor_area = possible_rect.size.area();
					auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

					if (armor_area > _para.armor_min_area
						&& armor_ratio > 1 // _para.small_armor_min_ratio   // 1.5
				   		&& armor_ratio < 4.5 // _para.armor_max_ratio   // 3.0
				   		&& abs(armor_angle) < _para.armor_max_angle
				   		&& armor_light_angle_diff < _para.armor_light_angle_diff) // 应该要更为严格
					{
						LINE_INFO("angle_1",lights[i].angle)
						LINE_INFO("angle_2",lights[j].angle)
						LINE_INFO("angle_diff",angle_diff)
						LINE_INFO("height_diff", height_diff)
						LINE_INFO("width_diff",width_diff)
						LINE_INFO("armor_ratio",armor_ratio)
						LINE_INFO("armor_angle",armor_angle)
						LINE_INFO("armor_light_angle_diff",armor_light_angle_diff)
						Armor_Twist armor_twist = STILL;
						LINE("[几乎静止] armor")
						armor_info armor(possible_rect, armor_twist);
						armor_vector.push_back(armor);
					}
				}
			} // 灯条(误差) 并不同侧
			else {
				cv::RotatedRect possible_rect;
				// 2灯条不平行 中速移动
				if (1 < light_aspect_ratio1 && light_aspect_ratio1 < 1.5
					&& 1 < light_aspect_ratio2 && light_aspect_ratio2 < 1.5
					&& abs(light_aspect_ratio1 - light_aspect_ratio2) < 0.5 )
				{
					// 2灯条不平行 中速移动
					if (abs(lights[i].angle) > 60 &&  + abs(lights[j].angle) > 60)
					{
						if (lights[i].center.x < lights[j].center.x)
							possible_rect = boundingRRectFast(lights[i], lights[j]);
						else
				    		possible_rect = boundingRRectFast(lights[j], lights[i]);

						auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
						auto armor_angle = possible_rect.angle;
						auto armor_area = possible_rect.size.area();
						//auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

						if (armor_area > _para.armor_min_area
							&& armor_ratio > 1 // _para.small_armor_min_ratio   // 1.5
				   			&& armor_ratio < _para.armor_max_ratio   // 3.0
				   			&& abs(armor_angle) < _para.armor_max_angle
				   			// && armor_light_angle_diff < _para.armor_light_angle_diff // 应该要更为严格
						){
							Armor_Twist armor_twist = MID_MOVE;
							LINE("[中速运动] armor")
							armor_info armor(possible_rect, armor_twist);
							armor_vector.push_back(armor);
						}
					}// 2灯条不平行 中速移动
				}
				else if (1 < light_aspect_ratio1 && light_aspect_ratio1 < 2.5
						 && 1 < light_aspect_ratio2 && light_aspect_ratio2 < 2.5
						 && abs(light_aspect_ratio1 - light_aspect_ratio2) < 1)
				{
					if (abs(lights[i].angle) > 60 && abs(lights[j].angle) > 60)
					{
						if (lights[i].center.x < lights[j].center.x)
							possible_rect = boundingRRectSlow(lights[i], lights[j]);
						else
				    		possible_rect = boundingRRectSlow(lights[j], lights[i]);

						auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
						auto armor_angle = possible_rect.angle;
						auto armor_area = possible_rect.size.area();
						auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

						if (armor_area > _para.armor_min_area
							&& armor_ratio > 1 // _para.small_armor_min_ratio   // 1.5
				   			&& armor_ratio < _para.armor_max_ratio   // 3.0
				   			&& abs(armor_angle) < _para.armor_max_angle
				   			&& armor_light_angle_diff < _para.armor_light_angle_diff // 应该要更为严格
						){
							Armor_Twist armor_twist = LOW_MOVE;
							LINE("[慢速运动] armor")
							armor_info armor(possible_rect, armor_twist);
							armor_vector.push_back(armor);
						}
					}// 2灯条近乎平行 慢速移动
				}// 2灯条近乎平行 快速移动
			}
				
		} // for j loop
	} // for i loop

	// speed_test_end("choose_armor_from_light 用时 = ", "ms");
}

void ArmorDetector::calu_meanstdev(cv::Mat& mask, const cv::RotatedRect& rect, double& m, double& stddev)
{
	// speed_test_reset();
	cv::Point pts[4];
	cv::Point2f temp[4];
	rect.points(temp);
	for (unsigned int j = 0; j < 4; j++) 
	{
		pts[j].x = (int) temp[j].x;
		pts[j].y = (int) temp[j].y;
	}
	cv::fillConvexPoly(mask, pts, 4, cv::Scalar(255), 8, 0);  // 多边形匹配

	cv::Mat mat_mean;
	cv::Mat mat_stddev;
	cv::meanStdDev(gray_img_, mat_mean, mat_stddev, mask);
	m = mat_mean.at<double>(0,0);
	stddev = mat_stddev.at<double>(0, 0);
	// speed_test_end("ArmorDetector::calu_meanstdev 用时 = ", "ms");

}

double ArmorDetector::armor_hist_diff(cv::Mat& mask, const cv::RotatedRect& rect, bool Visual)
{
	// speed_test_reset();
	cv::Point pts[4];
	cv::Point2f temp[4];
	rect.points(temp);
   	for (unsigned int j = 0; j < 4; j++) 
	{
    	pts[j].x = (int) temp[j].x;
    	pts[j].y = (int) temp[j].y;
    }
    cv::fillConvexPoly(mask, pts, 4, cv::Scalar(255), 8, 0);   // 多边形匹配

	int histSize = 256;  
    float range[] = { 0, 256 };  
    const float* histRange = { range };  
    int channels[] = {0};  // 针对灰度图就可以了,RGB图虽然更加准确,但是不划算  
    bool uniform = true; bool accumulate = false; 
	cv::Mat hist;
	cv::calcHist(&gray_img_, 1, channels, mask, hist, 1,&histSize, &histRange, uniform, accumulate); // use 0.7ms
	if(Visual) // 可视化
	{
		int hist_w = gray_img_.cols, hist_h = gray_img_.rows;  
		int bin_w = cvRound((double)hist_w / histSize);  
	
		Mat histImage(hist_w, hist_h, CV_8UC3, Scalar(0, 0, 0));  
	
		//直方图归一化范围[0，histImage.rows]  
		normalize(hist, hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());  
	
		for (int i = 1; i < histSize; ++i)  
		{  
			//cvRound：类型转换。 这里hist为256*1的一维矩阵，存储的是图像中各个灰度级的归一化值  
			line(histImage, Point(bin_w*(i - 1), hist_h - cvRound(hist.at<float>(i - 1))),  
				Point(bin_w*(i), hist_h - cvRound(hist.at<float>(i))),  
				Scalar(0, 0, 255), 2, 8, 0);  
		}  
		imshow("histImage", histImage); 
	}
	double val = cv::compareHist( armor_2_hist, hist, HISTCMP_CORREL);
	// speed_test_end("ArmorDetector::armor_hist_diff 用时 = ", "ms");
	return val;	
}

float ArmorDetector::small_armor_svm(Mat& img_roi)
{
	// speed_test_reset();
    cv::Mat gradient_lst;
    HOGDescriptor hog;
	cv::Size wsize =  cv::Size(60,25);
	cv::resize(img_roi, img_roi, cv::Size(60,25));
    hog.winSize = wsize / 8 * 8;
    std::vector< float > descriptors;
    
    Rect roi = Rect((img_roi.cols - wsize.width ) / 2,
                    (img_roi.rows - wsize.height ) / 2,
                   	 wsize.width,
                  	 wsize.height);
    hog.compute( img_roi(roi), descriptors, Size(8,8), Size(0,0));   
	float response = svm_small->predict(descriptors);
	// speed_test_end("armor_svm 用时 = ", "ms");
	return response;
}

float ArmorDetector::armor_svm(Mat& img_roi)
{
	// speed_test_reset();
    cv::Mat gradient_lst;
    HOGDescriptor hog;
	cv::Size wsize =  cv::Size(100,25);
	cv::resize(img_roi, img_roi, cv::Size(100,25));
    hog.winSize = wsize / 8 * 8;
    std::vector< float > descriptors;
    
    Rect roi = Rect((img_roi.cols - wsize.width ) / 2,
                    (img_roi.rows - wsize.height ) / 2,
                   	 wsize.width,
                  	 wsize.height);
    hog.compute( img_roi(roi), descriptors, Size(8,8), Size(0,0));   
	float response = svm_big->predict(descriptors);
	// speed_test_end("armor_svm 用时 = ", "ms");
	return response;
}

float ArmorDetector::get_armor_roi(cv::RotatedRect& rect, bool visual)
{
	auto center = rect.center;
	cv::Mat rot_mat = cv::getRotationMatrix2D(rect.center, rect.angle, 1); 
	cv::Mat img;
	warpAffine(gray_img_, img, rot_mat, img.size(), INTER_LINEAR, BORDER_CONSTANT); // warpAffine use 2ms
	// cv::imshow("warpaffine", img);
	cv::Rect roi = cv::Rect(center.x - (rect.size.width / 2), 
						    center.y - (rect.size.height / 2), 
					  	    rect.size.width, rect.size.height);
	if (makeRectSafe(roi, img.size()) == true)
	{
		cv::Mat armor_roi = img(roi);
		if(visual) cv::imshow("armor_roi", armor_roi);

		float wh = roi.width, gh = roi.height;
		float ratio = wh/gh;

		// 这里简单地根据装甲片的长宽比例判定了大小装甲，然后最终用的是SVM分类器
		float val;
		if(ratio < 3.5)
			val = small_armor_svm(armor_roi);
		else
			val = armor_svm(armor_roi);
		// std::cout << "svm lebel:" << val << std::endl;
		return val;
	}
	else
		return 0;
}

void ArmorDetector::FilterArmors(std::vector<armor_info> &armors) 
{
  	// speed_test_reset();
	std::vector<bool> is_armor(armors.size(), true);

	for (int i = 0; i < armors.size(); i++) {
	  	for (int j = i + 1; j < armors.size(); j++) {
			int ojbk = armorToarmorTest(armors[i].rect, armors[j].rect);
			if(ojbk == 1) is_armor[i] = false;
			else if(ojbk == 2) is_armor[j] = false;
		}
	}

	double dis;
#pragma omp parallel
  	for (int i = 0; i < armors.size(); i++) {
#pragma omp for
	  	for (int j = i + 1; j < armors.size(); j++) //  && (is_armor[j] == true)
		{
			dis = POINT_DIST(armors.at(i).rect.center,armors.at(j).rect.center);
		
			if (dis <= std::max(armors.at(i).rect.size.width, armors.at(i).rect.size.height)+ 
					   std::max(armors.at(j).rect.size.width, armors.at(j).rect.size.height) )
			{
				double armor_ratio1 = std::max(armors.at(i).rect.size.width, armors.at(i).rect.size.height) / 
									  std::min(armors.at(i).rect.size.width, armors.at(i).rect.size.height); 
    			double armor_ratio2 = std::max(armors.at(j).rect.size.width, armors.at(j).rect.size.height) / 
									  std::min(armors.at(j).rect.size.width, armors.at(j).rect.size.height); 

				if (is_armor[i] == true && is_armor[j] == true)
				{
					if (abs(armor_ratio1 - armor_ratio2) > 0.4)
					{
						if(armor_ratio1 - armor_ratio2 > 0)
						{
				 			is_armor[i] = false;
						}
						else{
				 			is_armor[j] = false;
						}
					}
					else if (abs(abs(armors.at(i).rect.angle) - abs(armors.at(j).rect.angle)) > 15) 
					{
						if (abs(armors.at(i).rect.angle) > abs(armors.at(j).rect.angle))
						{
							is_armor[i] = false;
						}
						else{
							is_armor[j] = false;
						}
					}
					else if (armors.at(i).rect.size.area() - armors.at(j).rect.size.area() > 100)
					{
						is_armor[i] = false;
					}
					else{
						float val_i = get_armor_roi(armors[i].rect);
						float val_j = get_armor_roi(armors[j].rect);

						if(val_i > 0 && val_j < 0)
						{
							is_armor[j] = false;
						}
						else if(val_i < 0 && val_j > 0)
						{
							is_armor[i] = false;
						}
						else {
							is_armor[i] = false;
							is_armor[j] = false;
						}
					}
				}// if both
			}    // dis < dist
    	} 		 // for j
  	} 			 // for i

	filte_rects.clear();
	for( int i = 0; i < is_armor.size();++i)
	{
		if(is_armor[i]) filte_rects.push_back(armors[i]);
	}
	armors = filte_rects;
  	// speed_test_end("FilterArmor 用时 = ", "ms");
}

static int cnt = 0;  // 临时写的，用同一段视频调试时，根据有多少帧识别到了来评价算法好坏

bool ArmorDetector::detect(cv::Mat & src, std::vector<armor_info> & armors_candidate) {
	// speed_test_reset();
	std::vector<cv::RotatedRect> lights;
    DetectLights(src, lights);   //  5ms
	if (lights.size() > 200) return false; // 防止被敌方激光瞄到视野（画面内太多红光确实会一下子灯条数量爆表，然后程序崩掉）
	FilterLights(lights);        //  0.1ms

	if (lights.size() > 1)
	{
		choose_target_from_lights(lights, armors_candidate);  // 0.001 ms
		// for(auto armor : armors_candidate) draw_rotated_rect(src, armor.rect, Scalar(0,0,255), 2);
		// std::cout << "before filter:" << armors_candidate.size() <<std::endl;
		/*
		for(int i =0; i<armors_candidate.size();++i)
		{
			char str[5];
			sprintf(str, "%d", i);
			draw_rotated_rect(src, armors_candidate[i].rect, Scalar(0,0,255), 1);
			putText(src, str, armors_candidate[i].rect.center, CV_FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(0,205,0), 1);
		}
		*/
		FilterArmors(armors_candidate);
		/*
		for(int i =0;i < armors_candidate.size();++i)
		{
			char str[5];
			sprintf(str, "%d", i);
			putText(src,str, armors_candidate[i].rect.center, CV_FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(0,205,0), 1);
 		}*/
		// std::cout << "after filter:" << armors_candidate.size() <<std::endl;

		// 进行一次多目标的 SHOW_DEBUG
		for(auto armor : armors_candidate) {
			draw_rotated_rect(src, armor.rect, Scalar(0,255,0), 2);
			if(armor.rect.size.area()) cnt++;
		}
        // std::cout << "识别数:\t" << cnt << std::endl;
		// speed_test_end("Vision_mul use = ", "ms");

		return true; // 识别到灯条
	}
	else{
		// speed_test_end("Vision_mul use = ", "ms");
		return false; // 没有识别到
	}
}

} // namespace vision_mul
} // namespace autocar
