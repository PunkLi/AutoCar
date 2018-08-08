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
#include "detect.hpp"

#define POINT_DIST(p1,p2) std::sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y))

void draw_rotated_rect(const cv::Mat &image, const cv::RotatedRect &rect, const cv::Scalar &color, int thickness)
{
    cv::Point2f vertex[4];

    rect.points(vertex);
    for (int i=0; i<4; i++)
        cv::line(image, vertex[i], vertex[(i+1)%4], color, thickness);
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
cv::RotatedRect armor_sample::boundingRRect(const cv::RotatedRect & left, const cv::RotatedRect & right){
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
cv::RotatedRect armor_sample::boundingRRectFast(const cv::RotatedRect & left, const cv::RotatedRect & right){
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
cv::RotatedRect armor_sample::boundingRRectSlow(const cv::RotatedRect & left, const cv::RotatedRect & right){
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

void armor_sample::DetectLights(const cv::Mat &src, std::vector<cv::RotatedRect> &lights) {

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
   	 	thresh = 70;
  	else
    	thresh = 40;  //50
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
          			is_processes[j] = true;
          			break;
        		}
      		}
    	} // for j loop
  	} // for i loop
}

void armor_sample::FilterLights(std::vector<cv::RotatedRect> &lights) 
{

  	//std::vector<cv::RotatedRect> light_rects;
	light_rects.clear();
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
		}
        // 针对灯条细小的情况, 没有最大比例的判断, 较为理想的灯条
		else if(armor_light.size.area() >= _para.light_min_area // 1.0
		   		&& armor_light.size.area() < 100000  //_para.light_max_area * src_img_.size().height * src_img_.size().width // 0.04
		   		&& abs(angle) < _para.light_max_angle) // 与垂直的偏角17.5 , 这里是可以取消/2的,进一步细化
		{
			light_rects.push_back(armor_light); // 接近于垂直的灯条, 由于阈值不够合理, 细小的灯条

	    }
        // 检测最为平凡的情况
    	else if (//light_aspect_ratio < _para.light_max_aspect_ratio  // 6.8
                 armor_light.size.area() >= _para.light_min_area // 1.0
			     && armor_light.size.area() < _para.light_max_area * src_img_.size().height * src_img_.size().width // 0.04
			     && abs(angle) < _para.light_max_angle) // 与垂直的偏角35 
        {
      		light_rects.push_back(armor_light);
		}
		
  	}
		lights = light_rects;
}

/**
 * @brief: multi-detect for armor_lights, which needed filter later.
 */
void armor_sample::choose_target_from_lights(std::vector<cv::RotatedRect> &lights, std::vector<armor_info> &armor_vector)
{
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
				//auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

				if (armor_area > _para.armor_min_area 
					&& armor_ratio < 4.5 // ok ! param! 步兵
					&& abs(armor_angle) < 20 )          // ok ! param! 步兵
				{	
					Armor_Twist armor_twist = FAST_MOVE;
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
				
				//auto armor_light_angle_diff = abs(lights[i].angle) == 90 ? 
				//							  abs(armor_angle) + abs(armor_angle - lights[j].angle - 90); // 左右灯条的积累差

				if (armor_area > _para.armor_min_area
				    && armor_ratio < 4.5 // ok ! param! 步兵
					&& abs(armor_angle) < 20 )          // ok ! param! 步兵
				{	
					Armor_Twist armor_twist = FAST_MOVE;
					armor_info armor(possible_rect, armor_twist);
					armor_vector.push_back(armor);
				} // get_armor
			}// 2个当中有一个略不平行

// 快速平移的情况 Fast Move
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 中速平移的情况 Mid Move
		// 2个严格平行
			else if ( ( (light_aspect_ratio1 == 1 && light_aspect_ratio2 <= 1.5) 
					 || (light_aspect_ratio1 <= 1.5 && light_aspect_ratio2 == 1) ) // 其中一个为正方形
					 && static_cast<int>(abs(angle_diff)) % 90 == 0               	 // 角度差为0
					 && static_cast<int>(abs(lights[i].angle)) % 90 == 0          	 // 角度为0 或 90
					 && static_cast<int>(abs(lights[j].angle)) % 90 == 0          	 // 角度为0 或 90
					 && height_diff < 0.5 && width_diff < 0.5)               	 // 形状差异
			{
				cv::RotatedRect possible_rect;
				if (lights[i].center.x < lights[j].center.x)
					possible_rect = boundingRRectFast(lights[i], lights[j]);
				else
					possible_rect = boundingRRectFast(lights[j], lights[i]);

				auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
				auto armor_angle = possible_rect.angle;
				auto armor_area = possible_rect.size.area();
				//auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

				if (armor_area > _para.armor_min_area			// ok ! param! 步兵
					&& armor_ratio < 4      // ok ! param! 步兵         //_para.armor_max_ratio , 步兵应该只有3, 英雄可能会到5
					&& abs(armor_angle) <  20 )  // ok ! param! 步兵
					{
						Armor_Twist armor_twist = MID_MOVE;
						armor_info armor(possible_rect, armor_twist);
						armor_vector.push_back(armor);
					} // get_armor
			}
			// 2个严格平行

			// 1个竖着 1个横着
			else if (1 < light_aspect_ratio1 && light_aspect_ratio1 < 1.3
					 && 1 < light_aspect_ratio2 && light_aspect_ratio2 < 1.3
					 && static_cast<int>(abs(angle_diff)) % 180 == 90 // 角度差为0
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
				//auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

				if (armor_area > _para.armor_min_area			// ok ! param! 步兵
					&& armor_ratio < 4      // ok ! param! 步兵         //_para.armor_max_ratio , 步兵应该只有3, 英雄可能会到5
					&& abs(armor_angle) <  20 )  // ok ! param! 步兵
					{
						Armor_Twist armor_twist = MID_MOVE;
						armor_info armor(possible_rect, armor_twist);
						armor_vector.push_back(armor);
					} // get_armor
			}
			else if (1 < light_aspect_ratio1 && light_aspect_ratio1 < 1.5
					 && 1 < light_aspect_ratio2 && light_aspect_ratio2 < 1.5
					 && static_cast<int>(abs(angle_diff)) % 180 == 0 // 角度差为0
					 && abs(lights[i].angle) == 0 && abs(lights[j].angle) == 0        	     // 角度为0 或 180
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
				//auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

				if (armor_area > _para.armor_min_area			// ok ! param! 步兵
					&& armor_ratio < 4      // ok ! param! 步兵                           //_para.armor_max_ratio , 步兵应该只有3, 英雄可能会到5
					&& abs(armor_angle) <  20 )  // ok ! param! 步兵
					{
						Armor_Twist armor_twist = LOW_MOVE;
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

				if (armor_area > _para.armor_min_area			// ok ! param! 步兵
					&& armor_ratio < 4      // ok ! param! 步兵                           //_para.armor_max_ratio , 步兵应该只有3, 英雄可能会到5
					&& abs(armor_angle) <  20    // ok ! param! 步兵
					&& armor_light_angle_diff < 20 )
					{
						Armor_Twist armor_twist = LOW_MOVE;
						armor_info armor(possible_rect, armor_twist);
						armor_vector.push_back(armor);
					} // get_armor
			}
		// 其中一块略有倾斜

// 慢速移动的情况 Low Move //////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
// 平凡的情况
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// 灯条近乎平行,至少在同一侧
			else if (lights[i].angle * lights[j].angle >= 0            // 灯条近乎同侧 , 或者有一个为0
				     && abs(angle_diff) < 30                            //  _para.light_max_angle_diff   // 20   // 18   这些都要换成相对值
					//&& height_diff < _para.light_max_height_diff  	   // 20  不需要宽度
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
							&& armor_ratio > 1 // _para.small_armor_min_ratio   // 1.5
				   			&& armor_ratio < _para.armor_max_ratio   // 3.0
				   			&& abs(armor_angle) < _para.armor_max_angle
				   			&& armor_light_angle_diff < _para.armor_light_angle_diff // 应该要更为严格
						)
						{
							Armor_Twist armor_twist = MID_MOVE;
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
							&& armor_ratio > 1 // _para.small_armor_min_ratio   // 1.5
				   			&& armor_ratio < _para.armor_max_ratio   // 3.0
				   			&& abs(armor_angle) < _para.armor_max_angle
				   			&& armor_light_angle_diff < _para.armor_light_angle_diff // 应该要更为严格
						)
						{
							Armor_Twist armor_twist = LOW_MOVE;
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
							&& armor_ratio > 1 // _para.small_armor_min_ratio   // 1.5
				   			&& armor_ratio < _para.armor_max_ratio   // 3.0
				   			&& abs(armor_angle) < _para.armor_max_angle
						)
						{
							Armor_Twist armor_twist = FAST_MOVE;
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
							&& armor_ratio > 1 // _para.small_armor_min_ratio   // 1.5
				   			&& armor_ratio < _para.armor_max_ratio   // 3.0
				   			&& abs(armor_angle) < _para.armor_max_angle
				   			&& armor_light_angle_diff < _para.armor_light_angle_diff // 应该要更为严格
						)
						{
							Armor_Twist armor_twist = LOW_MOVE;
							armor_info armor(possible_rect, armor_twist);
							armor_vector.push_back(armor);
						}
					}// 2灯条近乎平行 慢速移动
				}// 2灯条近乎平行 快速移动

				else if (_para.light_min_aspect_ratio < light_aspect_ratio1 // && light_aspect_ratio1 < _para.light_max_aspect_ratio
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
						&& armor_ratio > 1 // _para.small_armor_min_ratio   // 1.5
				   		&& armor_ratio < 4.5 // _para.armor_max_ratio   // 3.0
				   		&& abs(armor_angle) < _para.armor_max_angle
				   		&& armor_light_angle_diff < _para.armor_light_angle_diff ) // 应该要更为严格
					{
						Armor_Twist armor_twist = STILL;
						armor_info armor(possible_rect, armor_twist);
						armor_vector.push_back(armor);
					}
				}

			} // 灯条严格平行

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			
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
							&& armor_ratio > 1 // _para.small_armor_min_ratio   // 1.5
				   			&& armor_ratio < _para.armor_max_ratio   // 3.0
				   			&& abs(armor_angle) < _para.armor_max_angle
				   			&& armor_light_angle_diff < _para.armor_light_angle_diff // 应该要更为严格
						)
						{
							Armor_Twist armor_twist = MID_MOVE;
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
						Armor_Twist armor_twist = STILL;
						armor_info armor(possible_rect, armor_twist);
						armor_vector.push_back(armor);
					}
				}
			} // 灯条(误差) 并不同侧
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
				   			//&& armor_light_angle_diff < _para.armor_light_angle_diff // 应该要更为严格
						)
						{
							Armor_Twist armor_twist = MID_MOVE;
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
						)
						{
							Armor_Twist armor_twist = LOW_MOVE;
							armor_info armor(possible_rect, armor_twist);
							armor_vector.push_back(armor);
						}
					}// 2灯条近乎平行 慢速移动
				}// 2灯条近乎平行 快速移动
			}
				
		} // for j loop
	} // for i loop
	//speed_test_end("choose_armor_from_light 用时 = ", "ms");
} // end func
int img_idx = 11394;
void armor_sample::FilterArmors(std::vector<armor_info> &armors, int index) 
{
	std::vector<bool> is_armor(armors.size(), true);

  	cv::Mat mask = cv::Mat::zeros(gray_img_.size(), CV_8UC1);
  
  	for (int i = 0; i < armors.size(); i++) // const auto &armor : armors
	{
		cv::RotatedRect rect = armors[i].rect;
		auto center = rect.center;
		cv::Mat rot_mat = cv::getRotationMatrix2D(rect.center,rect.angle,1); 
		cv::Mat img;
		warpAffine(gray_img_, img, rot_mat, img.size(), INTER_LINEAR, BORDER_CONSTANT);  // warpAffine use 2ms
		cv::Rect target = cv::Rect(center.x - (rect.size.width / 2), 
								   center.y - (rect.size.height/2), 
								   rect.size.width, rect.size.height);
		if (makeRectSafe(target,img.size()) == true)
		{
			cv::Mat armor_roi = img(target);
			cv::resize(armor_roi, armor_roi, cv::Size(100,25));
			//cv::resize(armor_roi, armor_roi, cv::Size(60,25));
			char str[100];
            sprintf(str, "../../../../sample/armor_2/armor_%d.jpg", img_idx++);
			cv::imwrite(str, armor_roi);		
		}
	}
}

static int cnt = 0;

short armor_sample::detect(cv::Mat & src, int index) {

	std::vector<cv::RotatedRect> lights;
    DetectLights(src, lights);
	FilterLights(lights);
	std::vector<armor_info> armors_candidate;

	if (lights.size() > 1)
	{
		choose_target_from_lights(lights, armors_candidate);
		for(int i =0;i<armors_candidate.size();++i)
		{
			char str[5];
			sprintf(str, "%d", i);
			draw_rotated_rect(src, armors_candidate[i].rect, Scalar(0,0,255), 1);
			putText(src, str, armors_candidate[i].rect.center, CV_FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(0,205,0), 1);
		}
		FilterArmors(armors_candidate, index);
		// 进行一次多目标的SHOW_DEBUG
		for(auto armor : armors_candidate) {
			draw_rotated_rect(src, armor.rect, Scalar(0,255,0), 2);
			if(armor.rect.size.area()) cnt++;
		}
		return true; // 识别到灯条
	}
	else{
		return false; // 没有识别到
	}
}