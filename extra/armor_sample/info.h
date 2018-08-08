/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2018, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#ifndef ARMOR_INFO_H
#define ARMOR_INFO_H 

#include <opencv2/opencv.hpp>

/**
 * @brief: 装甲板算法的阈值参数
 */
class armor_param
{
public:
    armor_param();

    void read_params();
	float light_threshold_val;      // 环境亮度
	
    float light_min_aspect_ratio;   // 灯条最小纵横比
	float light_max_aspect_ratio;   // 灯条最大纵横比

	float light_min_area;           // 灯条最小面积
	float light_max_area;           // 灯条最大面积

	float light_max_angle;          // 灯条最大角度

	float light_max_angle_diff;     // 灯条最大角度差值
    float light_max_height_diff;    // 灯条最大长度差
    float light_max_width_diff;     // 灯条最大宽度差

    float armor_min_ratio;   	    // 小装甲板最小纵横比
    float armor_max_ratio;          // 小装甲板最大纵横比

	float armor_light_angle_diff;   // 装甲板两灯条角度的累计数值差
  
	float armor_max_angle;
	float armor_min_area;           // 装甲板最小面积
	float armor_max_area;           // 装甲板最大面积

	float filter_armor_area;
	
	float armor_min_aspect_ratio;   // 装甲板最小纵横比
	float armor_max_aspect_ratio;   // 装甲板最大纵横比
	float armor_max_pixel_val;
	float armor_max_stddev;
	float armor_width;				// 装甲板的宽度
	float armor_height;			    // 装甲板的高度
	bool enemy_color;               // 颜色
	
	int blue_color_diff;
	int red_color_diff;
};


/**
 * @brief: 装甲板的位姿信息
 */
struct armor_pos
{
	int Flag;   // 标志位
	double angle_x;  // Yaw角度
	double angle_y;  // Ptich角度
	double angle_z;  // 距离信息

	void reset_pos()
	{
		Flag = 0;
		angle_x = 0;
		angle_y = 0;
		angle_z = 0;
	}
	armor_pos()
	{
		Flag = 0;
		angle_x = 0;
		angle_y = 0;
		angle_z = 0;
	}
};

enum Armor_Twist { STILL = 1, LOW_MOVE = 2, MID_MOVE = 3, FAST_MOVE = 4 }; // 速度信息
/**
 * @brief: 装甲板结构体
 */
struct armor_info
{
	cv::RotatedRect rect;  // 识别到rectangle
	Armor_Twist state;
	armor_info(cv::RotatedRect t, Armor_Twist st): rect(t), state(st) { }

	armor_info()
	{
		rect = cv::RotatedRect();
		state = STILL;
	}
};
#endif // ARMOR_INFO_H