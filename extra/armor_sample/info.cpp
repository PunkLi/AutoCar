/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2018, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#include "info.h"

#define SHOW_DEBUG

armor_param::armor_param()
{
	read_params();
}

void armor_param::read_params()
{
    cv::FileStorage fs("../armor_params.xml", cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        std::cout << "In armor_param 函数: Cannot open armor_params.xml, please check if the file is exist." << std::endl;
    }
    cv::FileNode root = fs.root();
    
    //vision_mul
    cv::FileNode vision_mul = root["vision_mul"];

	vision_mul["light_threshold_val"] >> light_threshold_val;

	vision_mul["light_min_aspect_ratio"] >> light_min_aspect_ratio;
	vision_mul["light_max_aspect_ratio"] >> light_max_aspect_ratio;

    vision_mul["light_min_area"] >> light_min_area;
	vision_mul["light_max_area"] >> light_max_area;
	
	vision_mul["light_max_angle"] >> light_max_angle;

	vision_mul["light_max_angle_diff"] >> light_max_angle_diff;
	vision_mul["light_max_height_diff"] >> light_max_height_diff;
	vision_mul["light_max_width_diff"] >> light_max_width_diff;

	vision_mul["armor_min_ratio"] >> armor_min_ratio;
    vision_mul["armor_max_ratio"] >> armor_max_ratio;

	vision_mul["armor_light_angle_diff"] >> armor_light_angle_diff;

	vision_mul["filter_armor_area"] >> filter_armor_area;

	vision_mul["armor_max_angle"] >> armor_max_angle;
	vision_mul["armor_min_area"] >> armor_min_area;
	vision_mul["armor_max_area"] >> armor_max_area;

	vision_mul["armor_max_aspect_ratio"] >> armor_max_aspect_ratio;
	vision_mul["armor_max_pixel_val"] >> armor_max_pixel_val;
	vision_mul["armor_max_stddev"] >> armor_max_stddev;
	vision_mul["width"] >> armor_width;
	vision_mul["height"] >> armor_height;
	vision_mul["enemy_color"] >> enemy_color;

	vision_mul["blue_color_diff"] >> blue_color_diff;
	vision_mul["red_color_diff"] >> red_color_diff;

#ifdef SHOW_DEBUG
	std::cout << "successful load armor_param"<<std::endl;
#endif
}

