/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2018, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#include <armor_info.h>

#define SHOW_DEBUG

namespace detect_mul
{
armor_param::armor_param()
{
	read_params();
}

void armor_param::read_params()
{
    cv::FileStorage fs("../config/armor_params.xml", cv::FileStorage::READ);
    if(!fs.isOpened())
    {
        std::cout << "In armor_param 函数: Cannot open armor_params.xml, please check if the file is exist." << std::endl;
    }
    cv::FileNode root = fs.root();
    
    //detect_mul
    cv::FileNode detect_mul = root["detect_mul"];

	detect_mul["light_threshold_val"] >> light_threshold_val;

	detect_mul["light_min_aspect_ratio"] >> light_min_aspect_ratio;
	detect_mul["light_max_aspect_ratio"] >> light_max_aspect_ratio;

    detect_mul["light_min_area"] >> light_min_area;
	detect_mul["light_max_area"] >> light_max_area;
	
	detect_mul["light_max_angle"] >> light_max_angle;

	detect_mul["light_max_angle_diff"] >> light_max_angle_diff;
	detect_mul["light_max_height_diff"] >> light_max_height_diff;
	detect_mul["light_max_width_diff"] >> light_max_width_diff;

	detect_mul["armor_min_ratio"] >> armor_min_ratio;
    detect_mul["armor_max_ratio"] >> armor_max_ratio;

	detect_mul["armor_light_angle_diff"] >> armor_light_angle_diff;

	detect_mul["filter_armor_area"] >> filter_armor_area;

	detect_mul["armor_max_angle"] >> armor_max_angle;
	detect_mul["armor_min_area"] >> armor_min_area;
	detect_mul["armor_max_area"] >> armor_max_area;

	detect_mul["armor_max_aspect_ratio"] >> armor_max_aspect_ratio;
	detect_mul["armor_max_pixel_val"] >> armor_max_pixel_val;
	detect_mul["armor_max_stddev"] >> armor_max_stddev;
	detect_mul["width"] >> armor_width;
	detect_mul["height"] >> armor_height;
	detect_mul["enemy_color"] >> enemy_color;

	detect_mul["blue_color_diff"] >> blue_color_diff;
	detect_mul["red_color_diff"] >> red_color_diff;

#ifdef SHOW_DEBUG
	std::cout << "successful load armor_param"<<std::endl;
#endif
}

} // namespace detect_mul
