/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2018, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#include "info.h"
#include "detect.hpp"

int main()
{
    cv::VideoCapture capture_camera_forward("../../../video/armor_20180429_011146.avi");
    int index = 0;
    cv::Mat frame_forward;
    armor_sample armor_detector;
   
    while (true) {
        capture_camera_forward >> frame_forward;
		armor_detector.detect(frame_forward, index);
        try {
		    imshow("result", frame_forward);
            cv::waitKey(1);
		}
		catch (cv::Exception e) {
			std::cout << "show image error" << std::endl;
		}
        cv::waitKey(0);
	}
}