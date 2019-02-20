#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include "uvc_camera.h"

using namespace cv;
using namespace std;

#define USE_VIDEO

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uvc_camera");
    
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera_info", 1);
    cv_bridge::CvImagePtr frame = boost::make_shared< cv_bridge::CvImage >();
    frame->encoding = sensor_msgs::image_encodings::BGR8;

    VideoCapture cap;
#ifdef USE_VIDEO
    cap.open("/home/linux/demo.mp4");
#else
    driver_mul::uvc_camera config;
    std::string str = "480P_120_MH.xml";
    config.open_camera(cap, str);
#endif

    ros::Rate loop_rate(30);

    while(ros::ok())
    {
        cap >> frame->image;       
        if ( frame->image.empty() )
        {
            ROS_ERROR_STREAM( "Failed to capture frame!" );
            ros::shutdown();
        }
        frame->header.stamp = ros::Time::now();
        pub.publish(frame->toImageMsg());

        // cv::imshow("img", frame->image);
        // cv::waitKey(1);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    cap.release();

    return 0;
}
