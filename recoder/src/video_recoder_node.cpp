
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <video_recoder.h>

recoder_mul::video_recoder recoder("/home/linux/", 640, 480);

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        recoder.save_frame(frame); 
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Recoding");

    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub = it.subscribe("camera_info", 1, imageCallback);

    ros::spin();
    
    return 0;
}
