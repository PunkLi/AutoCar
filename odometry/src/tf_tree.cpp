
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_publisher");
    ros::NodeHandle n;
    ros::Rate r(100);
    tf::TransformBroadcaster broadcaster;
    
    tf::Quaternion lidar_Q(0, 0, 0, 1);
    tf::Vector3    lidar_T(-0.1, 0.0, 0.1);

    while(n.ok())
    {
        broadcaster.sendTransform(
            tf::StampedTransform(tf::Transform(lidar_Q, lidar_T), 
                                 ros::Time::now(), 
                                "base_link", "laser")
                    );
                    
        r.sleep();
    }
}
