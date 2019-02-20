
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "serial/chassis_info.h"
#include "vel2odom.h"

odom_mul::vel2odom solver;

void speed2pos(const serial::chassis_info car_vel)
{
    solver.angle  = car_vel.angle;
    solver.v_x    = car_vel.v_x;
    solver.v_y    = car_vel.v_y;
    solver.v_r    = car_vel.v_r;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry_publisher");

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("chassis_info", 1000, speed2pos);
    ros::Publisher pub  = n.advertise<nav_msgs::Odometry>("odom", 50);

    tf::TransformBroadcaster odom_broadcaster;
    solver.last_time = ros::Time::now();

    ros::Rate r(50);

    while(n.ok())
    {
        // check for incoming messages
        ros::spinOnce();
        solver.current_time = ros::Time::now();
        solver.cal_odom();

        //publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = solver.current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id  = "base_link";

        odom_trans.transform.translation.x = solver.x;
        odom_trans.transform.translation.y = solver.y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation      = solver.odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp    = solver.current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x  = solver.x;
        odom.pose.pose.position.y  = solver.y;
        odom.pose.pose.position.z  = 0.0;
        odom.pose.pose.orientation = solver.odom_quat;

        //set the velocity
        odom.child_frame_id        = "base_link";
        odom.twist.twist.linear.x  = solver.v_x;
        odom.twist.twist.linear.y  = solver.v_y;
        odom.twist.twist.angular.z = solver.v_r;

        //publish the message
        pub.publish(odom);

        solver.last_time = solver.current_time;
        r.sleep();
    }
}
