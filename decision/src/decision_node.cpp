
#include <tf/transform_listener.h>

#include <decision_node.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "decision_node");
    //tf::TransformListener tf(ros::Duration(10), true);

    decision_mul::decision_node decision;

    ros::MultiThreadedSpinner spinner(4);
    ros::spin();

    return 0;
}
