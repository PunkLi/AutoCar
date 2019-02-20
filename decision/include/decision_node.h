

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <detect/enemy_info.h>
#include <multiwar/share_info.h>

namespace decision_mul
{

class decision_node
{
    ros::Subscriber sub_mine_pos;
    ros::Subscriber sub_enemy_info;
    ros::Subscriber sub_share_info;
    ros::Publisher  pub_share_info;

public:
    decision_node()
    {
        ros::NodeHandle nh;

        sub_mine_pos    = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 5, 
                                            boost::bind(&decision_node::pos_callback, this, _1));

        sub_enemy_info  = nh.subscribe<detect::enemy_info>("enemy_info", 5, 
                                            boost::bind(&decision_node::enemy_callback, this, _1));

        sub_share_info  = nh.subscribe<multiwar::share_info>("team_info", 5, 
                                            boost::bind(&decision_node::multiwar_callback, this, _1));

        pub_share_info  = nh.advertise<multiwar::share_info>("my_info", 1000);
        
    }
    void pos_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pos)
    {
        ; // todo
    }

    void enemy_callback(const detect::enemy_info::ConstPtr & enemy)
    {
        ; // todo
    }

    void multiwar_callback(const multiwar::share_info::ConstPtr & share_msg)
    {
        ; // todo
    }

private:
    //geometry_msgs::PoseStamped enemy_goal_;
    //geometry_msgs::PoseStamped enemy_global_goal_[2]; // history
    //geometry_msgs::PoseStamped car_position;

};

} // namespace decision_mul