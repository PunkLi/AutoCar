

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <move_base_msgs/MoveBaseAction.h>

#include <detect/armor_goal.h>
#include <serial/car_info.h>
#include <multiwar/share_info.h>
#include <decision/shoot_info.h>

#include <actionlib/client/simple_action_client.h>

namespace decision_mul
{

using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

class decision_node
{
    move_base_msgs::MoveBaseGoal goal;
    move_base_msgs::MoveBaseGoal goal_pose;

    geometry_msgs::PoseStamped enemy_goal_;
    geometry_msgs::PoseStamped enemy_global_goal_; // history
    geometry_msgs::PoseStamped car_position;

    ros::Subscriber sub_car,
                    sub_mine_pos,
                    sub_armor,
                    sub_network;

    ros::Publisher  pub_goal,
                    pub_shoot,
                    pub_network;
public:
    decision_node(): ac_("move_base", true)
    {
        ros::NodeHandle nh;
 
        sub_car      = nh.subscribe<serial::car_info>("car_info", 5, boost::bind(&decision_node::car_callback, this, _1));
        sub_mine_pos = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 5, boost::bind(&decision_node::pos_callback, this, _1));
        sub_armor    = nh.subscribe<detect::armor_goal>("armor_info", 5, boost::bind(&decision_node::enemy_callback, this, _1));
        sub_network  = nh.subscribe<multiwar::share_info>("team_info", 5, boost::bind(&decision_node::multiwar_callback, this, _1));

        pub_goal     = nh.advertise<move_base_msgs::MoveBaseGoal>("nav_goal", 10);
        pub_shoot    = nh.advertise<decision::shoot_info>("shoot_info", 10);
        pub_network  = nh.advertise<multiwar::share_info>("my_info", 10);

        ac_.waitForServer();
    }

    void car_callback(const serial::car_info::ConstPtr & info)
    {
        ; // 
    }

    void pos_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pos)
    {
        car_position.pose = pos->pose.pose;
        car_position.header = pos->header;
    }

    void enemy_callback(const detect::armor_goal::ConstPtr & enemy)
    {
        // ac_.sendGoal(goal);

        if(enemy->detected)
        {
            // std::msgs/Header header
            // geometry_msg/Point point

            // enemy_global_goal_ = goalToGlobalFrame(enemy_goal_);  // inportant

            //tf::Pose pose;
            //tf::poseMsgToTF(enemy_global_goal_.pose, pose);
            //double yaw = tf::getYaw(pose.getRotation());
            /*
            if(distance(enemy_global_goal_, old_enemy_global_goal_) > 1.5)
            {
                p_mission_mode_ = p_shooting_mode_;
                //goal_.target_pose.pose.orientation.w = 1.0;
                //goal_.target_pose.pose.orientation.z = 0.0;
                car_state_ = OBTAINED_GOAL_STATE;
                old_enemy_global_goal_ = enemy_global_goal_;
                goal_.target_pose = enemy_global_goal_;
                goal_.target_pose.header.stamp = ros::Time::now();
                goal_.target_pose.pose.position.z = 2;
            }*/
        }
        else{
            ;// no enemy
        }
    }

    void multiwar_callback(const multiwar::share_info::ConstPtr & share_msg)
    {
        ; // todo
    }

private:
    MoveBaseClient ac_;
    tf::TransformListener tf_;

    geometry_msgs::PoseStamped goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg)
    {
        std::string global_frame = "map";
        tf::Stamped<tf::Pose> goal_pose, global_pose;
        poseStampedMsgToTF(goal_pose_msg, goal_pose);

        //just get the latest available transform... for accuracy they should send
        //goals in the frame of the planner
        goal_pose.stamp_ = ros::Time();

        try{
            tf_.transformPose(global_frame, goal_pose, global_pose);
        }
        catch(tf::TransformException& ex){
            ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
                    goal_pose.frame_id_.c_str(), global_frame.c_str(), ex.what());
            return goal_pose_msg;
        }

        geometry_msgs::PoseStamped global_pose_msg;
        tf::poseStampedTFToMsg(global_pose, global_pose_msg);
        return global_pose_msg;
    }

};

} // namespace decision_mul