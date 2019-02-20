#include "shooting_mode.h"
#include "logical_core.h"

ShootingMode::ShootingMode(LogicalCore *logical_core)
{
  logical_core_ = logical_core;
}

void ShootingMode::obtainedGoalState()
{
  ROS_INFO("[SHOOTINT MODE] [OBTAINED_GOAL_STATE]");
  logical_core_->ac_.sendGoal(logical_core_->goal_);
  logical_core_->car_state_ = BUSY_STATE;
  ROS_INFO("[SHOOTINT MODE] [BUSY_STATE]");
}

void ShootingMode::busyState()
{
//    //tf::Pose pose;
//    //tf::poseMsgToTF(enemy_global_goal_.pose, pose);
//    //double yaw = tf::getYaw(pose.getRotation());
//    if(logical_core_->distance(logical_core_->enemy_global_goal_, logical_core_->old_enemy_global_goal_) > 1.5)
//    {
//        p_mission_mode_ = p_shooting_mode_;
//        //goal_.target_pose.pose.orientation.w = 1.0;
//        //goal_.target_pose.pose.orientation.z = 0.0;
//        logical_core_->car_state_ = OBTAINED_GOAL_STATE;
//        logical_core_->old_enemy_global_goal_ = logical_core_->enemy_global_goal_;
//        logical_core_->goal_.target_pose = logical_core_->enemy_global_goal_;
//        logical_core_->goal_.target_pose.header.stamp = ros::Time::now();
//        logical_core_->goal_.target_pose.pose.position.z = 2;
//        ROS_INFO("[SHOOTINT MODE] [BUSY_STATE]");
//    }
}

void ShootingMode::idleState()
{
  ROS_INFO("[SHOOTINT MODE] [IDLE_STATE]");
  logical_core_->p_mission_mode_ = logical_core_->p_patrol_mode_;
}
