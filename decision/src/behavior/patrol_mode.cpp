#include "patrol_mode.h"
#include "logical_core.h"
#include <ros/ros.h>

PatrolMode::PatrolMode(LogicalCore* logical_core)
{
  logical_core_ = logical_core;
}

void PatrolMode::obtainedGoalState()
{
  ROS_INFO("[PATROL MODE] [OBTAINED_GOAL_STATE]");
  logical_core_->ac_.sendGoal(logical_core_->goal_);
  logical_core_->car_state_ = BUSY_STATE;
  ROS_INFO("[PATROL MODE] [BUSY_STATE]");

}

void PatrolMode::busyState()
{
  //patrol model 下goal是全局的
  if (logical_core_->ac_.getState()
      == actionlib::SimpleClientGoalState::SUCCEEDED ||
      logical_core_->distance(logical_core_->car_position, logical_core_->goal_.target_pose) < 0.5)
  {
    logical_core_->car_state_ = IDLE_STATE;
    logical_core_->patrol_list_.pop_back();
    //ROS_INFO("Current goal reached，switch to IDLE_STATE!");
  }
  else if (logical_core_->ac_.getState() == actionlib::SimpleClientGoalState::LOST)
    std::cout<<"Cannot make a plan!"<<std::endl;
}

void PatrolMode::idleState()
{
  ROS_INFO("[PATROL MODE] [IDLE_STATE]");
  logical_core_->goal_      = logical_core_->patrol_list_.back();
  logical_core_->car_state_ = OBTAINED_GOAL_STATE;
}
