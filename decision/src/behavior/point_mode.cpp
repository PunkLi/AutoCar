#include "point_mode.h"
#include "logical_core.h"

PointMode::PointMode(LogicalCore *logical_core)
{
  MissionMode();
  logical_core_ = logical_core;
}

void PointMode::obtainedGoalState()
{
  ROS_INFO("Switch to OBTAINED_GOAL_STATE under POINT_MODE");
  logical_core_->ac_.sendGoal(logical_core_->goal_);
  logical_core_->car_state_ = BUSY_STATE;
  ROS_INFO("Switch to BUSY_STATE");
}

void PointMode::busyState()
{
  if (logical_core_->ac_.getState()
      == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    logical_core_->car_state_ = IDLE_STATE;
    ROS_INFO("Point goal reached，switch to IDLE_STATE!");
  }
  else if (logical_core_->ac_.getState() == actionlib::SimpleClientGoalState::LOST)
    std::cout<<"Cannot make a plan!"<<std::endl;
}

void PointMode::idleState()
{
  ROS_INFO("Switch to the PATROL_MODE!");
  logical_core_->p_mission_mode_ = logical_core_->p_patrol_mode_;
}
