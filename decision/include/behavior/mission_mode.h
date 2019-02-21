#ifndef MISSIONMODE_H
#define MISSIONMODE_H
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
/**
 * @brief There are three missions of the autocar: point state, patrol state, shooting state.
 * and the autocar also have three state: OBTAINED_GOAL_STATE, BUSY_STATE, IDLE_STATE
 */
class MissionMode
{
public:
  MissionMode();
  virtual void obtainedGoalState();
  virtual void busyState();
  virtual void idleState();
};

#endif // MISSIONMODE_H
