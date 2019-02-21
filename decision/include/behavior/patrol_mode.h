#ifndef PATROLMODE_H
#define PATROLMODE_H
#include <move_base_msgs/MoveBaseAction.h>
#include "mission_mode.h"

class LogicalCore;
class PatrolMode: public MissionMode
{
public:
  PatrolMode(LogicalCore* logical_core);
  void obtainedGoalState();
  void busyState();
  void idleState();
private:
  LogicalCore *logical_core_;
};

#endif // PATROLMODE_H
