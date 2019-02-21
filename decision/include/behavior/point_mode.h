#ifndef POINTMODE_H
#define POINTMODE_H
#include "mission_mode.h"

class LogicalCore;
class PointMode: public MissionMode
{
public:
  PointMode(LogicalCore *logical_core);
  void obtainedGoalState();
  void busyState();
  void idleState();
private:
  LogicalCore *logical_core_;
};

#endif // POINTMODE_H
