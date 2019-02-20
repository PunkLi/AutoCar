#ifndef SHOOTINGMODE_H
#define SHOOTINGMODE_H
#include "mission_mode.h"

class LogicalCore;
class ShootingMode: public MissionMode
{
public:
  ShootingMode(LogicalCore *logical_core);
  void obtainedGoalState();
  void busyState();
  void idleState();
private:
  LogicalCore *logical_core_;
};

#endif // SHOOTINGMODE_H
