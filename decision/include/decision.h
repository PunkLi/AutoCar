#ifndef LOGICALCORE_H
#define LOGICALCORE_H


//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace decision_mul
{

enum RobotState
{
    IDLE_STATE,
    ATTACK_STATE,
    BACK_STATE
};

}

#endif // LOGICALCORE_H
