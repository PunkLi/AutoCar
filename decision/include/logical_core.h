#ifndef LOGICALCORE_H
#define LOGICALCORE_H


//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace decision_mul
{

enum RobotState
{
    IDLE_STATE,        // 空闲
    ATTACK_STATE,         // 攻击 
    BACK_STATE           // 防御
};


}
#endif // LOGICALCORE_H
