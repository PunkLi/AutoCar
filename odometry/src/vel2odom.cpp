#include "vel2odom.h"

namespace odom_mul
{
vel2odom::vel2odom():x(0.0), y(0.0), angle(0.0), v_x(0.0), v_y(0.0), v_r(0.0)
{
}

void vel2odom::cal_odom()
{
    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (v_x*cos(angle)-v_y*sin(angle)) * dt;
    double delta_y = (v_x*sin(angle)+v_y*cos(angle)) * dt;
    x += delta_x;
    y += delta_y;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf::createQuaternionMsgFromYaw(angle);

    //std::cout<<"Vx  "<<v_x<<std::endl;
    //std::cout<<"Vy  "<<v_y<<std::endl;
    
    int thetaPeriodCount    = (int)(angle/2/3.1415926);
    float thetaNormal       = angle - thetaPeriodCount*(2*3.1415926);
    float thetaNormalDegree = thetaNormal * (180/3.1415926);
    
    //std::cout<<"theta   "<<thetaNormalDegree<<std::endl;
    //std::cout<<"X  "<<x<<std::endl;
    //std::cout<<"Y  "<<y<<std::endl;
}

} // namespace odom_mul

