#ifndef VEL2ODOM_H
#define VEL2ODOM_H
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

namespace odom_mul
{
class vel2odom
{
public:
  vel2odom();
  /**
   * @brief Calculating odom from velocity
   */
  void cal_odom();

public:
  double angle; ///< yaw-axis coordinates of the car in map
  double v_x;   ///< x-axis velocity of the car
  double v_y;   ///< y-axis velocity of the car
  double v_r;   ///< yaw-axis velocity of the car

  double x;     ///< X-axis coordinates of the car in map
  double y;     ///< y-axis coordinates of the car in map

  geometry_msgs::Quaternion odom_quat; //< the quaternion of the car attitude
  ros::Time current_time;
  ros::Time last_time;
};

}// namespace odom_mul

#endif // VEL2ODOM_H
