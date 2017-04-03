#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <eigen3/Eigen/Eigen>
#include <math.h>


// ROS cuustom msg

#include <qb_interface/cubeRef.h>
#include <qb_interface/cubeCurrent.h>
#include <qb_interface/cubePos.h>

# define PI 3.1416


class PID
{
public:
  PID();
  ~PID();


  void run();
  // Eigen::Quaterniond madgwick_kin(Eigen::Vector3d acc, Eigen::Vector3d gyro, Eigen::Quaterniond q_old, double State_joint_prev, Eigen::Vector3d Joint_prev, double State_joint_nxt, Eigen::Vector3d Joint_nxt);





private:

  void callback_gyro(const geometry_msgs::Vector3::ConstPtr& msg);
  void callback_imu_euler(const geometry_msgs::Vector3::ConstPtr& msg);
  void callback_gain_PID(const geometry_msgs::Vector3::ConstPtr& msg);
  void callback_corr(const qb_interface::cubeCurrent::ConstPtr& msg);
  void callback_meas(const qb_interface::cubePos::ConstPtr& msg);

  void callback_gain_PID1(const geometry_msgs::Vector3::ConstPtr& msg);
  // void callback_imu_acc(const qb_interface::inertialSensorArray::ConstPtr& msg);
  // void callback_imu_gyro(const qb_interface::inertialSensorArray::ConstPtr& msg);
  int sgn(double d);

  ros::Subscriber sub_gyro_, sub_euler_, sub_gain_;
  ros::Publisher pub_comm_, pub_to_matlab;
  ros::Subscriber sub_corr, sub_enc;
  geometry_msgs::Vector3 to_matlab;

  Eigen::Vector3d gyro_, euler_;
  double kp_, kd_, ki_;
  double PD, P, D, I;
  double int1, int2;
  double enc1_, enc2_, enc1_old_, enc2_old_, enc1_of_, enc2_of_;
  bool flag_run1_, flag_run2_, flag_run3_;
  ros::NodeHandle n_;

};//End of class SubscribeAndPublish