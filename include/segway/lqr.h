#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Eigen>
#include <math.h>


// ROS cuustom msg

#include <qb_interface/cubeRef.h>
#include <qb_interface/cubeCurrent.h>
#include <qb_interface/cubePos.h>

# define PI 3.1416
# define PI2 6.2832


class lqr
{
public:
  lqr();
  ~lqr();


  void run();
  // Eigen::Quaterniond madgwick_kin(Eigen::Vector3d acc, Eigen::Vector3d gyro, Eigen::Quaterniond q_old, double State_joint_prev, Eigen::Vector3d Joint_prev, double State_joint_nxt, Eigen::Vector3d Joint_nxt);

  double dt;



private:

  void callback_gyro(const geometry_msgs::Vector3::ConstPtr& msg);
  void callback_imu_euler(const geometry_msgs::Vector3::ConstPtr& msg);
  // void callback_gain_PID(const geometry_msgs::Vector3::ConstPtr& msg);
  // void callback_corr(const qb_interface::cubeCurrent::ConstPtr& msg);
  void callback_meas(const qb_interface::cubePos::ConstPtr& msg);

  // void callback_gain_PID_v(const geometry_msgs::Vector3::ConstPtr& msg);
  void callback_comm(const geometry_msgs::Vector3::ConstPtr& msg);
  // void callback_gain_PID_w(const geometry_msgs::Vector3::ConstPtr& msg);
  // void callback_cube(const geometry_msgs::Vector3::ConstPtr& msg);
  // void callback_myo(const sensor_msgs::Imu::ConstPtr& msg);

  // void callback_imu_acc(const qb_interface::inertialSensorArray::ConstPtr& msg);
  // void callback_imu_gyro(const qb_interface::inertialSensorArray::ConstPtr& msg);
  int sgn(double d);
  double unwrap(double previousAngle,double newAngle);
  double angleDiff(double a,double b);

  ros::Subscriber sub_gyro_, sub_euler_, sub_gain_, sub_gain_v_, sub_gain_w_, sub_comm;
  ros::Publisher pub_comm_, pub_to_matlab, pub_vel_;
  ros::Subscriber sub_corr, sub_enc;
  ros::Subscriber sub_cube_, sub_myo_;
  geometry_msgs::Vector3 to_matlab;
  geometry_msgs::Quaternion q_myo_;


  Eigen::Vector3d gyro_, euler_;
  // double kp_, kd_, ki_, kp_v_, kd_v_, ki_v_, kp_w_, kd_w_, ki_w_;
  // double Pid, P, D, I, P_v, I_v,D_v, PID_v, PID_v_old, PID_w, P_w, I_w,D_w;
  double vel_rif_, w_rif_, pos_rif_, yaw_rif_;
  double enc1_, enc2_, enc1_old_, enc2_old_, enc1_of_, enc2_of_, vel1_old_, vel2_old_, vel_old_, w_old_;
  bool flag_run1_, flag_run2_, flag_run3_;
  double th_eq_, th_pr_, m1_cube_, m2_cube_, encL_cube_;
  ros::NodeHandle n_;
  double com_R, com_L;
};//End of class SubscribeAndPublish