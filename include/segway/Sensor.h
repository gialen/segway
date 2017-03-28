#include <ros/ros.h>
#include <ros_myo/EmgArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <eigen3/Eigen/Eigen>
// ROS cuustom msg
#include <qb_interface/inertialSensor.h>
#include <qb_interface/inertialSensorArray.h>

# define PI 3.14159


class Sensor
{
public:
  Sensor();
  ~Sensor();


  void run();
  // Eigen::Quaterniond madgwick_kin(Eigen::Vector3d acc, Eigen::Vector3d gyro, Eigen::Quaterniond q_old, double State_joint_prev, Eigen::Vector3d Joint_prev, double State_joint_nxt, Eigen::Vector3d Joint_nxt);


  double dt;



private:

  void callback_imu_acc(const qb_interface::inertialSensorArray::ConstPtr& msg);
  void callback_imu_gyro(const qb_interface::inertialSensorArray::ConstPtr& msg);

  void offset_gyro(Eigen::Vector3d gyro1, Eigen::Vector3d gyro2);

  ros::Subscriber sub_imu_acc_, sub_imu_gyro_;
  ros::Publisher pub_gyro_, pub_acc_, pub_q_est_;

  Eigen::Vector3d acc_1_, gyro_1_;
  Eigen::Vector3d acc_2_, gyro_2_;
  int step_;
  Eigen::Vector3d offset_1_, offset_2_;
  bool flag_run1_, flag_run2_, flag_offset_;
  ros::NodeHandle n_; 

  int n_sample_;
  Eigen::MatrixXd data_1_, data_2_;
  Eigen::Vector3d gyro1_old_, gyro2_old_;

  Eigen::Vector3d gyro_old_;
  Eigen::Vector3d acc_old_;
  Eigen::Quaterniond q_old_;
  geometry_msgs::Quaternion q_est_pub_;

};//End of class SubscribeAndPublish