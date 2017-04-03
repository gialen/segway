#include <ros/ros.h>
#include <ros/rate.h>

// my headers
#include "PID.h"


//-----------------------------------------------------
//                                                 main
//-----------------------------------------------------
int main(int argc, char **argv)
{
  double rateHZ = 200;

  ros::init(argc, argv, "PID_node");
  ros::NodeHandle nh;

  PID Obj;

  ros::Rate r(rateHZ);


  while(ros::ok())
  {
    Obj.run();

    ros::spinOnce();
    r.sleep();
        
  }// end while()
return 0;
}