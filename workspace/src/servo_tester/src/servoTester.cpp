#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "time.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "servo_pan");//start node
  ros::NodeHandle n;//create handle
  ros::Publisher pan_pub = n.advertise<std_msgs::Int16>("servo_pan", 1000); //create a publisher
  ros::Publisher tilt_pub = n.advertise<std_msgs::Int16>("servo_tilt", 1000); //create a publisher

  ros::Rate loop_rate(60); //set goal refresh rate (hz)
 
  while (ros::ok())
  {
    std_msgs::Int16 cmd_msg; //Create the message
    cmd_msg.data = (int16_t) 5*sin(ros::Time::now().toSec()); //Calculate angle

    pan_pub.publish(cmd_msg); //publish message
    tilt_pub.publish(cmd_msg); //publish message

    ros::spinOnce();//Spin me right round

    loop_rate.sleep(); //Sleep to hit goal refresh rate
  }


  return 0;
}
