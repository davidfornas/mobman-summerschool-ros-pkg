#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>

int error_x=0;

void errorCallback(const std_msgs::Int32::ConstPtr& msg){
  error_x = msg->data;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "object_search");
  ros::NodeHandle nh;

  ros::Publisher wheels_pub;
  ros::Publisher gripper_pub;

  ros::Subscriber error_x_sub;

  wheels_pub = nh.advertise<std_msgs::Int32MultiArray>("/wheels_cmd", 1);
  gripper_pub = nh.advertise<std_msgs::Int32>("/gripper", 1);

  error_x_sub = nh.subscribe("/error_x", 1, errorCallback);

  ros::Rate r(1);

  while(ros::ok()){

    std_msgs::Int32MultiArray control;
    std_msgs::Int32 controlGripper;

    control.data.push_back(90);
    control.data.push_back(90);


    if(error_x > 600){
    control.data[0] = 80;
    control.data[1] = 80;
    wheels_pub.publish(control);
    }else if(error_x < 400){
      control.data[0] = 100;
      control.data[1] = 100;
      wheels_pub.publish(control);
    }

    ros::Duration(0.5).sleep();

    control.data[0] = 90;
    control.data[1] = 90;
    wheels_pub.publish(control);

    //Alternatively, use Gripper to notify user

    /*
    if(error_x > 100){
        controlGripper.data = 1;
        gripper_pub.publish(controlGripper);
    }else if(error_x < 100){
      controlGripper.data = 0;
      gripper_pub.publish(controlGripper);
    }
    */

    r.sleep();
    ros::spinOnce();
  }
} // end main


