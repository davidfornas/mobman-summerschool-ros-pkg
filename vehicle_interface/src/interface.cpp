#include <ros/ros.h>
#include <vehicle_interface/arduino_interface.h>


int main(int argc, char *argv[])
{

  ros::NodeHandle nh;

  std::string device("/dev/rfcomm1");
  nh.getParam("device", device);

  ArduinoInterface interface(device.c_str());

  ros::Rate r(10);

  while(ros::ok()){
    interface.publishSonar();
    r.sleep();
    ros::spinOnce();
  }

}

