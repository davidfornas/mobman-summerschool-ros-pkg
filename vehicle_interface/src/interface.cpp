#include <ros/ros.h>
#include <vehicle_interface/arduino_interface.h>


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "vehicle_inrface");
  ros::NodeHandle nh;

  std::string device("/dev/rfcomm1");
  nh.getParam("device", device);

  ArduinoInterface interface(device.c_str());

  ros::Rate r(2);

  while(ros::ok()){
    //interface.publishSonar();
    //interface.performSonarScan(10,10);
    r.sleep();
    ros::spinOnce();
  }
}

