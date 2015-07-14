#include <ros/ros.h>
#include <vehicle_interface/arduino_interface.h>


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "vehicle_inrface");
  ros::NodeHandle nh;

  std::string device("/dev/rfcomm1");
  nh.getParam("device", device);

  ArduinoInterface interface(device.c_str());

  ros::Rate r(25);
  int count = 0;

  interface.moveWheels(90, 90);

  while(ros::ok()){
    if(count % 100 == 0) interface.publishSonar();
    //interface.publishLaserScan(10,10);
    r.sleep();
    ros::spinOnce();
    count++;
  }
}

