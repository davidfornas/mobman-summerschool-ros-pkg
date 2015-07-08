/**
 * Arduino Interface class -- used to bridge ROS and ARDUINO
 *
 * Authored in C by David Fornas (IRSLab) 06/2015
 *
 */
#ifndef __ARDUINOINTERFACE_H__
#define __ARDUINOINTERFACE_H__

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8MultiArray.h>

#include <sensor_msgs/LaserScan.h>

#include <vehicle_interface/arduino_serial.h>
#include <boost/make_shared.hpp>

/** Interface between ROS and Arduino */
class ArduinoInterface
{

  ros::NodeHandle nh_;
  boost::shared_ptr<ArdunioSerial> arduino_;

  ros::Subscriber wheels_sub_, sonar_servo_sub_;
  ros::Publisher sonar_pub_, sonar_servo_pub_;

  //Sensors...
  int sonar_servo_state_;
  int sonar_;

  //Timeout to comunicate with ArduinoSerial
  int timeout_;

public:

  /** Constructor.
   * @param serialport Serial device, usually similar to /det/ttyACM0 or /dev/rfcomm0
   * */
  ArduinoInterface( const char* serialport  )
  {
    arduino_ = boost::make_shared<ArdunioSerial>(serialport, 9600);
    if(!arduino_->ok()) ROS_ERROR("Could not open port.");

    //Use params to configure topics...
    //Publish sensors
    sonar_pub_= nh_.advertise<std_msgs::Int32>("/sonar", 1);
    sonar_servo_pub_= nh_.advertise<std_msgs::Int32>("/sonar_servo_state", 1);

    //Subscribe to vehicle commands
    wheels_sub_= nh_.subscribe("/wheels_cmd", 1, &ArduinoInterface::wheelsCallback, this);
    sonar_servo_sub_= nh_.subscribe("/sonar_servo_cmd", 1, &ArduinoInterface::sonarServoCallback, this);

    //Init sensors querying them...
    sonar_servo_state_ = 0;

    timeout_ = 500; //Serial read timeout, Default 5000;
    publishSonar();

    ROS_INFO("Vehicle topics initialized. Issue commands now.");
  }
  ~ArduinoInterface()
  {
  }

  void publishSonar(){
    querySonar();
    std_msgs::Int32 s, ss;
    s.data = sonar_;
    ss.data = sonar_servo_state_;
    sonar_pub_.publish(s);
    sonar_servo_pub_.publish(ss);
  }

  //This operations are better done with the vehicle stopped.
  void performSonarScan( float, float );

private:

  void wheelsCallback(const std_msgs::Int8MultiArray::ConstPtr& msg);
  void sonarServoCallback(const std_msgs::Int32::ConstPtr& msg);

  void sendCmd(std::string, int, int);
  void querySonar();

};

#endif  /* __ARDUINOINTERFACE_H__ */

