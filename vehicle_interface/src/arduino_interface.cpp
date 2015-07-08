#include <vehicle_interface/arduino_interface.h>

#include <sstream>
#include <string>

void ArduinoInterface::wheelsCallback(const std_msgs::Int8MultiArray::ConstPtr& msg)
{
  sendCmd("WHEELS", msg->data[0], msg->data[1]);
  ROS_INFO_STREAM("Sent " << msg->data[0] << ", " << msg->data[1]  << " to the wheels.");
}

void ArduinoInterface::sonarServoCallback(const std_msgs::Int32::ConstPtr& msg)
{
  sendCmd("SERVO", msg->data, 0);
  ROS_INFO_STREAM("Sent " << msg->data << " to sonar servo.");
  sonar_servo_state_ = msg->data;
}

void ArduinoInterface::sendCmd(std::string cmd, int arg1, int arg2)
{
  std::ostringstream s;
  s << cmd << "+" << arg1 << "+" << arg2;
  const std::string full_cmd(s.str());

  const int buf_max = 256;
  char buf[buf_max];
  int rc;

  //Write
  strncpy(buf, full_cmd.c_str(), buf_max);

  rc = arduino_->writeText(buf);
  if (rc == -1)
    ROS_ERROR("error writing");
  else
    ROS_DEBUG("write ok");

}

void ArduinoInterface::querySonar()
{
  sendCmd("SONAR", 0, 0);

  const int buf_max = 256;
  char eolchar = '\n';
  char buf[buf_max];
  int rc, n = 50;

  //Read
  memset(buf, 0, buf_max); //
  arduino_->readUntil(buf, eolchar, buf_max, timeout_);
  sonar_ = atof(buf);
  ROS_INFO_STREAM("Read: "<< buf);
}

sensor_msgs::LaserScan ArduinoInterface::performSonarScan( float angle_min, float angle_increment )
{
  sensor_msgs::LaserScan scan;
  scan.angle_min = angle_min*3.14/180;
  scan.angle_max = (160 - angle_min)*3.14/180;
  scan.angle_increment = angle_increment*3.14/180;

  scan.header.frame_id = "/sonar";
  scan.header.stamp = ros::Time::now();

  scan.range_min=0.0;
  scan.range_max=10.0;

  //For this application angle_max is mirrored from 90 deg.
  for(float angle = angle_min; angle <= 160 - angle_min; angle += angle_increment){
    sendCmd("SERVO", (int) angle, 0);
    ros::Duration(2.0).sleep();
    double value=0;
    for(int i=0;i<10;i++){
      querySonar();
      ros::Duration(0.3).sleep();
      value+=sonar_;
    }
    scan.ranges.push_back(value*0.0001);
  }
  ROS_INFO_STREAM(scan);

  return scan;
}


void ArduinoInterface::publishLaserScan(float init_angle=10, float angle_incr=10){
  sensor_msgs::LaserScan scan=performSonarScan(10,10);
  laserScan_pub.publish(scan);
}

