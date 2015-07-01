#include <vehicle_interface/arduino_interface.h>

void ArduinoInterface::leftWheelCallback(const std_msgs::Int32::ConstPtr& msg)
{
  sendCmd("LEFT", msg->data);
  ROS_INFO_STREAM("Sent " << msg->data << " to left wheel.");
}
void ArduinoInterface::rightWheelCallback(const std_msgs::Int32::ConstPtr& msg)
{
  sendCmd("RIGHT", msg->data);
  ROS_INFO_STREAM("Sent " << msg->data << " to right wheel.");
}
void ArduinoInterface::sonarServoCallback(const std_msgs::Int32::ConstPtr& msg)
{
  sendCmd("SERVO", msg->data);
  ROS_INFO_STREAM("Sent " << msg->data << " to sonar servo.");
  sonar_servo_state_ = msg->data;
}

void ArduinoInterface::sendCmd(std::string cmd, int arg)
{

  const int buf_max = 256;
  char buf[buf_max];
  int rc;

  //Write
  strncpy(buf, cmd.c_str(), buf_max);

  rc = arduino_->writeText(buf);
  if (rc == -1)
    ROS_ERROR("error writing");
  else
    ROS_DEBUG("write ok");

}

void ArduinoInterface::querySonar()
{
  sendCmd("SONAR", 0);

  const int buf_max = 256;
  char eolchar = '\n';
  char buf[buf_max];
  int rc, n = 50;

  //Read
  memset(buf, 0, buf_max); //
  arduino_->readUntil(buf, eolchar, buf_max, timeout_);
  sonar_ = atoi(buf);
  ROS_DEBUG_STREAM("Read: "<< buf);
}

