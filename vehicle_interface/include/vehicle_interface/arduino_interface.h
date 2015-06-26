/**
 * Arduino Interface class -- used to bridge ROS and ARDUINO
 *
 * Authored in C by David Fornas (IRSLab) 06/2015
 *
 */
#ifndef __ARDUINOINTERFACE_H__
#define __ARDUINOINTERFACE_H__

#include <vehicle_interface/arduino_serial.h>

/** Interface between ROS and Arduino */
class ArdunioInterface
{

  boost::shared_ptr<ArdunioSerial> arduino_;

public:

  /** Constructor.
   * @param serialport Serial device, usually similar to /det/ttyACM0 or /dev/rfcomm0
   * */
  ArdunioInterface()
  {

  }
  ~ArdunioInterface()
  {
  }

private:


};

#endif  /* __ARDUINOINTERFACE_H__ */

