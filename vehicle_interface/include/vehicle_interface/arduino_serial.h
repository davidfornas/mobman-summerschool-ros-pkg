/**
 * Arduino Serial class -- simple library for reading/writing serial ports
 *
 * Authored in C by 2006-2013, Tod E. Kurt, http://todbot.com/blog/
 * Improved and C++ class by David Fornas (IRSLab)
 *
 */
#ifndef __ARDUINOSERIAL_H__
#define __ARDUINOSERIAL_H__

#include <stdint.h>   // Standard types 
#include <stdio.h>    // Standard input/output definitions
#include <unistd.h>   // UNIX standard function definitions
#include <fcntl.h>    // File control definitions
#include <errno.h>    // Error number definitions
#include <termios.h>  // POSIX terminal control definitions
#include <string.h>   // String function definitions
#include <sys/ioctl.h>

/** Connect to a serial port and offer  read/write operations. */
class ArdunioSerial
{

  int fd_;

public:

  /** Constructor.
   * @param serialport Serial device, usually similar to /det/ttyACM0 or /dev/rfcomm0
   * */
  ArdunioSerial(const char* serialport, int baud)
  {
    init(serialport, baud);
  }

  /** Write binary data into serial port */
  int writeByte(uint8_t b);

  /** Write a text string into serial port */
  int writeText(const char* str);

  /** Read until the buffer is full or the timeout has finished */
  int readUntil(char* buf, char until, int buf_max, int timeout);

  /** Empty read buffer */
  int flush();

  /** Check if the port has been opende perfectly */
  bool ok()
  {
    return fd_ != -1;
  }

  ~ArdunioSerial()
  {
    close(fd_);
  }

private:

  int init(const char* serialport, int baud);

};

#endif  /* __ARDUINOSERIAL_H__ */

