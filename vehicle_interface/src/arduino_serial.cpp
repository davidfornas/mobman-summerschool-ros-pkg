//
// arduino-serial-lib -- simple library for reading/writing serial ports
//
// 2006-2013, Tod E. Kurt, http://todbot.com/blog/
// Improved by David Fornas (C++)
//

#include <vehicle_interface/arduino_serial.h>

// uncomment this to debug reads
//#define SERIALPORTDEBUG 

// takes the string name of the serial port (e.g. "/dev/tty.usbserial","COM1")
// and a baud rate (bps) and connects to that port at that speed and 8N1.
// opens the port in fully raw mode so you can send binary data.
// returns valid fd_, or -1 on error
int ArdunioSerial::init(const char* serialport, int baud)
{
  struct termios toptions;

  //fd_ = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY);
  fd_ = open(serialport, O_RDWR | O_NONBLOCK);

  if (fd_ == -1)
  {
    perror("serialport_init: Unable to open port ");
    return -1;
  }

  //int iflags = TIOCM_DTR;
  //ioctl(fd_, TIOCMBIS, &iflags);     // turn on DTR
  //ioctl(fd_, TIOCMBIC, &iflags);    // turn off DTR

  if (tcgetattr(fd_, &toptions) < 0)
  {
    perror("serialport_init: Couldn't get term attributes");
    return -1;
  }
  speed_t brate = baud; // let you override switch below if needed
  switch (baud)
  {
    case 4800:
      brate = B4800;
      break;
    case 9600:
      brate = B9600;
      break;
#ifdef B14400
      case 14400: brate=B14400; break;
#endif
    case 19200:
      brate = B19200;
      break;
#ifdef B28800
      case 28800: brate=B28800; break;
#endif
    case 38400:
      brate = B38400;
      break;
    case 57600:
      brate = B57600;
      break;
    case 115200:
      brate = B115200;
      break;
  }
  cfsetispeed(&toptions, brate);
  cfsetospeed(&toptions, brate);

  // 8N1
  toptions.c_cflag &= ~PARENB;
  toptions.c_cflag &= ~CSTOPB;
  toptions.c_cflag &= ~CSIZE;
  toptions.c_cflag |= CS8;
  // no flow control
  toptions.c_cflag &= ~CRTSCTS;

  //toptions.c_cflag &= ~HUPCL; // disable hang-up-on-close to avoid reset

  toptions.c_cflag |= CREAD | CLOCAL; // turn on READ & ignore ctrl lines
  toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

  toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
  toptions.c_oflag &= ~OPOST; // make raw

  // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
  toptions.c_cc[VMIN] = 0;
  toptions.c_cc[VTIME] = 0;
  //toptions.c_cc[VTIME] = 20;

  tcsetattr(fd_, TCSANOW, &toptions);
  if (tcsetattr(fd_, TCSAFLUSH, &toptions) < 0)
  {
    perror("init_serialport: Couldn't set term attributes");
    return -1;
  }
  return fd_;
}

//
int ArdunioSerial::writeByte(uint8_t b)
{
  int n = write(fd_, &b, 1);
  if (n != 1)
    return -1;
  return 0;
}

//
int ArdunioSerial::writeText(const char* str)
{
  if(!ok()) return -1;
  int len = strlen(str);
  int n = write(fd_, str, len);
  if (n != len)
  {
    perror("serialport_write: couldn't write whole string\n");
    return -1;
  }
  return n;
}

//
int ArdunioSerial::readUntil(char* buf, char until, int buf_max, int timeout)
{
  if(!ok()) return -1;
  char b[1]; // read expects an array, so we give it a 1-byte array
  int i = 0;
  do
  {
    int n = read(fd_, b, 1); // read a char at a time
    if (n == -1)
      return -1; // couldn't read
    if (n == 0)
    {
      usleep(1 * 1000); // wait 1 msec try again
      timeout--;
      continue;
    }
#ifdef SERIALPORTDEBUG  
    printf("serialport_read_until: i=%d, n=%d b='%c'\n",i,n,b[0]); // debug
#endif
    buf[i] = b[0];
    i++;
  } while (b[0] != until && i < buf_max && timeout > 0);

  buf[i] = 0; // null terminate the string
  return 0;
}

//
int ArdunioSerial::flush()
{
  if(!ok()) return -1;
  sleep(2); //required to make flush work, for some reason
  return tcflush(fd_, TCIOFLUSH);
}
