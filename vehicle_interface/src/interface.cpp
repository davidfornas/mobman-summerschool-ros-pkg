#include <stdio.h>    // Standard input/output definitions
#include <stdlib.h>
#include <string.h>   // String function definitions
#include <unistd.h>   // for usleep()
#include <getopt.h>

#include <vehicle_interface/arduino_serial.h>

//
void error(char* msg)
{
  fprintf(stderr, "%s\n", msg);
  exit(EXIT_FAILURE);
}

int main(int argc, char *argv[])
{
  const int buf_max = 256;

  int fd = -1;
  char serialport[buf_max];
  int baudrate = 9600; // default
  char quiet = 0;
  char eolchar = '\n';
  int timeout = 5000;
  char buf[buf_max];
  int rc, n=50;

  //If open, close it
  if (fd != -1)
  {
    serialport_close(fd);
    if (!quiet)
      printf("closed port %s\n", serialport);
  }

  //Init port
  fd = serialport_init("/dev/rfcomm1", baudrate);
  if (fd == -1)
    error("couldn't open port");
  if (!quiet)
    printf("opened port %s\n", serialport);
  serialport_flush(fd);

  //Write
  sprintf(buf, "SAYONARA; BABY");
  if (fd == -1)
    error("serial port not opened");
  rc = serialport_write(fd, buf);
  if (rc == -1)
    error("error writing");
  else
    printf("write ok\n");


  //Read
  if (fd == -1)
    error("serial port not opened");
  memset(buf, 0, buf_max); //
  serialport_read_until(fd, buf, eolchar, buf_max, timeout);
  if (!quiet)
    printf("read string:");
  printf("%s\n", buf);

  //Flush
  if (fd == -1)
    error("serial port not opened");
  if (!quiet)
    printf("flushing receive buffer\n");
  serialport_flush(fd);

  while(1){
    //ASK FOR SENSORS...

    //SEND MOTOR CTRL...

  }

} // end main

