#include <stdio.h>    // Standard input/output definitions
#include <stdlib.h>
#include <string.h>   // String function definitions
#include <unistd.h>   // for usleep()
#include <getopt.h>

#include <vehicle_interface/arduino_serial.h>
#include <vehicle_interface/arduino_interface.h>

//
void error(char* msg)
{
  fprintf(stderr, "%s\n", msg);
  exit(EXIT_FAILURE);
}

int arduino_serial_test(int argc, char *argv[]){

  const int buf_max = 256;

  int fd = -1;
  char serialport[buf_max];
  int baudrate = 9600; // default
  char eolchar = '\n';
  int timeout = 5000;
  char buf[buf_max];
  int rc, n = 50;

  ArdunioSerial serial("/dev/rfcomm1", baudrate);
  if (!serial.ok())
    return -1;

  //Write
  sprintf(buf, "SENSE");
  rc = serial.writeText(buf);
  if (rc == -1)
    error("error writing");
  else
    printf("write ok\n");

  //Read
  memset(buf, 0, buf_max); //
  serial.readUntil(buf, eolchar, buf_max, timeout);
  printf("%s\n", buf);

  //Write
  sprintf(buf, "MOTOR+3");
  rc = serial.writeText(buf);
  if (rc == -1)
    error("error writing");
  else
    printf("write ok\n");

  //Read
  memset(buf, 0, buf_max); //
  serial.readUntil(buf, eolchar, buf_max, timeout);
  printf("%s\n", buf);
}

int main(int argc, char *argv[])
{

  //return arduino_serial_test(argc, argv);
  ArduinoInterface interface("/dev/rfcomm1");


  //while(1){
  //ASK FOR SENSORS...

  //SEND MOTOR CTRL...

  //}

} // end main

