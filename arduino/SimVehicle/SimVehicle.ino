	#include <SoftwareSerial.h>

	SoftwareSerial mySerial(2, 3); // RX, TX
	String command = "", param=""; // Stores response of bluetooth device
	                     // which simply allows \n between each
	                     // response.

        void moveMotor(String param){
          //Reading parameters
          Serial.print("Moving wheel: ");
          Serial.println(param);
          
          //Response
          mySerial.println("MOVED");
        }
        
        void sense(String param){
          //Reading parameters
          Serial.println("Sensing...");
          
          //Response
          mySerial.println("TEMPERATURE+25");
        }        
        
        void issueCommand(String command, String param){
         if(command=="MOTOR"){
          moveMotor(param);
         }else if(command=="SENSE"){
           sense(param);
         }else{   
          mySerial.println("BAD COMMAND");
         }        
   
        }

	void setup()  
	{
	  // Open serial communications and wait for port to open:
	  Serial.begin(9600);
	  Serial.println("Let's see if I receive something over BT...");

	  // SoftwareSerial "com port" data rate. JY-MCU v1.03 defaults to 9600.
	  mySerial.begin(9600);
	}

	void loop()
	{
	  // Polling. Read BT input if available.
	  if (mySerial.available()) {
            boolean readingParam=false;
	    while(mySerial.available()) { // While there is more to be read, keep reading.
              char r = (char)mySerial.read();
	      if(r!='+'&&!readingParam){
                command += r; 
              }
              if(readingParam){
               param += r; 
              }
              if(r=='+'){
               readingParam=true; 
              }
	    }
	    Serial.print("Received command: ");
	    Serial.println(command);
	    Serial.print("With parameter: ");
	    Serial.println(param);
            issueCommand(command, param);
            command = ""; // No repeats
            param="";
	  }

	}// END loop()
