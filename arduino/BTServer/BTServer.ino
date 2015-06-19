	#include <SoftwareSerial.h>

	SoftwareSerial mySerial(2, 3); // RX, TX
	String command = ""; // Stores response of bluetooth device
	                     // which simply allows \n between each
	                     // response.

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
	  // Read BT input if available.
	  if (mySerial.available()) {
	    Serial.println("Receiving from BT: ");
	    while(mySerial.available()) { // While there is more to be read, keep reading.
	      command += (char)mySerial.read();
	    }
	    Serial.println(command);
	    command = ""; // No repeats
            delay(50);            
	    Serial.println("OK");
            mySerial.println("OK");
	  }
	
          // Read user input from PC if available an send if through BT.
	  if (Serial.available()) {
            delay(10);
	    while(Serial.available()) { // While there is more to be read, keep reading.
	      command += (char)Serial.read();
	    }
            Serial.println("Received from the computer: ");
	    Serial.println(command);
            mySerial.println(command);
	    command = ""; // No repeats
	  }

	}// END loop()
