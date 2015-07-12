#include <Servo.h>
#include <SoftwareSerial.h>

// Bluetooth virtual serial port
SoftwareSerial mySerial(6, 7); // RX, TX

Servo left_servo, right_servo, jaw_servo; // Servos for the wheels
Servo sonar_servo;               // Servo for the sonar ranger

#define SENSOR 0     // A/D sonar port
#define MEASURE 2   // Pin to enable sonar measuring

//Make actions depending on the received command
void issueCommand(String command, String param1, String param2){
	if(command=="SERVO"){
		moveSonar(param1);   
	}else if(command=="WHEELS"){
		moveWheels(param1, param2);
	}else if(command=="SONAR"){
		// WARNING! Choose 5V or 12V depending on the sonar model.
		sense5v();
	}else{   
		mySerial.println("Unrecognized Command");
	}        
} 

//Move wheel servos
void moveWheels(String lspd, String rspd){
	Serial.print("Moving left wheel at speed: ");
	Serial.println(lspd.toInt());
	left_servo.write(lspd.toInt());  

	Serial.print("Moving right wheel at speed: ");
	Serial.println(rspd.toInt());
	right_servo.write(rspd.toInt());
}   

//Move sonar servo to a given position
void moveSonar(String spd){   
	Serial.print("Moving sonar to position: ");
	Serial.println(spd.toInt());
	sonar_servo.write(spd.toInt());          
}  

//Use 5V sonar (analog)
void sense5v(){
	digitalWrite(MEASURE, HIGH);  // Enable measuring
	delay(10);                          
	
        Serial.println("Sensing...");
	int distance = analogRead(SENSOR);    // Read measure
	Serial.print("Distance... ");
	Serial.print(distance);
	Serial.print(", which is ");
	float mm = (float)distance / 2 * 25.4 / 100;    // Conver D/A valuer to mm
	Serial.print(mm);
	Serial.println(" mm");
	mySerial.println(mm);

	digitalWrite(MEASURE, LOW);          // Disable measuring  
}    

//Use 12V sonar (PWM)
void sense12v(){ 

	int ok = LOW;  // Valid measure
	unsigned long before = micros();

	digitalWrite(MEASURE, HIGH);          // Activate distance sonar
	delay(3);                           // >2ms pulse to activate measuring
	digitalWrite(MEASURE, LOW);           // Stop measuring pulse
	int pos = 0;
	do
	{
		ok = digitalRead(3);
		pos += 1;
	}
	while (ok);                       // Wait for echo to disapear
	do
	{

		ok = digitalRead(3);
		pos +=1;
	}
	while (!ok);                       // Wait for measure

	unsigned long now = micros();     // Echo receive time
	int count = now - before;             // Measuring time in us
	Serial.print("pos = ");
	Serial.println(pos);
	Serial.print("Distance... ");
	Serial.print(count);
	Serial.print(", which is ");
	float mm = (float)count * (float)0.170;    // Conviert PWM value to mm
	Serial.print(mm);
	Serial.println(" mm.");
	mySerial.println(mm);
}   

void setup()
{
	// Servo pins
	right_servo.attach(9);        // Right servo is pin 9 (OUTPUT 2 of TINKERKIT)
	left_servo.attach(10);       // Left servo is pin 10 (OUTPUT 1 of TINKERKIT)
	sonar_servo.attach(11);    // Sonar servo is pin 11 (OUTPUT 0 of TINKERKIT)
	//jaw_servo.attach(5);    // Sonar servo is pin 11 (OUTPUT 0 of TINKERKIT)

	//  Init position
	moveWheels("90", "90");
	moveSonar("90");    

	pinMode(MEASURE, OUTPUT);    // Measuring start signal
	digitalWrite(MEASURE, LOW);      // Avoid start measuring

	// Open serial communications and wait for port to open:
	Serial.begin(9600);
	Serial.println("Let's see if I receive something over BT...");

	// SoftwareSerial "com port" data rate
	mySerial.begin(9600);
}

void loop()
{
        //The main loop processes command one by one, the commands have the following
        //structure: COMMAND+PARAM1+PARAM2.
        
	String command = "", param1="", param2=""; // Stores response of bluetooth device
	// Polling. Read BT input if available.
	if (mySerial.available()) {
		boolean readingParam1=false, readingParam2=false;
		while(mySerial.available()) { // While there is more to be read, keep reading.
			char r = (char)mySerial.read();
			if( r != '+' && !readingParam1 && !readingParam2){
				command += r; 
			}
			if( r != '+' && readingParam1 && !readingParam2){
				param1 += r; 
			}
			if( r != '+' && readingParam2){
				param2 += r; 
			}
			if(r=='+'){
				if(readingParam1) readingParam2=true;
				readingParam1=true; 
			}
		}
		Serial.print("Received command: ");    Serial.println(command);
		Serial.print("With parameter 1: ");       Serial.println(param1);
		Serial.print("With parameter 2: ");       Serial.println(param2);
		issueCommand(command, param1, param2);
	}
	delay(20); 
}







