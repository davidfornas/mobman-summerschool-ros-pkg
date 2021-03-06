#include <Servo.h>

Servo left_servo, right_servo;   // Servos for the wheels
Servo sonar_servo;               // Servo for the sonar ranger
Servo jaw_servo;


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
		//sense12v();
        /*}else if(command=="OPEN"){
		openGripper();
        }else if(command=="CLOSE"){
		closeGripper();*/
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


void openGripper(){   
	Serial.print("Opening gripper.");
	jaw_servo.write(80);          
} 

void closeGripper(){   
	Serial.print("Closing gripper.");
	jaw_servo.write(40);          
} 

void setup()
{
	// Servo pins
	right_servo.attach(9);        // Right servo is pin 9 (OUTPUT 2 of TINKERKIT)
	left_servo.attach(10);       // Left servo is pin 10 (OUTPUT 1 of TINKERKIT)
	sonar_servo.attach(11);    // Sonar servo is pin 11 (OUTPUT 0 of TINKERKIT)
	jaw_servo.attach(5);    // Gripper servo is pin 5 (OUTPUT 4 of TINKERKIT)

	//  Init position
	moveWheels("100", "90");
	moveSonar("70"); 
	///openGripper();     


	// Open serial communications and wait for port to open:
	Serial.begin(9600);
	Serial.println("Let's start");

}

void loop()
{

  
	moveWheels("100", "90");
  
	delay(20); 
}







