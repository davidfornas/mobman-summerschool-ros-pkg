#include <Servo.h>

Servo left_servo, right_servo;   // Servos for the wheels
Servo sonar_servo;               // Servo for the sonar ranger
Servo jaw_servo;

int i=0;


#define SENSOR 0     // A/D sonar port
#define MEASURE 2   // Pin to enable sonar measuring


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
	jaw_servo.write(20);          
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
	closeGripper();     


	// Open serial communications and wait for port to open:
	Serial.begin(9600);
	Serial.println("Let's start");

}

void loop()
{
        if(i==50) {openGripper(); }
        if(i==150) {closeGripper(); }
        
        if(i==100) {moveSonar("50");}
        if(i==200) {moveSonar("90");}
  
        if(i<=25) {moveWheels("70", "110");}
        if(i>25 && i<=50) {moveWheels("90", "110");}
        if(i>75 && i<=100) {moveWheels("110", "90");}
        if(i>125 && i<=150) {moveWheels("70", "110");}
        if(i>150 && i<=175) {moveWheels("120", "80");}
        if(i>200 && i<=225) {moveWheels("70", "100");}
        if(i>225 && i<=250) {moveWheels("90", "90");}
        if(i>250 && i<=275) {moveWheels("70", "110");}
        if(i>275 && i<=300) {moveWheels("90", "130");}
        if(i==301) {i=0;}
  
        i=i+1;
	delay(20); 
}







