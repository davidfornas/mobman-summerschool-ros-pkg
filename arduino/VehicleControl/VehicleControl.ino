#include <Servo.h>
#include <SoftwareSerial.h>
 
SoftwareSerial mySerial(6, 7); // RX, TX

void issueCommand(String command, String param1, String param2){
  if(command=="SERVO"){
    moveSonar(param1);
  }else if(command=="WHEELS"){
    moveWheels(param1, param2);
  }else if(command=="SONAR"){
    sense();
  }else{   
    mySerial.println("BAD COMMAND");
  }        
} 
 
Servo left_servo, right_servo;     // Servos for the wheels
Servo sonar_servo;  // Servo for the sonar ranger
int pos = 90;     // Right servo position

// Uso de interrupciones para medir 
#define INTERRUPT_INPUT 3        // Entrada asignada a la interrupción
#define SENSOR 0        // Puerto A/D del medidor de distancias
#define MEASURE 2          // Pin para activar la medición

int distance = 0;      // A/D conversion value
float mm = 0;          // A/D value to mm
int servo_counter=0; //To shutdown the servos after x miliseconds

volatile int count = 0;
volatile int ok = LOW;  // Valid measure
volatile unsigned long now = 0;
volatile unsigned long before = 0;

void moveWheels(String lspd, String rspd){

  Serial.print("Moving left wheel at speed: ");
  Serial.println(lspd.toInt());
  left_servo.write(lspd.toInt());  
  
  Serial.print("Moving right wheel at speed: ");
  Serial.println(rspd.toInt());
  right_servo.write(rspd.toInt());  
  mySerial.println("MOVED");
  servo_counter=0;
}   

 void moveSonar(String spd){   
  //Reading parameters
  Serial.print("Moving sonar to position: ");
  Serial.println(spd.toInt());
  sonar_servo.write(spd.toInt());  
  mySerial.println("MOVED");                
}  
        
void sense(){
  attachInterrupt(1, measure, CHANGE);   // Activa interrupción en pin de entrada (3)  
  digitalWrite(MEASURE, HIGH);          // Activa medidor de distancia
  delay(10);                          // Para dar tiempo a realizar la primera medición

  //Reading parameters
  Serial.println("Sensing...");
  distance = analogRead(SENSOR);    // Realiza medición de distancia
  Serial.print("Distancia... ");
  Serial.print(distance);
  Serial.print(" cuentas = ");
  mm = distance / 2 * 25,4 / 100;    // Convierte valor del D/A a mm
  Serial.print(mm);
  Serial.println(" milimetros");
  
  while (!ok); // Esperamos a una medición válida
  Serial.print("Distancia... ");
  Serial.print(count);
  Serial.print(" cuentas = ");
  mm = count / 5,7874 - 170;    // Convierte valor del PWM a mm
  Serial.print(mm);
  Serial.println(" milimetros");
  Serial.println(" ");
  //Response
  mySerial.println(mm);
  
  digitalWrite(MEASURE, LOW);          // Inhibe mediciones  
  detachInterrupt(1);            // Inhabilita interrupción
}        

void setup()
{
    // Asigna pines a los servos
  right_servo.attach(9);        // Asigna el servo derecho al pin 9 (OUTPUT 2 de la TINKERKIT)
  left_servo.attach(10);       // Asigna el servo izquierdo al pin 10 (OUTPUT 1 de la TINKERKIT)
  sonar_servo.attach(11);    // Asigna el miniservo al pin 11 (OUTPUT 0 de la TINKERKIT)
    
  //  Init position
  moveWheels("90", "90");
  moveSonar("90");    
  
  pinMode(INTERRUPT_INPUT, INPUT);           // Pin de entrada del medidor
  pinMode(MEASURE, OUTPUT);            // Señal de disparo del medidor
  digitalWrite(MEASURE, LOW);          // Inhibe mediciones
  
  	  // Open serial communications and wait for port to open:
	  Serial.begin(9600);
	  Serial.println("Let's see if I receive something over BT...");

	  // SoftwareSerial "com port" data rate. JY-MCU v1.03 defaults to 9600.
	  mySerial.begin(9600);
}
 
void loop()
{
     String command = "", param1="", param2=""; // Stores response of bluetooth device
	                     // which simply allows \n between each
	                     // response.
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
	    Serial.print("Received command: ");
	    Serial.println(command);
	    Serial.print("With parameter 1: ");
	    Serial.println(param1);
	    Serial.print("With parameter 2: ");
	    Serial.println(param2);
            issueCommand(command, param1, param2);

	  }
  delay(20);                   // Insertamos una espera
 
}

// Rutina de servicio de la interrupción
void measure()
{
  ok = digitalRead(INTERRUPT_INPUT);
  if (ok){
    before = micros();
    ok = LOW;
  } 
  else {
    now = micros();
    count = now - before;
    ok = HIGH;
  }
}
  



