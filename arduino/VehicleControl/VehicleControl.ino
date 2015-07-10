#include <Servo.h>
#include <SoftwareSerial.h>

// Bluetooth virtual serial port
SoftwareSerial mySerial(6, 7); // RX, TX

Servo left_servo, right_servo;     // Servos for the wheels
Servo sonar_servo;  // Servo for the sonar ranger

#define SENSOR 0        // A/D sonar port
#define MEASURE 2          // Pin to enable measuring

//Make actions depending on the command
void issueCommand(String command, String param1, String param2){
  if(command=="SERVO"){
      moveSonar(param1);   
  }else if(command=="WHEELS"){
    moveWheels(param1, param2);
  }else if(command=="SONAR"){
    //Choose 5V or 12V depending on the sonar model.
    sense12v();
  }else{   
    mySerial.println("Unrecognized Command");
  }        
} 

void moveWheels(String lspd, String rspd){
  Serial.print("Moving left wheel at speed: ");
  Serial.println(lspd.toInt());
  left_servo.write(lspd.toInt());  

  Serial.print("Moving right wheel at speed: ");
  Serial.println(rspd.toInt());
  right_servo.write(rspd.toInt());
}   

void moveSonar(String spd){   
  //Reading parameters
  Serial.print("Moving sonar to position: ");
  Serial.println(spd.toInt());
  sonar_servo.write(spd.toInt());          
}  

//Use 5V sonar, analog
void sense5v(){
  
  digitalWrite(MEASURE, HIGH);          // Activa medidor de distancia
  delay(10);                          // Para dar tiempo a realizar la primera medición

  //ANALOG READING, 5V  sonar
  //Reading parameters
  Serial.println("Sensing...");
  int distance = analogRead(SENSOR);    // Realiza medición de distancia
  Serial.print("Distancia... ");
  Serial.print(distance);
  Serial.print(" cuentas = ");
  float mm = (float)distance / 2 * 25.4 / 100;    // Convierte valor del D/A a mm
  Serial.print(mm);
  Serial.println(" milimetros");
  mySerial.println(mm);

  digitalWrite(MEASURE, LOW);          // Inhibe mediciones  
}    

//Use 12V sonar, PWM
void sense12v(){ 
  
  int ok = LOW;  // Valid measure
  unsigned long before = micros();
  
  digitalWrite(MEASURE, HIGH);          // Activate distance sonar
  delay(3);                           // Pulso de más de 2ms de duración para provocar medición
  digitalWrite(MEASURE, LOW);           // Stop pulse to measure
  
  int pos = 0;
  do
  {
    ok = digitalRead(3);
    pos += 1;
  }
  while (ok);                       // Espera desaparición detección propio pulso
  do
  {
    
    ok = digitalRead(3);
    pos +=1;
  }
  while (!ok);                       // Espera medición

  unsigned long now = micros();                    // Instante recepción eco
  int count = now - before;             // Tiempo transcurrido durante la medición en microsegundos
  Serial.print("pos = ");
  Serial.println(pos);
  Serial.print("Distancia... ");
  Serial.print(count);
  Serial.print(" us= ");
  float mm = (float)count * (float)0.170;    // Convierte valor del PWM a mm
  Serial.print(mm);
  Serial.println(" mm.");
  Serial.println(" ");
  mySerial.println(mm);
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
    Serial.print("Received command: ");    Serial.println(command);
    Serial.print("With parameter 1: ");       Serial.println(param1);
    Serial.print("With parameter 2: ");       Serial.println(param2);
    issueCommand(command, param1, param2);
  }
  delay(20); 
}







