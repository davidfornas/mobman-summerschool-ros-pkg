#include <Servo.h>

Servo left_servo, right_servo;     // Servos for the wheels
Servo sonar_servo;  // Servo for the sonar ranger
int pos = 90;     // Right servo position

// Uso de interrupciones para medir 
#define INTERRUPT_INPUT 3        // Entrada asignada a la interrupción
#define SENSOR 0        // Puerto A/D del medidor de distancias
#define MEASURE 2          // Pin para activar la medición

int distance = 0;      // A/D conversion value
float mm = 0;          // A/D value to mm
int incomingByte;

volatile int count = 0;
volatile int ok = LOW;  // Valid measure
volatile unsigned long now = 0;
volatile unsigned long before = 0;

     

void setup()
{
  // Asigna pines a los servos
  right_servo.attach(9);        // Asigna el servo derecho al pin 9 (OUTPUT 2 de la TINKERKIT)
  left_servo.attach(10);       // Asigna el servo izquierdo al pin 10 (OUTPUT 1 de la TINKERKIT)
  sonar_servo.attach(11);    // Asigna el miniservo al pin 11 (OUTPUT 0 de la TINKERKIT)

  // Inmoviliza servos  
  right_servo.write(90);  
  left_servo.write(90);  
  sonar_servo.write(90);    
  
  pinMode(INTERRUPT_INPUT, INPUT);           // Pin de entrada del medidor
  pinMode(MEASURE, OUTPUT);            // Señal de disparo del medidor
  Serial.begin(9600);                // Inicializamos la comunicación serie  
  digitalWrite(MEASURE, LOW);          // Inhibe mediciones
}
 
void loop()
{
  attachInterrupt(1, measure, CHANGE);   // Activa interrupción en pin de entrada (3)  
  digitalWrite(MEASURE, HIGH);          // Activa medidor de distancia
  delay(10);                          // Para dar tiempo a realizar la primera medición
  
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

  digitalWrite(MEASURE, LOW);          // Inhibe mediciones  
  
  detachInterrupt(1);            // Inhabilita interrupción
 
  // Mueve servo derecho adelante
  
  for(pos = 90; pos<=180; pos+=10)     
  {                                
    right_servo.write(pos);      
    left_servo.write(pos);   
    sonar_servo.write(pos-45);              
    delay(100);                       
  }
  for(pos = 180; pos>=90; pos-=10)     
  {                                
    right_servo.write(pos);              
    left_servo.write(pos);   
    sonar_servo.write(pos-45);              
    delay(100);                       
  }  

      sonar_servo.write(90);
  delay(5000);                   // Insertamos una espera
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
  



