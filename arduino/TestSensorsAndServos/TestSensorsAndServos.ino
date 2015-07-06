// Declaraciones relativas a los servos

#include <Servo.h>
 
Servo ServoD;     // Servo lado derecho
Servo ServoI;     // Servo lado izquierdo
Servo MiniServo;  // MiniServo del medidor
int pos = 90;     // Para almacenar la posición del servo derecho

// Uso de interrupciones para medir 

#define entrada 3        // Entrada asignada a la interrupción
#define medidor 0        // Puerto A/D del medidor de distancias
#define medir 2          // Pin para activar la medición

int distancia = 0;      // Valor obtenido en la conversión A/D
float mm = 0;           // Valor de la medición convertido a mm
int incomingByte;


volatile int cuentas = 0;
volatile int vale = LOW;  // Medición válida
volatile unsigned long ahora = 0;
volatile unsigned long antes = 0;

void setup()
{
  // Asigna pines a los servos
  ServoD.attach(9);        // Asigna el servo derecho al pin 9 (OUTPUT 2 de la TINKERKIT)
  ServoI.attach(10);       // Asigna el servo izquierdo al pin 10 (OUTPUT 1 de la TINKERKIT)
  MiniServo.attach(11);    // Asigna el miniservo al pin 11 (OUTPUT 0 de la TINKERKIT)

  // Inmoviliza servos  
  ServoD.write(90);  
  ServoI.write(90);  
  MiniServo.write(90);    
  
  pinMode(entrada, INPUT);           // Pin de entrada del medidor
  pinMode(medir, OUTPUT);            // Señal de disparo del medidor
  Serial.begin(9600);                // Inicializamos la comunicación serie  
  digitalWrite(medir, LOW);          // Inhibe mediciones
}
 
void loop()
{
  attachInterrupt(1, mide, CHANGE);   // Activa interrupción en pin de entrada (3)  
  digitalWrite(medir, HIGH);          // Activa medidor de distancia
  delay(10);                          // Para dar tiempo a realizar la primera medición
  
  distancia = analogRead(medidor);    // Realiza medición de distancia
  Serial.print("Distancia... ");
  Serial.print(distancia);
  Serial.print(" cuentas = ");
  mm = distancia / 2 * 25,4 / 100;    // Convierte valor del D/A a mm
  Serial.print(mm);
  Serial.println(" milimetros");
  
  while (!vale); // Esperamos a una medición válida
  Serial.print("Distancia... ");
  Serial.print(cuentas);
  Serial.print(" cuentas = ");
  mm = cuentas / 5,7874 - 170;    // Convierte valor del PWM a mm
  Serial.print(mm);
  Serial.println(" milimetros");
  Serial.println(" ");
  
/*  while (Serial.available() > 0) {
                // read the incoming byte:
                incomingByte = Serial.read();

                // say what you got:
                Serial.print(incomingByte, HEX);
                Serial.print(" ");
        }  
        
  Serial.println(" ");
  Serial.println(" ");
*/  
  digitalWrite(medir, LOW);          // Inhibe mediciones  
  
  detachInterrupt(1);            // Inhabilita interrupción
 
  // Mueve servo derecho adelante
  
  for(pos = 90; pos<=180; pos+=10)     
  {                                
    ServoD.write(pos);      
    ServoI.write(pos);   
    MiniServo.write(pos-45);              
    delay(100);                       
  }
  for(pos = 180; pos>=90; pos-=10)     
  {                                
    ServoD.write(pos);              
    ServoI.write(pos);   
    MiniServo.write(pos-45);              
    delay(100);                       
  }  

      MiniServo.write(90);
/*  
  // Mueve servo derecho atrás  
  for(pos = 90; pos>=0; pos-=10)     
  {                                
    ServoD.write(pos); 
    ServoI.write(pos);   
    MiniServo.write(pos);              
    delay(100);                       
  }
  for(pos = 0; pos<=90; pos+=10)     
  {                                
    ServoD.write(pos);   
    ServoI.write(pos);   
    MiniServo.write(pos);              
    delay(100);                       
  }
/*
 
  // Mueve servo izquierdo adelante
  for(pos = 90; pos<=180; pos+=10)     
  {                                
    ServoI.write(pos);              
    delay(100);                       
  }
  for(pos = 180; pos>=90; pos-=10)     
  {                                
    ServoI.write(pos);              
    delay(100);                       
  }
  // Mueve servo izquierdo atrás    
  for(pos = 90; pos>=0; pos-=10)     
  {                                
    ServoI.write(pos);              
    delay(100);                       
  }
  for(pos = 0; pos<=90; pos+=10)     
  {                                
    ServoI.write(pos);              
    delay(100);                       
  }


  // Mueve miniservo a la derecha
  for(pos = 90; pos<=180; pos+=1)     
  {                                
    MiniServo.write(pos);              
    delay(100);                       
  }
  for(pos = 180; pos>=90; pos-=1)     
  {                                
    MiniServo.write(pos);              
    delay(100);                       
  }
  // Mueve miniservo a la izquierda
  for(pos = 90; pos>=1; pos-=1)     
  {                                
    MiniServo.write(pos);              
    delay(100);                       
  }
  for(pos = 0; pos<=90; pos+=1)     
  {                                
    MiniServo.write(pos);              
    delay(100);                       
  }
*/
  delay(5000);                   // Insertamos una espera
}

// Rutina de servicio de la interrupción
void mide()
{
  vale = digitalRead(entrada);
  if (vale){
    antes = micros();
    vale = LOW;
  } 
  else {
    ahora = micros();
    cuentas = ahora - antes;
    vale = HIGH;
  }
}
  



