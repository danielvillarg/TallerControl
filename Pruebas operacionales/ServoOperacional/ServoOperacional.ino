// Codigo operacional del servo motor, indica con luz verde cuando se inicializo correctamente
// luego, espera a que la variable "altura" llegue a 12 para moverse los grados necesario para 
// accionar le sistema que desancla el globo
#include <ESP32Servo.h>
Servo ServoGlobo;  // create servo object to control a servo
 
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
const int ServoPin = 27;
const int ledListo = 32;
const int ledServo = 26;
int altura = 0;
int pos;

void setup() {
  pinMode(ledListo, OUTPUT);
  pinMode(ledServo, OUTPUT);
  ServoGlobo.setPeriodHertz(50); 
  ServoGlobo.attach(ServoPin);
  Serial.begin(9600);
  Serial.println("Servo Listo");
  digitalWrite(ledListo, HIGH); 
}
float tiem;
void loop() {
  
  delay(1000);
  if (altura < 6){
    Serial.print("Altura ");
    Serial.println(altura);
    altura = altura + 1;
  }  
    
  if (altura == 6){
    altura = altura + 1;
    Serial.println("Motor activo");
    digitalWrite (ledServo, HIGH);  // prender servo y led
    for (tiem = 0; tiem<10; tiem+=1);
      {
        for (pos = 0; pos <= 1250; pos += 1) {
          ServoGlobo.write(pos);
          delay(10);
        }
      }
    digitalWrite (ledServo, LOW);  
  }
}
