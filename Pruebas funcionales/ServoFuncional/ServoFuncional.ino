// Codigo funcional del servomotor, permite mover el servo de 0 a 180 grados e indica con el led rojo cuando se va a empezar a mover
#include <ESP32Servo.h>
 
Servo myservo;  // create servo object to control a servo
 
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
const int servoPin = 27;
const int ledServo = 26;
int pos = 0;    // variable to store the servo position

void setup() {
  //myservo.setPeriodHertz(50); 
  myservo.attach(servoPin);
  pinMode(ledServo, OUTPUT);
  digitalWrite (ledServo, HIGH);  // turn on the LED
}
 
void loop() {
   for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  //digitalWrite (ledServo, LOW);  // turn on the LED
}
