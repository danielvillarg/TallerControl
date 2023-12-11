// Test funcional BMP180
#include <Wire.h>
#include <Adafruit_BMP085.h>

Adafruit_BMP085 bmp;
 
void setup() {
 Serial.begin(9600);
 if (!bmp.begin()) {
 Serial.println("Could not find a valid BMP085/BMP180 sensor, check wiring!");
 while (1) {}
 }
}
 
void loop() {
 Serial.print("Temperatura = ");
 Serial.print(bmp.readTemperature());
 Serial.println(" *C");
   
 Serial.print("Presion = ");
 Serial.print(bmp.readPressure());
 Serial.println(" Pa");
   
 // Calculate altitude assuming 'standard' barometric
 // pressure of 1013.25 millibar = 101325 Pascal
 Serial.print("Altitud = ");
 Serial.print(bmp.readAltitude());
 Serial.println(" m");

 Serial.print("Presion al nivel del mar = ");
 Serial.print(bmp.readSealevelPressure());
 Serial.println(" Pa");
   
 Serial.println();
 delay(500);
}
