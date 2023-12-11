// Test operacional BMP180
#include <Wire.h>
#include <Adafruit_BMP085.h>

Adafruit_BMP085 bmp;
float altura;

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
      
  // Para una medida mas precisa de la altitud se considero
  // la presion a nivel del mar actual que es de 1018
  altura = bmp.readAltitude(101800);
  Serial.print("Altitud = ");
  Serial.print(altura);
  Serial.println(" m");

  Serial.print("Presion = ");
 Serial.print(bmp.readPressure());
 Serial.println(" Pa");
 
  if (altura > 2600){
    Serial.println("Altura de desacople alcanzada");
  }
    
  Serial.println();
  delay(100);
}