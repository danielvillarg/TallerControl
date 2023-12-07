/*
 * This ESP32 code is created by esp32io.com
 *
 * This ESP32 code is released in the public domain
 *
 * For more detail (instruction and wiring diagram), visit https://esp32io.com/tutorials/esp32-gps
 */
#include <Wire.h>
#include <TinyGPS.h>

#define GPS_BAUDRATE 9600  // The default baudrate of NEO-6M is 9600
#define RXD2 16
#define TXD2 17

unsigned long chars;
unsigned short sentences, failed_checksum;

HardwareSerial neogps(1);

TinyGPS gps;  // the TinyGPS++ object

void setup() {
  Serial.begin(115200);
  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);

  Serial.println(F("ESP32 - GPS module"));
}

void loop()
{
  while(neogps.available()) 
  {
    int c = neogps.read();
 
    if(gps.encode(c))  
    {
      float latitude, longitude;
      gps.f_get_position(&latitude, &longitude);
      Serial.print("Latitud/Longitud: "); 
      Serial.print(latitude,5); 
      Serial.print(", "); 
      Serial.println(longitude,5);
      Serial.print("Altitud (metros): ");
      Serial.println(gps.f_altitude()); 
      Serial.print("Rumbo (grados): "); Serial.println(gps.f_course()); 
      Serial.print("Velocidad(kmph): ");
      Serial.println(gps.f_speed_kmph());
      gps.stats(&chars, &sentences, &failed_checksum);  
    }
  }
}