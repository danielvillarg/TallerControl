/*
 * This ESP32 code is created by esp32io.com
 *
 * This ESP32 code is released in the public domain
 *
 * For more detail (instruction and wiring diagram), visit https://esp32io.com/tutorials/esp32-gps
 */
#include <Wire.h>
#include <TinyGPS++.h>

#define GPS_BAUDRATE 9600  // The default baudrate of NEO-6M is 9600
#define RXD2 16
#define TXD2 17

unsigned long chars;
unsigned short sentences, failed_checksum;

HardwareSerial neogps(1);

TinyGPSPlus gps;  // the TinyGPS++ object

// variable definitions
char Time[]  = "TIME: 00:00:00";
char Date[]  = "DATE: 00-00-2000";
byte last_second, Second, Minute, Hour, Day, Month;
int Year;

void setup() {
  Serial.begin(9600);
  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);
  delay(2000);
  Serial.println(F("ESP32 - GPS module"));
}

void loop()
{
  while(neogps.available()) 
  {
    int c = neogps.read();
 
    if(gps.encode(c))  
    {
      if (gps.location.isUpdated()) {
        Serial.print("Latitud/Longitud: ");
        Serial.print(gps.location.lat(), 6);
        Serial.print(", ");
        Serial.println(gps.location.lng(), 6);
      }
      if (gps.altitude.isUpdated()) {
        Serial.print("Altitud (metros): ");
        Serial.println(gps.altitude.meters());
      }
      if (gps.course.isUpdated()) {
        Serial.print("Rumbo (grados): ");
        Serial.println(gps.course.deg());
      }
      if (gps.speed.isUpdated()) {
        Serial.print("Velocidad (kmph): ");
        Serial.println(gps.speed.kmph());
      }
    // Raw date in DDMMYY format (u32)
      int year, month, day, hour, minute, second;

      // Obtiene la fecha del GPS
      Serial.print("Raw date DDMMYY = ");
      Serial.println(gps.date.value()); 

      // Year (2000+) (u16)
      Serial.print("Year = "); 
      Serial.println(gps.date.year()); 
      // Month (1-12) (u8)
      Serial.print("Month = "); 
      Serial.println(gps.date.month()); 
      // Day (1-31) (u8)
      Serial.print("Day = "); 
      Serial.println(gps.date.day()); 

      // Raw time in HHMMSSCC format (u32)
      Serial.print("Raw time in HHMMSSCC = "); 
      Serial.println(gps.time.value()); 

      // Hour (0-23) (u8)
      Serial.print("Hour = "); 
      Serial.println(gps.time.hour()-5); 
      // Minute (0-59) (u8)
      Serial.print("Minute = "); 
      Serial.println(gps.time.minute()); 
      // Second (0-59) (u8)
      Serial.print("Second = "); 
      Serial.println(gps.time.second()); 
    }
  }
}