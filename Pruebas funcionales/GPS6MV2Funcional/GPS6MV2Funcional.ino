#include <Wire.h>
#include <TinyGPS.h>

#define GPS_BAUDRATE 9600  // The default baudrate of NEO-6M is 9600
#define RXD2 16
#define TXD2 17

char dato=' ';

HardwareSerial neogps(1);

TinyGPS gps;  // the TinyGPS++ object

void setup() {
  Serial.begin(115200);
  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);

  Serial.println(F("ESP32 - GPS module"));
}

void loop()
{
  if(neogps.available())
  {
    dato=neogps.read();
    Serial.print(dato);
    delay (50);
  }
}