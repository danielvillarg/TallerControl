/*
Codigo operacional de la SD, el codigo genera un archivo llamado SDOperacional.txt en el cual
almacena 20 lineas de datos separados por coma, simulando lo que se almacenaria en una prueba. 
*/

#include <SPI.h>
#include <SD.h>

File myFile;
const int CS = 5;
const int ledListo = 32;

float latit, longit, ang, vel, x, y, z, azim, bea, direct, accelx, accely, altitud, temp, carga, dt, tiemp;

void WriteFile(const char * path, const char * message){
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open(path, FILE_WRITE);
  // if the file opened okay, write to it:
  if (myFile) {
    Serial.printf("Writing to %s ", path);
    myFile.println(message);
    myFile.println("Latitud, Longitud, Angulo, Velocidad, X, Y, Z, Azimuth, Bearing, Direction, AccelX, AccelY, Altitud, Temperatura, Carga, Dt, Tiempo"); // write number to file
    for (int i = 0; i <= 30; i++) {
      latit = random(1000)/100.0; // generate random number between 0 and 9  
      longit = random(1000)/100.0; // generate random number between 0 and 9
      ang = random(18000)/100.0; // generate random number between 0 and 9
      vel = random(1000)/100.0; // generate random number between 0 and 9
      x = random(1000)/100.0;
      y = random(1000)/100.0;
      z = random(1000)/100.0;
      azim = random(1000)/100.0;
      bea = random(1000)/100.0;
      direct = random(1000)/100.0;
      accelx = random(1000)/100.0;
      accely = random(1000)/100.0;
      altitud = random(1000)/100.0;
      temp = random(1000)/100.0;
      carga = random(1000)/100.0;
      dt = random(1000)/100.0;
      tiemp = random(1000)/100.0;
      myFile.println(String(latit)+","+String(longit)+","+String(ang)+","+String(vel)+","+String(x)+","+String(y)+","+String(z)+","+String(azim)+","+String(bea)+","+String(direct)+","+String(accelx)+","+String(accely)+","+String(altitud)+","+String(temp)+","+String(carga)+","+String(dt)+","+String(tiemp)); // write number to file
      
    }
    myFile.close(); // close the file:
    Serial.println("completed.");
    digitalWrite(ledListo, LOW);
  } 
  // if the file didn't open, print an error:
  else {
    Serial.println("error opening file ");
    Serial.println(path);
  }
}


void ReadFile(const char * path){
  // open the file for reading:
  myFile = SD.open(path);
  if (myFile) {
     Serial.printf("Reading file from %s\n", path);
     // read from the file until there's nothing else in it:
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    myFile.close(); // close the file:
  } 
  else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}

void setup() {
  Serial.begin(9600);    // Set serial baud rate to 9600
  pinMode(ledListo, OUTPUT);
  delay(2000);
  while (!Serial) { ; }  // wait for serial port to connect. Needed for native USB port only
  Serial.println("Initializing SD card...");
  if (!SD.begin(CS)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done."); 
  digitalWrite(ledListo, HIGH);
  WriteFile("/SDOperacional.txt", "Datos almacenados:");
  //ReadFile("/SDOperacional.txt");
}

void loop() {
  // nothing happens after setup
}
