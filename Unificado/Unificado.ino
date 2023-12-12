#include <SPI.h>
#include <SD.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <QMC5883LCompass.h>
#include <ESP32Servo.h>
#include <Adafruit_BMP085.h>

const int ledAnclado = 34;
const int ledListo = 32;
const int ledServo = 26;
const int Bateria = 25;
const int ServoPin = 27;

float temperatura;
float altura;
float presion;
int pos;
float carga;

#define I2C_SDA 21
#define I2C_SCL 22
#define GPS_BAUDRATE 9600  // The default baudrate of NEO-6M is 9600
#define RXD2 16 
#define TXD2 17


Adafruit_BMP085 bmp;
Adafruit_MPU6050 mpu;

File myFile;
const int CS = 5;

unsigned long chars;
unsigned short sentences, failed_checksum;
long tiempo_prev, dt;
long t = 0;

// Mode Control (MODE)
const byte qmc5883l_mode_stby = 0x00;
const byte qmc5883l_mode_cont = 0x01;
// Output Data Rate (ODR)
const byte qmc5883l_odr_10hz  = 0x00;
const byte qmc5883l_odr_50hz  = 0x04;
const byte qmc5883l_odr_100hz = 0x08;
const byte qmc5883l_odr_200hz = 0x0C;
// Full Scale Range (RNG)
const byte qmc5883l_rng_2g    = 0x00;
const byte qmc5883l_rng_8g    = 0x10;
// Over Sample Ratio (OSR)
const byte qmc5883l_osr_512   = 0x00;
const byte qmc5883l_osr_256   = 0x40;
const byte qmc5883l_osr_128   = 0x80;
const byte qmc5883l_osr_64    = 0xC0;

QMC5883LCompass compass;
HardwareSerial neogps(1);

TinyGPSPlus gps;  // the TinyGPS++ object
Servo ServoGlobo;  // create servo object to control a servo
 

void WriteFile(const char * path, float latitude, float longitude, float angulo, float velocidad, float altitudGPS, String fecha_y_hora, float x_value, float y_value, float z_value, float azimuth,float a_x,float a_y,float a_z,float g_x,float g_y,float g_z,float presion, float altitud,float temperatura, float carga, float dt, float tiempo){
  // Si el archivo es nuevo
  if (!SD.exists(path)) {
    myFile = SD.open(path, FILE_WRITE);
  // escribir titulos de las columnas si se esta creando un archivo nuevo:
    if (myFile) {
      myFile.println("Latitud (°), Longitud (°), Angulo (°), Velocidad (km/h), altitudGPS (m), fecha_y_hora, X (G), Y (G), Z (G), Azimuth (°), AccelX (m/s^2), AccelY (m/s^2), AccelZ (m/s^2), GyroX (°/s), GyroY (°/s), GyroZ (°/s), Presion (Pa), Altura (m), Temperatura (°C), Carga (V), Dt (ms), Tiempo (ms)"); // write number to file
      myFile.print("\n");
      myFile.close();
    }
    else {
      Serial.println("Error abriendo el archivo");
      return; 
      }
  }
  // Si el archivo ya existe
   myFile = SD.open(path, FILE_APPEND);

    if (myFile){
    myFile.print(String(latitude));
    myFile.print(",");
    myFile.print(String(longitude));
    myFile.print(",");
    myFile.print(String(angulo));
    myFile.print(",");
    myFile.print(String(velocidad));
    myFile.print(",");
    myFile.print(String(altitudGPS));
    myFile.print(",");
    myFile.print(String(fecha_y_hora));
    myFile.print(",");
    myFile.print(String(x_value));
    myFile.print(",");
    myFile.print(String(y_value));
    myFile.print(",");
    myFile.print(String(z_value));
    myFile.print(",");
    myFile.print(String(azimuth));
    myFile.print(",");
    myFile.print(String(a_x));
    myFile.print(",");
    myFile.print(String(a_y));
    myFile.print(",");
    myFile.print(String(a_z));
    myFile.print(",");
    myFile.print(String(g_x));
    myFile.print(",");
    myFile.print(String(g_y));
    myFile.print(",");
    myFile.print(String(g_z));
    myFile.print(",");
    myFile.print(String(presion));
    myFile.print(",");
    myFile.print(String(altitud));
    myFile.print(",");
    myFile.print(String(temperatura));
    myFile.print(",");
    myFile.print(String(floor(carga*10)/10-1));
    myFile.print(",");
    myFile.print(String(dt));
    myFile.print(",");
    myFile.print(String(tiempo));
    myFile.print("\n");
    //","+ String(accel_ang_x)+ ","+String(accel_ang_y)+","+String(temperatura) ); // write number to file
   

    Serial.print(String(latitude));
    Serial.print(",");
    Serial.print(String(longitude));
    Serial.print(",");
    Serial.print(String(angulo));
    Serial.print(",");
    Serial.print(String(velocidad));
    Serial.print(",");
    Serial.print(String(altitudGPS));
    Serial.print(",");
    Serial.print(String(fecha_y_hora));
    Serial.print(",");
    Serial.print(String(x_value));
    Serial.print(",");
    Serial.print(String(y_value));
    Serial.print(",");
    Serial.print(String(z_value));
    Serial.print(",");
    Serial.print(String(azimuth));
    Serial.print(",");
    Serial.print(String(a_x));
    Serial.print(",");
    Serial.print(String(a_y));
    Serial.print(",");
    Serial.print(String(a_z));
    Serial.print(",");
    Serial.print(String(g_x));
    Serial.print(",");
    Serial.print(String(g_y));
    Serial.print(",");
    Serial.print(String(g_z));
    Serial.print(",");
    Serial.print(String(presion));
    Serial.print(",");
    Serial.print(String(altitud));
    Serial.print(",");
    Serial.print(String(temperatura));
    Serial.print(",");
    Serial.print(String(floor(carga*10)/10-1));
    Serial.print(",");
    Serial.print(String(dt));
    Serial.print(",");
    Serial.print(String(tiempo));
    Serial.print("\n");


    Serial.println("Linea");
    myFile.close();
      }
    else {
    Serial.println("error opening file ");
    Serial.println(path);
  }
}

void setupSensores(){
  
  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.println(F("ESP32 - GPS module ready"));
  Serial.println("Inicio conexion a QMC");
  compass.init();
  compass.setCalibration(-45, 1657, -1040, 716, 317, 2010);
  Serial.println("Magnetometro listo");

  Serial.println("Adafruit MPU6050 test!");

    // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);

  //BMP
  
  if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP085/BMP180 sensor, check wiring!");
  while (1) {}
  }
  Serial.println("BMP 180 ready");

  //Servo
  ServoGlobo.attach(ServoPin);
  Serial.println("Servo ready");

  digitalWrite(ledListo, HIGH);
}

void setup() {
  Serial.begin(9600);    // Set serial baud rate to 9600
  delay(2000);
  pinMode(ledServo, OUTPUT);
  pinMode(ledListo, OUTPUT);
  while (!Serial) { ; }  // wait for serial port to connect. Needed for native USB port only
  Serial.println("Initializing SD card...");
  if (!SD.begin(CS)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("SD initialization done.");
  setupSensores();

}

void loop() {
  
  dt = millis()-tiempo_prev;
  tiempo_prev=millis();
  t = t + dt;
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  //float accel_ang_x=atan(a.acceleration.x/sqrt(pow(a.acceleration.y,2) + pow(a.acceleration.z,2)))*(180.0/3.14);
  //float accel_ang_y=atan(a.acceleration.y/sqrt(pow(a.acceleration.x,2) + pow(a.acceleration.z,2)))*(180.0/3.14);
  //Mostrar los angulos separadas por un [tab]
  //Serial.print("Inclinacion en X: ");
  //Serial.print(accel_ang_x); 
  //Serial.print(", Inclinacion en Y:");
  //Serial.println(accel_ang_y);
  float a_x = a.acceleration.x;
  float a_y = a.acceleration.y;
  float a_z = a.acceleration.z;
  float g_x = g.gyro.x;
  float g_y = g.gyro.y;
  float g_z = g.gyro.z;

  int x_value;
  int y_value;
  int z_value;
  int azimuth;  // 0° - 359°
  byte bearing; // 0 - 15 (N, NNE, NE, ENE, E, ...)
  char direction[strlen("NNE") + 1];
  char buffer[strlen("X=-99999 | Y=-99999 | Z=-99999 | A=259° | B=15 | D=NNE") + 1]; 
  
  compass.read(); // Read compass values via I2C

  x_value   = compass.getX();
  y_value   = compass.getY();
  z_value   = compass.getZ();
  azimuth   = compass.getAzimuth(); // Calculated from X and Y value 
  bearing   = compass.getBearing(azimuth);
  
  compass.getDirection(direction, azimuth);
  direction[3] = '\0';
//  Serial.println("Buffer y otros");
//  sprintf(buffer,
//          "X=%6d | Y=%6d | Z=%6d | A=%3d° | B=%02hu | %s",
//          x_value,
//          y_value,
//          z_value,
//          azimuth,
//          bearing,
//          direction);
//  Serial.println(buffer);
  float latitude, longitude, rumbo, velocidad;
  float altitudGPS;
  String fecha_y_hora;
  while(neogps.available()) 
  {
    int c = neogps.read();
 
    if(gps.encode(c))  
    {
      float latitud = (gps.location.lat(), 6);
      float longitud = (gps.location.lng(), 6);
      float rumbo = gps.course.deg();
      float velocidad = gps.speed.kmph();
      float altitudGPS = gps.altitude.meters();
      int year, month, day, hour, minute, second;
      
      /*Serial.print("Raw date DDMMYY = ");
      Serial.println(gps.date.value()); 
      */
      // Year (2000+) (u16)
      //Serial.print("Year = "); 
      float anio = gps.date.year(); 
      // Month (1-12) (u8)
      //Serial.print("Month = "); 
      float mes = gps.date.month(); 
      // Day (1-31) (u8)
      //Serial.print("Day = "); 
      float dia = gps.date.day(); 

      // Raw time in HHMMSSCC format (u32)
      /*Serial.print("Raw time in HHMMSSCC = "); 
      Serial.println(gps.time.value()); 
*/
      // Hour (0-23) (u8)
      //Serial.print("Hour = "); 
      //Serial.println(gps.time.hour()-5); 
      float hora = gps.time.hour()-5;
      // Minute (0-59) (u8)
      //Serial.print("Minute = "); 
      //Serial.println(gps.time.minute()); 
      float minuto = gps.time.minute();
      // Second (0-59) (u8)
      //Serial.print("Second = "); 
      //Serial.println(gps.time.second()); 
      float segundo = gps.time.second();

      String str_anio = String(anio, 0); // 0 indica que no queremos decimales
      String str_mes = String(mes, 0);
      String str_dia = String(dia, 0);
      String str_hora = String(hora, 0);
      String str_minuto = String(minuto, 0);
      String str_segundo = String(segundo, 0);

      // Formar la cadena final concatenando los valores con "/"
      String fecha_y_hora = str_dia + "/" + str_mes + "/" + str_anio + "//" + str_hora + "/" + str_minuto + "/" + str_segundo;
    }
//      Serial.print("Latitud/Longitud: "); 
//      Serial.print(latitude,5); 
//      Serial.print(", "); 
     
////      Serial.println(gps.f_altitude()); 
//      Serial.print("Rumbo (grados): "); Serial.println(gps.f_course()); 
//      Serial.print("Velocidad(kmph): ");
//      Serial.println(gps.f_speed_kmph());
    }
  
  delay(500);

  //BMP 180
  temperatura = bmp.readTemperature();
//  Serial.print("Temperatura = ");
//  Serial.print(temperatura);
//  Serial.println(" *C");
//      
  // Para una medida mas precisa de la altitud se considero
  // la presion a nivel del mar actual que es de 1018
  float altitud = bmp.readAltitude(101800);
  presion = bmp.readPressure();
//  Serial.print("Alttitud = ");
//  Serial.print(altura);
//  Serial.println(" m");
//    
  //Serial.println();
  delay(500);

  //Servo

  
  if (altitud <= 12){
    Serial.print("Altura ");
    Serial.println(altitud);
  }  
    
  if (altitud == 12){
    digitalWrite (ledServo, HIGH);  // turn on the LED
    Serial.println("Motor activo");
    for (pos = 0; pos <= 180; pos += 1) {
    ServoGlobo.write(pos);
    delay(10);
    }
    digitalWrite (ledServo, LOW); // turn off the LED
    }
    
  carga = analogRead(Bateria);
  carga = (carga/4095)*3.3; 
  carga = carga* (5 + 9.6)/5;  // R2 = 5K, R1 = 9.6K
  //Serial.println(carga);
  
  WriteFile("/test8.txt", latitude, longitude, rumbo, velocidad, altitudGPS, fecha_y_hora, x_value, y_value, z_value,azimuth, a_x, a_y, a_z, g_x, g_y, g_z, presion, altura, temperatura, carga, dt, t);
//  ReadFile("/test1.txt");

  if (altitud == 0){
    myFile.close(); // close the file
    
  }
}
