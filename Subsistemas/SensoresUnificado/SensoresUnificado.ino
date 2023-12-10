#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <TinyGPS.h>
#include <QMC5883LCompass.h>

#define I2C_SDA 21
#define I2C_SCL 22
#define GPS_BAUDRATE 9600  // The default baudrate of NEO-6M is 9600
#define RXD2 16
#define TXD2 17

Adafruit_MPU6050 mpu;

unsigned long chars;
unsigned short sentences, failed_checksum;
long tiempo_prev, dt;

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

TinyGPS gps;  // the TinyGPS++ object


void setup(void) {
  delay(3000);
  Serial.begin(115200);
  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.println(F("ESP32 - GPS module"));
  Serial.println("Inicio conexion a QMC");
  compass.init();
  compass.setCalibration(-45, 1657, -1040, 716, 317, 2010);

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
}

void loop() {
  dt = millis()-tiempo_prev;
  tiempo_prev=millis();
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  Adafruit_I2CDevice *i2c_dev = NULL;
  Adafruit_BusIO_Register chip_id =
      Adafruit_BusIO_Register(i2c_dev, MPU6050_WHO_AM_I, 1);
  delete i2c_dev;

  float accel_ang_x=atan(a.acceleration.x/sqrt(pow(a.acceleration.y,2) + pow(a.acceleration.z,2)))*(180.0/3.14);
  float accel_ang_y=atan(a.acceleration.y/sqrt(pow(a.acceleration.x,2) + pow(a.acceleration.z,2)))*(180.0/3.14);
  //Mostrar los angulos separadas por un [tab]
  Serial.print("Inclinacion en X: ");
  Serial.print(accel_ang_x); 
  Serial.print(", Inclinacion en Y:");
  Serial.println(accel_ang_y);

  Serial.print("Temperature: ");
  Serial.print(temp.temperature - 20);
  Serial.println(" degC");

  Serial.println("");


  
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

  sprintf(buffer,
          "X=%6d | Y=%6d | Z=%6d | A=%3d° | B=%02hu | %s",
          x_value,
          y_value,
          z_value,
          azimuth,
          bearing,
          direction);
  Serial.println(buffer);
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
  delay(500);
}
