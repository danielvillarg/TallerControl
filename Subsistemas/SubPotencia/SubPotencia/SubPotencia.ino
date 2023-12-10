// Codigo operacional del subsitema de potencia. El led verde se enciende si a la ESP32 se le esta suministrando un voltaje adecuado
const int ledListo = 32;
const int Bateria = 25;

float carga;

void setup() {
  Serial.begin(9600);
  // setup pin 5 as a digital output pin
  pinMode (ledListo, OUTPUT);
  pinMode(Bateria, INPUT);
  digitalWrite(ledListo, HIGH);
}

void loop() {
  
  delay(2000); // wait for half a second or 500 milliseconds

  carga = analogRead(Bateria);
  carga = (carga/4095)*3.3; 
  carga = carga* (5 + 9.6)/5;  // R2 = 5K, R1 = 9.6K
  Serial.println(carga); 
}
