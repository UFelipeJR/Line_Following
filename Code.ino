#include <QTRSensors.h>
#include <EEPROMex.h>
#include <Servo.h>

// EEPROM Addressing for calibration storage
#define addrCalibratedMinimumOn 0
#define addrCalibratedMaximumOn 100

#define lineUmbral 500

const int velBaseIni = 250;
const int velBaseMax = 255;

float kP = 0.05;
float kD = 0.005;
float kI = 0.0000955;

QTRSensors qtr;

const uint8_t SensorCount = 8;
unsigned int sensorValues[SensorCount];
bool sensor[SensorCount];

int P = 0;
long I = 0;
int D = 0;

int LAST = 0;
float vel;

int velIzq = 0;
int velDer = 0;

int STBY = 7;
int PWMA = 10; 
int AIN1 = 9;
int AIN2 = 8; 
int PWMB = 11; 
int BIN1 = 13; 
int BIN2 = 12;

int pinSensor = A0;
int pinBuzzer = A4;
int val = 0;

bool left = false;
bool right = true;

int wait = 450;
int waitL = 3000;

int minValue = 600;
int maxValue = 900;

Servo servo;

void setup() {
  Serial.begin(9600);
  pinMode(STBY, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  servo.attach(A5);  
  servo.write(15);
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A3, A2, A1, 6, 5, 4, 3, 2}, SensorCount);
  recallQTR();
}

void loop() {  
  readSensors();
  val = analogRead(pinSensor);
  Serial.println(val);
  /*
  leerSerial();
  imprimirValores();
  return;
  */
  
  //Motor 1 derecho
  //Motor 2 izquierdo

  programSpeed(velBaseIni, velBaseMax);
  drive(velIzq, velDer); 

  if(lineColor(0)&&lineColor(7)){
    move(1,0,1);
    move(2,0,1);
    descarga();
    delay(waitL);
    if(right){
      right = false;
      left = true;
      move(1,velBaseMax/2,2);
      move(2,velBaseMax,1);
      delay(wait);
      move(1,0,1);
      move(2,0,1);
      delay(50);
      move(1,velBaseMax,1);
      move(2,velBaseMax,1);
      delay(wait);
    }
    else{
      move(1,velBaseMax,1);
      move(2,velBaseMax/2,2);
      descarga();
      delay(wait-100);
      move(1,0,1);
      move(2,0,1);
      delay(50);
      move(1,velBaseMax,1);
      move(2,velBaseMax,1);
      delay(wait-100);
      right = true;
      left = false;
    }
    
  }


 
}

void calibrateQTR() {
  delay(2000);

  for (int i = 0; i < SensorCount; i++) {
    qtr.calibrationOn.minimum[i] = 1000;
    qtr.calibrationOn.maximum[i] = 0;
  }

  Serial.println();
  Serial.println("Beginning Calibration Process...");

  for (int i = 0; i < 200; i++) {
    qtr.calibrate();
  }

  Serial.println("Calibration Complete");
}

void readQTR() {
  Serial.println();
  Serial.println("Reading Calibration Data...");

  for (int i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  for (int i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }

  delay(3000);
}

void storeQTR() {
  Serial.println();
  Serial.println("Storing Calibration Data into EEPROM...");

  EEPROM.writeBlock<unsigned int>(addrCalibratedMinimumOn, qtr.calibrationOn.minimum, 8);
  EEPROM.writeBlock<unsigned int>(addrCalibratedMaximumOn, qtr.calibrationOn.maximum, 8);

  Serial.println("EEPROM Storage Complete");
}

void recallQTR() {
  Serial.println();
  Serial.println("Recalling Calibration Data from EEPROM...");

  qtr.calibrate();
  EEPROM.readBlock<unsigned int>(addrCalibratedMinimumOn, qtr.calibrationOn.minimum, 8);
  EEPROM.readBlock<unsigned int>(addrCalibratedMaximumOn, qtr.calibrationOn.maximum, 8);

  Serial.println("EEPROM Recall Complete");
}

void leerSerial() {
  if (Serial.available()) {
    char ch = Serial.read();

    if (ch == 'c' || ch == 'C') {
      calibrateQTR();
    }
    else if (ch == 'r' || ch == 'R') {
      readQTR();
    }
    else if (ch == 's' || ch == 'S') {
      storeQTR();
    }
    else if (ch == 'e' || ch == 'E') {
      recallQTR();
    }
  }
}

void readSensors() {
  unsigned int position = qtr.readLineBlack(sensorValues);
  P = (position - 3500);

}

void imprimirValores()
{

  for (int i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print("\t");
  }
  Serial.println();

  delay(500);

}

void move(int motor, int speed, int direction) {
  digitalWrite(STBY, HIGH);
  boolean inPin1 = LOW;
  boolean inPin2 = HIGH;

  if (direction == 1) {
    inPin1 = HIGH;
    inPin2 = LOW;
  }

  if (motor == 1) {
    digitalWrite(AIN1, inPin1);
    digitalWrite(AIN2, inPin2);
    analogWrite(PWMA, speed);
  }
  else {
    digitalWrite(BIN1, inPin1);
    digitalWrite(BIN2, inPin2);
    analogWrite(PWMB, speed);
  }
}

void stop() {
  digitalWrite(STBY, LOW);
}

void drive(int speedl, int speedr) {
  if (speedl > 0) {
    move(1, speedl, 1);
  } else {
    move(1, speedl * -1, 2);
  }
  if (speedr > 0) {
    move(2, speedr, 1);
  } else {
    move(2, speedr * -1, 2);
  }
}

void programSpeed(int velIni, int velMax) {
  D = (P - LAST);
  I = (I + P);
  if (I * P < 0) I = 0;
  vel = (P * kP) + (D * kD) + (I * kI);
  velIzq = velIni - vel;
  velDer = velIni + vel;
  if (velIzq > velMax) {
    velIzq = velMax;
  } else if (velIzq < -velMax) {    velIzq = -velMax;
  }
  if (velDer > velMax) {
    velDer = velMax;
  } else if (velDer < -velMax) {
    velDer = -velMax;
  }
  LAST = P;
}


bool lineColor(int numSensor){
  if(sensorValues[numSensor] >= lineUmbral){
    return true;
  }
  else{
    return false;
  }

}

void descarga(){
  if(val > minValue && val < maxValue){
    servo.write(90); 
    delay(3000);
    servo.write(15);
    delay(5);
  }
  else{
    servo.write(15); 
  }
  
}
