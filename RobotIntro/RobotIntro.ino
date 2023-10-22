#include <QTRSensors.h>
#include <EEPROMex.h>

//EEPROM Addressing for calibration storage
#define addrCalibratedMinimumOn 0
#define addrCalibratedMaximumOn 100

#define blanco true
#define negro false

const int velBaseIni = 200;
const int velBaseMax = 255;


float kP = 0.05;
float kD = 0.005;
float kI = 0.0000955;


// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensors qtr;

const uint8_t SensorCount = 8;
unsigned int sensorValues[SensorCount];
bool sensor[SensorCount];

bool j2value;
bool j1value;

byte programa = 0;

unsigned long previousMillis = 0;

int cuentaInter = 0;
int resetCuentaInter = 0;
int maxResetCuentaInter = 4;
int bloqueaLectura = 0;

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

void setup(){
  Serial.begin(9600);
  pinMode(STBY, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A3,A2,A1,6,5,4,3,2}, SensorCount);
  qtr.setEmitterPin(1);
  recallQTR();
}

void loop(){
  readSensors();

  /*
  leerSerial();
  imprimirValores();
  return;
  */
 
  programSpeed(velBaseIni, velBaseMax);
  drive(velIzq, velDer); 
}






void calibrateQTR()
{
  //drive(0, 0);

  delay(2000);
  for (int i = 0; i < SensorCount; i++)
  {
    qtr.calibrationOn.minimum[i] = 1000;
    qtr.calibrationOn.maximum[i] = 0;

  }

  Serial.println();
  Serial.println("Beginning Calibration Process...");

  for (int i = 0; i < 200; i++)  // make the calibration take about 10 seconds
  {
    qtr.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }

  Serial.println("Calibration Complete");

}
//**************************************************************************************************************
void readQTR()
{
  Serial.println();
  Serial.println("Reading Calibration Data...");

  for (int i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }

  delay(3000);

}
//**************************************************************************************************************
void storeQTR()
{
  Serial.println();
  Serial.println("Storing Calibration Data into EEPROM...");

  EEPROM.writeBlock<unsigned int>(addrCalibratedMinimumOn, qtr.calibrationOn.minimum, 8);
  EEPROM.writeBlock<unsigned int>(addrCalibratedMaximumOn, qtr.calibrationOn.maximum, 8);

  Serial.println("EEPROM Storage Complete");
}
//**************************************************************************************************************
void recallQTR()
{
  Serial.println();
  Serial.println("Recalling Calibration Data from EEPROM...");

  qtr.calibrate();
  EEPROM.readBlock<unsigned int>(addrCalibratedMinimumOn, qtr.calibrationOn.minimum, 8);
  EEPROM.readBlock<unsigned int>(addrCalibratedMaximumOn, qtr.calibrationOn.maximum, 8);

  Serial.println("EEPROM Recall Complete");
}
//**************************************************************************************************************
void leerSerial() {

  if (Serial.available())
  {
    char ch = Serial.read();

    if (ch == 'c' || ch == 'C')
    {
      calibrateQTR();
    }
    else if (ch == 'r' || ch == 'R')
    {
      readQTR();
    }
    else if (ch == 's' || ch == 'S')
    {
      storeQTR();
    }
    else if (ch == 'e' || ch == 'E')
    {
      recallQTR();
    }
  }

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

void readSensors() {
  unsigned int position = qtr.readLineBlack(sensorValues);

  P = (position - 3500);

}

void move(int motor, int speed, int direction){

  digitalWrite(STBY, HIGH); 
  boolean inPin1 = LOW;
  boolean inPin2 = HIGH;

 

  if(direction == 1){
    inPin1 = HIGH;
    inPin2 = LOW;
  }


  if(motor == 1){
    digitalWrite(AIN1, inPin1);
    digitalWrite(AIN2, inPin2);
    analogWrite(PWMA, speed);
  }
  else{
    digitalWrite(BIN1, inPin1);
    digitalWrite(BIN2, inPin2);
    analogWrite(PWMB, speed);
  }

}

void stop(){
  digitalWrite(STBY, LOW);
}

void drive(int speedl, int speedr)
{
  if (speedl > 0)
  {
    move(1,speedl,1);

  } else {
    move(1,speedl*-1,2);
  }
  if (speedr > 0)
  {
    move(2,speedr,1);
  } else {
    move(2,speedr*-1,2);
  }
}

void programSpeed(int velIni, int velMax)
{

  D = (P - LAST); /// ERROR MENOS EL ERROR ANTERIOR , DERIVATIVO
  I = (I + P); //INTEGRAL

  if (I * P < 0) I = 0;

  vel = (P * kP) + (D * kD) + (I * kI); // para velocidad 120//////estaba en 0.0925

  velIzq = velIni - vel;
  velDer = velIni + vel;

  if (velIzq > velMax) {
    velIzq = velMax;
  } else if (velIzq < -velMax) {
    velIzq = -velMax;
  }

  if (velDer > velMax) {
    velDer = velMax;
  } else if (velDer < -velMax) {
    velDer = -velMax;
  }

  LAST = P;

}
