//**************************************************************************************//
//  Gestes Spine2 w/ Mongoose IMU (Accelerometer + Gyroscope + Magnetometer)            //
//  Joseph Malloch                                                                      //
//  Input Devices and Music Interaction Laboratory                                      //
//  Modified : 2012/01/30                                                               //
//  Notes   : Using modified sensor reading functions from base Mongoose firmware v 1.1 //
//            EEPROM read/write functions adapted from code by Halley @arduino.cc       //
//**************************************************************************************//

#include <HMC58X3.h>
#include "ADXL345.h"
#include <EEPROM.h>
#include <Wire.h>
#include "eeprom_stuff.h"
#include "imu.h"

ADXL345 adxl345;

#define STATUS_LED 4  //PD4 on the Atmega328. Red LED
#define XBEE_SLEEP 6
#define SPI_SS 10
#define CALIBRATE_NONE 0
#define CALIBRATE_ACCEL 1
#define CALIBRATE_MAG 2

#define IS_SPI_MASTER 1
#define XBEE_COMMS 1

//#define ID 100 // Prototype 1.2
//#define ID 101 // Prototype 4.1
#define ID 102 // Prototype 4.3

t_quaternion quat, quat_remote1, quat_remote2;
t_axes accel, accel_remote1, accel_remote2;
t_axes gyro, gyro_remote1, gyro_remote2;
t_axes mag, mag_remote1, mag_remote2;

// make some byte array pointers for convenient serial output
unsigned char *quat_bytes = (unsigned char *)&quat;
unsigned char *quat_remote1_bytes = (unsigned char *)&quat_remote1;
unsigned char *quat_remote2_bytes = (unsigned char *)&quat_remote2;
unsigned char *accel_bytes = (unsigned char *)&accel;
unsigned char *accel_remote1_bytes = (unsigned char *)&accel_remote1;
unsigned char *accel_remote2_bytes = (unsigned char *)&accel_remote2;
unsigned char *gyro_bytes = (unsigned char *)&gyro;
unsigned char *gyro_remote1_bytes = (unsigned char *)&gyro_remote1;
unsigned char *gyro_remote2_bytes = (unsigned char *)&gyro_remote2;
unsigned char *mag_bytes = (unsigned char *)&mag;
unsigned char *mag_remote1_bytes = (unsigned char *)&mag_remote1;
unsigned char *mag_remote2_bytes = (unsigned char *)&mag_remote2;

double accel_calibration[3][4] = {{0, 0, 1, 0},
                                  {0, 0, 1, 0},
                                  {0, 0, 1, 0}};
double mag_calibration[3][4] = {{0, 0, 1, 0},
                                {0, 0, 1, 0},
                                {0, 0, 1, 0}};

//define values for slip coding
byte escapeChar = 101;
byte delimiterChar = 100;

unsigned int gyro_time;
double period = 0.0;
double weight = 0.99;

byte calibrate = CALIBRATE_NONE;

byte polled = 0;
byte counter = 0;

void setup()
{
  //readSettings();

  if (IS_SPI_MASTER) {
    if (XBEE_COMMS)
      Serial.begin(57600);
    else
      Serial.begin(115200);
  }

  pinMode (STATUS_LED, OUTPUT);  // Status LED

  Wire.begin();    //Init the I2C
  if (IS_SPI_MASTER) {
    SPI_MASTER_Init();
    if (XBEE_COMMS)
      pinMode (XBEE_SLEEP, OUTPUT); // XBee sleep
      digitalWrite(XBEE_SLEEP, LOW);
    delay(500);
  }
  else {
    SPI_SLAVE_Init();
    delay(20);
  }

  setupAccelerometer();
  setupGyroscope();
  setupMagnetometer();

  delay(500);

  quaternion_init(&quat);
  quaternion_init(&quat_remote1);
  quaternion_init(&quat_remote2);
}

void loop() //Main Loop
{
  if (IS_SPI_MASTER)
    checkSerial();

  readAccelerometer();
  readGyroscope();
  readMagnetometer();

  sensor_fusion(&quat, &accel, &mag, &gyro, weight);
  //digitalWrite(STATUS_LED,HIGH);

  if (IS_SPI_MASTER) {
    //for (int i=0; i < 16; i++) {
    //  digitalWrite(SPI_SS, LOW);
    //  quat_remote2_bytes[i] = SPI_ReadWrite(counter++);
    //  digitalWrite(SPI_SS, HIGH);
    //}
    //for (int i=0; i < 12; i++) {
    //  digitalWrite(SPI_SS, LOW);
    //  accel_remote2_bytes[i] = SPI_ReadWrite(counter++);
    //  digitalWrite(SPI_SS, HIGH);
    //}
    //for (int i=0; i < 12; i++) {
    //  digitalWrite(SPI_SS, LOW);
    //  gyro_remote2_bytes[i] = SPI_ReadWrite(counter++);
    //  digitalWrite(SPI_SS, HIGH);
    //}
    for (int i=0; i < 16; i++) {
      digitalWrite(SPI_SS, LOW);
      quat_remote1_bytes[i] = SPI_ReadWrite(counter++);
      digitalWrite(SPI_SS, HIGH);
    }
    for (int i=0; i < 12; i++) {
      digitalWrite(SPI_SS, LOW);
      accel_remote1_bytes[i] = SPI_ReadWrite(counter++);
      digitalWrite(SPI_SS, HIGH);
    }
    for (int i=0; i < 12; i++) {
      digitalWrite(SPI_SS, LOW);
      gyro_remote1_bytes[i] = SPI_ReadWrite(counter++);
      digitalWrite(SPI_SS, HIGH);
    }
    if (polled) {
      digitalWrite(STATUS_LED, HIGH);
      slipOutByte(ID);
      slipOutDouble(quat_remote1.w);
      slipOutDouble(quat_remote1.x);
      slipOutDouble(quat_remote1.y);
      slipOutDouble(quat_remote1.z);
      slipOutDouble(accel_remote1.x);
      slipOutDouble(accel_remote1.y);
      slipOutDouble(accel_remote1.z);
      slipOutDouble(gyro_remote1.x);
      slipOutDouble(gyro_remote1.y);
      slipOutDouble(gyro_remote1.z);
      //slipOutDouble(mag_remote1.x);
      //slipOutDouble(mag_remote1.y);
      //slipOutDouble(mag_remote1.z);

      //slipOutDouble(quat_remote2.w);
      //slipOutDouble(quat_remote2.x);
      //slipOutDouble(quat_remote2.y);
      //slipOutDouble(quat_remote2.z);
      //slipOutDouble(accel_remote2.x);
      //slipOutDouble(accel_remote2.y);
      //slipOutDouble(accel_remote2.z);
      //slipOutDouble(gyro_remote2.x);
      //slipOutDouble(gyro_remote2.y);
      //slipOutDouble(gyro_remote2.z);
      //slipOutDouble(mag_remote2.x);
      //slipOutDouble(mag_remote2.y);
      //slipOutDouble(mag_remote2.z);

      slipOutDouble(quat.w);
      slipOutDouble(quat.x);
      slipOutDouble(quat.y);
      slipOutDouble(quat.z);
      slipOutDouble(accel.x);
      slipOutDouble(accel.y);
      slipOutDouble(accel.z);
      slipOutDouble(gyro.x);
      slipOutDouble(gyro.y);
      slipOutDouble(gyro.z);
      //slipOutDouble(mag.x);
      //slipOutDouble(mag.y);
      //slipOutDouble(mag.z);

      //slipOutDouble(period);
      Serial.write(delimiterChar);

      polled = 0;
      digitalWrite(STATUS_LED, LOW);
    }
    delay(1);
  }
  else {
    for(int i=0; i<16; i++) {
      quat_remote1_bytes[i] = SPI_ReadWrite(quat_bytes[i]);
    }
    for(int i=0; i<12; i++) {
      accel_remote1_bytes[i] = SPI_ReadWrite(accel_bytes[i]);
    }
    for(int i=0; i<12; i++) {
      gyro_remote1_bytes[i] = SPI_ReadWrite(gyro_bytes[i]);
    }
    //for(int i=0; i<12; i++) {
    //  mag_remote1_bytes[i] = SPI_ReadWrite(mag_bytes[i]);
    //}
    //for(int i=0; i<16; i++) {
    //  SPI_ReadWrite(quat_remote1_bytes[i]);
    //}
    //for(int i=0; i<12; i++) {
    //  SPI_ReadWrite(accel_remote1_bytes[i]);
    //}
    //for(int i=0; i<12; i++) {
    //  SPI_ReadWrite(gyro_remote1_bytes[i]);
    //}
    //for(int i=0; i<12; i++) {
    //  SPI_ReadWrite(mag_remote1_bytes[i]);
    //}
    StatusLEDToggle();
  }
}

