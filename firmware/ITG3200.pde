/* ******************************************************* */
/* I2C code for ITG-3200 Gyro                              */
/*                                                         */
/* ******************************************************* */

//I2C addresses
#define GyroAddress 0x68      //Write:0xD0  Read:0xD1

// IGT-3200 Sensitivity (from datasheet) => 14.375 LSBs/deg/s => 0.250891079974185 LSBs/rad/sec
// Tested values :
#define Gyro_Gain_X   823.626830500558425 //X axis Gyro gain
#define Gyro_Gain_Y   823.626830500558425 //Y axis Gyro gain
#define Gyro_Gain_Z   823.626830500558425 //Z axis Gyro gain

double gyro_ema[3] = {0, 0, 0};
double gyro_emd[3] = {0, 0, 0};
double gyro_bias[3] = {0, 0, 0};
double gyro_raw[3] = {0, 0, 0};

//============================================
// Gyro
//============================================
void setupGyroscope()
{
  //Set the sample rate divider
  Wire.beginTransmission(GyroAddress);
  Wire.send(0x15);  // Sample rate divider register
  Wire.send(99);  // Sample rate divider is 1 (register value + 1)
  Wire.endTransmission();
  delay(20);

  //Set the DLPF
  Wire.beginTransmission(GyroAddress);
  Wire.send(0x16);  // DLPF register
  Wire.send( (0x03<<3) | (0x00<<0) );  // Set the full-scale range to +/- 2000deg/sec, and the low pass filter to 256Hz
  Wire.endTransmission();
  delay(20);	

  //Setup the clock reference
  Wire.beginTransmission(GyroAddress);
  Wire.send(0x3E);  // Power and clock register
  Wire.send(1);  // Use the PLL with the X Gyro as the clock reference
  Wire.endTransmission();
  delay(20);	

  // Because our main loop runs at 50Hz we adjust the output data rate to 50Hz (25Hz bandwith)
  //Wire.beginTransmission(AccelAddress);
  //Wire.send(0x2C);  // Rate
  //Wire.send(0x09);  // set to 50Hz, normal operation
  //Wire.endTransmission();

  gyro_time = millis();
}

// Reads the angular rates from the Gyro
void readGyroscope()
{
  byte i = 0, j;
  byte buff[8];  //6 bytes of angular rate data, and 2 bytes of temperature data

  Wire.beginTransmission(GyroAddress);
  Wire.send(0x1B);        //The temperature and gyro data starts at address 0x1B
  Wire.endTransmission(); //end transmission

  Wire.requestFrom(GyroAddress, 8);    // request 8 bytes from device
  while(Wire.available())   // ((Wire.available())&&(i<6))
  {
    buff[i] = Wire.receive();  // receive one byte
    i++;
  }

  unsigned int elapsed = gyro_time;
  gyro_time = millis();
  if (gyro_time > elapsed)
    elapsed = gyro_time - elapsed;
  else
    elapsed = 0;

  period = period * 0.9 + elapsed * 0.1;

  if (i==8)  // All bytes received?
    {
      //get the raw data
      gyro_raw[0] = ((((int)buff[2]) << 8) | buff[3])/Gyro_Gain_X;    // X axis
      gyro_raw[1] = ((((int)buff[4]) << 8) | buff[5])/Gyro_Gain_Y;    // Y axis
      gyro_raw[2] = ((((int)buff[6]) << 8) | buff[7])/Gyro_Gain_Z;    // Z axis

      // update exponential moving average, deviation, and bias estimates
      for (i=0; i < 3; i++) {
        gyro_ema[i] = gyro_raw[i] * 0.1 + gyro_ema[i] * 0.9;
        gyro_emd[i] = abs(gyro_raw[i] - gyro_ema[i]) * 0.1 + gyro_emd[i] * 0.9;
        j = gyro_emd[i] < 1 ? 0.01 : 0.00001;
        gyro_bias[i] = gyro_raw[i] * j + gyro_bias[i] * (1 - j);
        gyro_raw[i] -= gyro_bias[i];
      }
      gyro.x = gyro_raw[0] * elapsed * 0.001;
      gyro.z = gyro_raw[1] * elapsed * 0.001;
      gyro.y = gyro_raw[2] * elapsed * -0.001;
    }
  //else
    // transmit error code using DEBUG_LED ?
}

