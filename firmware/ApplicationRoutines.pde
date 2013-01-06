/**************************************************************************
 *                                                                         *
 * Application routines for Arduino-based IMU                              *
 *                                                                         *
 ***************************************************************************
 *                                                                         *
 * This program is free software; you can redistribute it and/or modify    *
 * it under the terms of the GNU License.                                  *
 * This program is distributed in the hope that it will be useful,         *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 * GNU License V2 for more details.                                        *
 *                                                                         *
 ***************************************************************************/

void checkSerial() {
  if(!Serial.available())
    return;

  char c = Serial.read();

  switch (c) {
    case 'p':          // poll message, for now as fast as possible
      polled = 1;
      break;
    case 'c':          // calibrate message
      while (Serial.available() < 1) {}
      switch (Serial.read()) {
        case 1:
          // accelerometer calibration
          for (c=0; c<3; c++) {
            accel_calibration[c][0] = 512;
            accel_calibration[c][1] = -512;
          }
          calibrate = CALIBRATE_ACCEL;
          break;
        case 2:
          // magnetometer calibration
          for (c=0; c<3; c++) {
            mag_calibration[c][0] = 2047;
            mag_calibration[c][1] = -2048;
          }
          calibrate = CALIBRATE_MAG;
          break;
        default:
          calibrate = CALIBRATE_NONE;
          break;
      }
      break;
    case 'w':          // write settings
      while (Serial.available() < 1) {}
      c = Serial.read();
      switch (c) {
        case 'w':
          writeSettings();
          break;
        default:
        break;
      }
  }
}

void setupAccelerometer(){
  // Start accelerometer
  adxl345.powerOn();
}

void readAccelerometer() {
  int temp[3], i;
  adxl345.readAccel(temp);
  for (i=0; i<3; i++) {
    if (calibrate == CALIBRATE_ACCEL) {
      accel_calibration[i][0] = min(accel_calibration[i][0], temp[i]);
      accel_calibration[i][1] = max(accel_calibration[i][1], temp[i]);
      accel_calibration[i][2] = compute_scale(accel_calibration[i][0], accel_calibration[i][1], -1.0, 1.0);
      accel_calibration[i][3] = compute_offset(accel_calibration[i][0], accel_calibration[i][1], -1.0, 1.0);
    }
  }
  accel.x = (double)temp[0] * accel_calibration[0][2] + accel_calibration[0][3];
  accel.z = (double)temp[1] * accel_calibration[1][2] + accel_calibration[1][3];
  accel.y = (double)temp[2] * accel_calibration[2][2] + accel_calibration[2][3];

  accel.x *= -1.0;
}

double compute_scale(double src_min, double src_max, double dest_min, double dest_max) {
  if (src_min == src_max)
    return 0;
  return (dest_min - dest_max) / (src_min - src_max);
}

double compute_offset(double src_min, double src_max, double dest_min, double dest_max) {
  if (src_min == src_max)
    return dest_min;
  return (dest_max * src_min - dest_min * src_max) / (src_min - src_max);
}

boolean readSettings() {
  byte written;
  int n = EEPROM_readAnything(0, written);    //checks if any data written before overwriting defaults

  if (written == 100) {
    n = EEPROM_readAnything(n, accel_calibration);
    n = EEPROM_readAnything(n, mag_calibration);
  }
}

boolean writeSettings() {
  byte written = 100;
  int n = EEPROM_writeAnything(0, written);

  n = EEPROM_writeAnything(n, accel_calibration);
  n = EEPROM_writeAnything(n, mag_calibration);
}

void StatusLEDToggle()
{
  static unsigned int counter = 0;
  static char state = 0;

  counter++;
  if (counter > 20)
  {
    counter=0;
    if(state)
    {
      digitalWrite(STATUS_LED,LOW);
      state = 0;
    }
    else
    {
      digitalWrite(STATUS_LED,HIGH);
      state = 1;
    }

  }
}
