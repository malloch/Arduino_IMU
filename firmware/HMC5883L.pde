/* ******************************************************* */
/* I2C code for HMC5583 magnetometer                       */
/*                                                         */
/* ******************************************************* */
#include <HMC58X3.h>

//I2C addresses
int CompassAddress = 0x1E;   //Write:0x3C  Read:0x3D

HMC58X3 magn;

void setupMagnetometer()
{
  // no delay needed as we have already a delay(5) in HMC5843::init()
  magn.init(false); // Dont set mode yet, we'll do that later on.
  // Calibrate HMC using self test, not recommended to change the gain after calibration.
  magn.calibrate(0); // Use gain 1=default, valid 0-7, 7 not recommended.
  // Single mode conversion was used in calibration, now set continuous mode
  magn.setMode(0);
}

void readMagnetometer()
{
  int temp[3];
  byte i;

  magn.getRaw(&temp[0],&temp[1],&temp[2]);

  if (calibrate == CALIBRATE_MAG) {
    for (i=0; i<3; i++) {
      mag_calibration[i][0] = min(mag_calibration[i][0], (double)temp[i]);
      mag_calibration[i][1] = max(mag_calibration[i][1], (double)temp[i]);
      mag_calibration[i][2] = compute_scale(mag_calibration[i][0],
                                            mag_calibration[i][1],
                                            -0.5,
                                            0.5);
      mag_calibration[i][3] = compute_offset(mag_calibration[i][0],
                                            mag_calibration[i][1],
                                            -0.5,
                                            0.5);
    }
  }
  mag.x = (double)temp[0] * mag_calibration[0][2] + mag_calibration[0][3];
  mag.z = (double)temp[1] * mag_calibration[1][2] + mag_calibration[1][3];
  mag.y = (double)temp[2] * mag_calibration[2][2] + mag_calibration[2][3];

  // change signs?
}










