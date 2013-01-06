/**************************************************************************
 *                                                                         *
 * Sensor Fusion code for estimating orientation of Arduino-based IMU      *
 * 2011 Joseph Malloch / Input Devices and Music Interaction Laboratory    *
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

#ifdef __cplusplus
extern "C" {
#endif

// Data structures
typedef struct _quaternion
{
    double  w;
    double  x;
    double  y;
    double  z;
} t_quaternion;

typedef struct _axes
{
    double  x;
    double  y;
    double  z;
    double  roll;
    double  tilt;
    double  magnitude;
} t_axes;

// Function prototypes
void sensor_fusion(t_quaternion *orientation, t_axes *accel, t_axes *mag, t_axes *gyro, double weight);
void quaternion_init(t_quaternion *q);
void quaternion_multiply(t_quaternion *l, t_quaternion *r, t_quaternion *o);
void quaternion_inverse(t_quaternion *i, t_quaternion *o);
void quaternion_conjugate(t_quaternion *i, t_quaternion *o);
void quaternion_slerp(t_quaternion *l, t_quaternion *r, t_quaternion *o, double weight);
double quaternion_dot_product(t_quaternion *l, t_quaternion *r);
void quaternion_normalize(t_quaternion *q);
void quaternion_minimum_distance(t_quaternion *old, t_quaternion *newer);
void quaternion_copy(t_quaternion *original, t_quaternion *newer);

#ifdef __cplusplus
}
#endif
