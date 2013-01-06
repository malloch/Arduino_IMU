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

#include "imu.h"

// Constants
double PI = 3.141592653589793;
double halfPI = 1.570796326794897;
double twoPI = 6.283185307179586;

// *********************************************************
// -(anything)----------------------------------------------
void sensor_fusion(t_quaternion *orientation, t_axes *accel, t_axes *mag, t_axes *gyro, double weight)
{
    // calculate polar representation of accelerometer data
    accel->roll = atan2(accel->z, accel->y);
    accel->magnitude = sqrt(pow(accel->z, 2) + pow(accel->y, 2));
    accel->tilt = atan2(accel->x, accel->magnitude);
    accel->magnitude = sqrt(pow(accel->x, 2) + pow(accel->magnitude, 2));
    accel->magnitude *= 0.00390625;

    // calculate accelerometer quaternions
    t_quaternion qar, qar_inv, qat, qat_inv, qa;
    qar.w = cos(accel->roll * 0.5);
    qar.y = sin(accel->roll * 0.5);
    qar.x = qar.z = 0;
    quaternion_inverse(&qar, &qar_inv);
    qat.w = cos(accel->tilt * 0.5);
    qat.x = sin(accel->tilt * 0.5);
    qat.y = qat.z = 0;
    quaternion_inverse(&qat, &qat_inv);
    quaternion_multiply(&qat, &qar, &qa);

    // calculate polar representation of magnetometer data
    mag->roll = atan2(mag->z, mag->y) - PI;
    mag->magnitude = sqrt(pow(mag->z, 2) + pow(mag->y, 2));
    mag->tilt = atan2(mag->x, mag->magnitude);

    // calculate magnetometer quaternions
    t_quaternion qmr, qmt, qm;
    qmr.w = cos(mag->roll * 0.5);
    qmr.y = sin(mag->roll * 0.5);
    qmr.x = qmr.z = 0;
    qmt.w = cos(mag->tilt * 0.5);
    qmt.x = sin(mag->tilt * 0.5);
    qmt.y = qmt.z = 0;
    quaternion_multiply(&qmt, &qmr, &qm);

    // rotate the magnetometer quaternion
    quaternion_multiply(&qm, &qar_inv, &qm);
    quaternion_multiply(&qm, &qat_inv, &qm);

    // extract azimuth
    double azimuth = atan2(qm.x, qm.y) + PI;
    if (qm.w > 0.)
        azimuth += PI;
    azimuth = fmod(azimuth, twoPI) - PI;

    // replace qm with just azimuth
    qm.w = cos(azimuth * 0.5);
    qm.x = qm.y = 0;
    qm.z = sin(azimuth * 0.5);

    // construct quaternion from combined accelerometer and magnetometer azimuth data
    t_quaternion qam;
    quaternion_multiply(&qm, &qa, &qam);

    // construct quaternion from gyroscope axes
    t_quaternion qg;
    qg.w = cos((gyro->x + gyro->y + gyro->z) * 0.5);
    qg.x = sin(gyro->z * 0.5);
    qg.y = sin(gyro->x * 0.5);
    qg.z = sin(gyro->y * 0.5);
    /*
     // calculate ema and emd
     x->average = accel->magnitude * x->weight + x->average * (1 - x->weight);
     float temp = abs(accel->magnitude - x->average);
     x->deviation = temp * x->weight + x->deviation * (1 - x->weight);
     float weight;
     if (accel->magnitude < 0.05)
     weight = 1;
     else {
     weight = (accel->magnitude - 0.004) * 0.045918367346939 + 0.99;
     weight = weight < 0.9999 ? weight : 0.9999;
     }
     */

    // complementary filter:
    // integrate latest gyro quaternion with stored orientation
    quaternion_multiply(orientation, &qg, orientation);
    // SLERP between stored orientation and new accel + mag estimate
    t_quaternion delayed;
    quaternion_copy(orientation, &delayed);
    quaternion_slerp(&qam, orientation, orientation, weight);

    // use the shortest distance from previous orientation
    quaternion_minimum_distance(&delayed, orientation);
}

// *********************************************************
// -(t_quaternion initialization)---------------------------
void quaternion_init(t_quaternion *q)
{
   q->w = 1;
   q->x = 0;
   q->y = 0;
   q->z = 0;
}

// *********************************************************
// -(quaternion multiplication)-----------------------------
void quaternion_multiply(t_quaternion *l, t_quaternion *r, t_quaternion *o)
{
    double temp[4];
    temp[0] = l->w * r->w - l->x * r->x - l->y * r->y - l->z * r->z;
    temp[1] = l->x * r->w + l->w * r->x + l->y * r->z - l->z * r->y;
    temp[2] = l->w * r->y - l->x * r->z + l->y * r->w + l->z * r->x;
    temp[3] = l->w * r->z + l->x * r->y - l->y * r->x + l->z * r->w;
    o->w = temp[0];
    o->x = temp[1];
    o->y = temp[2];
    o->z = temp[3];
}

// *********************************************************
// -(quaternion inverse)------------------------------------
void quaternion_inverse(t_quaternion *i, t_quaternion *o)
{
    float norm_squared = i->w * i->w + i->x * i->x + i->y * i->y + i->z * i->z;
    if (norm_squared == 0)
      norm_squared = 0.0000001;
    o->w = i->w / norm_squared;
    o->x = i->x * -1 / norm_squared;
    o->y = i->y * -1 / norm_squared;
    o->z = i->z * -1 / norm_squared;
}

// *********************************************************
// -(quaternion conjugate)----------------------------------
void quaternion_conjugate(t_quaternion *i, t_quaternion *o)
{
    o->w = i->w;
    o->x = i->x * -1;
    o->y = i->y * -1;
    o->z = i->z * -1;
}

// *********************************************************
// -(quaternion slerp)--------------------------------------
void quaternion_slerp(t_quaternion *l, t_quaternion *r, t_quaternion *o, double weight)
{
    double dot = quaternion_dot_product(l, r);
    if (dot > 0.9995) {
        o->w = l->w + (r->w - l->w) * weight;
        o->x = l->x + (r->x - l->x) * weight;
        o->y = l->y + (r->y - l->y) * weight;
        o->z = l->z + (r->z - l->z) * weight;
        quaternion_normalize(o);
        return;
    }

    if (dot > 1)
        dot = 1;
    else if (dot < -1)
        dot = -1;

    double theta_0 = acos(dot);
    double theta = (0. < theta_0 && theta_0 < halfPI) ? theta_0 * weight : (theta_0 - PI) * weight;

    o->w = r->w - l->w * dot;
    o->x = r->x - l->x * dot;
    o->y = r->y - l->y * dot;
    o->z = r->z - l->z * dot;

    quaternion_normalize(o);

    o->w = l->w * cos(theta) + o->w * sin(theta);
    o->x = l->x * cos(theta) + o->x * sin(theta);
    o->y = l->y * cos(theta) + o->y * sin(theta);
    o->z = l->z * cos(theta) + o->z * sin(theta);
}

// *********************************************************
// -(quaternion dot product)--------------------------------
double quaternion_dot_product(t_quaternion *l, t_quaternion *r)
{
    return l->w * r->w + l->x * r->x + l->y * r->y + l->z * r->z;
}

// *********************************************************
// -(quaternion normalization)------------------------------
void quaternion_normalize(t_quaternion *q)
{
    double norm = sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    if (norm == 0)
      norm = 0.0000001;
    q->w /= norm;
    q->x /= norm;
    q->y /= norm;
    q->z /= norm;
}

// *********************************************************
// -(quaternion minimum distance)---------------------------
void quaternion_minimum_distance(t_quaternion *old, t_quaternion *newer)
{
    // use the shortest distance
    if (quaternion_dot_product(old, newer) < 0) {
        newer->w = newer->w * -1;
        newer->x = newer->x * -1;
        newer->y = newer->y * -1;
        newer->z = newer->z * -1;
    }
}

// *********************************************************
// -(quaternion copy)---------------------------------------
void quaternion_copy(t_quaternion *original, t_quaternion *newer)
{
    newer->w = original->w;
    newer->x = original->x;
    newer->y = original->y;
    newer->z = original->z;
}
