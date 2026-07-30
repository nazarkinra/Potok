#ifndef MADGWICK_AHRS_H
#define MADGWICK_AHRS_H

#include <math.h>

extern float q0, q1, q2, q3;

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);

#endif
