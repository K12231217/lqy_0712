#ifndef __IMU660_H_
#define __IMU660_H_

#include "zf_common_headfile.h"

extern float yaw;
extern float filtered_yaw;
extern float Actual_gyro_z;
extern int16 gyro_bias_z;

void  imu660_yaw_calibration(void);
void  imu660_yaw_reset(float angle);
float imu660_yaw_update_5ms(void);
float imu660_get_yaw(void);
float imu660_get_yaw_360(void);
float imu660_get_delta_yaw(float start_yaw, float current_yaw);

void  yaw_calibration(void);
float get_yaw_angle(void);
float get_quan(float angle);

#endif
