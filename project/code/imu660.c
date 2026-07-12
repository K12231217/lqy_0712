#include "imu660.h"

#define IMU660_YAW_DT_S              (0.005f)
#define IMU660_GYRO_DEADBAND_DPS     (0.5f)
#define IMU660_YAW_SCALE             (1.0f)

int16 gyro_bias_z = 0;
float yaw = 0.0f;
float filtered_yaw = 0.0f;
float Actual_gyro_z = 0.0f;

static float imu660_limit_yaw(float angle)
{
    while(angle >= 180.0f)
    {
        angle -= 360.0f;
    }

    while(angle < -180.0f)
    {
        angle += 360.0f;
    }

    return angle;
}

static float imu660_to_360(float angle)
{
    angle = imu660_limit_yaw(angle);

    if(angle < 0.0f)
    {
        angle += 360.0f;
    }

    return angle;
}

void imu660_yaw_calibration(void)
{
    uint8 i;
    int32 bias_sum = 0;

    for(i = 0; i < 100; i++)
    {
        imu660rb_get_gyro();
        bias_sum += imu660rb_gyro_z;
        system_delay_ms(10);
    }

    gyro_bias_z = (int16)(bias_sum / 100);
    imu660_yaw_reset(0.0f);
}

void imu660_yaw_reset(float angle)
{
    yaw = imu660_limit_yaw(angle);
    filtered_yaw = yaw;
    Actual_gyro_z = 0.0f;
}

float imu660_yaw_update_5ms(void)
{
    int16 gyro_z_raw;
    float gyro_z_dps;
    float yaw_next;

    gyro_z_raw = (int16)(imu660rb_gyro_z - gyro_bias_z);
    gyro_z_dps = imu660rb_gyro_transition(gyro_z_raw);

    if(gyro_z_dps > -IMU660_GYRO_DEADBAND_DPS &&
       gyro_z_dps < IMU660_GYRO_DEADBAND_DPS)
    {
        gyro_z_dps = 0.0f;
    }

    Actual_gyro_z = gyro_z_dps;

    yaw_next = yaw + gyro_z_dps * IMU660_YAW_DT_S * IMU660_YAW_SCALE;
    yaw_next = imu660_limit_yaw(yaw_next);

    yaw = yaw_next;
    filtered_yaw = yaw;

    return yaw;
}

float imu660_get_yaw(void)
{
    return yaw;
}

float imu660_get_yaw_360(void)
{
    return imu660_to_360(yaw);
}

float imu660_get_delta_yaw(float start_yaw, float current_yaw)
{
    return imu660_limit_yaw(imu660_to_360(current_yaw) - imu660_to_360(start_yaw));
}

void yaw_calibration(void)
{
    imu660_yaw_calibration();
}

float get_yaw_angle(void)
{
    return imu660_yaw_update_5ms();
}

float get_quan(float angle)
{
    return imu660_to_360(angle);
}
