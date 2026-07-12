#include "test.h"
#include "Ring.h"
#include "imu660.h"

#define PROJECT_SELF_TEST_ENABLE 0

#if PROJECT_SELF_TEST_ENABLE
uint8 num1;
uint8 num2;
uint8 num3;
uint8 num4;

uint16 nums5;

uint8 imu_delta_wrap_positive_ok;
uint8 imu_delta_wrap_negative_ok;
uint8 imu_yaw_reset_limit_ok;
uint8 imu_yaw_integrates_5ms_ok;

void test_num(void)
{
	uint8 i;

	num1 = Ring_ShouldBeepOnStepEnter(0, 1);
	num2 = Ring_ShouldBeepOnStepEnter(1, 1);
	num3 = Ring_ShouldBeepOnStepEnter(10, 0);
	num4 = Ring_ShouldBeepOnStepEnter(7, 8);

	nums5 = Ring_Debug_CanUseCachedLeftBreak(1, 40, 60);

	imu_delta_wrap_positive_ok =
		(imu660_get_delta_yaw(350.0f, 10.0f) > 19.0f &&
		 imu660_get_delta_yaw(350.0f, 10.0f) < 21.0f);
	imu_delta_wrap_negative_ok =
		(imu660_get_delta_yaw(10.0f, 350.0f) < -19.0f &&
		 imu660_get_delta_yaw(10.0f, 350.0f) > -21.0f);

	imu660_yaw_reset(190.0f);
	imu_yaw_reset_limit_ok =
		(imu660_get_yaw() > -171.0f &&
		 imu660_get_yaw() < -169.0f);

	imu660_yaw_reset(0.0f);
	gyro_bias_z = 0;
	imu660ra_gyro_z = (int16)(100.0f * imu660ra_transition_factor[1]);
	for(i = 0; i < 200; i++)
	{
		imu660_yaw_update_5ms();
	}
	imu_yaw_integrates_5ms_ok =
		(imu660_get_yaw() > 99.0f &&
		 imu660_get_yaw() < 101.0f);

	imu660ra_gyro_z = 0;
	imu660_yaw_reset(0.0f);
}
#endif

