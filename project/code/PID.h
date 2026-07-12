#ifndef __PID_H_
#define __PID_H_

#include "zf_common_headfile.h"

#define OUT_MAX     6000
#define OUT_MIN    -6000

extern volatile uint16 Out_servo;
typedef struct
{
    float kp;       /* 比例系数 */
    float ki;       /* 积分系数 */
    float kd;       /* 微分系数 */
	  float kg;		/* 陀螺仪系数 */
    float kp2;      /* 二次项系数，如果后轮不用就填0 */
    
    float err1;     /* 上一次误差 */
    float err2;     /* 上上次误差 */
    float err_sum;  /* 积分项，前轮位置式 PID 用，后轮增量式可不用 */

    float out;      /* 输出 */
} PID_tf;


extern volatile PID_tf pid_rf;
extern volatile PID_tf pid_lf;
extern volatile PID_tf servo_pidf;
void PID_servof(volatile PID_tf *pid);
void Increment_PIDf(volatile PID_tf *pid, float tar_val, float act_val);
#endif

