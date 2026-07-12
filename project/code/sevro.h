#ifndef __SEVRO_H_
#define __SEVRO_H_
#include "zf_common_headfile.h"
#define SERVO_PWM1              (PWME_CH1P_PA0)                         // 定义主板上舵机对应引脚
#define SERVO_PWM2              (PWME_CH2P_PA2)                         // 定义主板上舵机对应引脚
#define SERVO_PWM3              (PWME_CH3P_PA4)                         // 定义主板上舵机对应引脚
#define SERVO_PWM4              (PWME_CH4P_PA6)                         // 定义主板上舵机对应引脚

#define SERVO_FREQ              (300)                                           // 定义主板上舵机频率  请务必注意范围 50-300

// ------------------ 舵机占空比计算方式 ------------------
//
// 舵机对应的 0-180 活动角度对应 控制脉冲的 0.5ms-2.5ms 高电平
// 那么不同频率下的占空比计算方式就是
// PWM_DUTY_MAX/(1000/freq)*(0.5+Angle*90) 在 50hz 时就是 PWM_DUTY_MAX/(1000/50)*(1+Angle/180)
// 斜率是K=（2.5-0.5）/（180-0）=1/90    截距是0.5
// 那么 100hz 下 90度的打角 即高电平时间1.5ms 计算套用为
// PWM_DUTY_MAX/(1000/100)*(0.5+Angle*90) = PWM_DUTY_MAX/10*1.5
// ------------------ 舵机占空比计算方式 ------------------
#define SERVO_DUTY(x)         ((float)PWM_DUTY_MAX / (1000.0 / (float)SERVO_FREQ) * (0.5 + (float)(x) / 90.0))

#if ((SERVO_FREQ < 50) || (SERVO_FREQ > 300))
    #error "SERVO_MOTOR_FREQ ERROE!"
#endif


#define SERVO_DUTY_MID   4400
#define SERVO_DUTY_MIN   3850   //向右
#define SERVO_DUTY_MAX   4900   //向左 
#define SERVO_CENTER 4650
#define MAX_TURN  500     //(((SERVO_DUTY_MAX)-(SERVO_DUTY_MIN))>>1)

#define controlReferenceLine 80
#define Mid_Col 94  

void servo_init(void);
void servo_setAngle(pwm_channel_enum sever,float angle);
void Servo_Loop(void);

#endif


