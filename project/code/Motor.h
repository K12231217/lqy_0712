#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "zf_common_headfile.h"
extern int16 encoder_data_r;
extern int16 encoder_data_l;
extern int16 tar_speed;

/* DRV8701 resources from the target vehicle. Keep PWM_L/PWM_R aliases because
 * the reference control logic routes wheel commands through Motor_control(). */
#define MOTOR_LEFT_EN_PWM    ( PWMB_CH3_P76 )
#define MOTOR_LEFT_PH        ( IO_P75 )
#define MOTOR_LEFT_NSLEEP    ( IO_P74 )
#define MOTOR_RIGHT_EN_PWM   ( PWMD_CH1_P50 )
#define MOTOR_RIGHT_PH       ( IO_P77 )
#define MOTOR_RIGHT_NSLEEP   ( IO_P51 )

#define PWM_L                ( MOTOR_LEFT_EN_PWM )
#define PWM_R                ( MOTOR_RIGHT_EN_PWM )
#define MOTOR_PWM_FREQ       ( 15000 )

/* The target vehicle wiring is inverted relative to positive vehicle speed.
 * Encoder signs are migrated separately and must be verified with the wheels raised. */
#define MOTOR_LEFT_COMMAND_SIGN   ( -1 )
#define MOTOR_RIGHT_COMMAND_SIGN  ( -1 )

/* Target vehicle encoder wiring: right=TIM17(P8.0,P4.4), left=TIM18(P9.0,P4.6). */
#define ENCODER_RIGHT_TIMER            ( TIM17_ENCODER )
#define ENCODER_RIGHT_PULSE_PIN        ( TIM17_ENCODER_CH1_P80 )
#define ENCODER_RIGHT_DIR_PIN          ( IO_P44 )
#define ENCODER_LEFT_TIMER             ( TIM18_ENCODER )
#define ENCODER_LEFT_PULSE_PIN         ( TIM18_ENCODER_CH1_P90 )
#define ENCODER_LEFT_DIR_PIN           ( IO_P46 )

/* Keep Init.c's reference-project interface unchanged. */
#define ENCODER_DIR_1                  ( ENCODER_RIGHT_TIMER )
#define ENCODER_DIR_PULSE_1            ( ENCODER_RIGHT_PULSE_PIN )
#define ENCODER_DIR_DIR_1              ( ENCODER_RIGHT_DIR_PIN )
#define ENCODER_DIR_2                  ( ENCODER_LEFT_TIMER )
#define ENCODER_DIR_PULSE_2            ( ENCODER_LEFT_PULSE_PIN )
#define ENCODER_DIR_DIR_2              ( ENCODER_LEFT_DIR_PIN )

//250
#define MAX_SPEED 150
#define MIN_SPEED 65


#define STRAIGHT_SPEED      120
#define NORMAL_CURVE_SPEED  95
#define U_CURVE_SPEED       95
#define RING_SPEED          95

#define CURVE_MIN_LEVEL     30
#define CURVE_MID_LEVEL     105
#define CURVE_MAX_LEVEL     200

#define SPEED_ACC_STEP      1
#define SPEED_DEC_STEP      6

#define U_DIFF_START        75
#define DIFF_RATIO          22
#define DIFF_MAX            22


//后轮驱动代码
void Motor_Init(void);
void motor_test(void);
void Encoder_GetValue(void);
void Motor_Loop(void);
void Motor_control(pwm_channel_enum wheel,int16 speed);

extern float diff_kp;
//差速
void Motor_different(void);


#endif


