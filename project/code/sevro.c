#include "sevro.h"
//舵机中值87.5°   最右最左12.5°
void servo_setAngle(pwm_channel_enum sever,float angle)
{
	if(angle>12.5|| angle<-12.5)return;
	angle=87.5-angle;
	pwm_set_duty(sever,SERVO_DUTY(angle));
}


void servo_init(void)
{
	pwm_init(SERVO_PWM1,SERVO_FREQ,4650);
	pwm_set_duty(SERVO_PWM1,SERVO_DUTY_MID);//
}

volatile uint16 huQiqiangServo(volatile uint16 input)
{
	//4450
	if((input<4680) && (input>5630))
	{
		return (volatile uint16)4760;
	}
	return input;
	
}


void Servo_Loop(void)
{
	
    PID_servof(&servo_pidf);
	//Out_servo=huQiqiangServo(Out_servo);
    pwm_set_duty(SERVO_PWM1, (uint32)Out_servo);
}

