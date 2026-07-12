#include "Motor.h"
#include "Ring.h"

#define MAX_DUTY            ( 20 )                             
int8 duty = 0;
int8 dir = 1;

#define ENCODER_DIR_1                 (PWMA_ENCODER)              // 带方向编码器对应使用的编码器接口 
#define ENCODER_DIR_PULSE_1           (PWMA_ENCODER_CH1P_P60)     // PULSE 对应的引脚
#define ENCODER_DIR_DIR_1             (PWMA_ENCODER_CH2P_P62)     // DIR 对应的引脚

#define ENCODER_DIR_2                 	(PWMC_ENCODER)              // 带方向编码器对应使用的编码器接口
#define ENCODER_DIR_PULSE_2       		(PWMC_ENCODER_CH1P_P40)     // PULSE 对应的引脚
#define ENCODER_DIR_DIR_2           	(PWMC_ENCODER_CH2P_P42)     // DIR 对应的引脚

int16 encoder_data_r=0;
int16 encoder_data_l=0;

void Motor_Init(void)
{
	/* Keep both drivers asleep until PH and PWM are configured. */
	gpio_init(MOTOR_LEFT_NSLEEP, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(MOTOR_RIGHT_NSLEEP, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(MOTOR_LEFT_PH, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(MOTOR_RIGHT_PH, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(PWM_L, MOTOR_PWM_FREQ, 0);
    pwm_init(PWM_R, MOTOR_PWM_FREQ, 0);
    gpio_set_level(MOTOR_LEFT_NSLEEP, GPIO_HIGH);
    gpio_set_level(MOTOR_RIGHT_NSLEEP, GPIO_HIGH);
}


//
//左轮正转1秒反转1秒蜂鸣器鸣叫，右轮正转1秒反转1秒蜂鸣器鸣叫
#define M_TEST  1000
void motor_test(void)
{
		Motor_control(PWM_L, M_TEST);
		system_delay_ms(1000);
		Motor_control(PWM_L, -M_TEST);
		system_delay_ms(1000);
		
		Motor_control(PWM_L, 0);
		gpio_set_level(IO_P65, 1);system_delay_ms(200);
		gpio_set_level(IO_P65, 0);
		
		Motor_control(PWM_R, M_TEST);
		system_delay_ms(1000);
		Motor_control(PWM_R, -M_TEST);
		system_delay_ms(1000);
		Motor_control(PWM_R, 0);
		gpio_set_level(IO_P65, 1);system_delay_ms(200);
		gpio_set_level(IO_P65, 0);

}
void Motor_control(pwm_channel_enum wheel,int16 speed)
{
	int16 command = speed;
	pwm_channel_enum en_pwm;
	gpio_pin_enum ph_pin;

	if(wheel == PWM_L)
	{
		command = (int16)(command * MOTOR_LEFT_COMMAND_SIGN);
		en_pwm = PWM_L;
		ph_pin = MOTOR_LEFT_PH;
	}
	else
	{
		command = (int16)(command * MOTOR_RIGHT_COMMAND_SIGN);
		en_pwm = PWM_R;
		ph_pin = MOTOR_RIGHT_PH;
	}

	if(command >= 0)
	{
		gpio_set_level(ph_pin, GPIO_HIGH);
		pwm_set_duty(en_pwm, command);
	}
	else
	{
		gpio_set_level(ph_pin, GPIO_LOW);
		pwm_set_duty(en_pwm, -command);
	}
}

void Encoder_GetValue(void)
{
    encoder_data_r = encoder_get_count(ENCODER_DIR_1);                  // 获取编码器计数
    encoder_data_l = -encoder_get_count(ENCODER_DIR_2);
	
	
    encoder_clear_count(ENCODER_DIR_1);                                		// 清空编码器计数
    encoder_clear_count(ENCODER_DIR_2);  
}

int16 tar_speed=STRAIGHT_SPEED;
int16 diff_left,diff_right;
float diff_kp=-0.0;//0.30.25
void Dream_speed(void)
{						
    uint16 tp_turn;
	int8 delta = (int8)(err_sum - Mid_Col);
    // 计算舵机偏离中值的程度
    if (Out_servo >  SERVO_CENTER)//左拐
    {
        tp_turn = Out_servo - SERVO_CENTER;
		diff_left  = (int16)(- delta * diff_kp);
		diff_right = (int16)( 0.76f *  delta * diff_kp);
    }
    else			//右拐
    {
        tp_turn = SERVO_CENTER - Out_servo;
		delta = -delta;   // 转为正偏差量 (Mid_Col - err_sum)
		diff_left  = (int16)(   0.76f* delta * diff_kp);
		diff_right = (int16)(-  delta * diff_kp);
    }

    // 限幅，防止超过最大转向量
    if (tp_turn > MAX_TURN)
    {
        tp_turn = MAX_TURN;
    }

    // 直道：tp_turn = 0        → tar_speed = MAX_SPEED
    // 弯道：tp_turn 越大      → tar_speed 越低
    // 最大转向：tp_turn=MAX_TURN → tar_speed = MIN_SPEED
    tar_speed = MAX_SPEED -
                (uint32)(MAX_SPEED - MIN_SPEED) * tp_turn / MAX_TURN;
}

int16 tar_speed_l = STRAIGHT_SPEED;
int16 tar_speed_r = STRAIGHT_SPEED;

static int16 abs_i16(int16 x)
{
    return x < 0 ? -x : x;
}

static int16 limit_i16(int16 x, int16 min_v, int16 max_v)
{
    if(x < min_v) return min_v;
    if(x > max_v) return max_v;
    return x;
}

static int16 ramp_i16(int16 now, int16 target, int16 acc_step, int16 dec_step)
{
    if(target > now)
    {
        if(target - now > acc_step) return now + acc_step;
        else return target;
    }
    else
    {
        if(now - target > dec_step) return now - dec_step;
        else return target;
    }
}

static int16 calc_target_speed(uint16 curve_level)
{
    int16 target;

    if(curve_level <= CURVE_MIN_LEVEL)
    {
        target = STRAIGHT_SPEED;
    }
    else if(curve_level <= CURVE_MID_LEVEL)
    {
        // 普通弯：130 -> 118，速度基本不怎么掉
        target = STRAIGHT_SPEED -
            (int32)(STRAIGHT_SPEED - NORMAL_CURVE_SPEED) *
            (curve_level - CURVE_MIN_LEVEL) /
            (CURVE_MID_LEVEL - CURVE_MIN_LEVEL);
    }
    else if(curve_level <= CURVE_MAX_LEVEL)
    {
        // 大弯/U弯：118 -> 78
        target = NORMAL_CURVE_SPEED -
            (int32)(NORMAL_CURVE_SPEED - U_CURVE_SPEED) *
            (curve_level - CURVE_MID_LEVEL) /
            (CURVE_MAX_LEVEL - CURVE_MID_LEVEL);
    }
    else
    {
        target = U_CURVE_SPEED;
    }

    return target;
}
void Smart_Dream_speed(void)
{
    static float curve_filter = 0.0f;

    int16 e_far, e_mid, e_near;
    int16 bend;
    uint16 turn_abs;
    uint16 curve_level_raw;
    uint16 curve_level;
    int16 target_speed;

    e_far  = (int16)mid_line[30]  - Mid_Col;
    e_mid  = (int16)mid_line[80]  - Mid_Col;
    e_near = (int16)mid_line[100] - Mid_Col;

    bend = e_near - e_far;

    if(Out_servo > SERVO_CENTER)
        turn_abs = Out_servo - SERVO_CENTER;
    else
        turn_abs = SERVO_CENTER - Out_servo;

    if(turn_abs > MAX_TURN)
        turn_abs = MAX_TURN;

    curve_level_raw = abs_i16(e_mid) * 2 + abs_i16(bend) * 2 + turn_abs / 12;

    // 关键：弯道强度滤波，防止速度一会快一会慢
    curve_filter = curve_filter * 0.75f + curve_level_raw * 0.25f;
    curve_level = (uint16)curve_filter;

    if(current_step > 0 && current_step < 11)
    {
        target_speed = RING_SPEED;
    }
    else
    {
        target_speed = calc_target_speed(curve_level);
    }

	
    tar_speed = ramp_i16(tar_speed, target_speed, SPEED_ACC_STEP, SPEED_DEC_STEP);
	
	
    if(curve_level > U_DIFF_START)
	{
		int16 diff;
		int16 turn_signed;

		turn_signed = (int16)Out_servo - SERVO_CENTER;

		diff = (int16)((int32)tar_speed * DIFF_RATIO * turn_abs / (100 * MAX_TURN));
		diff = limit_i16(diff, 0, DIFF_MAX);

		if(turn_signed > 0)  // 左弯
		{
			tar_speed_l = tar_speed - diff;
			tar_speed_r = tar_speed + diff;
		}
		else if(turn_signed < 0)  // 右弯
		{
			tar_speed_l = tar_speed + diff;
			tar_speed_r = tar_speed - diff;
		}
		else
		{
			tar_speed_l = tar_speed;
			tar_speed_r = tar_speed;
		}
	}
	else
	{
		// 普通弯不开差速，避免贴内
		tar_speed_l = tar_speed;
		tar_speed_r = tar_speed;
	}

	tar_speed_l = limit_i16(tar_speed_l, 50, STRAIGHT_SPEED + 10);
	tar_speed_r = limit_i16(tar_speed_r, 50, STRAIGHT_SPEED + 10);
}

void Motor_Loop(void)
{
    if(out_track_flag)
    {
		tar_speed = 0;
		tar_speed_l = 0;
		tar_speed_r = 0;

		pid_lf.out = 0;
		pid_rf.out = 0;
		
		pwm_set_duty(PWM_L, 0);
		pwm_set_duty(PWM_R, 0);
        return;
    }

    Smart_Dream_speed();

    Increment_PIDf(&pid_lf,(float)tar_speed_l,encoder_data_l);
    Increment_PIDf(&pid_rf,(float)tar_speed_r,encoder_data_r);
    
    Motor_control(PWM_L,(int16)pid_lf.out);
    Motor_control(PWM_R,(int16)pid_rf.out);	
}

