#include "Init.h"

void pit_handler (void);

#if INIT_WIFI_COMMAND_DEBUG_ENABLE
uint32 _t=0;
uint8 motor_pit=5;
volatile uint8 wifi_spi_get_data_buffer[256] = {0};
uint8 data_len;
#endif
extern uint8 ControlFlag;
void System_Init(void)
{
	  clock_init(SYSTEM_CLOCK_96M); 				// 时钟配置及系统初始化<务必保留>
    debug_init();                              // 调试串口信息初始化

		//display_init();

        // 蜂鸣器初始化
		gpio_init(IO_P65, GPO, 0, GPO_PUSH_PULL);
	
		//激光初始化
		laser_init();
//		
//      // 陀螺仪初始化
		imu660rb_init();
		
		//编码器初始化
        encoder_dir_init(ENCODER_DIR_1, ENCODER_DIR_PULSE_1, ENCODER_DIR_DIR_1);   // 初始化编码器模块与引脚，带方向增量编码器模式
        encoder_dir_init(ENCODER_DIR_2, ENCODER_DIR_PULSE_2, ENCODER_DIR_DIR_2);   // 初始化编码器模块与引脚，带方向增量编码器模式
	
//    encoder_dir_init(ENCODER_DIR_1, ENCODER_DIR_DIR_1, ENCODER_DIR_PULSE_1);   // 初始化编码器模块与引脚，带方向增量编码器模式
//      encoder_dir_init(ENCODER_DIR_2, ENCODER_DIR_DIR_2, ENCODER_DIR_PULSE_2);   // 初始化编码器模块与引脚，带方向增量编码器模式

        // 按钮初始化
		gpio_init(KEY1_PIN, GPI, 1, GPI_PULL_UP);
		gpio_init(KEY2_PIN, GPI, 1, GPI_PULL_UP);
		gpio_init(KEY3_PIN, GPI, 1, GPI_PULL_UP);
		gpio_init(KEY4_PIN, GPI, 1, GPI_PULL_UP);
	
		gpio_init(SWITCH1_PIN, GPI, 1, GPI_PULL_UP);
		gpio_init(SWITCH2_PIN, GPI, 1, GPI_PULL_UP);
	
        // 舵机初始化
		servo_init();
		
        // 电机与 PID 初始化
		Motor_Init();
		
        // 电池电压检测初始化，接口 P10
//		adc_init(ADC1_CH0_P10,ADC_12BIT);
		

		
//		//初始化摄像头
		while(1)
		{
			if(!mt9v03x_init())
			{
				gpio_set_level(IO_P65, 0);
				break;
			}
			gpio_set_level(IO_P65, 1);
			system_delay_ms(500);                                
		}
		
#if !APP_RACE_MODE
		if(!ControlFlag){
            // WIFI SPI 初始化
			while(wifi_spi_init("TCP", "123456789"))
			{
				gpio_set_level(IO_P65, 1);
                system_delay_ms(100);                                                   // 初始化失败，等待 100ms
			}gpio_set_level(IO_P65, 0);		
			
			
            // 逐飞助手初始化，数据传输使用高速 WIFI SPI
			seekfree_assistant_init();
			seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
			seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
			seekfree_assistant_camera_boundary_config(X_BOUNDARY, MT9V03X_H, left_control_line,right_control_line, mid_line,NULL,NULL,NULL);		
		}
#endif

		
		
////			
//		////按下按键4 || wifi发送才始开
		//Begin();
//		
//      /********************* 定时器初始化 **********************/
        pit_ms_init(TIM_0,1, pit_handler);         // 初始化 PIT_CH0 为周期中断，5ms 周期
		
}

#if INIT_WIFI_COMMAND_DEBUG_ENABLE
static uint8 send_buffer[256] = {0};
int16 just_i,just_p,just_tar,just_angle;
static int16 begin=0;
void test_ss(void)
{
//		sprintf(send_buffer,"Date:%d, %d, %d\n", encoder_data_l,encoder_data_r,tar_speed);
//		wifi_spi_send_buffer(send_buffer,sizeof(send_buffer));
	
	
		data_len=wifi_spi_read_buffer(wifi_spi_get_data_buffer,20);
	
		if (wifi_spi_get_data_buffer[0] == 'I') {
			sscanf(wifi_spi_get_data_buffer, "I:%d", &just_i);
		}
		else if (wifi_spi_get_data_buffer[0] == 'P') {
			sscanf(wifi_spi_get_data_buffer, "P:%d", &just_p); 
		}
		else if(wifi_spi_get_data_buffer[0] == 'T'){
		    sscanf(wifi_spi_get_data_buffer, "T:%u", &just_tar);
				
		}
		else if(wifi_spi_get_data_buffer[0] == 'A'){
				sscanf(wifi_spi_get_data_buffer, "A:%d", &just_angle);
		}
		else if(wifi_spi_get_data_buffer[0] == 'S'){
				sscanf(wifi_spi_get_data_buffer, "S:%d", &tar_speed);
		}	
		else if(wifi_spi_get_data_buffer[0] == '1'){
				key_anlysis1=1;
		}			
		else if(wifi_spi_get_data_buffer[0] == '2'){
				key_anlysis2=1;
		}
		else if(wifi_spi_get_data_buffer[0] == '3'){
				key_anlysis3=1;
		}
		
		
//		   		if(sizeof(wifi_spi_get_data_buffer)!=0){
//					wifi_spi_send_buffer(wifi_spi_get_data_buffer,data_len);
//                  memset(wifi_spi_get_data_buffer, 0,20);           // 数据发送完成，清空数据
//				}
		
		
		
}

#endif

void Buzzero_O(void)
{
	uint8 i;
	for(i=50;i>0;i--)
	{
		gpio_set_level(IO_P65, 1);
		system_delay_ms(i);
		gpio_set_level(IO_P65, 0);
		system_delay_ms(i);
	}
	gpio_set_level(IO_P65, 1);system_delay_ms(200);gpio_set_level(IO_P65,0);
	gpio_set_level(IO_P65, 1);system_delay_ms(200);gpio_set_level(IO_P65,0);system_delay_ms(200);
	gpio_set_level(IO_P65, 1);system_delay_ms(200);gpio_set_level(IO_P65,0);
}

void Buzzer_On(void)
{
	gpio_set_level(IO_P65, 1);
}

void Buzzer_Off(void)
{
	gpio_set_level(IO_P65, 0);
}

#define BUZZER_PULSE_TICKS	6
#define BUZZER_GAP_TICKS	2

static volatile uint8 buzzer_pending_count = 0;
static volatile uint8 buzzer_on_ticks = 0;
static volatile uint8 buzzer_gap_ticks = 0;

void Buzzer_RequestPulse(void)
{
	if(buzzer_pending_count < 20)
	{
		buzzer_pending_count++;
	}
}

void Buzzer_Task5ms(void)
{
	if(buzzer_on_ticks > 0)
	{
		buzzer_on_ticks--;
		if(buzzer_on_ticks == 0)
		{
			Buzzer_Off();
			buzzer_gap_ticks = BUZZER_GAP_TICKS;
		}
		return;
	}

	if(buzzer_gap_ticks > 0)
	{
		buzzer_gap_ticks--;
		return;
	}

	if(buzzer_pending_count > 0)
	{
		buzzer_pending_count--;
		buzzer_on_ticks = BUZZER_PULSE_TICKS;
		Buzzer_On();
	}
}

void o_O(const char dat[])
{
	uint8 str[100];
	Buzzero_O();
	sprintf(str,"%s  Successful!",dat);
	while(1)
	{
		tft180_show_string(0,104,"Test While");
		Key_GetNum();
		if(key4_flag)
		{	
			if(key4_flag)
			{
				key4_flag=0;
				break;
			}
		}
	}
	tft180_show_string(0,104,str);
	gpio_set_level(IO_P65, 1);system_delay_ms(100);gpio_set_level(IO_P65,0);system_delay_ms(100);
	gpio_set_level(IO_P65, 1);system_delay_ms(100);gpio_set_level(IO_P65,0);
	tft180_clear(RGB565_WHITE);
}
static float s=0;
void x_X(void)
{
	if(s>12)
		s=-12;
	s+=0.5;
	servo_setAngle(SERVO_PWM1,s);
}

uint8 k4;
#if INIT_WIFI_COMMAND_DEBUG_ENABLE
void Begin(void)
{
	while(1)
	{
		//Buzzer_On();
		data_len=wifi_spi_read_buffer(wifi_spi_get_data_buffer,20);
		Key_Anlysis();
		if(wifi_spi_get_data_buffer[0] == 'B')
		{
			//Buzzero_O();
			break;
		}
		else if(k4==1)
		{
			k4=0;
			//Buzzero_O();
			break;
		}
	}
}

#endif

void Buzzer_test(uint8 count)
{
	uint8 i;
	for(i=0;i<count;i++){
		gpio_set_level(IO_P65, 1);
		system_delay_ms(200);
		gpio_set_level(IO_P65,0);
		system_delay_ms(200);
	}

}



