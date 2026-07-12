 /*********************************************************************************************************************
* STC32G144K Opensource Library（STC32G144K 开源库），基于官方 SDK 接口的智能车工程
* 修改记录                               代码仍在调试
* 日期              作者                   备注
* 2026-3-24      项目成员                fengge
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "imu660.h"
// **************************** 全局变量 ****************************
uint8 ControlFlag=1;  // Race mode: image processing and 5 ms motor control
uint32 time_1ms=0;
char display_buf[32];
void main(void)
{

	System_Init();
//	laser_on(0);laser_on(1);laser_on(2);laser_on(3);laser_on(4);
//	gpio_set_level(IO_PA0,1);
	
    while(1)
    {

//		printf("%d,%d,%d\r\n", tar_speed, encoder_data_l, encoder_data_r);
//     system_delay_ms(20);
			
//        printf("encoder_data_r counter_r %d .\r\n", encoder_data_r);      // 右编码器调试输出
	    //motor_test();
		//tft180_show_string(0,64,"VBAT:");
		//tft180_show_float(32,64,Vbat*0.008826,4,6);
				
		//test_ss();	// WIFI SPI 接收调试信息	
		//tft_image_tset();
		
		//Key_Anlysis();//按键处理
		
		//Out_servo=seekfree_assistant_parameter[0];

		//printf("%d, %d\r\n",imu660ra_gyro_z,encoder_data_l);
		//printf("%d\r\n",imu660ra_gyro_z);
//		tar_speed = seekfree_assistant_parameter[0];
//		pid_lf.kp=seekfree_assistant_parameter[1];
//		pid_lf.ki=seekfree_assistant_parameter[2];
//		pid_lf.kd=seekfree_assistant_parameter[3];		
		//seekfree_assistant_data_analysis();
		
		//图像处理
		//每帧图像完成后执行一次赛道识别与误差计算，控制计算在 pit_handler 中断中以 5ms 周期执行
		if(mt9v03x_finish_flag){
			//memcpy(image_copy[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
			get_reference_point(mt9v03x_image[0]);
			Out_Check(mt9v03x_image[0]);
			if(!out_track_flag)
			{
				Search_reference_col(mt9v03x_image[0]);
				Search_line(mt9v03x_image[0]);
				Ring();
				Fitted_Midline();
				Error_sum();
				Find_target();
				attack_target();
			}
			if(!ControlFlag){
				seekfree_assistant_camera_send();	
			}
				
			mt9v03x_finish_flag=0;
		}
		
//		if(mt9v03x_finish_flag){
//			memcpy(base_image[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
//			Threshold=otsuThreshold(base_image[0]);
//			base_image_threshold(Threshold);
//			
//			if(!ControlFlag){
//				seekfree_assistant_camera_send();	
//			}
//			
//			mt9v03x_finish_flag=0;
//		}
		


//		sprintf(display_buf, "s:%d k3:%d d:%d p:%d", key_anlysis1, key_anlysis3, current_ring_dir, ring_pre_repair_flag);
//		ips200_show_string(0, 120, display_buf);

//		sprintf(display_buf, "yaw:%.1f", imu660_get_yaw());
//		ips200_show_string(0, 140, display_buf);		
//         sprintf(display_buf, "vcl:%u wb:%u", vision_curve_level, vision_weight_blend);
//         ips200_show_string(0, 156, display_buf);
//			sprintf(display_buf, "step:%d k2:%d", key_anlysis1, key_anlysis2);
//			ips200_show_string(0, 120, display_buf);
//		
//			sprintf(display_buf, "yaw: %.1f", imu660_get_yaw());
//			ips200_show_string(0, 140, display_buf);
		///laser_off();
		
				//laser_on();
				//image_send_seekfree();//向逐飞助手发送图像
//        {
//            mt9v03x_finish_flag = 0;
//			//iiiii=otsu_fast((const uint8 *)mt9v03x_image);
//			get_reference_point(mt9v03x_image[0]);
//			Search_reference_col(mt9v03x_image[0]);
//			Search_line(mt9v03x_image[0]);
//			Ring();
//			Fitted_Midline();
//			
//			
//			//Pre_Scan();
//			Target_find(pre_find_offset);
//			ips200_show_gray_image(0,0,(const uint8*)mt9v03x_image,188,120,188,120,tar_th);
//			
//			ips200_show_string(0,120,"Find_offset:");
//			ips200_show_int8(90,120,pre_find_offset);
//			
//			
//			ips200_show_string(0,136,"L R T U:");
//			ips200_show_uint8(70,136,l_d);
//			ips200_show_uint8(102,136,r_d);
//			ips200_show_uint8(134,136,top_d);
//			ips200_show_uint8(166,136,under_d);

//			ips200_show_string(0,152,"Th S:");
//			ips200_show_uint8(48,152,tar_th);
//			ips200_show_uint8(96,152,debug_stage);

//			ips200_show_string(0,168,"X  Y:");
//			ips200_show_uint8(50,168,tar_x);
//			ips200_show_uint8(82,168,tar_y);


//			if(tar_flag)
//			{
//				isp_200_draw_col(tar_x,120,0,RGB565_BLUE);
//				isp_200_draw_row(tar_y,188,0,RGB565_BLUE);
//			}
//		}
				//gpio_set_level(IO_P65, 1);			
				//image_send_seekfree();			
				//motor_test();							

		
//				system_delay(1000);
				
		
//				if(ring_l)			
//					Buzzer_On();			
//				else 					
//					Buzzer_Off();				

    }
}

//
void pit_handler(void)
{
	time_1ms++;
	if(time_1ms % 5 == 0)
	{
		Buzzer_Task5ms();

		if(ControlFlag){
			imu660ra_get_gyro();
			imu660_yaw_update_5ms();
			Encoder_GetValue();
			
			Motor_Loop();
		}
		Servo_Loop();
	}
}
