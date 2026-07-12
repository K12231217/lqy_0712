#ifndef __Init_H_
#define __INIT_H_
#define INIT_WIFI_COMMAND_DEBUG_ENABLE 0

#include "zf_common_headfile.h"

#if INIT_WIFI_COMMAND_DEBUG_ENABLE
extern uint8 motor_pit;
extern uint32 _t;
extern volatile uint8 wifi_spi_get_data_buffer[256];
extern uint8 data_len;
extern int16 just_i,just_p,just_tar,just_angle;
#endif
extern uint8 k4;
void System_Init(void);

// 系统初始化

// WIFI 命令调试接口
#if INIT_WIFI_COMMAND_DEBUG_ENABLE
void test_ss(void);
// 接收并解析 WIFI 调试数据，等待开始信号
void Begin(void);
#endif
// 蜂鸣器控制
void Buzzer_On(void);
void Buzzer_Off(void);
void Buzzer_RequestPulse(void);
void Buzzer_Task5ms(void);
void Buzzero_O(void);
void Buzzer_test(uint8 count);
// 测试和提示接口
void o_O(const char dat[]);
void x_X(void); // 舵机测试
#endif





