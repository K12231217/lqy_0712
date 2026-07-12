#ifndef __DISPLAY_H_
#define __DISPLAY_H_
#include "zf_common_headfile.h"

#define DISPLAY_IPS200PRO_DEMO_ENABLE 0

#if DISPLAY_IPS200PRO_DEMO_ENABLE
extern uint16 camera_id;

extern ips200pro_image_line_uint8_struct border1[MT9V03X_H];
extern ips200pro_image_line_uint8_struct border2[MT9V03X_H];
extern uint16 rectangle_x ;
extern uint8  temp_start;
void display_init_pro(void);
#endif
void display_init(void);
void isp_200_draw_col(uint8 col ,uint8 h,uint8 start,uint16 color);
void isp_200_draw_row(uint8 col ,uint8 w,uint8 start,uint16 color);
void display_motor(void);



#endif



