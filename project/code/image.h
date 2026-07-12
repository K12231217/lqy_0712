#ifndef __IMAGE_H_
#define __IMAGE_H_
#include "zf_common_headfile.h"
#define SEARCH_IMAGE_W  	188  //搜线宽度
#define SEARCH_IMAGE_H  	120  //搜线高度
#define BALACK_POINT     	100  //黑点值
#define WHITE_MAX_MUL        13  //白点最大值
#define WHITE_MIN_MUL      	  7  //白点最小值

#define REFRENCE_ROW    	  4   //参考行数
#define SEARCH_RANGE    	  16  //搜索半径
#define STOP_ROW         	  8   //搜线停止行
#define PIXEL_OFFSET          4   //偏移量

#define ROAD_WIDTH_EST  96
#define IMAGE_COPY_ENABLE 0

extern uint8 reference_point;		//参考点
extern uint8 white_max_point;		//白点最大
extern uint8 white_min_point;		//白点最小

extern uint8 refenence_col_line[SEARCH_IMAGE_H]; 	//图传
extern uint8 reference_contrast_ratio;         		//对比度阈值
extern uint8 reference_col;					     	//参考列
extern uint8 left_edge_line[SEARCH_IMAGE_H];		//左边线存储数组
extern uint8 right_edge_line[SEARCH_IMAGE_H];		//右边线存储数组
extern uint8 mid_line[SEARCH_IMAGE_H];				//中线数组
extern uint8 left_control_line[SEARCH_IMAGE_H];		//左操作数组
extern uint8 right_control_line[SEARCH_IMAGE_H];	//右操作数组
extern uint8 lost_left,lost_right;


extern uint8 cross_flag;							//十字标志位

extern uint16 encoder_enter;

#if IMAGE_COPY_ENABLE
extern uint8 far image_copy[MT9V03X_H][MT9V03X_W];
#endif
extern uint8 binary_image[MT9V03X_H/2][MT9V03X_W/2];
extern uint8 th;
extern int32 err_sum; 

extern float center_line;
extern uint16 vision_curve_level;
extern uint8 vision_weight_blend;
extern volatile uint8 out_track_flag;

//用tft屏幕进行测试
void tft_image_tset(void);
//对比度搜线算法
void get_reference_point(const uint8 *image);
void Search_reference_col(const uint8 *image);
void Search_line(const uint8 *image);
void Fitted_Midline(void);
void Cross_analysis(void);
void Connect_point(void);
void insert_val(void);//插值函数
void Out_Check(const uint8* image);

void straightAccelerate(void);
void Error_sum(void);


#endif



