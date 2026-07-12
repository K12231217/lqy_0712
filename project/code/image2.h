#ifndef __IMAGE2_H_
#define __IMAGE2_H_

#include "zf_common_headfile.h"
#define GrayScale 256   //所有像素值
#define OTSU_USE_LIMIT    0   // 1=开启阈值限幅，0=关闭（保留原有逻辑）
#define OTSU_LIMIT_MIN    90  // 限幅下限
#define OTSU_LIMIT_MAX    200 // 限幅上限

#define threshold_max	255*5//此参数可根据自己的需求调节
#define threshold_min	255*2//此参数可根据自己的需求调节

#define IMAGE2_OTSU_ENABLE 0

#if IMAGE2_OTSU_ENABLE
void base_image_threshold(uint8 Threshold);
uint8 otsuThreshold(uint8 *mt9v03x_image_first);

extern uint8 Threshold;
extern uint8 base_image[MT9V03X_H][MT9V03X_W];
#endif

#endif /* CODE_RING_H_ */


