#ifndef __LASER_H_/* CODE_LASER_H_ */
#define __LASER_H_/* CODE_LASER_H_ */
#include "zf_common_headfile.h"

void laser_init(void);
void laser_on(uint8 postition);
void laser_off(uint8 postition);

void Find_target(void);
void attack_target(void);

extern uint8 tar_flag; 
extern uint8 tar_x;
extern uint8 tar_y;

#endif /* CODE_LASER_H_ */

