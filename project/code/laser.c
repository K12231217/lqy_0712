#include "Laser.h"

/*
先看路：你得知道赛道在哪，别撞墙。

扫一眼找靶子：大概看看靶子在哪个方向，离中心线多远。

盯着靶子调整方向：把车头对准它。

靠近了再精细瞄准：离得越近，瞄得越准。

开火：距离合适、方向对准时，打开激光打过去。
*/

//小车动态的话 靶子的每行和列都会发生变化 因此我们需要每帧独立


//1.测试编码器当前的速度 当目标速度为直道速度的时候 去测试当前编码器的速度
//2.同理测试U型弯 普通弯道 圆环速度
//3.根据对应的速度去测试不同的激光点亮时间和激光亮起的目标位置

extern uint32 time_1ms;

//后期根据white_min_point去调试
uint8 ring_black_th = 160;  // 黑环灰度上限（<160算黑）
uint8 ring_white_th = 200;  // 白底/内孔下限（>200算白）

uint8 tar_flag = 0; 		//用于是否找到靶
uint8 attack_flag;

uint8 tar_x,tar_y;

uint32 laser_time = 0;

uint8 fired = 0; 			//表示激光打过没有 只打一次

uint32 cooldown_end = 0;
#define COOLDOWN_MS  600   // 600ms内不再触发，防止多次打

uint8 target_confirm_cnt = 0;
#define CONFIRM_FRAMES 3    // 连续3帧找到才认为有效

uint8 tar_laser = 2;			//打靶所用的激光
uint8 table[] = {IO_P44,IO_PA6,IO_PA3,IO_PA4,IO_PA1};
void laser_init(void)
{
	uint8 i;
	for(i = 0;i < 5;i++)
	{
		gpio_init(table[i], GPO, 0, GPO_PUSH_PULL);
	}
}
void laser_on(uint8 postition)
{
	gpio_set_level(table[postition], 1);
}

void laser_off(uint8 postition)
{
	gpio_set_level(table[postition],0);
}

//选择哪个激光
//1.粗扫找靶(从近端往远端扫)
void Find_target(void)
{
	uint8 start_row = SEARCH_IMAGE_H - 1;
	uint8 end_row = 10; //或者Stop_row
	uint8 step = 2;	//每次扫线的步长
	int16 L,R;
	uint8 row,i;
	uint8 has_vaild = 0;
	
	int16 L_vaild = 10,R_valid = SEARCH_IMAGE_W - 10;
	uint8 state = 0;
	
	uint8 p;
	
	uint8 l = 0,r = 0; //靶的内环的左右边界(列)
	
//	ring_black_th = white_min_point + 25;
//	ring_white_th = white_min_point - 20;
	
	tar_flag = 0;
	
	for(row = start_row; row > end_row;row -= step)
	{
		state = 0;
		l = 0; r = 0;
		//判断靶是否存在于十字的情况(双边丢线)
		if(left_edge_line[row] == 0 && right_edge_line[row] == SEARCH_IMAGE_W - 1)
		{
			//如果有历史有效就继承
			if(has_vaild)
			{
				L = L_vaild - 5;
				R = R_valid + 5;
				//限制 先不加
				if(L < 0) L = 0;
				if(R >= SEARCH_IMAGE_W) R = SEARCH_IMAGE_W - 1;
			}
			else
			{
				L = 10;
				R = SEARCH_IMAGE_W - 10;
			}
		}
		else
		{
			//赛道边界内缩去找
			L = left_edge_line[row] + 5;
			R = right_edge_line[row] - 5;
			L_vaild = left_edge_line[row];
			R_valid = right_edge_line[row];
			has_vaild = 1;
		}
			
		if(L >= R) continue;
		
		for(i = L; i < R;i++)
		{
			p = mt9v03x_image[row][i];
			if(state == 0 && p < ring_black_th) state = 1;
			else if(state == 1 && p > ring_white_th){state = 2; l = i;}
			else if(state == 2 && p < ring_black_th){state = 3; r = i;break;}
		}
		
		if(state == 3 && l > 0 && r > l && (r - l) > 5 && (r - l) < 80)
		{
            tar_x = (l + r) >> 1;
            tar_y = row;
            tar_flag = 1;
            return;
        }
	}
	
}

uint8 get_tary(uint16 speed)
{
	if(speed >= 135) return 75;
	else if(speed >= 90)return 80;
	else if(speed >= 70)return 82;
	return 90;
}

uint8 get_laser_duration(uint16 speed)
{
    if(speed >= 135) return 15;    // ms
    else if(speed >= 90) return 15;
    else if(speed >= 70) return 15;
    return 15;
}

//打靶
void attack_target(void)
{
	int16 offset;
	uint8 trigg_y;
	uint8 duration;
	uint16 abs_l,abs_r,cur_speed;
	// 冷却期，直接返回
    if(time_1ms < cooldown_end) return;

	abs_l = (encoder_data_l < 0) ? (uint16)(-encoder_data_l) : (uint16)(encoder_data_l);
	abs_r = (encoder_data_r < 0) ? (uint16)(-encoder_data_r) : (uint16)(encoder_data_r);
	cur_speed = (abs_l + abs_r) >> 1;
	
	if(attack_flag)
	{
		duration = get_laser_duration(cur_speed);
		//根据具体赛道去选择打靶时间
		if(time_1ms - laser_time >= duration)
		{
			laser_off(tar_laser);
			attack_flag = 0;
			tar_flag = 0;
			cooldown_end = time_1ms + COOLDOWN_MS;
		}
		return;
	}
	
    if(!tar_flag)
    {
        target_confirm_cnt = 0;   // 丢靶，清零
        return;
    }
    else
    {
        if(target_confirm_cnt < CONFIRM_FRAMES)
        {
            target_confirm_cnt++;
            return;   // 未满3帧，不触发
        }
    }
	
	trigg_y = get_tary(cur_speed);
	
	//判断行 (判断就是tar_y是否离现在的靶太远了 是否打)
	if(tar_y > trigg_y) //这个阈值要根据实际速度调我觉得 tar_y越大越靠近
	{
		
		//选择那个激光打靶 根据offset 偏移图像中心
		offset = (int16)(tar_x - 94);   // 图像中心
		if(offset < -25)      tar_laser = 0;
		else if(offset < -10) tar_laser = 1;
		else if(offset <= 10) tar_laser = 2;
		else if(offset <= 25) tar_laser = 3;
		else                  tar_laser = 4;
		laser_on(tar_laser);
		laser_time = time_1ms;
		attack_flag = 1;
	
	}
	
}
