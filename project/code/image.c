#include "image.h"
#include "Ring.h"

volatile uint8 out_track_flag = 0;   // 1 表示小车已出界，锁定停车

// 权重数组
static const uint8 straight_weight_array[SEARCH_IMAGE_H] = {
	1,1,1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,1,1,
	19,19,19,19,19,19,19,19,19,19,
	19,19,19,19,19,19,19,19,19,19,
	19,19,19,19,19,19,19,19,19,19,
	19,19,19,19,19,19,19,19,19,19,
	6,6,6,6,6,6,6,6,6,6,
	6,6,6,6,6,6,6,6,6,6,
	6,6,6,6,6,6,6,6,6,6,
	19,19,19,19,19,19,19,19,19,19
};
static const uint8 curve_weight_array[SEARCH_IMAGE_H] = {
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
	1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
	10,10,10,10,10,10,10,10,10,10,
    10,10,10,10,10,10,10,10,10,10,
	20,20,20,20,20,20,20,20,20,20,
    20,20,20,20,20,20,20,20,20,20,
    20,20,20,20,20,20,20,20,20,20,
    20,20,20,20,20,20,20,20,20,20
};
static const uint8 ring_weight_array[SEARCH_IMAGE_H] = {
    1,1,1,1,1,1,1,1,1,1,
    2,2,2,2,2,3,3,3,3,3,
    4,4,5,5,6,7,8,9,10,10,
    10,10,10,10,10,10,10,10,10,10,
    13,13,13,13,13,13,13,13,13,13,
    16,16,16,16,16,16,16,16,16,16,
    17,17,17,17,17,17,17,17,17,17,
    18,18,18,18,18,18,18,18,18,18,
    18,18,18,18,18,18,18,18,18,18,
    17,17,17,17,17,17,17,17,17,17,
    13,13,13,13,13,13,13,13,13,13,
    10,10,10,10,10,10,10,10,10,10
};

float center_line;
uint16 vision_curve_level = 0;
uint8 vision_weight_blend = 0;

// 图像备份数组，用于发送或调试时保存当前图像
#if IMAGE_COPY_ENABLE
uint8 far image_copy[MT9V03X_H][MT9V03X_W];
#endif

uint16 get_contrast(uint8 temp1,uint8 temp2)
{
    int16 diff = (int16)temp1 - (int16)temp2;
    return  (uint16)((diff < 0 ? -diff : diff) * 200 / (temp1 + temp2 + 1)); 
}


// 参考点和阈值说明：uint8 不能超过 255，计算时需要先用更大类型承接中间值
// 寻找参考灰度，作为图像二值判断的基础
uint8 reference_point;        // 参考灰度
uint8 white_max_point;        // 白色上阈值
uint8 white_min_point;        // 白色下阈值
void get_reference_point(const uint8 *image)
{
	uint16 i;
    uint8 *p= image+(SEARCH_IMAGE_H-REFRENCE_ROW)*SEARCH_IMAGE_W; // 指向用于统计的起始行
    uint16 temp =0; // 统计像素数量
    uint32 tempSum=0; // 像素灰度累加和
    temp = REFRENCE_ROW * SEARCH_IMAGE_W; // 需要统计的像素总数
	for(i=0;i<temp;i++){
        tempSum+= *(p+i); // 累加像素灰度
	}
    reference_point = (uint8)(tempSum/temp); // 求平均灰度，作为图像参考点
    // 根据参考点计算黑白阈值
	white_max_point = (uint8)func_limit_ab(reference_point*WHITE_MAX_MUL/10,BALACK_POINT,230);
	white_min_point = (uint8)func_limit_ab(reference_point*WHITE_MIN_MUL/10,BALACK_POINT,230);
}



uint8 refenence_col_line[SEARCH_IMAGE_H];   // 参考列线
uint8 reference_contrast_ratio = 32;         // 对比度阈值
uint8 reference_col;                        // 参考列
void Search_reference_col(const uint8 *image)
{
	uint8 col,row,temp1,temp2,i;
    uint8 globe_remote=120,globe_remote_min=120; // 临时记录各列最靠上的边界点
	uint16 contrast=0;
	reference_col = SEARCH_IMAGE_W / 2;

    // 按列扫描，寻找最合适的参考列
	for(col=0;col<SEARCH_IMAGE_W;col+=PIXEL_OFFSET)
	{
        globe_remote=120; // 恢复默认值
		for(row =SEARCH_IMAGE_H-1;row>PIXEL_OFFSET;row-=PIXEL_OFFSET)
		{
            temp1 =*(image + row*SEARCH_IMAGE_W+col);               // 当前像素
            temp2 =*(image + (row-PIXEL_OFFSET) * SEARCH_IMAGE_W +col); // 上方对比像素

            if(temp2 > white_max_point) // 上方仍为白色，继续向上扫描
			{
				continue;
			}
				
            else if(temp1 < white_min_point) // 当前点为黑色，记录边界
			{
				if(globe_remote>row)
					globe_remote=row;
				break;
			}
			
            contrast = get_contrast(temp1,temp2);   // 计算灰度对比度
			
            if(contrast > reference_contrast_ratio || row == STOP_ROW) // 对比度超过阈值，认为找到边界
			{
				if(globe_remote>row)
					globe_remote=row;
			}

		}	
            // 每一列扫描结束后更新全局最小行
			if(globe_remote<globe_remote_min)
			{
				globe_remote_min=globe_remote;
                reference_col= col; // 记录最靠上的边界所在列
			}
				
	}
	
    // 得到参考列后，为每一行填充同一个参考列
    for(i=0 ;i<SEARCH_IMAGE_H;i++) // 将参考列写入整条参考线
		refenence_col_line[i] = reference_col;
	
}



uint8 left_edge_line [SEARCH_IMAGE_H];        // 左边界存储数组
uint8 right_edge_line[SEARCH_IMAGE_H];        // 右边界存储数组
uint8 mid_line[SEARCH_IMAGE_H];               // 中线数组
uint8 lost_left,lost_right;                   // 左右边界丢失行记录
void Search_line(const uint8 *image)
{
    const uint8 *p = image;                   // 图像指针
    uint8 row_max = SEARCH_IMAGE_H -1;         // 最大扫描行
	uint8 row_min = STOP_ROW;
	uint8 col_max = SEARCH_IMAGE_W - 1;
	uint8 col_min = 0;
	
    uint8 left_start_col  = reference_col;     // 左边界搜索起始列
    uint8 right_start_col = reference_col+10;  // 右边界搜索起始列，右侧适当偏移
    uint8 left_end_col   = col_min;            // 左边界搜索终止列
    uint8 right_end_col  = SEARCH_IMAGE_W-1;   // 右边界搜索终止列

    uint8 search_time;                         // 搜索次数，用于边界丢失后重搜
    uint8 temp1,temp2;                         // 临时保存相邻像素灰度
    int16 contrast;                            // 对比度
	
    uint8 left_stop=0,right_stop=0,stop_point=0; // 左右停止标志和填充点
	uint8 col,row;
	
	lost_left=STOP_ROW;
    lost_right=STOP_ROW; // 边界丢失行默认值
	for(row = row_max;row >= row_min;row -- )
	{
		left_edge_line[row]  = 0;
		right_edge_line[row] = SEARCH_IMAGE_W-1;
	}
	//o_O("1.for");
	
    // 从下往上逐行扫描左右边界
	for(row = row_max;  row >= row_min; row-=PIXEL_OFFSET)
	{
        p=image + row * SEARCH_IMAGE_W; // 当前行起始指针
        if(!left_stop) // 左边界未停止搜索
		{
			search_time =2;
			do
			{
				if(search_time == 1)
				{
					left_start_col  = reference_col;
					left_end_col    = col_min;
				}
				search_time--;
                // 从参考列向左搜索边界
				for(col = left_start_col;col > left_end_col+PIXEL_OFFSET;col-=PIXEL_OFFSET)
				{
                    temp1 = *( p + col );              // 当前像素
                    temp2 = *( p + col-PIXEL_OFFSET);  // 左侧相邻像素

                    // 起始位置已经是黑色时，认为左边界丢失并填充到图像边缘
					if(temp1 < white_min_point && col == left_start_col && left_start_col == reference_col)
					{
						left_stop =1;
                        search_time=0; // 停止本行搜索
                        // 用左边缘填充后续行
						for(stop_point = row;stop_point>1;stop_point--)
						{
                            left_edge_line[stop_point] = col_min; // 左边界贴边
						}
                        lost_left=row;  // 记录左边界丢失行
						break;
					}
                    // 当前像素为黑色，直接作为左边界
					if(temp1 < white_min_point)
					{						
						left_edge_line[row]=col;						
						break;
					}
                    // 左侧仍为白色，继续搜索
					if(temp2 > white_max_point)
					{
						continue;
					}
                    // 计算对比度
                    contrast = get_contrast(temp1,temp2);   // 灰度对比度
					
					if(contrast > reference_contrast_ratio || col == col_min)
					{
						left_edge_line[row]=col;
						left_start_col = (uint8)func_limit_ab(col+SEARCH_RANGE,col,col_max);
                        left_end_col   = (uint8)func_limit_ab(col-SEARCH_RANGE,col_min,col); // 更新下一行搜索范围
						search_time =0;
						break;
					}
				}
			}while(search_time);
		}
		
        if(!right_stop) // 右边界未停止搜索
		{
			search_time =2;
			do
			{
				if(search_time == 1)
				{
					right_start_col  = reference_col;
					right_end_col    = col_max;
				}
				search_time--;
                // 从参考列向右搜索边界
				for(col = right_start_col;col <=right_end_col-PIXEL_OFFSET;col+=PIXEL_OFFSET)
				{
                    temp1 = *( p + col );              // 当前像素
                    temp2 = *( p + col + PIXEL_OFFSET); // 右侧相邻像素

                    // 起始位置已经是黑色时，认为右边界丢失并填充到图像边缘
					if(temp1 < white_min_point && col == right_start_col && right_start_col == reference_col)
					{
						right_stop =1;
                        search_time=0; // 停止本行搜索
                        // 用右边缘填充后续行
						for(stop_point=row;stop_point>row_min;stop_point--)
						{
                            right_edge_line[stop_point] = SEARCH_IMAGE_W-1; // 右边界贴边
						}
                        lost_right=row; // 记录右边界丢失行
						break;
					}
                    // 当前像素为黑色，直接作为右边界
					if(temp1 < white_min_point)
					{

						right_edge_line[row]=col;
						break;
					}
                    // 右侧仍为白色，继续搜索
					if(temp2 > white_max_point)
					{
						continue;
					}
                    // 计算对比度
					contrast = get_contrast(temp1,temp2);
					if(contrast > reference_contrast_ratio || col >= col_max - PIXEL_OFFSET)
					{
						
						right_edge_line[row]=col;
						right_start_col = (uint8)func_limit_ab(col-SEARCH_RANGE,col_min,col);
                        right_end_col   = (uint8)func_limit_ab(col+SEARCH_RANGE,col,col_max); // 更新下一行搜索范围
						search_time =0;
						break;
					}
				}
			}while(search_time);
		}		
	}
	
    // 对边界中间缺失的行进行插值
	insert_val();
    // 将原始边界复制到控制边界
	memcpy(left_control_line, left_edge_line, sizeof(left_edge_line)); 
	memcpy(right_control_line,right_edge_line, sizeof(right_edge_line)); 
}

// 边界插值
void insert_val(void)
{
	uint8 i,j;
    uint8 front_p,cur_p; // 上一采样点和当前采样点
	uint8 step;
    // 左边界插值
	front_p=left_edge_line[SEARCH_IMAGE_H-1];
	for(i = SEARCH_IMAGE_H-PIXEL_OFFSET-1;i>= STOP_ROW;i-=PIXEL_OFFSET){
		cur_p=left_edge_line[i];
		if(cur_p >= front_p){
            step=(cur_p-front_p+PIXEL_OFFSET-1)/PIXEL_OFFSET; // 计算插值步长
			for(j=1;j<PIXEL_OFFSET;j++){
				left_edge_line[i-j+PIXEL_OFFSET]=front_p+step*j;
			}				
		}
		else{
			step=(front_p-cur_p+PIXEL_OFFSET-1)/PIXEL_OFFSET;
			for(j=1;j<PIXEL_OFFSET;j++){
				left_edge_line[i-j+PIXEL_OFFSET]=front_p-step*j;
			}
		}
        front_p=cur_p; // 更新上一点
	}
	
    // 右边界插值
	front_p=right_edge_line[SEARCH_IMAGE_H-1];
	for(i = SEARCH_IMAGE_H-PIXEL_OFFSET-1;i>= STOP_ROW;i-=PIXEL_OFFSET){
		cur_p=right_edge_line[i];
		if(cur_p >= front_p){
			step=(cur_p-front_p+PIXEL_OFFSET-1)/PIXEL_OFFSET;
			for(j=1;j<PIXEL_OFFSET;j++){
				right_edge_line[i-j+PIXEL_OFFSET]=front_p+j*step;
			}				
		}
		else{
			step=(front_p-cur_p+PIXEL_OFFSET-1)/PIXEL_OFFSET;
			for(j=1;j<PIXEL_OFFSET;j++){
				right_edge_line[i-j+PIXEL_OFFSET]=front_p-j*step;
			}
		}
		front_p=cur_p;
	}
}

// int16 绝对值函数
static int16 abs_i16_image(int16 x)
{
    return x < 0 ? -x : x;
}

static uint8 clamp_blend_u16(uint16 x)
{
    if(x > 100)
    {
        return 100;
    }
    return (uint8)x;
}

static uint8 get_control_center(uint8 row)
{
    return (uint8)(((uint16)left_control_line[row] + (uint16)right_control_line[row]) >> 1);
}

static int16 avg_control_center(uint8 row_start, uint8 row_end)
{
    uint8 row;
    uint16 sum = 0;
    uint8 cnt = 0;

    if(row_end >= SEARCH_IMAGE_H)
    {
        row_end = SEARCH_IMAGE_H - 1;
    }

    for(row = row_start; row <= row_end; row++)
    {
        sum += get_control_center(row);
        cnt++;
    }

    if(cnt == 0)
    {
        return Mid_Col;
    }

    return (int16)(sum / cnt);
}

static uint8 map_curve_level_to_blend(uint16 level)
{
	// if(level <= 30)
	// {
	// 	return 0;
	// }

	// if(level >= 80)
	// {
	// 	return 100;
	// }

	// return clamp_blend_u16((uint16)((level - 30) * 100 / (80 - 30)));

    // if(level <= 45)
    // {
    //     return 0;
    // }

    // if(level >= 100)
    // {
    //     return 100;
    // }

    // return clamp_blend_u16((uint16)((level - 45) * 100 / (100 - 45)));

	if(level <= 60)
	{
		return 0;
	}

	if(level >= 120)
	{
		return 100;
	}

	return clamp_blend_u16((uint16)((level - 60) * 100 / (120 - 60)));
}

static void update_vision_weight_blend(void)
{
    static float vision_curve_filter = 0.0f;

    int16 far_mid;
    int16 mid_mid;
    int16 near_mid;
    int16 e_far;
    int16 e_mid;
    int16 e_near;
    int16 bend;
    uint16 raw_level;

    if(current_step > 0 && current_step <= 10)
    {
        vision_curve_filter = 0.0f;
        vision_curve_level = 0;
        vision_weight_blend = 0;
        return;
    }

    far_mid = avg_control_center(30, 45);
    mid_mid = avg_control_center(65, 85);
    near_mid = avg_control_center(90, 110);

    e_far = far_mid - Mid_Col;
    e_mid = mid_mid - Mid_Col;
    e_near = near_mid - Mid_Col;
    bend = e_near - e_far;

    raw_level = (uint16)(abs_i16_image(e_mid) * 2 + abs_i16_image(bend) * 2);

    vision_curve_filter = vision_curve_filter * 0.75f + (float)raw_level * 0.25f;
    vision_curve_level = (uint16)vision_curve_filter;
    vision_weight_blend = map_curve_level_to_blend(vision_curve_level);
}

static uint8 get_dynamic_weight(uint8 row)
{
    uint16 straight_part;
    uint16 curve_part;
    uint16 weight;

    if(current_step > 0 && current_step <= 10)
    {
        return ring_weight_array[row];
    }

    straight_part = (uint16)straight_weight_array[row] * (uint16)(100 - vision_weight_blend);
    curve_part = (uint16)curve_weight_array[row] * (uint16)vision_weight_blend;
    weight = (straight_part + curve_part + 50) / 100;

    if(weight == 0)
    {
        weight = 1;
    }

    return (uint8)weight;
}
void Fitted_Midline(void)
{
    uint8 i;
    uint8 row_weight;
    uint32 weight_sum = 0;
    float center;
    float sum = 0.0f;

    update_vision_weight_blend();

    for(i = 0; i < SEARCH_IMAGE_H; i++)
    {
        center = (float)get_control_center(i);
        row_weight = get_dynamic_weight(i);

        sum += (center - (float)Mid_Col) * (float)row_weight;
        weight_sum += row_weight;
        mid_line[i] = (uint8)center;
    }

    if(weight_sum == 0)
    {
        center_line = 0.0f;
    }
    else
    {
        center_line = sum / (float)weight_sum;
    }
}

int32 err_sum=0;
void Error_sum(void)
{
    uint8 i;
    int32 sum1 = 0, sum2 = 0, sum3 = 0;
    uint8 cnt1 = 0, cnt2 = 0, cnt3 = 0;

    for(i = 8; i < 28; i++) {
        sum1 += mid_line[i];
        cnt1++;
    }

    for(i = 28; i < 89; i++) {
        sum2 += mid_line[i];
        cnt2++;
    }

    for(i = 89; i < 111; i++) {
        sum3 += mid_line[i];
        cnt3++;
    }
	
	//2:5:3
    err_sum = ((sum1 / cnt1) * 20 + (sum2 / cnt2) * 50 + (sum3 / cnt3) * 30) / 100;
}


void straightAccelerate(void)
{
	uint8 isContinue_left,isContinue_right;
	isContinue_left=isContinueLine(left_edge_line);
	isContinue_right=isContinueLine(right_edge_line);

	if((isContinue_left && isContinue_right) )
	{
		Buzzer_On();
		servo_pidf.kp=15;
	}
	else 
	{
		Buzzer_Off();
		servo_pidf.kp=70;
	}
	
	
}

void Out_Check(const uint8* image)
{
    uint16 row, col;
    uint16 black_cnt = 0;
    uint16 total_cnt = 0;
    static uint8 out_cnt = 0;

    if(out_track_flag)
    {
        return;
    }

    for(row = 80; row < SEARCH_IMAGE_H; row += 2)
    {
        for(col = 0; col < SEARCH_IMAGE_W; col += 2)
        {
            total_cnt++;

            if(*(image + row * SEARCH_IMAGE_W + col) < 100)
            {
                black_cnt++;
            }
        }
    }

    if(total_cnt == 0)
    {
        return;
    }

    if((uint32)black_cnt * 100 >= (uint32)total_cnt * 88)
    {
        if(out_cnt < 4)
        {
            out_cnt++;
        }
    }
    else
    {
        out_cnt = 0;
    }

    if(out_cnt >= 4)
    {
        out_track_flag = 1;
    }
}
