#include "Ring.h"
#include "Init.h"
#include "imu660.h"

// 这几个角点是你给定的固定补线终点
// 坐标规则// x 从左到右递增，y 从上到下递增
#define RING_LEFT_CORNER_X        2
#define RING_LEFT_CORNER_Y        118
#define RING_RIGHT_CORNER_X       186
#define RING_RIGHT_CORNER_Y       118

// 按图像的大致区域，把“顶中部/底部/底部基点”分开判断
// 这些阈值后面如果图像效果不稳定，可以继续微调
#define RING_TOP_ROW_MIN          STOP_ROW
#define RING_PRE_ENTRY_TOP_ROW_MIN 5
#define RING_TOP_ROW_MAX          44
#define RING_MID_ROW_MIN          48
#define RING_MID_ROW_MAX          76
#define RING_BOTTOM_ROW_MIN       84
#define RING_BASE_ROW_MIN         100

#define LEFT_VALID_MIN_X          2
#define RIGHT_VALID_MAX_X         (SEARCH_IMAGE_W - 3)
#define LEFT_BASE_MIN_X           8
#define RIGHT_BASE_MAX_X          (SEARCH_IMAGE_W - 8)
#define BREAK_DELTA_X             10
#define RING_ENTRY_TREND_MIN_VALID 5
#define RING_ENTRY_TREND_MIN_DELTA 8
#define RING_ENTRY_TOP_STABLE_DELTA 5
#define RING_ENTRY_TOP_BREAK_DELTA  5
#define RING_ENTRY_TOP_MIN_ABOVE    3
#define RING_ENTRY_RIGHT_MIN_VALID 4
#define RING_ENTRY_RIGHT_BOTTOM_RUN 10
#define LEFT_STATE3_BOTTOM_VALID_MIN 1
#define RING_BOTTOM_TREND_MAX_REVERSE 2
#define RING_BOTTOM_TREND_MAX_REVERSE_COUNT 1
#define RIGHT_STATE7_TREND_MIN_VALID 3
#define RIGHT_STATE7_TREND_MIN_RISE 8
#define RIGHT_STATE7_TREND_MIN_FALL 8
#define RIGHT_STATE7_TREND_MIN_FALL_SAMPLES 1
#define RIGHT_STATE7_PEAK_ROW_MIN 60
#define RIGHT_STATE7_BREAK_ROW_MIN 60
#define RIGHT_STATE7_BREAK_MIN_DELTA_X 8
#define RIGHT_STATE7_BREAK_ABOVE_SCAN 16
#define RIGHT_STATE7_BREAK_ABOVE_INVALID_MIN 4
#define RIGHT_STATE7_BREAK_BELOW_SCAN 16
#define RIGHT_STATE7_BREAK_BELOW_VALID_MIN 3
#define RIGHT_STATE6_SCAN_ROWS 70
#define RIGHT_STATE6_SCAN_STEP 4
#define RIGHT_STATE6_PEAK_LOST_CONFIRM 2
#define RIGHT_STATE4_5_BREAK_LOST_CONFIRM 2
#define LEFT_STATE4_5_BREAK_LOST_CONFIRM 2
#define LEFT_STATE5_YAW_MIN_DEG (-80.0f)
#define LEFT_STATE5_YAW_MAX_DEG (-70.0f)
#define LEFT_STATE8_YAW_MIN_DEG (20.0f)
#define LEFT_STATE8_YAW_MAX_DEG (50.0f)
#define RIGHT_STATE5_YAW_MIN_DEG (70.0f)
#define RIGHT_STATE5_YAW_MAX_DEG (80.0f)
#define RIGHT_STATE8_YAW_MIN_DEG (-50.0f)
#define RIGHT_STATE8_YAW_MAX_DEG (-20.0f)
#define RING_UNUSED_DEBUG_HELPERS_ENABLE 0

// left_control_line / right_control_line 是最终参与控制和回传显示的线
// left_edge_line / right_edge_line 是常规搜线得到的原始边界
uint8 left_control_line[SEARCH_IMAGE_H];
uint8 right_control_line[SEARCH_IMAGE_H];

// current_step 是圆环状态机
// 0 表示普通搜线，1~11 对应当前定义的 11 个圆环状态
uint8 current_step = 0;
// 0: no ring, 1: left ring, 2: right ring
uint8 current_ring_dir = 0;
uint8 key_anlysis1 = 0, key_anlysis2 = 0, key_anlysis3 = 0;
// 0: none, 1: left line would repair, 2: right line would repair, 3: both
uint8 ring_pre_repair_flag = 0;

// 状态 9/10 使用左拐点缓存，避免拐点一帧有一帧无导致补线突然撤掉
static uint8 ring_last_left_break_valid = 0;
static uint8 ring_last_left_break_row = 0;
static uint8 ring_last_left_break_col = 0;
static uint8 ring_last_right_break_valid = 0;
static uint8 ring_last_right_break_row = 0;
static uint8 ring_last_right_break_col = 0;
static uint8 right_state6_peak_valid = 0;
static uint8 right_state6_peak_row = 0;
static uint8 right_state6_peak_col = 0;
static uint8 right_state6_peak_lost_cnt = 0;
static uint8 left_state4_5_break_lost_cnt = 0;
static uint8 right_state4_5_break_lost_cnt = 0;
static uint8 cross_ring_suspect_hold_seen = 0;

// 下面这些标志位用于兼容原工程其他代码
uint8 ring_preMeet_flag = 0;
uint8 first_meeting_flag = 0;
uint8 ring_l = 0, ring_r = 0;
uint8 ring_enter_flag = 0;
uint8 ring_turn_flag = 0;
uint8 Out_flag = 0;
uint8 Straighten_flag = 0;
uint8 ring_over_flag = 0;
uint16 cnt_over = 0;

uint8 Ring_ShouldBeepOnStepEnter(uint8 last_step, uint8 current_step_now)
{
    if(last_step == current_step_now)
    {
        return 0;
    }

    if(current_step_now > 11)
    {
        return 0;
    }

    return 1;
}

uint8 Ring_Debug_CanUseCachedLeftBreak(uint8 cache_valid, uint8 cache_row, uint8 cache_col)
{
    return (cache_valid && cache_row < SEARCH_IMAGE_H && cache_col < SEARCH_IMAGE_W);
}

static void ring_clear_left_break_cache(void)
{
    ring_last_left_break_valid = 0;
    ring_last_left_break_row = 0;
    ring_last_left_break_col = 0;
}

static void ring_save_left_break_cache(uint8 break_row, uint8 break_col)
{
    ring_last_left_break_valid = 1;
    ring_last_left_break_row = break_row;
    ring_last_left_break_col = break_col;
}

static void ring_clear_right_break_cache(void)
{
    ring_last_right_break_valid = 0;
    ring_last_right_break_row = 0;
    ring_last_right_break_col = 0;
}

static void ring_save_right_break_cache(uint8 break_row, uint8 break_col)
{
    ring_last_right_break_valid = 1;
    ring_last_right_break_row = break_row;
    ring_last_right_break_col = break_col;
}

static void right_state6_clear_peak_cache(void)
{
    right_state6_peak_valid = 0;
    right_state6_peak_row = 0;
    right_state6_peak_col = 0;
    right_state6_peak_lost_cnt = 0;
}

static void right_state4_5_clear_break_lost_cache(void)
{
    right_state4_5_break_lost_cnt = 0;
}

static void left_state4_5_clear_break_lost_cache(void)
{
    left_state4_5_break_lost_cnt = 0;
}

static uint8 ring_can_use_cached_right_break(void)
{
    return (ring_last_right_break_valid &&
            ring_last_right_break_row < SEARCH_IMAGE_H &&
            ring_last_right_break_col < SEARCH_IMAGE_W);
}

static void ring_set_step(uint8 next_step)
{
    if(Ring_ShouldBeepOnStepEnter(current_step, next_step))
    {
        Buzzer_RequestPulse();
    }

    current_step = next_step;
}

// 限刜列坐标，避免补线旜衑出图像范
static uint8 clamp_col_int16(int16 val)
{
    if(val < 0) return 0;
    if(val >= SEARCH_IMAGE_W) return (SEARCH_IMAGE_W - 1);
    return (uint8)val;
}

static uint8 clamp_row_int16(int16 val)
{
    if(val < 0) return 0;
    if(val >= SEARCH_IMAGE_H) return (SEARCH_IMAGE_H - 1);
    return (uint8)val;
}

// 左螚界有效不是贴在最左螚
static uint8 is_left_valid(uint8 x)
{
    return (x > LEFT_VALID_MIN_X);
}

// 右螚界有效不是贴在最右螚
static uint8 is_right_valid(uint8 x)
{
    return (x < RIGHT_VALID_MAX_X);
}

// 用两个点直接画一条直线，to_flag = 0 写入左控制线
// to_flag = 1 写入右控制线
static void draw_line_by_point(uint8 row0, uint8 col0, uint8 row1, uint8 col1, uint8 to_flag)
{
    uint8 row_start, row_end;
    uint8 col_start, col_end;
    uint8 row;
    float slope;

    row0 = clamp_row_int16(row0);
    row1 = clamp_row_int16(row1);
    col0 = clamp_col_int16(col0);
    col1 = clamp_col_int16(col1);

    if(row0 == row1)
    {
        if(!to_flag)
        {
            left_control_line[row0] = col0;
        }
        else
        {
            right_control_line[row0] = col0;
        }
        return;
    }

    if(row0 < row1)
    {
        row_start = row0;
        row_end = row1;
        col_start = col0;
        col_end = col1;
    }
    else
    {
        row_start = row1;
        row_end = row0;
        col_start = col1;
        col_end = col0;
    }

    slope = (float)((int16)col_end - (int16)col_start) / (float)((int16)row_end - (int16)row_start);

    for(row = row_start; row <= row_end; row++)
    {
        uint8 col_now = clamp_col_int16((int16)((float)col_start + slope * (float)(row - row_start) + 0.5f));
        if(!to_flag)
        {
            left_control_line[row] = col_now;
        }
        else
        {
            right_control_line[row] = col_now;
        }
    }
}

// 这个接口保留给旧代码使用
// 根据两行里已经存在的控制线点，把中间补成一条线
void draw_line(uint8 left_point, uint8 right_point, uint8 to_flag)
{
    if(!to_flag)
    {
        draw_line_by_point(left_point, left_control_line[left_point], right_point, left_control_line[right_point], 0);
    }
    else
    {
        draw_line_by_point(left_point, right_control_line[left_point], right_point, right_control_line[right_point], 1);
    }
}

// connect_point 的作用和 draw_line 类似，只是名字更接近“连接上下两个点”
void connect_point(uint8 under, uint8 top, uint8 l_r)
{
    if(!l_r)
    {
        draw_line_by_point(top, left_control_line[top], under, left_control_line[under], 0);
    }
    else
    {
        draw_line_by_point(top, right_control_line[top], under, right_control_line[under], 1);
    }
}

// 判断一条边线是否整体连续；相邻采样点跳变太大则认为不连续
uint8 isContinueLine(uint8* arr)
{
    int16 i;
    for(i = SEARCH_IMAGE_H - 1 - PIXEL_OFFSET; i > 7 * PIXEL_OFFSET; i -= PIXEL_OFFSET)
    {
        uint8 curr = arr[i];
        uint8 prev = arr[i - PIXEL_OFFSET];
        if(abs((int16)curr - (int16)prev) > 40)
        {
            return 0;
        }
    }
    return 1;
}

// 在左螚界顶部区域里，x 最大的那个// 䚟尹是你说的“顶部能扞到x 轴最大的点
static uint8 find_left_top_max_point(uint8 *row_out, uint8 *col_out)
{
    uint8 row;
    uint8 best_row = 0;
    uint8 best_col = 0;
    uint8 found = 0;

    for(row = RING_TOP_ROW_MIN; row <= RING_TOP_ROW_MAX; row += PIXEL_OFFSET)
    {
        if(is_left_valid(left_edge_line[row]) && (!found || left_edge_line[row] >= best_col))
        {
            best_row = row;
            best_col = left_edge_line[row];
            found = 1;
        }
    }

    if(found)
    {
        *row_out = best_row;
        *col_out = best_col;
    }
    return found;
}

// 在右螚界顶部区域里，x 最小的那个
static uint8 find_right_top_min_point(uint8 *row_out, uint8 *col_out)
{
    uint8 row;
    uint8 best_row = 0;
    uint8 best_col = 0;
    uint8 found = 0;

    for(row = RING_TOP_ROW_MIN; row <= RING_TOP_ROW_MAX; row += PIXEL_OFFSET)
    {
        if(is_right_valid(right_edge_line[row]) && (!found || right_edge_line[row] <= best_col))
        {
            best_row = row;
            best_col = right_edge_line[row];
            found = 1;
        }
    }

    if(found)
    {
        *row_out = best_row;
        *col_out = best_col;
    }
    return found;
}

// 预入环保护允许从更靠上的行开始找，但只采纳有效左边线点
#if RING_UNUSED_DEBUG_HELPERS_ENABLE
static uint8 find_left_pre_entry_top_point(uint8 *row_out, uint8 *col_out)
{
    uint8 row;
    uint8 best_row = 0;
    uint8 best_col = 0;
    uint8 found = 0;

    for(row = RING_PRE_ENTRY_TOP_ROW_MIN; row <= RING_TOP_ROW_MAX; row++)
    {
        if(is_left_valid(left_edge_line[row]) && (!found || left_edge_line[row] >= best_col))
        {
            best_row = row;
            best_col = left_edge_line[row];
            found = 1;
        }
    }

    if(found)
    {
        *row_out = best_row;
        *col_out = best_col;
    }
    return found;
}

// 仅用于让状态判断更直观
#endif

static uint8 left_top_exist(void)
{
    uint8 row, col;
    return find_left_top_max_point(&row, &col);
}

static uint8 right_top_exist(void)
{
    uint8 row, col;
    return find_right_top_min_point(&row, &col);
}

// 在左螚界底部区域里，x 最大的那个// 䚟尹是你说的“底部能扞到x 轴最大的点
static uint8 find_left_bottom_max_point(uint8 *row_out, uint8 *col_out)
{
    uint8 row;
    uint8 best_row = 0;
    uint8 best_col = 0;
    uint8 found = 0;

    for(row = RING_BOTTOM_ROW_MIN; row < SEARCH_IMAGE_H; row += PIXEL_OFFSET)
    {
        if(is_left_valid(left_edge_line[row]) && (!found || left_edge_line[row] >= best_col))
        {
            best_row = row;
            best_col = left_edge_line[row];
            found = 1;
        }
    }

    if(found)
    {
        *row_out = best_row;
        *col_out = best_col;
    }
    return found;
}

// 在右螚界底部区域里，x 最小的那个
static uint8 find_right_bottom_min_point(uint8 *row_out, uint8 *col_out)
{
    uint8 row;
    uint8 best_row = 0;
    uint8 best_col = 0;
    uint8 found = 0;

    for(row = RING_BOTTOM_ROW_MIN; row < SEARCH_IMAGE_H; row += PIXEL_OFFSET)
    {
        if(is_right_valid(right_edge_line[row]) && (!found || right_edge_line[row] <= best_col))
        {
            best_row = row;
            best_col = right_edge_line[row];
            found = 1;
        }
    }

    if(found)
    {
        *row_out = best_row;
        *col_out = best_col;
    }
    return found;
}

// 判断左螚界中部是不是“扞不到线// 这里的思衯不是只看一个点，而是看中部一掾区域里是同连睭出现垈多无效
static uint8 left_middle_missing(void)
{
    uint8 row;
    uint8 invalid_cnt = 0;
    uint8 longest_invalid = 0;
    uint8 current_invalid = 0;

    for(row = RING_MID_ROW_MIN; row <= RING_MID_ROW_MAX; row += PIXEL_OFFSET)
    {
        if(is_left_valid(left_edge_line[row]))
        {
            current_invalid = 0;
        }
        else
        {
            invalid_cnt++;
            current_invalid++;
            if(current_invalid > longest_invalid)
            {
                longest_invalid = current_invalid;
            }
        }
    }

    return (invalid_cnt >= 4 || longest_invalid >= 3);
}

// 判断右边界中部是否找不到线，服务状态 6 到 7 的右侧判断
static uint8 right_middle_missing(void)
{
    uint8 row;
    uint8 invalid_cnt = 0;
    uint8 longest_invalid = 0;
    uint8 current_invalid = 0;

    for(row = RING_MID_ROW_MIN; row <= RING_MID_ROW_MAX; row += PIXEL_OFFSET)
    {
        if(is_right_valid(right_edge_line[row]))
        {
            current_invalid = 0;
        }
        else
        {
            invalid_cnt++;
            current_invalid++;
            if(current_invalid > longest_invalid)
            {
                longest_invalid = current_invalid;
            }
        }
    }

    return (invalid_cnt >= 4 || longest_invalid >= 3);
}

// 判断右边界是否还基本存在，因为状态 1 建立在右边界有线的前提上
static uint8 right_boundary_exist(void)
{
    uint8 row;
    uint8 valid_cnt = 0;

    for(row = RING_TOP_ROW_MIN; row < SEARCH_IMAGE_H; row += PIXEL_OFFSET)
    {
        if(is_right_valid(right_edge_line[row]))
        {
            valid_cnt++;
        }
    }

    return (valid_cnt >= 8);
}

static uint8 right_region_has_valid_points(uint8 row_min, uint8 row_max, uint8 min_valid)
{
    uint8 row;
    uint8 valid_cnt = 0;

    for(row = row_min; row <= row_max && row < SEARCH_IMAGE_H; row += PIXEL_OFFSET)
    {
        if(is_right_valid(right_edge_line[row]))
        {
            valid_cnt++;
        }
    }

    return (valid_cnt >= min_valid);
}

static uint8 right_bottom_has_continuous_rows(uint8 min_run)
{
    uint8 row;
    uint8 current_run = 0;
    uint8 longest_run = 0;

    for(row = RING_BOTTOM_ROW_MIN; row < SEARCH_IMAGE_H; row++)
    {
        if(is_right_valid(right_edge_line[row]))
        {
            current_run++;
            if(current_run > longest_run)
            {
                longest_run = current_run;
            }
        }
        else
        {
            current_run = 0;
        }
    }

    return (longest_run >= min_run);
}

static uint8 right_side_valid_for_state1(void)
{
    if(!right_bottom_has_continuous_rows(RING_ENTRY_RIGHT_BOTTOM_RUN))
    {
        return 0;
    }

    if(!right_region_has_valid_points(RING_TOP_ROW_MIN, RING_TOP_ROW_MAX, RING_ENTRY_RIGHT_MIN_VALID))
    {
        return 0;
    }

    if(!right_region_has_valid_points(RING_MID_ROW_MIN, RING_MID_ROW_MAX, RING_ENTRY_RIGHT_MIN_VALID))
    {
        return 0;
    }

    return 1;
}

static uint8 left_bottom_has_continuous_rows(uint8 min_run)
{
    uint8 row;
    uint8 current_run = 0;
    uint8 longest_run = 0;

    for(row = RING_BOTTOM_ROW_MIN; row < SEARCH_IMAGE_H; row++)
    {
        if(is_left_valid(left_edge_line[row]))
        {
            current_run++;
            if(current_run > longest_run)
            {
                longest_run = current_run;
            }
        }
        else
        {
            current_run = 0;
        }
    }

    return (longest_run >= min_run);
}

static uint8 left_region_has_valid_points(uint8 row_min, uint8 row_max, uint8 min_valid)
{
    uint8 row;
    uint8 valid_cnt = 0;

    for(row = row_min; row <= row_max && row < SEARCH_IMAGE_H; row += PIXEL_OFFSET)
    {
        if(is_left_valid(left_edge_line[row]))
        {
            valid_cnt++;
        }
    }

    return (valid_cnt >= min_valid);
}

static uint8 left_side_valid_for_right_ring_entry(void)
{
    if(!left_bottom_has_continuous_rows(10))
    {
        return 0;
    }

    if(!left_region_has_valid_points(RING_TOP_ROW_MIN, RING_TOP_ROW_MAX, RING_ENTRY_RIGHT_MIN_VALID))
    {
        return 0;
    }

    if(!left_region_has_valid_points(RING_MID_ROW_MIN, RING_MID_ROW_MAX, RING_ENTRY_RIGHT_MIN_VALID))
    {
        return 0;
    }

    return 1;
}

static uint8 left_bottom_trend_increasing_for_state1(void)
{
    int16 row;
    uint8 valid_cnt = 0;
    uint8 reverse_cnt = 0;
    uint8 first_col = 0;
    uint8 last_col = 0;
    uint8 prev_col = 0;

    for(row = SEARCH_IMAGE_H - 2; row >= RING_BASE_ROW_MIN; row -= PIXEL_OFFSET)
    {
        uint8 col = left_edge_line[row];

        if(!is_left_valid(col))
        {
            return 0;
        }

        if(valid_cnt == 0)
        {
            first_col = col;
        }
        else
        {
            int16 diff = (int16)col - (int16)prev_col;

            if(diff < 0)
            {
                if(-diff > RING_BOTTOM_TREND_MAX_REVERSE)
                {
                    return 0;
                }

                reverse_cnt++;
                if(reverse_cnt > RING_BOTTOM_TREND_MAX_REVERSE_COUNT)
                {
                    return 0;
                }
            }
        }

        prev_col = col;
        last_col = col;
        valid_cnt++;
    }

    if(valid_cnt < RING_ENTRY_TREND_MIN_VALID)
    {
        return 0;
    }

    if((int16)last_col - (int16)first_col < RING_ENTRY_TREND_MIN_DELTA)
    {
        return 0;
    }

    return 1;
}

static uint8 right_bottom_trend_decreasing_for_state1(void)
{
    int16 row;
    uint8 valid_cnt = 0;
    uint8 reverse_cnt = 0;
    uint8 first_col = 0;
    uint8 last_col = 0;
    uint8 prev_col = 0;

    for(row = SEARCH_IMAGE_H - 2; row >= RING_BASE_ROW_MIN; row -= PIXEL_OFFSET)
    {
        uint8 col = right_edge_line[row];

        if(!is_right_valid(col))
        {
            return 0;
        }

        if(valid_cnt == 0)
        {
            first_col = col;
        }
        else
        {
            int16 diff = (int16)col - (int16)prev_col;

            if(diff > 0)
            {
                if(diff > RING_BOTTOM_TREND_MAX_REVERSE)
                {
                    return 0;
                }

                reverse_cnt++;
                if(reverse_cnt > RING_BOTTOM_TREND_MAX_REVERSE_COUNT)
                {
                    return 0;
                }
            }
        }

        prev_col = col;
        last_col = col;
        valid_cnt++;
    }

    if(valid_cnt < RING_ENTRY_TREND_MIN_VALID)
    {
        return 0;
    }

    if((int16)first_col - (int16)last_col < RING_ENTRY_TREND_MIN_DELTA)
    {
        return 0;
    }

    return 1;
}

static uint8 find_left_top_entry_break_point(uint8 *break_row_out,
                                             uint8 *break_col_out,
                                             uint8 *top_max_row_out,
                                             uint8 *top_max_col_out)
{
    uint8 row;
    uint8 valid_above = 0;
    uint8 top_max_row = 0;
    uint8 top_max_col = 0;

    for(row = RING_TOP_ROW_MIN; row <= RING_TOP_ROW_MAX; row += PIXEL_OFFSET)
    {
        uint8 col = left_edge_line[row];

        if(is_left_valid(col))
        {
            valid_above++;
            if(top_max_col == 0 || col > top_max_col)
            {
                top_max_row = row;
                top_max_col = col;
            }
        }

        if(row > RING_TOP_ROW_MIN && row + PIXEL_OFFSET <= RING_TOP_ROW_MAX)
        {
            uint8 prev = left_edge_line[row - PIXEL_OFFSET];
            uint8 curr = left_edge_line[row];
            uint8 next = left_edge_line[row + PIXEL_OFFSET];

            if(!is_left_valid(prev) || !is_left_valid(curr) || !is_left_valid(next))
            {
                continue;
            }

            if(valid_above < RING_ENTRY_TOP_MIN_ABOVE)
            {
                continue;
            }

            if((int16)curr - (int16)next > BREAK_DELTA_X)
            {
                *break_row_out = row;
                *break_col_out = curr;
                *top_max_row_out = top_max_row;
                *top_max_col_out = top_max_col;
                return 1;
            }
        }
    }

    *break_row_out = 0;
    *break_col_out = 0;
    *top_max_row_out = 0;
    *top_max_col_out = 0;
    return 0;
}

static uint8 find_right_top_entry_break_point(uint8 *break_row_out,
                                              uint8 *break_col_out,
                                              uint8 *top_min_row_out,
                                              uint8 *top_min_col_out)
{
    uint8 row;
    uint8 valid_above = 0;
    uint8 top_min_row = 0;
    uint8 top_min_col = 0;
    uint8 found_min = 0;

    for(row = RING_TOP_ROW_MIN; row <= RING_TOP_ROW_MAX; row += PIXEL_OFFSET)
    {
        uint8 col = right_edge_line[row];

        if(is_right_valid(col))
        {
            valid_above++;
            if(!found_min || col < top_min_col)
            {
                top_min_row = row;
                top_min_col = col;
                found_min = 1;
            }
        }

        if(row > RING_TOP_ROW_MIN && row + PIXEL_OFFSET <= RING_TOP_ROW_MAX)
        {
            uint8 prev = right_edge_line[row - PIXEL_OFFSET];
            uint8 curr = right_edge_line[row];
            uint8 next = right_edge_line[row + PIXEL_OFFSET];

            if(!is_right_valid(prev) || !is_right_valid(curr) || !is_right_valid(next))
            {
                continue;
            }

            if(valid_above < RING_ENTRY_TOP_MIN_ABOVE)
            {
                continue;
            }

            if((int16)next - (int16)curr > BREAK_DELTA_X)
            {
                *break_row_out = row;
                *break_col_out = curr;
                *top_min_row_out = top_min_row;
                *top_min_col_out = top_min_col;
                return 1;
            }
        }
    }

    *break_row_out = 0;
    *break_col_out = 0;
    *top_min_row_out = 0;
    *top_min_col_out = 0;
    return 0;
}

// 右环入环顶部连接点：
// 先从上到下扞右螚界突变点，再从突变点垀上回x 最小的有效点
static uint8 find_right_top_entry_connect_point(uint8 *row_out, uint8 *col_out)
{
    uint8 break_row, break_col, unused_row, unused_col;
    uint8 row;
    uint8 best_row = 0;
    uint8 best_col = 0;
    uint8 found = 0;

    if(!find_right_top_entry_break_point(&break_row, &break_col, &unused_row, &unused_col))
    {
        return 0;
    }

    for(row = RING_TOP_ROW_MIN; row <= break_row; row += PIXEL_OFFSET)
    {
        if(is_right_valid(right_edge_line[row]) && (!found || right_edge_line[row] <= best_col))
        {
            best_row = row;
            best_col = right_edge_line[row];
            found = 1;
        }
    }

    if(found)
    {
        *row_out = best_row;
        *col_out = best_col;
    }

    return found;
}

static uint8 find_left_top_entry_connect_point(uint8 *row_out, uint8 *col_out)
{
    uint8 break_row, break_col, unused_row, unused_col;
    uint8 row;
    uint8 best_row = 0;
    uint8 best_col = 0;
    uint8 found = 0;

    if(!find_left_top_entry_break_point(&break_row, &break_col, &unused_row, &unused_col))
    {
        return 0;
    }

    for(row = RING_TOP_ROW_MIN; row <= break_row; row += PIXEL_OFFSET)
    {
        if(is_left_valid(left_edge_line[row]) && (!found || left_edge_line[row] >= best_col))
        {
            best_row = row;
            best_col = left_edge_line[row];
            found = 1;
        }
    }

    if(found)
    {
        *row_out = best_row;
        *col_out = best_col;
    }

    return found;
}

static uint8 left_top_has_entry_break_for_state1(void)
{
    uint8 break_row, break_col, top_max_row, top_max_col;

    return find_left_top_entry_break_point(&break_row, &break_col, &top_max_row, &top_max_col);
}

static uint8 right_top_has_entry_break_for_state1(void)
{
    uint8 top_min_row, top_min_col;

    return find_right_top_entry_connect_point(&top_min_row, &top_min_col);
}

// 判断左底部基点是同存// 这里把底部一小掾区域里，能不能扞到明昞猝开左螚楆的点，作为“左基点存在
static uint8 left_bottom_base_found(void)
{
    uint8 row;
    for(row = RING_BASE_ROW_MIN; row < SEARCH_IMAGE_H; row += PIXEL_OFFSET)
    {
        if(left_edge_line[row] >= LEFT_BASE_MIN_X)
        {
            return 1;
        }
    }
    return 0;
}

// 判断右底部基点是同存
static uint8 left_bottom_base_found_for_state3(void)
{
    uint8 row;
    uint8 valid_cnt = 0;

    for(row = RING_BASE_ROW_MIN; row < SEARCH_IMAGE_H; row += PIXEL_OFFSET)
    {
        if(is_left_valid(left_edge_line[row]))
        {
            valid_cnt++;
            if(valid_cnt >= LEFT_STATE3_BOTTOM_VALID_MIN)
            {
                return 1;
            }
        }
    }

    return 0;
}

static uint8 right_bottom_base_found(void)
{
    uint8 row;
    for(row = RING_BASE_ROW_MIN; row < SEARCH_IMAGE_H; row += PIXEL_OFFSET)
    {
        if(right_edge_line[row] <= RIGHT_BASE_MAX_X)
        {
            return 1;
        }
    }
    return 0;
}

// 判断右螚底部区域是同还能扞到螚线
// 这里不强制找“基点”，只要底部还有有效右边界即可
static uint8 right_bottom_line_found(void)
{
    uint8 row;
    for(row = RING_BOTTOM_ROW_MIN; row < SEARCH_IMAGE_H; row += PIXEL_OFFSET)
    {
        if(is_right_valid(right_edge_line[row]))
        {
            return 1;
        }
    }
    return 0;
}

static uint8 left_bottom_line_found_for_right_ring(void)
{
    int16 row;
    uint8 valid_cnt = 0;

    for(row = SEARCH_IMAGE_H - 1; row >= (int16)(SEARCH_IMAGE_H - 30); row--)
    {
        if(is_left_valid(left_edge_line[row]))
        {
            valid_cnt++;
        }
    }

    return (valid_cnt >= 10);
}

// 从下垀上扞右螚界珏一个有效点
// 状态 7 到 8 的条件：最后一个有效点的 x >= 100
#if RING_UNUSED_DEBUG_HELPERS_ENABLE
static uint8 left_edge_rise_then_fall_for_right_state7(uint8 *peak_row_out, uint8 *peak_col_out)
{
    int16 row;
    uint8 valid_cnt = 0;
    uint8 fall_samples = 0;
    uint8 start_col = 0;
    uint8 peak_row = 0;
    uint8 peak_col = 0;
    uint8 found_rise = 0;

    for(row = SEARCH_IMAGE_H - 1; row >= STOP_ROW; row--)
    {
        uint8 col = left_edge_line[row];

        if(!is_left_valid(col))
        {
            continue;
        }

        if(valid_cnt == 0)
        {
            start_col = col;
            peak_col = col;
            peak_row = (uint8)row;
            valid_cnt++;
            continue;
        }

        valid_cnt++;

        if(col > peak_col)
        {
            peak_col = col;
            peak_row = (uint8)row;
            fall_samples = 0;

            if((int16)peak_col - (int16)start_col >= RIGHT_STATE7_TREND_MIN_RISE)
            {
                found_rise = 1;
            }
            continue;
        }

        if(found_rise && (int16)peak_col - (int16)col >= RIGHT_STATE7_TREND_MIN_FALL)
        {
            fall_samples++;
            if(valid_cnt >= RIGHT_STATE7_TREND_MIN_VALID &&
               fall_samples >= RIGHT_STATE7_TREND_MIN_FALL_SAMPLES)
            {
                *peak_row_out = peak_row;
                *peak_col_out = peak_col;
                return 1;
            }
        }
    }

    *peak_row_out = 0;
    *peak_col_out = 0;
    return 0;
}

#endif

static uint8 find_right_state6_bottom_left_peak(uint8 *peak_row_out, uint8 *peak_col_out)
{
    int16 row;
    int16 row_min = (int16)(SEARCH_IMAGE_H - 1) - RIGHT_STATE6_SCAN_ROWS;
    uint8 valid_cnt = 0;
    uint8 fall_samples = 0;
    uint8 start_col = 0;
    uint8 peak_row = 0;
    uint8 peak_col = 0;
    uint8 found_rise = 0;

    if(row_min < STOP_ROW)
    {
        row_min = STOP_ROW;
    }

    for(row = SEARCH_IMAGE_H - 1; row >= row_min; row -= RIGHT_STATE6_SCAN_STEP)
    {
        uint8 col = left_edge_line[row];

        if(!is_left_valid(col))
        {
            if(found_rise && valid_cnt >= RIGHT_STATE7_TREND_MIN_VALID)
            {
                *peak_row_out = peak_row;
                *peak_col_out = peak_col;
                return 1;
            }
            continue;
        }

        if(valid_cnt == 0)
        {
            start_col = col;
            peak_col = col;
            peak_row = (uint8)row;
            valid_cnt++;
            continue;
        }

        valid_cnt++;

        if(col > peak_col)
        {
            peak_col = col;
            peak_row = (uint8)row;
            fall_samples = 0;

            if((int16)peak_col - (int16)start_col >= RIGHT_STATE7_TREND_MIN_RISE)
            {
                found_rise = 1;
            }
            continue;
        }

        if(found_rise && (int16)peak_col - (int16)col >= RIGHT_STATE7_TREND_MIN_FALL)
        {
            fall_samples++;
            if(valid_cnt >= RIGHT_STATE7_TREND_MIN_VALID &&
               fall_samples >= RIGHT_STATE7_TREND_MIN_FALL_SAMPLES)
            {
                *peak_row_out = peak_row;
                *peak_col_out = peak_col;
                return 1;
            }
        }
    }

    *peak_row_out = 0;
    *peak_col_out = 0;
    return 0;
}

static uint8 right_state6_should_enter_state7(uint8 *debug_row_out, uint8 *debug_col_out)
{
    uint8 peak_row = 0;
    uint8 peak_col = 0;

    if(find_right_state6_bottom_left_peak(&peak_row, &peak_col))
    {
        right_state6_peak_valid = 1;
        right_state6_peak_row = peak_row;
        right_state6_peak_col = peak_col;
        right_state6_peak_lost_cnt = 0;
        *debug_row_out = peak_row;
        *debug_col_out = peak_col;
        return 0;
    }

    if(!right_state6_peak_valid)
    {
        *debug_row_out = 0;
        *debug_col_out = 0;
        return 0;
    }

    if(right_state6_peak_lost_cnt < 255)
    {
        right_state6_peak_lost_cnt++;
    }

    *debug_row_out = right_state6_peak_row;
    *debug_col_out = right_state6_peak_col;

    return (right_state6_peak_lost_cnt >= RIGHT_STATE6_PEAK_LOST_CONFIRM);
}

static uint8 right_state5_yaw_can_enter_state6(void)
{
    float yaw_now = imu660_get_yaw();

    return (yaw_now > RIGHT_STATE5_YAW_MIN_DEG &&
            yaw_now < RIGHT_STATE5_YAW_MAX_DEG);
}

static uint8 right_state8_yaw_can_enter_state9(void)
{
    float yaw_now = imu660_get_yaw();

    return (yaw_now > RIGHT_STATE8_YAW_MIN_DEG &&
            yaw_now < RIGHT_STATE8_YAW_MAX_DEG);
}

static uint8 left_state5_yaw_can_enter_state6(void)
{
    float yaw_now = imu660_get_yaw();

    return (yaw_now > LEFT_STATE5_YAW_MIN_DEG &&
            yaw_now < LEFT_STATE5_YAW_MAX_DEG);
}

static uint8 left_state8_yaw_can_enter_state9(void)
{
    float yaw_now = imu660_get_yaw();

    return (yaw_now > LEFT_STATE8_YAW_MIN_DEG &&
            yaw_now < LEFT_STATE8_YAW_MAX_DEG);
}

static uint8 find_first_right_point_from_bottom(uint8 *row_out, uint8 *col_out)
{
    int16 row;

    for(row = SEARCH_IMAGE_H - 2; row >= STOP_ROW; row -= PIXEL_OFFSET)
    {
        if(is_right_valid(right_edge_line[row]))
        {
            *row_out = (uint8)row;
            *col_out = right_edge_line[row];
            return 1;
        }
    }
    return 0;
}

#if RING_UNUSED_DEBUG_HELPERS_ENABLE
// 从下垀上扞左螚界珏一个有效点
static uint8 find_first_left_point_from_bottom(uint8 *row_out, uint8 *col_out)
{
    int16 row;

    for(row = SEARCH_IMAGE_H - 2; row >= STOP_ROW; row -= PIXEL_OFFSET)
    {
        if(is_left_valid(left_edge_line[row]))
        {
            *row_out = (uint8)row;
            *col_out = left_edge_line[row];
            return 1;
        }
    }
    return 0;
}

// 从上到下寻找左边界“拐点”：当前点与上一点变化不大，但与下一点变化突然变大
static uint8 left_has_lost_line_above(uint8 row_base)
{
    int16 row;
    int16 row_min = (int16)row_base - RIGHT_STATE7_BREAK_ABOVE_SCAN;
    uint8 invalid_cnt = 0;

    if(row_min < STOP_ROW)
    {
        row_min = STOP_ROW;
    }

    for(row = (int16)row_base - 1; row >= row_min; row--)
    {
        if(!is_left_valid(left_edge_line[row]))
        {
            invalid_cnt++;
        }
    }

    return (invalid_cnt >= RIGHT_STATE7_BREAK_ABOVE_INVALID_MIN);
}

static uint8 left_has_valid_line_below(uint8 row_base)
{
    uint8 row;
    uint8 row_max = (uint8)((int16)row_base + RIGHT_STATE7_BREAK_BELOW_SCAN);
    uint8 valid_cnt = 0;

    if(row_max >= SEARCH_IMAGE_H)
    {
        row_max = SEARCH_IMAGE_H - 1;
    }

    for(row = row_base; row <= row_max; row++)
    {
        if(is_left_valid(left_edge_line[row]))
        {
            valid_cnt++;
        }
    }

    return (valid_cnt >= RIGHT_STATE7_BREAK_BELOW_VALID_MIN);
}

static uint8 find_left_lost_then_break_point_for_right_state7(uint8 *row_out, uint8 *col_out)
{
    uint8 row;

    for(row = RIGHT_STATE7_BREAK_ROW_MIN; row < SEARCH_IMAGE_H; row++)
    {
        uint8 curr = left_edge_line[row];
        uint8 next = (row + 1 < SEARCH_IMAGE_H) ? left_edge_line[row + 1] : curr;

        if(!is_left_valid(curr))
        {
            continue;
        }

        if(!left_has_lost_line_above(row))
        {
            continue;
        }

        if(!left_has_valid_line_below(row))
        {
            continue;
        }

        if(!is_left_valid(next) ||
           abs((int16)next - (int16)curr) >= RIGHT_STATE7_BREAK_MIN_DELTA_X)
        {
            *row_out = row;
            *col_out = curr;
            return 1;
        }
    }

    *row_out = 0;
    *col_out = 0;
    return 0;
}

static uint8 right_state8_can_enter_state9(uint8 *left_bottom_row_out, uint8 *left_bottom_col_out)
{
    uint8 row = 0;
    uint8 col = 0;

    if(!right_bottom_base_found())
    {
        *left_bottom_row_out = 0;
        *left_bottom_col_out = 0;
        return 0;
    }

    if(!find_first_left_point_from_bottom(&row, &col))
    {
        *left_bottom_row_out = 0;
        *left_bottom_col_out = 0;
        return 0;
    }

    *left_bottom_row_out = row;
    *left_bottom_col_out = col;
    return (row >= 100);
}

#endif

static uint8 find_left_break_point(uint8 *row_out, uint8 *col_out)
{
    uint8 row;

    for(row = RING_TOP_ROW_MIN + PIXEL_OFFSET; row <= SEARCH_IMAGE_H - 1 - PIXEL_OFFSET; row += PIXEL_OFFSET)
    {
        uint8 prev = left_edge_line[row - PIXEL_OFFSET];
        uint8 curr = left_edge_line[row];
        uint8 next = left_edge_line[row + PIXEL_OFFSET];

        if(!is_left_valid(prev) || !is_left_valid(curr) || !is_left_valid(next))
        {
            continue;
        }

        if(abs((int16)curr - (int16)prev) < BREAK_DELTA_X &&
           abs((int16)next - (int16)curr) > BREAK_DELTA_X)
        {
            *row_out = row;
            *col_out = curr;
            return 1;
        }
    }

    return 0;
}

// 从上到下扞右螚“拐点
static uint8 find_right_break_point(uint8 *row_out, uint8 *col_out)
{
    uint8 row;

    for(row = RING_TOP_ROW_MIN + PIXEL_OFFSET; row <= SEARCH_IMAGE_H - 1 - PIXEL_OFFSET; row += PIXEL_OFFSET)
    {
        uint8 prev = right_edge_line[row - PIXEL_OFFSET];
        uint8 curr = right_edge_line[row];
        uint8 next = right_edge_line[row + PIXEL_OFFSET];

        if(!is_right_valid(prev) || !is_right_valid(curr) || !is_right_valid(next))
        {
            continue;
        }

        if(abs((int16)curr - (int16)prev) < BREAK_DELTA_X &&
           abs((int16)next - (int16)curr) > BREAK_DELTA_X)
        {
            *row_out = row;
            *col_out = curr;
            return 1;
        }
    }

    return 0;
}

// 状态 1 的进环条件：
// 1. 右螚界有// 2. 左螚顶部能扞到点
// 3. 左螚底部能扞到点
// 4. 左螚中部明昞丢线
// 5. 右边界中部不能明显丢线，否则更像十字，不进入左圆环
static uint8 is_state1_scene(void)
{
    uint8 top_row, top_col, bottom_row, bottom_col;

    if(!right_side_valid_for_state1())
    {
        return 0;
    }

    if(!find_left_top_entry_connect_point(&top_row, &top_col))
    {
        return 0;
    }

    if(!find_left_bottom_max_point(&bottom_row, &bottom_col))
    {
        return 0;
    }

    if(!left_bottom_trend_increasing_for_state1())
    {
        return 0;
    }

    if(right_middle_missing())
    {
        return 0;
    }

    return left_middle_missing();
}

static uint8 is_right_state1_scene(void)
{
    uint8 top_row, top_col, bottom_row, bottom_col;

    if(!left_side_valid_for_right_ring_entry())
    {
        return 0;
    }

    if(!find_right_top_entry_connect_point(&top_row, &top_col))
    {
        return 0;
    }

    if(!find_right_bottom_min_point(&bottom_row, &bottom_col))
    {
        return 0;
    }

    if(!right_bottom_trend_decreasing_for_state1())
    {
        return 0;
    }

    if(!right_top_has_entry_break_for_state1())
    {
        return 0;
    }

    if(left_middle_missing())
    {
        return 0;
    }

    return right_middle_missing();
}

static uint8 left_bottom_curve_decreasing_for_state0(void)
{
    int16 row;
    uint8 valid_cnt = 0;
    uint8 reverse_cnt = 0;
    uint8 invalid_cnt = 0;
    uint8 first_col = 0;
    uint8 last_col = 0;
    uint8 prev_col = 0;

    for(row = SEARCH_IMAGE_H - 2; row >= RING_BASE_ROW_MIN; row -= PIXEL_OFFSET)
    {
        uint8 col = left_edge_line[row];

        if(!is_left_valid(col))
        {
            invalid_cnt++;
            continue;
        }

        if(valid_cnt == 0)
        {
            first_col = col;
        }
        else
        {
            int16 diff = (int16)col - (int16)prev_col;

            if(diff > 0)
            {
                if(diff > RING_BOTTOM_TREND_MAX_REVERSE)
                {
                    return 0;
                }

                reverse_cnt++;
                if(reverse_cnt > RING_BOTTOM_TREND_MAX_REVERSE_COUNT)
                {
                    return 0;
                }
            }
        }

        prev_col = col;
        last_col = col;
        valid_cnt++;
    }

    if(valid_cnt < RING_ENTRY_TREND_MIN_VALID)
    {
        return 0;
    }

    if(invalid_cnt > valid_cnt)
    {
        return 0;
    }

    return ((int16)first_col - (int16)last_col >= RING_ENTRY_TREND_MIN_DELTA);
}

static uint8 right_bottom_curve_increasing_for_state0(void)
{
    int16 row;
    uint8 valid_cnt = 0;
    uint8 reverse_cnt = 0;
    uint8 invalid_cnt = 0;
    uint8 first_col = 0;
    uint8 last_col = 0;
    uint8 prev_col = 0;

    for(row = SEARCH_IMAGE_H - 2; row >= RING_BASE_ROW_MIN; row -= PIXEL_OFFSET)
    {
        uint8 col = right_edge_line[row];

        if(!is_right_valid(col))
        {
            invalid_cnt++;
            continue;
        }

        if(valid_cnt == 0)
        {
            first_col = col;
        }
        else
        {
            int16 diff = (int16)col - (int16)prev_col;

            if(diff < 0)
            {
                if(-diff > RING_BOTTOM_TREND_MAX_REVERSE)
                {
                    return 0;
                }

                reverse_cnt++;
                if(reverse_cnt > RING_BOTTOM_TREND_MAX_REVERSE_COUNT)
                {
                    return 0;
                }
            }
        }

        prev_col = col;
        last_col = col;
        valid_cnt++;
    }

    if(valid_cnt < RING_ENTRY_TREND_MIN_VALID)
    {
        return 0;
    }

    if(invalid_cnt > valid_cnt)
    {
        return 0;
    }

    return ((int16)last_col - (int16)first_col >= RING_ENTRY_TREND_MIN_DELTA);
}

static uint8 is_cross_or_ring_suspect_scene(void)
{
    uint8 row, col;

    if(left_bottom_curve_decreasing_for_state0() ||
       right_bottom_curve_increasing_for_state0())
    {
        return 0;
    }

    if(!left_middle_missing() && !right_middle_missing())
    {
        return 0;
    }

    if(find_left_top_max_point(&row, &col) &&
       find_left_bottom_max_point(&row, &col))
    {
        return 1;
    }

    if(find_right_top_min_point(&row, &col) &&
       find_right_bottom_min_point(&row, &col))
    {
        return 1;
    }

    return 0;
}

static uint8 is_cross_scene_for_ring_entry(void)
{
    uint8 left_row, left_col;
    uint8 right_row, right_col;

    if(left_bottom_curve_decreasing_for_state0() ||
       right_bottom_curve_increasing_for_state0())
    {
        return 0;
    }

    if(!left_middle_missing() && !right_middle_missing())
    {
        return 0;
    }

    if(!find_left_top_entry_connect_point(&left_row, &left_col))
    {
        return 0;
    }

    if(!find_right_top_entry_connect_point(&right_row, &right_col))
    {
        return 0;
    }

    return 1;
}

static uint8 apply_cross_ring_suspect_repair(void)
{
    uint8 top_row, top_col, bottom_row, bottom_col;
    uint8 repaired_flags = 0;

    if(!is_cross_or_ring_suspect_scene())
    {
        cross_ring_suspect_hold_seen = 0;
        ring_pre_repair_flag = 0;
        return 0;
    }

    if(find_left_top_max_point(&top_row, &top_col) &&
       find_left_bottom_max_point(&bottom_row, &bottom_col))
    {
        // Only record the pre-repair need here. Do not modify left_control_line.
        // draw_line_by_point(top_row, top_col, bottom_row, bottom_col, 0);
        repaired_flags |= 1;
    }

    if(find_right_top_min_point(&top_row, &top_col) &&
       find_right_bottom_min_point(&bottom_row, &bottom_col))
    {
        // Only record the pre-repair need here. Do not modify right_control_line.
        // draw_line_by_point(top_row, top_col, bottom_row, bottom_col, 1);
        repaired_flags |= 2;
    }

    if(repaired_flags)
    {
        cross_ring_suspect_hold_seen = 1;
        ring_pre_repair_flag = repaired_flags;
        key_anlysis2 = RING_MID_ROW_MIN;
        key_anlysis3 = repaired_flags;
        return 1;
    }

    ring_pre_repair_flag = 0;
    return 0;
}

#if RING_UNUSED_DEBUG_HELPERS_ENABLE
static uint8 is_pre_entry_protect_scene(void)
{
    uint8 top_row, top_col, bottom_row, bottom_col;

    if(!right_side_valid_for_state1())
    {
        return 0;
    }

    if(!left_middle_missing())
    {
        return 0;
    }

    if(!find_left_pre_entry_top_point(&top_row, &top_col))
    {
        return 0;
    }

    if(!find_left_bottom_max_point(&bottom_row, &bottom_col))
    {
        return 0;
    }

    return 1;
}

static uint8 apply_pre_entry_protect(void)
{
    uint8 top_row, top_col, bottom_row, bottom_col;

    if(!is_pre_entry_protect_scene())
    {
        return 0;
    }

    if(find_left_pre_entry_top_point(&top_row, &top_col) &&
       find_left_bottom_max_point(&bottom_row, &bottom_col))
    {
        draw_line_by_point(top_row, top_col, bottom_row, bottom_col, 0);
        key_anlysis2 = top_row;
        key_anlysis3 = top_col;
        return 1;
    }

    return 0;
}

// 状态 1：把左边界顶部最大点和左边界底部最大点连起来，作为左补线
#endif

static void apply_state1(void)
{
    uint8 top_row, top_col, bottom_row, bottom_col;

    if(find_left_top_entry_connect_point(&top_row, &top_col) &&
       find_left_bottom_max_point(&bottom_row, &bottom_col))
    {
        draw_line_by_point(top_row, top_col, bottom_row, bottom_col, 0);
        key_anlysis2 = top_row;
        key_anlysis3 = top_col;
    }
}

static void apply_right_state1(void)
{
    uint8 top_row, top_col, bottom_row, bottom_col;

    if(find_right_top_entry_connect_point(&top_row, &top_col) &&
       find_right_bottom_min_point(&bottom_row, &bottom_col))
    {
        draw_line_by_point(top_row, top_col, bottom_row, bottom_col, 1);
        key_anlysis2 = top_row;
        key_anlysis3 = top_col;
    }
}

// 状态 2：左底部基点丢失时，把左顶部最大点直接连到左下角 (2,118)
static void apply_state2(void)
{
    uint8 top_row, top_col;

    if(find_left_top_max_point(&top_row, &top_col))
    {
        draw_line_by_point(top_row, top_col, RING_LEFT_CORNER_Y, RING_LEFT_CORNER_X, 0);
        key_anlysis2 = top_row;
        key_anlysis3 = top_col;
    }
}

static void apply_right_state2(void)
{
    uint8 top_row, top_col;

    if(find_right_top_min_point(&top_row, &top_col))
    {
        draw_line_by_point(top_row, top_col, RING_RIGHT_CORNER_Y, RING_RIGHT_CORNER_X, 1);
        key_anlysis2 = top_row;
        key_anlysis3 = top_col;
    }
}

// 状态 3/4：找到左边界拐点后，把它连到右下角 (186,118)，作为右边界补线
// 同时把拐点坐标返回出来，给状态跳转和调试继续使用
static void left_state4_5_apply_fixed_left_line(void);

static uint8 apply_state3_or_4(uint8 *break_row_out, uint8 *break_col_out)
{
    uint8 break_row, break_col;

    left_state4_5_apply_fixed_left_line();

    if(find_left_break_point(&break_row, &break_col))
    {
        draw_line_by_point(break_row, break_col, RING_RIGHT_CORNER_Y, RING_RIGHT_CORNER_X, 1);
        *break_row_out = break_row;
        *break_col_out = break_col;
        return 1;
    }

    *break_row_out = 0;
    *break_col_out = 0;
    return 0;
}

static void left_state4_5_apply_fixed_left_line(void)
{
    draw_line_by_point(2, RING_LEFT_CORNER_X,
                       RING_LEFT_CORNER_Y, RING_LEFT_CORNER_X, 0);
}

static void left_state4_5_apply_fixed_right_line(void)
{
    draw_line_by_point(2, RING_LEFT_CORNER_X,
                       RING_RIGHT_CORNER_Y, RING_RIGHT_CORNER_X, 1);
}

static uint8 left_state4_5_apply_fixed_right_line_and_track_break(uint8 *break_row_out,
                                                                  uint8 *break_col_out)
{
    uint8 break_row = 0;
    uint8 break_col = 0;

    left_state4_5_apply_fixed_left_line();
    left_state4_5_apply_fixed_right_line();

    if(find_left_break_point(&break_row, &break_col))
    {
        left_state4_5_break_lost_cnt = 0;
        *break_row_out = break_row;
        *break_col_out = break_col;
        return 1;
    }

    if(left_state4_5_break_lost_cnt < 255)
    {
        left_state4_5_break_lost_cnt++;
    }

    *break_row_out = 0;
    *break_col_out = 0;
    return (left_state4_5_break_lost_cnt < LEFT_STATE4_5_BREAK_LOST_CONFIRM);
}

static void right_state4_5_apply_fixed_right_line(void)
{
    draw_line_by_point(2, RING_RIGHT_CORNER_X,
                       RING_RIGHT_CORNER_Y, RING_RIGHT_CORNER_X, 1);
}

static uint8 right_state4_5_apply_fixed_left_line_and_track_break(uint8 *break_row_out,
                                                                  uint8 *break_col_out)
{
    uint8 break_row = 0;
    uint8 break_col = 0;

    draw_line_by_point(2, RING_RIGHT_CORNER_X,
                       RING_LEFT_CORNER_Y, RING_LEFT_CORNER_X, 0);
    right_state4_5_apply_fixed_right_line();

    if(find_right_break_point(&break_row, &break_col))
    {
        right_state4_5_break_lost_cnt = 0;
        *break_row_out = break_row;
        *break_col_out = break_col;
        return 1;
    }

    if(right_state4_5_break_lost_cnt < 255)
    {
        right_state4_5_break_lost_cnt++;
    }

    *break_row_out = 0;
    *break_col_out = 0;
    return (right_state4_5_break_lost_cnt < RIGHT_STATE4_5_BREAK_LOST_CONFIRM);
}

// 状态 5：找到左边界拐点后，把它连到右下角 (186,118)，作为右边界补线
// 同时把拐点坐标返回出来，给状态停留与调试继续使用
static uint8 apply_state5(uint8 *break_row_out, uint8 *break_col_out)
{
    return left_state4_5_apply_fixed_right_line_and_track_break(break_row_out,
                                                                break_col_out);
}

// 状态 7：继续用左上角到右下角的兜底斜线作为右边界
static void apply_state7(void)
{
    draw_line_by_point(2, 2, RING_RIGHT_CORNER_Y, RING_RIGHT_CORNER_X, 1);
}

static void apply_right_state5_or_7_or_8(void)
{
    draw_line_by_point(2, RING_RIGHT_CORNER_X, RING_LEFT_CORNER_Y, RING_LEFT_CORNER_X, 0);
}

// 状态 9/10：找到左拐点后，把它连到左下角 (2,118)，作为左边界补线
// 同时把拐点坐标返回出来，给状态跳转和调试继续使用
static uint8 apply_state9_or_10(uint8 *break_row_out, uint8 *break_col_out)
{
    uint8 break_row, break_col;

    if(find_left_break_point(&break_row, &break_col))
    {
        ring_save_left_break_cache(break_row, break_col);
        draw_line_by_point(break_row, break_col, RING_LEFT_CORNER_Y, RING_LEFT_CORNER_X, 0);
        *break_row_out = break_row;
        *break_col_out = break_col;
        return 1;
    }

    if(Ring_Debug_CanUseCachedLeftBreak(ring_last_left_break_valid,
                                        ring_last_left_break_row,
                                        ring_last_left_break_col))
    {
        draw_line_by_point(ring_last_left_break_row, ring_last_left_break_col,
                           RING_LEFT_CORNER_Y, RING_LEFT_CORNER_X, 0);
        *break_row_out = ring_last_left_break_row;
        *break_col_out = ring_last_left_break_col;
        return 1;
    }

    *break_row_out = 0;
    *break_col_out = 0;
    return 0;
}

static uint8 apply_right_state9_or_10(uint8 *break_row_out, uint8 *break_col_out, uint8 allow_cache)
{
    uint8 break_row, break_col;

    if(find_right_break_point(&break_row, &break_col))
    {
        ring_save_right_break_cache(break_row, break_col);
        draw_line_by_point(break_row, break_col, RING_RIGHT_CORNER_Y, RING_RIGHT_CORNER_X, 1);
        *break_row_out = break_row;
        *break_col_out = break_col;
        return 1;
    }

    if(allow_cache && ring_can_use_cached_right_break())
    {
        draw_line_by_point(ring_last_right_break_row, ring_last_right_break_col,
                           RING_RIGHT_CORNER_Y, RING_RIGHT_CORNER_X, 1);
        *break_row_out = ring_last_right_break_row;
        *break_col_out = ring_last_right_break_col;
        return 1;
    }

    *break_row_out = 0;
    *break_col_out = 0;
    return 0;
}

// 兼容原工程的各种圆环标志；现在真正起作用的是 current_step
static uint8 apply_right_state9(uint8 *break_row_out, uint8 *break_col_out)
{
    return apply_right_state9_or_10(break_row_out, break_col_out, 0);
}

static uint8 apply_right_state10(uint8 *break_row_out, uint8 *break_col_out)
{
    return apply_right_state9_or_10(break_row_out, break_col_out, 1);
}

static void sync_ring_flags(void)
{
    ring_l = (current_ring_dir == 1 && current_step > 0 && current_step < 11);
    ring_r = (current_ring_dir == 2 && current_step > 0 && current_step < 11);

    ring_preMeet_flag = (current_step == 1);
    first_meeting_flag = (current_step == 2);
    ring_enter_flag = (current_step == 3 || current_step == 4);
    ring_turn_flag = (current_step >= 5 && current_step <= 8);
    Out_flag = (current_step >= 7 && current_step <= 10);
    Straighten_flag = (current_step == 9 || current_step == 10);
    ring_over_flag = (current_step == 11);
}

// 圆环结束，全部清零，回到普通搜线
void Ring_Over(void)
{
    ring_preMeet_flag = 0;
    first_meeting_flag = 0;
    ring_l = 0;
    ring_r = 0;
    ring_enter_flag = 0;
    ring_turn_flag = 0;
    Out_flag = 0;
    Straighten_flag = 0;
    ring_over_flag = 0;
    ring_pre_repair_flag = 0;
    current_ring_dir = 0;
    ring_clear_left_break_cache();
    ring_clear_right_break_cache();
    left_state4_5_clear_break_lost_cache();
    right_state4_5_clear_break_lost_cache();
    right_state6_clear_peak_cache();
    cross_ring_suspect_hold_seen = 0;
    ring_set_step(0);
    cnt_over = 0;
}

// 提供一个手动复位接口
void Ring_Reset(void)
{
    Ring_Over();
}

static void run_left_ring_state_machine(void)
{
    uint8 guard = 0;

    // guard 用于防止单帧连续跳状态过多，导致状态机失控
    while(guard++ < 4)
    {
        // 对前半段状态做一个保护：
        // 如果右边界整体都丢了，说明当前场景已经不适合进环逻辑，直接复位
        if(!right_boundary_exist() && current_step >= 1 && current_step <= 4)
        {
            Ring_Over();
            break;
        }

        switch(current_step)
        {
            case 1:
                // 状态 1 还需要左顶部继续存在，否则直接退出圆环状态
                // if(!left_top_exist())
                // {
                //     Ring_Over();
                //     break;
                // }

                // 状态 1 -> 状态 2：左底部基点丢失

                if(!left_bottom_base_found())
                {
                    ring_set_step(2);
                    continue;
                }

                // 留在状态 1，执行对应补线
                apply_state1();
                break;

            case 2:
                // 状态 2 同样需要左顶部还存在
                // if(!left_top_exist())
                // {
                //     Ring_Over();
                //     break;
                // }

                // 状态 2 -> 状态 3：左底部基点重新出现

                if(left_bottom_base_found())
                {
                    ring_set_step(3);
                    continue;
                }

                // 留在状态 2，执行对应补线
                apply_state2();
                break;

            case 3:
            {
                // 状态 3 需要顶部左边界继续存在

                // if(!left_top_exist())
                // {
                //     key_anlysis2 = 31;
                //     key_anlysis3 = 0;
                //     Ring_Over();
                //     break;
                // }

                // 状态 3 -> 状态 4：左底部基点再次丢失

                if(!left_bottom_base_found())
                {
                    key_anlysis2 = 34;
                    key_anlysis3 = 0;
                    left_state4_5_clear_break_lost_cache();
                    ring_set_step(4);
                    continue;
                }

                // 留在状态 3，保持正常搜线，不额外补线
                key_anlysis2 = 33;
                key_anlysis3 = 0;
                break;
            }

            case 4:
            {
                uint8 break_row = 0;
                uint8 break_col = 0;

                // 状态 4 -> 状态 5：右底部基点丢失

                if(!right_bottom_base_found())
                {
                    ring_set_step(5);
                    continue;
                }

                // 留在状态 4，继续按拐点连右下角

                if(left_state4_5_apply_fixed_right_line_and_track_break(&break_row, &break_col))
                {
                    key_anlysis2 = break_row;
                    key_anlysis3 = break_col;
                }
                else
                {
                    key_anlysis2 = 0;
                    key_anlysis3 = 0;
                }
                break;
            }

            case 5:
            {
                // uint8 break_row = 0;
                // uint8 break_col = 0;

                // // 留在状态 5，继续按左拐点连右下
                // // 如果找不到拐点了，说明车已经过弯，进入状态 6
                // if(apply_state5(&break_row, &break_col))
                // {
                //     key_anlysis2 = break_row;
                //     key_anlysis3 = break_col;
                // }
                // else
                // {
                //     key_anlysis2 = 0;
                //     key_anlysis3 = 0;
                //     ring_set_step(6);
                //     continue;
                // }
                // break;

                uint8 break_row = 0;
                uint8 break_col = 0;

                if(left_state5_yaw_can_enter_state6())
                {
                    key_anlysis2 = 0;
                    key_anlysis3 = 0;
                    ring_set_step(6);
                    continue;
                }

                if(apply_state5(&break_row, &break_col))
                {
                    key_anlysis2 = break_row;
                    key_anlysis3 = break_col;
                }
                else
                {
                    key_anlysis2 = 0;
                    key_anlysis3 = 0;
                }
                break;
            }

            case 6:
                // 状态 6 不补线，保持普通搜线；状态 6 -> 状态 7：右边界底部能找到边线，中部找不到
                if(right_bottom_line_found() && right_middle_missing())
                {
                    ring_set_step(7);
                    continue;
                }
                break;

            case 7:
            {
                // 状态 7 -> 状态 8：左右底部基点都丢失

                if(!left_bottom_base_found() && !right_bottom_base_found())
                {
                    ring_set_step(8);
                    continue;
                }

                // 留在状态 7，继续用左上到右下的兜底斜线
                apply_state7();
                break;
            }

            case 8:
            {
                // 状态 8 -> 状态 9：右底部恢复，并且左侧填充已经出现圆环补线可用的拐点
                /*
                uint8 row, col;
                uint8 break_row = 0;
                uint8 break_col = 0;

                if(find_first_right_point_from_bottom(&row, &col) &&
                   row >= 70 &&
                   find_left_break_point(&break_row, &break_col))
                {
                    key_anlysis2 = break_row;
                    key_anlysis3 = break_col;
                    ring_set_step(9);
                    continue;
                }
                else
                {
                    key_anlysis2 = 0;
                    key_anlysis3 = 0;
                }
                */

                if(left_state8_yaw_can_enter_state9())
                {
                    key_anlysis2 = 0;
                    key_anlysis3 = 0;
                    ring_set_step(9);
                    continue;
                }

                key_anlysis2 = 0;
                key_anlysis3 = 0;

                // 留在状态 8，继续用左上到右下的兜底斜线
                apply_state7();
                break;
            }

            case 9:
            {
                uint8 break_row = 0;
                uint8 break_col = 0;

                // 状态 9 -> 10 必须依赖当前帧真实找到的拐点；缓存只用于状态 10 防止补线闪断
                if(find_left_break_point(&break_row, &break_col))
                {
                    ring_save_left_break_cache(break_row, break_col);
                    draw_line_by_point(break_row, break_col, RING_LEFT_CORNER_Y, RING_LEFT_CORNER_X, 0);
                    key_anlysis2 = break_row;
                    key_anlysis3 = break_col;

                    // 状态 9 -> 状态 10：拐点的 y 已经大于 30

                    if(break_row > 30)
                    {
                        ring_set_step(10);
                    }
                }
                else
                {
                    key_anlysis2 = 0;
                    key_anlysis3 = 0;
                }
                break;
            }

            case 10:
            {
                uint8 break_row = 0;
                uint8 break_col = 0;

                // 状态 10 继续保持状态 9 的补线方式
                if(apply_state9_or_10(&break_row, &break_col))
                {
                    key_anlysis2 = break_row;
                    key_anlysis3 = break_col;
                }
                else
                {
                    key_anlysis2 = 0;
                    key_anlysis3 = 0;
                }

                // 状态 10 -> 状态 11：底部左右基点都恢复

                if(left_bottom_base_found() && right_bottom_base_found())
                {
                    ring_set_step(11);
                }
                break;
            }

            case 11:
                Ring_Over();
                break;

            default:
                // 兜底保护
                Ring_Over();
                break;
        }
        break;
    }

}

static void run_right_ring_state_machine(void)
{
    uint8 guard = 0;

    while(guard++ < 4)
    {
        switch(current_step)
        {
            case 1:
                // if(!right_top_exist())
                // {
                //     Ring_Over();
                //     break;
                // }

                if(!right_bottom_base_found())
                {
                    ring_set_step(2);
                    continue;
                }

                apply_right_state1();
                break;

            case 2:
                // if(!right_top_exist())
                // {
                //     Ring_Over();
                //     break;
                // }

                if(right_bottom_base_found())
                {
                    ring_set_step(3);
                    continue;
                }

                apply_right_state2();
                break;

            case 3:
            {
                // if(!right_top_exist())
                // {
                //     Ring_Over();
                //     break;
                // }

                if(!right_bottom_base_found())
                {
                    right_state4_5_clear_break_lost_cache();
                    ring_set_step(4);
                    continue;
                }

                key_anlysis2 = 0;
                key_anlysis3 = 0;
                break;
            }

            case 4:
            {
                uint8 break_row = 0;
                uint8 break_col = 0;

                // if(!right_top_exist())
                // {
                //     Ring_Over();
                //     break;
                // }

                if(!left_bottom_base_found())
                {
                    ring_set_step(5);
                    continue;
                }

                if(right_state4_5_apply_fixed_left_line_and_track_break(&break_row, &break_col))
                {
                    key_anlysis2 = break_row;
                    key_anlysis3 = break_col;
                }
                else
                {
                    key_anlysis2 = 0;
                    key_anlysis3 = 0;
                }
                break;
            }

            case 5:
            {
                // uint8 break_row = 0;
                // uint8 break_col = 0;

                // if(right_state4_5_apply_fixed_left_line_and_track_break(&break_row, &break_col))
                // {
                //     key_anlysis2 = break_row;
                //     key_anlysis3 = break_col;
                // }
                // else
                // {
                //     key_anlysis2 = 0;
                //     key_anlysis3 = 0;
                //     ring_set_step(6);
                //     continue;
                // }
                // break;

                uint8 break_row = 0;
                uint8 break_col = 0;

                if(right_state5_yaw_can_enter_state6())
                {
                    key_anlysis2 = 0;
                    key_anlysis3 = 0;
                    ring_set_step(6);
                    continue;
                }

                if(right_state4_5_apply_fixed_left_line_and_track_break(&break_row, &break_col))
                {
                    key_anlysis2 = break_row;
                    key_anlysis3 = break_col;
                }
                else
                {
                    key_anlysis2 = 0;
                    key_anlysis3 = 0;
                }
                break;
            }

            case 6:
                if(left_bottom_line_found_for_right_ring() && left_middle_missing())
                {
                    key_anlysis2 = 0;
                    key_anlysis3 = 0;
                    right_state6_clear_peak_cache();
                    ring_set_step(7);
                    continue;
                }
                break;

            case 7:
                if(!left_bottom_base_found() && !right_bottom_base_found())
                {
                    ring_set_step(8);
                    continue;
                }

                apply_right_state5_or_7_or_8();
                break;

            case 8:
                if(right_state8_yaw_can_enter_state9())
                {
                    key_anlysis2 = 0;
                    key_anlysis3 = 0;
                    ring_set_step(9);
                    continue;
                }

                key_anlysis2 = 0;
                key_anlysis3 = 0;
                apply_right_state5_or_7_or_8();
                break;

            case 9:
            {
                uint8 break_row = 0;
                uint8 break_col = 0;

                if(apply_right_state9(&break_row, &break_col))
                {
                    key_anlysis2 = break_row;
                    key_anlysis3 = break_col;

                    if(break_row > 30)
                    {
                        ring_set_step(10);
                    }
                }
                else
                {
                    key_anlysis2 = 0;
                    key_anlysis3 = 0;
                }
                break;
            }

            case 10:
            {
                uint8 break_row = 0;
                uint8 break_col = 0;

                if(apply_right_state10(&break_row, &break_col))
                {
                    key_anlysis2 = break_row;
                    key_anlysis3 = break_col;
                }
                else
                {
                    key_anlysis2 = 0;
                    key_anlysis3 = 0;
                }

                if(left_bottom_base_found() && right_bottom_base_found())
                {
                    ring_set_step(11);
                }
                break;
            }

            case 11:
                Ring_Over();
                break;

            default:
                Ring_Over();
                break;
        }
        break;
    }
}

void Ring(void)
{
    key_anlysis1 = current_step;

    if(current_step == 0)
    {
        if(is_cross_scene_for_ring_entry())
        {
            apply_cross_ring_suspect_repair();
        }
        else if(is_state1_scene())
        {
            current_ring_dir = 1;
            cross_ring_suspect_hold_seen = 0;
            ring_pre_repair_flag = 0;
            imu660_yaw_reset(0.0f);
            ring_set_step(1);
        }
        else if(is_right_state1_scene())
        {
            current_ring_dir = 2;
            cross_ring_suspect_hold_seen = 0;
            ring_pre_repair_flag = 0;
            imu660_yaw_reset(0.0f);
            ring_set_step(1);
        }
        else if(apply_cross_ring_suspect_repair())
        {
            // Hold this frame after suspect repair.
        }
        else
        {
            ring_pre_repair_flag = 0;
            key_anlysis2 = 0;
            key_anlysis3 = 0;
        }
    }

    if(current_step != 0)
    {
        if(current_ring_dir == 1)
        {
            run_left_ring_state_machine();
        }
        else if(current_ring_dir == 2)
        {
            run_right_ring_state_machine();
        }
        else
        {
            Ring_Over();
        }
    }

    key_anlysis1 = current_step;
    sync_ring_flags();
}

// 保留旧接口，内部直接复用新的状态 1 场景判断
uint8 Ring_Pre_Meet(void)
{
    return is_state1_scene();
}

uint8 Ring_Pre_Meet_use(void)
{
    return is_state1_scene();
}

// 保留旧接口，让原工程调用方式继续有效
void Ring_First_meeting(void)
{
    apply_state1();
}

void Ring_Enter(void)
{
    apply_state2();
}

void Ring_Turing(void)
{
    uint8 break_row = 0;
    uint8 break_col = 0;
    apply_state3_or_4(&break_row, &break_col);
}

void Ring_Ring_Ring(void)
{
    uint8 break_row = 0;
    uint8 break_col = 0;
    apply_state5(&break_row, &break_col);
}

void Ring_Out(void)
{
    apply_state7();
}

void Ring_Straighten(void)
{
    uint8 break_row = 0;
    uint8 break_col = 0;
    apply_state9_or_10(&break_row, &break_col);
}
