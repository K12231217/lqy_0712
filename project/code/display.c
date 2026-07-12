#include "display.h"

#if DISPLAY_IPS200PRO_DEMO_ENABLE
uint32 ips200pro_stack_size;

// 保存创建的页面ID，当需要对页面进行操作的时候则需要填写此ID
uint16 page_id[3];

uint16 camera_id;

ips200pro_image_line_uint8_struct border1[MT9V03X_H];
ips200pro_image_line_uint8_struct border2[MT9V03X_H];
uint8 i = 0;

uint16 rectangle_x = 0;
uint8  temp_start = 10;


void display_init_pro(void)
{

    // 初始化屏幕并创建一个页面
    page_id[0] = ips200pro_init("\xD7\xDC\xD7\xEA\xB7\xE7\xB2\xE2\xCA\xD4", IPS200PRO_TITLE_BOTTOM, 30);

    // 获取屏幕信息，根据个人需求调用 1064主板屏幕接口没有连接芯片的miso，所以无法使用获取的功能
    ips200pro_get_information(&ips200pro_information);
    ips200pro_get_free_stack_size(&ips200pro_stack_size);
    ips200pro_get_time(&ips200pro_time);

    // 设置全局字体为16号
    ips200pro_set_default_font(FONT_SIZE_16);

    // 屏幕设置为竖屏
    ips200pro_set_direction(IPS200PRO_PORTRAIT);

    // 设置编码格式
    ips200pro_set_format(IPS200PRO_FORMAT_GBK);

    // 设置选中页面的标题背景颜色，填写任意一个创建完成的页面ID即可
    ips200pro_set_color(page_id[0], COLOR_PAGE_SELECTED_BG, IPS200PRO_RGB888_TO_RGB565(0xFF, 0xE3, 0xE3));

    // 设置第一个页面的背景色
    ips200pro_set_color(page_id[0], COLOR_BACKGROUND, IPS200PRO_RGB888_TO_RGB565(0x43, 0x4A, 0x61));

    // 设置页面标题文字颜色，填写任意一个创建完成的页面ID即可
    ips200pro_set_color(page_id[0], COLOR_FOREGROUND, RGB565_BLACK);

    // 设置页面选中后的标题文字颜色，填写任意一个创建完成的页面ID即可
    ips200pro_set_color(page_id[0], COLOR_PAGE_SELECTED_TEXT, RGB565_WHITE);

    // 设置屏幕背光亮度，可以设置范围0-255
    ips200pro_set_backlight(255);

    // 切换页面，切换到第一个页面
    ips200pro_page_switch(page_id[0] ,PAGE_ANIM_ON);

    // 创建一个图像控件，图像控件没有任何的颜色属性设置
    camera_id = ips200pro_image_create(26, 20, MT9V03X_W, MT9V03X_H);

//后面是放在while里的flag之后的参考

//           // 重新准备边线数据，模拟每次找到的边界数据
//            for(i = 0; MT9V03X_H > i; i++)
//            {
//                // 计算左边界
//                border1[i].x = (MT9V03X_H - i) * temp_start / MT9V03X_H;
//                border1[i].y = (uint8)i;

//                // 计算右边界
//                border2[i].x = MT9V03X_W - ((MT9V03X_H - i) * temp_start / MT9V03X_H);
//                border2[i].y = (uint8)i;
//            }
//            temp_start++;
//            if(90 < temp_start)
//            {
//                temp_start = 10;
//            }


//            // 发送边线数据 务必先发送边线后发送图像 最多叠加10条边线
//            // 这里仅仅只是示意，当边线数据替换为每次搜索出来的边线，这样就可以非常方便的将边线叠加在原始图像上，从而使得调试效率增加
//            // IMAGE_LINE_TYPE_UINT8表示border1结构体是ips200pro_image_line_uint8_struct类型，表示每个元素都是8bit
//            // IMAGE_LINE_TYPE_UINT16表示border1结构体是ips200pro_image_line_uint16_struct类型，表示每个元素都是16bit
//            // 如果保存的边界数据会大于255(即坐标数据X或Y任意一个会大于255)，则应该使用ips200pro_image_line_uint16_struct进行定义
//            ips200pro_image_draw_line(camera_id, 1, border1, MT9V03X_H, IMAGE_LINE_TYPE_UINT8, RGB565_BLUE);
//            ips200pro_image_draw_line(camera_id, 2, border2, MT9V03X_H, IMAGE_LINE_TYPE_UINT8, RGB565_GREEN);

//            rectangle_x++;
//            if((MT9V03X_W - 30) < rectangle_x)
//            {
//                rectangle_x = 0;
//            }
//            // 发送矩形框数据 务必先发送矩形框后发送图像 最多叠加5个矩形框显示  可用于目标检测时快速框出对应物体
//            ips200pro_image_draw_rectangle(camera_id, 1, rectangle_x, 10, 30, 20, RGB565_RED);

//            // 显示图像 灰度模式 不启用二值化，发送图像数据给屏幕之后，屏幕会进入忙状态，导致发送其他数据给屏幕会出现10ms以上的等待
//            // 通常不仅要显示图像，还需要显示各种字符串等信息，因此在显示多种信息时为了保证屏幕的流畅性，建议先发送其他数据，图像数据最后再发送，图像数据发送之后等待10ms后再发送其他数据
//            ips200pro_image_display(camera_id, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, IMAGE_GRAYSCALE, 0);

//            // 如果需要测试二值化效果则打开下方语句，将上方的语句注释
//            // 显示图像 灰度模式并启用二值化，二值化阈值80
//            //ips200pro_image_display(camera_id, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, IMAGE_GRAYSCALE, 80);

}

#endif

void display_init(void)
{
	ips200_init();
}

void isp_200_draw_col(uint8 col ,uint8 h,uint8 start,uint16 color)
{
	uint8 i=0;
	for(i=start;i<start+h;i++){
		ips200_draw_point(col,i,color);
	}
}
void isp_200_draw_row(uint8 row ,uint8 w,uint8 start,uint16 color)
{
	uint8 i=0;
	for(i=start;i<start+w;i++){
		ips200_draw_point(i,row,color);
	}
}

void display_motor(void)
{
//	ips200_show_string(0,0,"Encoder L R:");
//	ips200_show_int16(100,0,encoder_data_l);
//	ips200_show_int16(164,0,encoder_data_r);
//	
//	ips200_show_string(0,32,"tar_speed:");
//	ips200_show_int16(80,32,tar_speed);
//	
//	ips200_show_string(0,64,"L Kp Ki:");
//	ips200_show_int16(80,64,pi_l.kp_q);
//	ips200_show_int16(128,64,pi_l.ki_q);
//	
//	ips200_show_string(0,96,"R Kp Ki:");
//	ips200_show_int16(80,96,pi_r.kp_q);
//	ips200_show_int16(128,96,pi_r.ki_q);	
}




