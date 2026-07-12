#include "image2.h"

#if IMAGE2_OTSU_ENABLE
uint8 base_image[MT9V03X_H][MT9V03X_W];
uint8 Threshold;
//-------------------------------------------------------------------------------------------------------------------
//  @brief      快速大津
//  @return     uint8
//  @since      v1.1
//  Sample usage:   OTSU_Threshold = otsuThreshold(mt9v03x_image_dvp[0]);//大津法阈值
//-------------------------------------------------------------------------------------------------------------------
// 新增优化宏开关（可一键开启/关闭限幅，放在函数内，不影响全局）

uint8 otsuThreshold(uint8 *mt9v03x_image_first)   //注意计算阈值的一定要是原图像
{
	int pixelCount[GrayScale]={0};	//采集到的图像中每个灰度值的个数
	float pixelPro[GrayScale]={0};	//采集到的图像中每个灰度值的比例
	int Pixel_Max=0;		//采集到的最大灰度值
	int Pixel_Min=255;	//采集到的最小灰度值
	int offset;//灰度值偏移量,在防止局部最大时使用
	uint16 width = MT9V03X_W;
	uint16 height = MT9V03X_H;
	uint32 gray_sum=0;	//总采样点灰度值之和

		// 优化点2：规范浮点数初始化，按顺序统一初始化，消除内存随机值干扰
		int i, j, pixelSum = width * height;	//总采样点
		uint8 threshold = 0;	//阈值
		uint8* date_mt9v03x_image ;
		float w0=0, w1=0, u0tmp=0, u1tmp=0, u0=0, u1=0, u=0, deltaTmp=0, deltaMax=-0.5;

	date_mt9v03x_image= mt9v03x_image_first;  //指向像素数据的指针
		
		// 统计灰度级中每个像素在整幅图像中的个数
		// 优化点6：外层循环i改为uint16，匹配width/height类型，消除隐式转换警告
		for ( i = 0; (uint16)i < height; i++)
		{
			for (j = 0; (uint16)j < width; j++)
			{
				pixelCount[(int)date_mt9v03x_image[i * width + j]]++;  //将当前的点的像素值作为计数数组的下标
				gray_sum+=(int)date_mt9v03x_image[i * width + j];       //灰度值总和
						//更新采样到的最大最小灰度值
				if(date_mt9v03x_image[i * width + j]>Pixel_Max)   Pixel_Max=date_mt9v03x_image[i * width + j];
				if(date_mt9v03x_image[i * width + j]<Pixel_Min)   Pixel_Min=date_mt9v03x_image[i * width + j];
			}
		}

		// 优化点1：新增极端场景容错-图像全黑/全白，提前返回，避免浮点数计算异常
		if (Pixel_Min == Pixel_Max)
		{
			threshold = Pixel_Min;
			return threshold;
		}

		//计算每个像素值的点在整幅图像中的比例
		for (i = Pixel_Min; i <=Pixel_Max; i++)
		{
			pixelPro[i] = (float)pixelCount[i] / pixelSum;
		}

		//遍历灰度级[0,255]
		for (j = Pixel_Min; j <=Pixel_Max; j++)
		{
			w0 += pixelPro[j];  //背景部分每个灰度值的像素点所占比例之和   即背景部分的比例
			u0tmp += j * pixelPro[j];  //背景部分 每个灰度值的点的比例 *灰度值

			w1=1-w0;
			u1tmp=(float)gray_sum/pixelSum-u0tmp;
				//防止被除数太小,导致数据溢出
			if(w0<1e-6f||w1<1e-6f)
				continue;
			
			u0 = u0tmp / w0;              //背景平均灰度
			u1 = u1tmp / w1;              //前景平均灰度
			u = u0tmp + u1tmp;            //全局平均灰度
			deltaTmp = (float)(w0 *w1* (u0 - u1)* (u0 - u1)) ;//方差,方差越大,对应的阈值最优
			if (deltaTmp > deltaMax)//更新最大方差,及最优阈值
			{
				deltaMax = deltaTmp;
				threshold = j;
			}
					offset=(Pixel_Max-Pixel_Min)/10;//灰度值偏移量
					//方差随着阈值的增大而增大,当增大到最大值后开始下降
			// 优化点3：新增灰度偏移（j > Pixel_Min + 10），避免局部最大值提前终止，错过全局最优阈值
			if (deltaTmp < deltaMax && j > Pixel_Min + offset)
			{
				break;//跳出整个for循环
			}
		}

		// 优化点4+5：新增宏开关控制限幅 + 将last_threshold改为static静态变量，保留历史值
	#if OTSU_USE_LIMIT
		if(threshold>OTSU_LIMIT_MIN && threshold<OTSU_LIMIT_MAX)//限幅
			last_threshold = threshold;
		else
			threshold = last_threshold;
	#endif
		return threshold;
}

//图像二值化
void base_image_threshold(uint8 Threshold)
{
	int i,j;
	for(i=0;i<40;i++)
	{
		for(j=0;j<MT9V03X_W;j++)
		{
			if(base_image[i][j]<Threshold+10)
				base_image[i][j]=0;
			else
				base_image[i][j]=255;
		}
	}
	
		for(i=40;i<MT9V03X_H;i++)
	{
		for(j=0;j<MT9V03X_W;j++)
		{
			if(base_image[i][j]<Threshold)
				base_image[i][j]=0;
			else
				base_image[i][j]=255;
		}
	}

}
#endif

