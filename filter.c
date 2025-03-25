#include "filter.h"
#include "struct_typedef.h"

//#define PI					3.14159265358979f
// 一阶低通滤波计算
float first_order_filter(first_order_filter_type_t *first_order_filter_type, fp32 input)
{
  first_order_filter_type->input = input;
  first_order_filter_type->out = first_order_filter_type->input * first_order_filter_type->num + (1 - first_order_filter_type->num) * first_order_filter_type->last_input;
  first_order_filter_type->last_input = first_order_filter_type->out;

  return first_order_filter_type->out;
}

// 一阶低通滤波初始化
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 num)
{
  if (first_order_filter_type == 0)
  {
    return;
  }

  first_order_filter_type->input = 0;
  first_order_filter_type->last_input = 0;
  first_order_filter_type->num = num;
  first_order_filter_type->out = 0;
}

/*
 *功能：滑动均值滤波参数初始化(浮点型)
 *输入：滤波对象结构体
 */
void sliding_mean_filter_init(sliding_mean_filter_type_t *mean_filter)
{
  mean_filter->count_num = 0;
  for (int i = 0; i < 20; i++)
    mean_filter->FIFO[i] = 0.0f;
  mean_filter->Input = 0.0f;
  mean_filter->Output = 0.0f;
  mean_filter->Sum = 0.0f;
  mean_filter->sum_flag = 0;
}

/*
 *功能：滑动均值滤波（浮点型）------抑制小幅度高频噪声
 *传入：1.滤波对象结构体  2.更新值 3.均值数量
 *传出：滑动滤波输出值（250次）
 */
float sliding_mean_filter(sliding_mean_filter_type_t *mean_filter, float Input, int num)
{
  // 更新
  mean_filter->Input = Input;
  mean_filter->FIFO[mean_filter->count_num] = mean_filter->Input;
  mean_filter->count_num++;

  if (mean_filter->count_num == num)
  {
    mean_filter->count_num = 0;
    mean_filter->sum_flag = 1;
  }
  // 求和
  if (mean_filter->sum_flag == 1)
  {
    for (int count = 0; count < num; count++)
    {
      mean_filter->Sum += mean_filter->FIFO[count];
    }
  }
  // 均值
  mean_filter->Output = mean_filter->Sum / num;
  mean_filter->Sum = 0;

  return mean_filter->Output;
}

/*************************************************************************************************************
*函数名称:void Second_order_Filter_Init(Second_order_Filter *Biquad_filter,u16 frequency,u16 sampleRate,float Q,u8 type)    //二阶低通滤波参数初始化
*入口参数:Second_order_Filter    参数指针地址
*        u16 frequency           中心点频率（截至频率）
*        u16 sampleRate          采样频率
*        float Q                 品质因数
*        u8 type                 通带类型选择
*函数功能:二阶低通滤波参数初始化
*************************************************************************************************************/

//Second_order_Filter Sin;
//Second_order_Filter Cos;
//Second_order_Filter AC_DC_V;
//Second_order_Filter DC_DC_V;
//Second_order_Filter DC_DC_I;
//void Second_order_Filter_Init(Second_order_Filter* Biquad_filter, uint16_t frequency, uint16_t sampleRate, float Q, uint8_t type)    //二阶低通滤波参数初始化
//{
//	Biquad_filter->wc = 2 * PI * frequency / sampleRate;
//	Biquad_filter->alpha = arm_sin_f32(Biquad_filter->wc) / (2 * Q);
//	switch (type)
//	{
//	case BIQUAD_LOWPASS:    //低通滤波器参数

//		Biquad_filter->b0 = (1 - arm_cos_f32(Biquad_filter->wc)) / 2;
//		Biquad_filter->b1 = Biquad_filter->b0 * 2;
//		Biquad_filter->b2 = Biquad_filter->b0;
//		Biquad_filter->a0 = 1 + Biquad_filter->alpha;
//		Biquad_filter->a1 = -2 * arm_cos_f32(Biquad_filter->wc);
//		Biquad_filter->a2 = 1 - Biquad_filter->alpha;
//		break;

//	case BIQUAD_HIGHPASS:    //高通滤波器参数

//		Biquad_filter->b0 = (1 + arm_cos_f32(Biquad_filter->wc)) / 2;
//		Biquad_filter->b1 = -Biquad_filter->b0 * 2;
//		Biquad_filter->b2 = Biquad_filter->b0;
//		Biquad_filter->a0 = 1 + Biquad_filter->alpha;
//		Biquad_filter->a1 = -2 * arm_cos_f32(Biquad_filter->wc);
//		Biquad_filter->a2 = 1 - Biquad_filter->alpha;
//		break;

//	case BIQUAD_BANDPASS_PEAK: //带通滤波器参数

//		Biquad_filter->b0 = Biquad_filter->alpha * Q;
//		Biquad_filter->b1 = 0;
//		Biquad_filter->b2 = -Biquad_filter->alpha * Q;
//		Biquad_filter->a0 = 1 + Biquad_filter->alpha;
//		Biquad_filter->a1 = -2 * arm_cos_f32(Biquad_filter->wc);
//		Biquad_filter->a2 = 1 - Biquad_filter->alpha;
//		break;
//	}

//	Biquad_filter->b0 = Biquad_filter->b0 / Biquad_filter->a0;
//	Biquad_filter->b1 = Biquad_filter->b1 / Biquad_filter->a0;
//	Biquad_filter->b2 = Biquad_filter->b2 / Biquad_filter->a0;
//	Biquad_filter->a1 = Biquad_filter->a1 / Biquad_filter->a0;
//	Biquad_filter->a2 = Biquad_filter->a2 / Biquad_filter->a0;


//}
/*************************************************************************************************************
*函数名称:u16 Second_order_filter(Second_order_Filter *Biquad_filter,u8 Channel)   // 二阶低通滤波器
*入口参数:Second_order_Filter  //参数指针地址
*        ADC_Value            //输入电压
*函数功能:二阶低通滤波
*函数说明:Y(n)=[b0X(n)+2b0X(n-1)+b0X(n-2)-a1Y(n-1)-a2Y(n-2)]/a0
*				 X(n)为本次采样值
*				 X(n-1)为上一采样值
*				 X(n-2)为上两采样值 
*				 Y(n)为本次滤波输出值
*				 Y(n-1)为上一次滤波输出值
*				 Y(n-2)为上两次滤波输出值
*************************************************************************************************************/

//float Second_order_filter(pll_t  *Biquad_filter, int ADC_Value)   // 二阶低通滤波器
//{
//	Biquad_filter->Input = ADC_Value;	    //获取ADC采集值
//	Biquad_filter->Output = Biquad_filter->b0 * Biquad_filter->Input + Biquad_filter->b1
//		* Biquad_filter->Last_IN + Biquad_filter->b2 * Biquad_filter->Last_two_IN - Biquad_filter->a1
//		* Biquad_filter->Last_OUT - Biquad_filter->a2 * Biquad_filter->Last_two_OUT;
	//储存滤波结果
//	Biquad_filter->Last_two_OUT = Biquad_filter->Last_OUT;
//	Biquad_filter->Last_OUT = Biquad_filter->Output;
//	Biquad_filter->Last_two_IN = Biquad_filter->Last_IN;
//	Biquad_filter->Last_IN = Biquad_filter->Input;
//	return Biquad_filter->Output;      //输出滤波结果
//}
