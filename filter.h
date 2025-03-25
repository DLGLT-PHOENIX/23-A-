#ifndef __FILTER_H__
#define __FILTER_H__
#include "struct_typedef.h"
//#include "arm_math.h"


//滑动均值滤波参数（浮点）
typedef __packed struct
{
    fp32 Input;        //当前取样值
    int32_t count_num; //取样次数
    fp32 Output;       //滤波输出
    fp32 Sum;          //累计总和
    fp32 FIFO[250];    //队列
    int32_t sum_flag;  //已经够250个标志
} sliding_mean_filter_type_t;

//一阶低通滤波参数
typedef __packed struct
{
    fp32 input;      //输入数据
    fp32 last_input; //上次数据
    fp32 out;        //滤波输出的数据
    fp32 num;        //滤波参数
} first_order_filter_type_t;

typedef enum 
{
    BIQUAD_LOWPASS,       //低通滤波
    BIQUAD_HIGHPASS,      //高通滤波
    BIQUAD_BANDPASS_PEAK, //带通滤波
} sample_t;               //此枚举用于定义二阶滤波器的类型



typedef struct     //PID初始化参数，每次一个PID都要重新定义一个新名称。
{
  float wc;
	float alpha;
	float b0;
	float b1;
	float b2;
	float	a0;
	float a1;
	float a2;
	float Input;          //输入
	float Last_IN;        //上一次输入
	float Last_two_IN;    //上两次输入
	float	Output;         //输出
	float Last_OUT;       //上一次输出
	float Last_two_OUT;		//上两次输出
	
} Second_order_Filter;

/* 低通滤波 */
extern float first_order_filter(first_order_filter_type_t *first_order_filter_type, fp32 input);
extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 num);

/* 平滑滤波 */
float sliding_mean_filter(sliding_mean_filter_type_t *mean_filter, fp32 Input, int32_t num); //均值滑窗滤波
void sliding_mean_filter_init(sliding_mean_filter_type_t *mean_filter);                      //均值滑窗滤波初始化（可不用，直接定义结构体时给初值）

void Second_order_Filter_Init(Second_order_Filter* Biquad_filter, uint16_t frequency, uint16_t sampleRate, float Q, uint8_t type);
float Second_order_filter(Second_order_Filter* Biquad_filter, int ADC_Value);

#endif
