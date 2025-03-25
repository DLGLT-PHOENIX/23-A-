#ifndef __FILTER_H__
#define __FILTER_H__
#include "struct_typedef.h"
//#include "arm_math.h"


//������ֵ�˲����������㣩
typedef __packed struct
{
    fp32 Input;        //��ǰȡ��ֵ
    int32_t count_num; //ȡ������
    fp32 Output;       //�˲����
    fp32 Sum;          //�ۼ��ܺ�
    fp32 FIFO[250];    //����
    int32_t sum_flag;  //�Ѿ���250����־
} sliding_mean_filter_type_t;

//һ�׵�ͨ�˲�����
typedef __packed struct
{
    fp32 input;      //��������
    fp32 last_input; //�ϴ�����
    fp32 out;        //�˲����������
    fp32 num;        //�˲�����
} first_order_filter_type_t;

typedef enum 
{
    BIQUAD_LOWPASS,       //��ͨ�˲�
    BIQUAD_HIGHPASS,      //��ͨ�˲�
    BIQUAD_BANDPASS_PEAK, //��ͨ�˲�
} sample_t;               //��ö�����ڶ�������˲���������



typedef struct     //PID��ʼ��������ÿ��һ��PID��Ҫ���¶���һ�������ơ�
{
  float wc;
	float alpha;
	float b0;
	float b1;
	float b2;
	float	a0;
	float a1;
	float a2;
	float Input;          //����
	float Last_IN;        //��һ������
	float Last_two_IN;    //����������
	float	Output;         //���
	float Last_OUT;       //��һ�����
	float Last_two_OUT;		//���������
	
} Second_order_Filter;

/* ��ͨ�˲� */
extern float first_order_filter(first_order_filter_type_t *first_order_filter_type, fp32 input);
extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 num);

/* ƽ���˲� */
float sliding_mean_filter(sliding_mean_filter_type_t *mean_filter, fp32 Input, int32_t num); //��ֵ�����˲�
void sliding_mean_filter_init(sliding_mean_filter_type_t *mean_filter);                      //��ֵ�����˲���ʼ�����ɲ��ã�ֱ�Ӷ���ṹ��ʱ����ֵ��

void Second_order_Filter_Init(Second_order_Filter* Biquad_filter, uint16_t frequency, uint16_t sampleRate, float Q, uint8_t type);
float Second_order_filter(Second_order_Filter* Biquad_filter, int ADC_Value);

#endif
