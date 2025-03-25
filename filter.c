#include "filter.h"
#include "struct_typedef.h"

//#define PI					3.14159265358979f
// һ�׵�ͨ�˲�����
float first_order_filter(first_order_filter_type_t *first_order_filter_type, fp32 input)
{
  first_order_filter_type->input = input;
  first_order_filter_type->out = first_order_filter_type->input * first_order_filter_type->num + (1 - first_order_filter_type->num) * first_order_filter_type->last_input;
  first_order_filter_type->last_input = first_order_filter_type->out;

  return first_order_filter_type->out;
}

// һ�׵�ͨ�˲���ʼ��
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
 *���ܣ�������ֵ�˲�������ʼ��(������)
 *���룺�˲�����ṹ��
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
 *���ܣ�������ֵ�˲��������ͣ�------����С���ȸ�Ƶ����
 *���룺1.�˲�����ṹ��  2.����ֵ 3.��ֵ����
 *�����������˲����ֵ��250�Σ�
 */
float sliding_mean_filter(sliding_mean_filter_type_t *mean_filter, float Input, int num)
{
  // ����
  mean_filter->Input = Input;
  mean_filter->FIFO[mean_filter->count_num] = mean_filter->Input;
  mean_filter->count_num++;

  if (mean_filter->count_num == num)
  {
    mean_filter->count_num = 0;
    mean_filter->sum_flag = 1;
  }
  // ���
  if (mean_filter->sum_flag == 1)
  {
    for (int count = 0; count < num; count++)
    {
      mean_filter->Sum += mean_filter->FIFO[count];
    }
  }
  // ��ֵ
  mean_filter->Output = mean_filter->Sum / num;
  mean_filter->Sum = 0;

  return mean_filter->Output;
}

/*************************************************************************************************************
*��������:void Second_order_Filter_Init(Second_order_Filter *Biquad_filter,u16 frequency,u16 sampleRate,float Q,u8 type)    //���׵�ͨ�˲�������ʼ��
*��ڲ���:Second_order_Filter    ����ָ���ַ
*        u16 frequency           ���ĵ�Ƶ�ʣ�����Ƶ�ʣ�
*        u16 sampleRate          ����Ƶ��
*        float Q                 Ʒ������
*        u8 type                 ͨ������ѡ��
*��������:���׵�ͨ�˲�������ʼ��
*************************************************************************************************************/

//Second_order_Filter Sin;
//Second_order_Filter Cos;
//Second_order_Filter AC_DC_V;
//Second_order_Filter DC_DC_V;
//Second_order_Filter DC_DC_I;
//void Second_order_Filter_Init(Second_order_Filter* Biquad_filter, uint16_t frequency, uint16_t sampleRate, float Q, uint8_t type)    //���׵�ͨ�˲�������ʼ��
//{
//	Biquad_filter->wc = 2 * PI * frequency / sampleRate;
//	Biquad_filter->alpha = arm_sin_f32(Biquad_filter->wc) / (2 * Q);
//	switch (type)
//	{
//	case BIQUAD_LOWPASS:    //��ͨ�˲�������

//		Biquad_filter->b0 = (1 - arm_cos_f32(Biquad_filter->wc)) / 2;
//		Biquad_filter->b1 = Biquad_filter->b0 * 2;
//		Biquad_filter->b2 = Biquad_filter->b0;
//		Biquad_filter->a0 = 1 + Biquad_filter->alpha;
//		Biquad_filter->a1 = -2 * arm_cos_f32(Biquad_filter->wc);
//		Biquad_filter->a2 = 1 - Biquad_filter->alpha;
//		break;

//	case BIQUAD_HIGHPASS:    //��ͨ�˲�������

//		Biquad_filter->b0 = (1 + arm_cos_f32(Biquad_filter->wc)) / 2;
//		Biquad_filter->b1 = -Biquad_filter->b0 * 2;
//		Biquad_filter->b2 = Biquad_filter->b0;
//		Biquad_filter->a0 = 1 + Biquad_filter->alpha;
//		Biquad_filter->a1 = -2 * arm_cos_f32(Biquad_filter->wc);
//		Biquad_filter->a2 = 1 - Biquad_filter->alpha;
//		break;

//	case BIQUAD_BANDPASS_PEAK: //��ͨ�˲�������

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
*��������:u16 Second_order_filter(Second_order_Filter *Biquad_filter,u8 Channel)   // ���׵�ͨ�˲���
*��ڲ���:Second_order_Filter  //����ָ���ַ
*        ADC_Value            //�����ѹ
*��������:���׵�ͨ�˲�
*����˵��:Y(n)=[b0X(n)+2b0X(n-1)+b0X(n-2)-a1Y(n-1)-a2Y(n-2)]/a0
*				 X(n)Ϊ���β���ֵ
*				 X(n-1)Ϊ��һ����ֵ
*				 X(n-2)Ϊ��������ֵ 
*				 Y(n)Ϊ�����˲����ֵ
*				 Y(n-1)Ϊ��һ���˲����ֵ
*				 Y(n-2)Ϊ�������˲����ֵ
*************************************************************************************************************/

//float Second_order_filter(pll_t  *Biquad_filter, int ADC_Value)   // ���׵�ͨ�˲���
//{
//	Biquad_filter->Input = ADC_Value;	    //��ȡADC�ɼ�ֵ
//	Biquad_filter->Output = Biquad_filter->b0 * Biquad_filter->Input + Biquad_filter->b1
//		* Biquad_filter->Last_IN + Biquad_filter->b2 * Biquad_filter->Last_two_IN - Biquad_filter->a1
//		* Biquad_filter->Last_OUT - Biquad_filter->a2 * Biquad_filter->Last_two_OUT;
	//�����˲����
//	Biquad_filter->Last_two_OUT = Biquad_filter->Last_OUT;
//	Biquad_filter->Last_OUT = Biquad_filter->Output;
//	Biquad_filter->Last_two_IN = Biquad_filter->Last_IN;
//	Biquad_filter->Last_IN = Biquad_filter->Input;
//	return Biquad_filter->Output;      //����˲����
//}
