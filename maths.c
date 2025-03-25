#include "maths.h"
#include <stddef.h>
#include "list_of_function.h"
#include "math.h"

/*****************************************************************************
*函数名称:int Rms_voltage( int *arr,int data_len )
*入口参数:int *arr               //输入数据指针
*         int data_len           //数据长度
*返回值:  rms                    //均方根值
*函数功能:取均方根值
*****************************************************************************/
float Rms_voltage(fp32 arr, fp32 data_len)
{
	static float sum = 0, rms = 0;
	int i;
	for (i = 0; i < data_len; i++)
	{
		sum += arr * arr;         //计算平方和
	}
	rms = sqrt(sum / data_len);  //平方和取平均，再开方
	sum = 0;
	return rms;
}


float invSqrt(float x) //平方根倒数速算法
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/*取两个数最大值的绝对值*/
int16_t max_abs(int16_t x, int16_t y)
{
    if (user_abs(x) >= user_abs(y))
        return user_abs(x);
    else
        return user_abs(y);
}

/*
 *功能：正负循环限制（16位）
 *传入：1.输入值  2.限制幅度(正数)
 *传出：限幅输出值
 *描述：将输入值限制在 +-限制幅度 的范围内
 */
int16_t loop_restriction_int16(int16_t num, int16_t limit_num)
{
    if (user_abs(num) > limit_num)
    {
        if (num >= 0)
            num -= limit_num;
        else
            num += limit_num;
    }
    return num;
}

/*
 *功能：正负循环限制（float）
 *传入：1.输入值  2.限制幅度(正数)
 *传出：限幅输出值
 *描述：将输入值限制在 +-限制幅度 的范围内
 */
float loop_restriction_float(float num, float limit_num)
{
    if (user_abs(num) > limit_num)
    {
        if (num >= 0)
            num -= limit_num;
        else
            num += limit_num;
    }
    return num;
}

/*循环限幅32*/
float loop_fp32_constrain(float Input, float minValue, float maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        float len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        float len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

//斜坡函数(加速度限制)
void data_accelerated_control(float *input, float acc)
{
    static int16_t last_num = 0;
    int16_t temp;
    temp = *input - last_num;

    if (user_abs(temp) > acc)
        *input = last_num + temp / user_abs(temp) * acc;

    last_num = *input;
}

// 限幅滤波函数
float limiting_filter(float new_value, float last_value, float delat_max)
{
    if ((new_value - last_value > delat_max) || (last_value - new_value > delat_max))
        return last_value;
    return new_value;
}

// sin函数查表法
float sin_calculate(float angle)
{
    float sin_angle;

    if (angle >= 0.0f && angle < 90.0f)
        sin_angle = (Trigonometric_Functions[(int)(user_abs(angle) * 10.0f)] / 100.0f);
    else if (angle >= 90.0f && angle < 180.f)
        sin_angle = (Trigonometric_Functions[(int)(user_abs(180.0f - angle) * 10.0f)] / 100.0f);
    else if (angle >= -180.0f && angle < -90.f)
        sin_angle = -(Trigonometric_Functions[(int)(user_abs(180.0f + angle) * 10.0f)] / 100.0f);
    else if (angle >= -90.0f && angle < 0.f)
        sin_angle = -(Trigonometric_Functions[(int)(user_abs(180.0f - (180.0f + angle)) * 10.0f)] / 100.0f);
    else if (angle == 180.f)
        sin_angle = 0.0f;

    return sin_angle;
}

// cos函数查表法
float cos_calculate(float angle)
{
    float cos_angle;

    angle = user_abs(angle);

    if (angle >= 0.0f && angle < 90.0f)
        cos_angle = (Trigonometric_Functions[(int)(user_abs(90.0f - angle) * 10.0f)] / 100.0f);
    else if (angle >= 90.0f && angle < 180.f)
        cos_angle = -(Trigonometric_Functions[(int)(user_abs(angle - 90.0f) * 10.0f)] / 100.0f);
    else if (angle == 180.f)
        cos_angle = -1.0f;

    return cos_angle;
}

float float_min_distance(float target, float actual, float minValue, float maxValue)
{
	if (maxValue < minValue)
    {
        return 0;
    }
	
	target = loop_fp32_constrain(target, minValue,maxValue);
	
	if (user_abs(target - actual) > (maxValue - minValue) / 2.0f)
	{
		if(maxValue - actual < (maxValue - minValue) / 2.0f)
		{
			return maxValue - actual + target - minValue;
		}
		else
		{
			return minValue - actual + target - maxValue;
		}
	}
	else 
	{
		return target - actual;
	}
	
}
