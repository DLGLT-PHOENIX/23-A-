#include "docal.h"
#include "adc.h"
#include "maths.h"

sogi_t Sin;
sogi_t Cos;
#define PI 3.1415296
#define Fx 20000//载波频率
#define F0 50   //基波频率
pll_t PLL;

extern vu16 AD_Value[ADC_SAMPLE_PNUM][ADC_SAMPLE_CNUM];
extern float M;

void V_for_current(detection_volume_t *Vout,detection_volume_t *cur,fp32 setVout, fp32 actual_Vout, fp32 actual_cur)
{
	
    //Vout->current_ture=double_loop(&cur->current_pid,&Vout->Vout_pid,setVout,actual_Vout,actual_cur);
    M=(float)double_loop(&cur->current_pid,&Vout->Vout_pid,setVout,actual_Vout,actual_cur);
}

void AC_V_detect(detection_volume_t *AC_V)//测量计算交流电压有效值
{
    AC_V->DCvalue = AD_Value[0][0]+AD_Value[1][0];
    AC_V->sum += AC_V->DCvalue;
    AC_V->ACvalue = ADC_Value(1);
    AC_V->ave=AC_V->sum/0.02;
    AC_V->ACvalue = AC_V->DCvalue-AC_V->ave;
    AC_V->rms=Rms_voltage(AC_V->ACvalue,0.02);
}

void PPL(pll_t *pll,fp32 alpha,fp32 beta,fp32 V_web)
{
    PidInit(&pll->ppl_pid,1,0.1,0,Output_Limit);
    PidInitMode(&pll->ppl_pid,Output_Limit,400,0);
    PidInitMode(&pll->ppl_pid,Integral_Limit,400,0);
    SOGI_Init(&Sin, 1);                //二阶广义积分参数初始化
	SOGI_Init(&Cos, 0);                //二阶广义积分参数初始化


    static float PLLSinAngle = 0;
	static float PLLLoopOut;

    static float Theta = 0;
	unsigned long Ratio = 0;
	Ratio = (Fx) / (F0);
    ClarkTransform(pll,Single_Phase);
	ParkTransform(pll);

    PLLLoopOut = Incremental_PID(&pll->ppl_pid,pll->VoltD, 0);
	PLLSinAngle += (PLLLoopOut / Ratio);
	PLLSinAngle = PLLSinAngle;
	if (PLLSinAngle > (2 * PI))
	{
		PLLSinAngle = PLLSinAngle - (2 * PI);
	}
	else if (PLLSinAngle < 0)
	{
		PLLSinAngle = PLLSinAngle + (2 * PI);
	}
	Theta = PLLSinAngle;
	//pll->sinwt = arm_sin_f32(Theta);
	//pll->coswt = arm_cos_f32(Theta);
	//return Theta;


}

/*************************************************************************************************************
*函数名称:void SOGI_Init(Second_order_Filter *Biquad_filter,u8 mode)    //二阶广义积分器参数初始化
*入口参数:Second_order_Filter    参数指针地址
*         u8 mode      当mode=1时不移相，mode=0时移相PI/2
*函数功能:二阶广义积分器参数初始化
*************************************************************************************************************/	
void SOGI_Init(sogi_t* Biquad_filter, uint8_t mode)
{

	Biquad_filter->wc = (2 * PI * F0) / Fx;

	switch (mode)
	{
	case 1:  //不移相

		Biquad_filter->b0 = 2 * Biquad_filter->wc;
		Biquad_filter->b1 = 0;
		Biquad_filter->b2 = -2 * Biquad_filter->wc;
		Biquad_filter->a0 = 4 + 2 * Biquad_filter->wc + Biquad_filter->wc * Biquad_filter->wc;
		Biquad_filter->a1 = 2 * Biquad_filter->wc * Biquad_filter->wc - 8;
		Biquad_filter->a2 = 4 + Biquad_filter->wc * Biquad_filter->wc - 2 * Biquad_filter->wc;

		break;

	case 0:   //移相 

		Biquad_filter->b0 = Biquad_filter->wc * Biquad_filter->wc;
		Biquad_filter->b1 = 2 * Biquad_filter->wc * Biquad_filter->wc;
		Biquad_filter->b2 = Biquad_filter->wc * Biquad_filter->wc;
		Biquad_filter->a0 = 4 + 2 * Biquad_filter->wc + Biquad_filter->wc * Biquad_filter->wc;
		Biquad_filter->a1 = 2 * Biquad_filter->wc * Biquad_filter->wc - 8;
		Biquad_filter->a2 = Biquad_filter->wc * Biquad_filter->wc - 2 * Biquad_filter->wc + 4;


		break;
	}



	Biquad_filter->b0 = Biquad_filter->b0 / Biquad_filter->a0;
	Biquad_filter->b1 = Biquad_filter->b1 / Biquad_filter->a0;
	Biquad_filter->b2 = Biquad_filter->b2 / Biquad_filter->a0;
	Biquad_filter->a1 = Biquad_filter->a1 / Biquad_filter->a0;
	Biquad_filter->a2 = Biquad_filter->a2 / Biquad_filter->a0;
}

//unsigned long VoltageAIndex=0,VoltageBIndex=0;
/*************************************************************************************************************
*函数名称:ClarkTransform(ST_AC_INPUT *stAc, ST_CLARK *stClack,u8 type)
*入口参数:ST_AC_INPUT *stAc    //输入量指针
*			  	ST_CLARK *stClack    //输出量指针
*         u8 type              //工作类型选择  Single_Phase为单相 Three_Phase为三相
*函数功能:产生正交分量
*************************************************************************************************************/																																					
void ClarkTransform(pll_t *pll, uint8_t type)
{
	static int ADC_V = 0;
	switch (type)
	{
		/*用二阶广义积分器产生正交信号   */
	case Single_Phase:  //单相

		ADC_V = (ADC_Value(0) - 2457) / 600;
		pll->Valpha = Second_order_filter(&Sin, ADC_V);
		pll->Vbeta = Second_order_filter(&Cos, ADC_V);
 		

		break;


		/*用在静止正交坐标系αβ下的旋转矢量V来模拟三相的相位和幅值大小*/
	case Three_Phase:    //三相

//		stClack->Ualpha = 0.3333 * (stAc->Ua + stAc->Ua - stAc->Ub - stAc->Uc);
//		stClack->Ubete = 0.57735 * (stAc->Ub - stAc->Uc);
//		stClack->Ugamma = 0;


		break;
	}

}


/*************************************************************************************************************
*函数名称:void ParkTransform(ST_CLARK *stClack, ST_PARK *stPark)  //park变换
*入口参数:ST_CLARK *stClack    //输入量指针
*			  	ST_PARK *stPark     //输出量指针
*函数功能://park变换
*函数说明：在αβ坐标系下的电压矢量V仍然会随时间相位变化，需要得到一个类似于静态的量来进行控制跟踪，
*因此引入了旋转坐标系dq。旋转坐标系实时跟随电压矢量V(电网角度)旋转。因此电网电压矢量V在d轴
*上的分量为他的幅值，在q轴上的分量为0。将静止正交坐标系αβ的分量投射到旋转坐标系dq上的过程
*称为2s/2r变换或者park变换
*************************************************************************************************************/


void ParkTransform(pll_t *pll)  //park变换
{
	pll->VoltD =   pll->coswt * pll->Valpha + pll->sinwt * pll->Vbeta;
    pll->VoltQ = - pll->sinwt * pll->Valpha + pll->coswt * pll->Vbeta;
}


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

float Second_order_filter(sogi_t  *Biquad_filter, int ADC_Value)   // 二阶低通滤波器
{
	Biquad_filter->Input = ADC_Value;	    //获取ADC采集值
	Biquad_filter->Output = Biquad_filter->b0 * Biquad_filter->Input + Biquad_filter->b1
		* Biquad_filter->Last_IN + Biquad_filter->b2 * Biquad_filter->Last_two_IN - Biquad_filter->a1
		* Biquad_filter->Last_OUT - Biquad_filter->a2 * Biquad_filter->Last_two_OUT;
	//储存滤波结果
	Biquad_filter->Last_two_OUT = Biquad_filter->Last_OUT;
	Biquad_filter->Last_OUT = Biquad_filter->Output;
	Biquad_filter->Last_two_IN = Biquad_filter->Last_IN;
	Biquad_filter->Last_IN = Biquad_filter->Input;
	return Biquad_filter->Output;      //输出滤波结果
}
