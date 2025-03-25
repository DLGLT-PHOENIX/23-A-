#include "PLL.h" 




#define TS_FREQ           20000 //采集频率
#define AC_INPUT_FREQ     50    //输入信号频率
#define NO_LOOPL_UP_TABLE 0     //当NO_LOOPL_UP_TABLE=0时为查表模式，NO_LOOPL_UP_TABLE=1时为三角函数运算

#define MAX_SIN_TAB     400
#define SIN_TAB_90      100
#define SIN_TAB_120     133
#define SIN_TAB_240     266


ST_AC        stAcInputVolt;
ST_CLARK     stClark; 
ST_PARK      stPark;
ST_PARK      Uabc_dq;
ST_AC        Incoming_Current;
ST_AC        dq_Uabc;

void Voltage_to_read(ST_AC* stAc, float Ua, float Ub, float Uc)
{

	stAc->Ua = Ua;
	stAc->Ub = Ub;
	stAc->Uc = Uc;

}




/*************************************************************************************************************
*函数名称:void SOGI_Init(Second_order_Filter *Biquad_filter,u8 mode)    //二阶广义积分器参数初始化
*入口参数:Second_order_Filter    参数指针地址
*         u8 mode      当mode=1时不移相，mode=0时移相PI/2
*函数功能:二阶广义积分器参数初始化
*************************************************************************************************************/	
void SOGI_Init(Second_order_Filter* Biquad_filter, uint8_t mode)
{

	Biquad_filter->wc = (2 * PI * AC_INPUT_FREQ) / TS_FREQ;

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

unsigned long VoltageAIndex=0,VoltageBIndex=0;
/*************************************************************************************************************
*函数名称:ClarkTransform(ST_AC_INPUT *stAc, ST_CLARK *stClack,u8 type)
*入口参数:ST_AC_INPUT *stAc    //输入量指针
*			  	ST_CLARK *stClack    //输出量指针
*         u8 type              //工作类型选择  Single_Phase为单相 Three_Phase为三相
*函数功能:产生正交分量
*************************************************************************************************************/																																					
void ClarkTransform(ST_AC* stAc, ST_CLARK* stClack, uint8_t type)
{
	static int ADC_V = 0;
	switch (type)
	{
		/*用二阶广义积分器产生正交信号   */
	case Single_Phase:  //单相

//		ADC_V = (ADC_Value(0) - 2457) / 600;
//		stClack->Ualpha = Second_order_filter(&Sin, ADC_V);
//		stClack->Ubete = Second_order_filter(&Cos, ADC_V);
//		stClack->Ugamma = 0;

		break;


		/*用在静止正交坐标系αβ下的旋转矢量V来模拟三相的相位和幅值大小*/
	case Three_Phase:    //三相

		stClack->Ualpha = 0.3333f * (stAc->Ua + stAc->Ua - stAc->Ub - stAc->Uc);
		stClack->Ubete = 0.57735f * (stAc->Ub - stAc->Uc);
		stClack->Ugamma = 0;


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


void ParkTransform(ST_CLARK* stClack, ST_PARK* stPark)  //park变换
{
	stPark->Ud = stClack->Ualpha * stPark->Sinwt - stClack->Ubete * stPark->Coswt;
	stPark->Uq = stClack->Ualpha * stPark->Coswt + stClack->Ubete * stPark->Sinwt;
	stPark->U0 = 0;
}


void Uabc_to_dq(ST_AC* stAc, ST_PARK* stPark)  //dq变换
{
	stPark->Ud = 0.66666667f * (stAc->Ua * stPark->Sinwt + stAc->Ub * stPark->Sinwt_Lagging + stAc->Uc * stPark->Sinwt_Leading);
	stPark->Uq = 0.66666667f * (stAc->Ua * stPark->Coswt + stAc->Ub * stPark->Coswt_Lagging + stAc->Uc * stPark->Coswt_Leading);
	stPark->U0 = 0;
}


void dq_to_Uabc(ST_PARK* stPark, ST_AC* stAc)  //dq反变换
{
	stAc->Ua = stPark->Uq * stPark->Coswt + stPark->Ud * stPark->Sinwt;
	stAc->Ub = stPark->Uq * stPark->Coswt_Lagging + stPark->Ud * stPark->Sinwt_Lagging;
	stAc->Uc = stPark->Uq * stPark->Coswt_Leading + stPark->Ud * stPark->Sinwt_Leading;
}



/*************************************************************************************************************
*函数名称:float pll(u8 type,u8 mode)            //锁相环程序
*入口参数:u8 type       //工作类型选择  Single_Phase为单相 Three_Phase为三相
*返回值：当为查表模式时返回值为表中数据的位置，当为三角函数运算时返回值为角度
*函数功能:锁相环程序
*************************************************************************************************************/
float PLL(uint8_t type)
{
	static float PLLSinAngle = 0;
	static float PLLLoopOut;
#if NO_LOOPL_UP_TABLE
	static float Theta = 0;
	unsigned long Ratio = 0;
	Ratio = (TS_FREQ) / (AC_INPUT_FREQ);
#else
	static uint16_t SinIndex = 0;
	static uint16_t CosIndex = 0;
	static float TempIndex;
#endif

	ClarkTransform(&stAcInputVolt, &stClark, type);
	ParkTransform(&stClark, &stPark);


#if NO_LOOPL_UP_TABLE

	PLLLoopOut = Incremental_PID(&PllLoop, 0, stPark.Uq)；
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
	stPark.Sinwt = arm_sin_f32(Theta);
	stPark.Coswt = arm_cos_f32(Theta);
	return Theta;

#else

	//PLLLoopOut = Incremental_PID(&PllLoop, 0, stPark.Uq) * ((float)1 / TS_FREQ);
	PLLSinAngle += PLLLoopOut;


	if (PLLSinAngle > (2 * PI))
	{
		PLLSinAngle = PLLSinAngle - (2 * PI);
	}
	else if (PLLSinAngle < 0)
	{
		PLLSinAngle = PLLSinAngle + (2 * PI);
	}


	/* sin角度计算查表索引值   */
	TempIndex = 63.662f * PLLSinAngle;// 将角度转换成index，2pi分成400个点，所以是 400/2pi

	if ((uint16_t)TempIndex < (uint16_t)(TempIndex + 0.5f))   TempIndex = (uint16_t)(TempIndex + 1);      //四舍五入
	else  TempIndex = (uint16_t)TempIndex;

	SinIndex = TempIndex;
	CosIndex = TempIndex + SIN_TAB_90;
	SinIndex %= MAX_SIN_TAB;
	CosIndex %= MAX_SIN_TAB;

	// 查正弦表，得到g_f32SinWT和g_f32CosWT
//	stPark.Sinwt = SinTab[SinIndex];
//	stPark.Coswt = SinTab[CosIndex];
	return  SinIndex;


#endif


}


