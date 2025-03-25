#include "PLL.h" 




#define TS_FREQ           20000 //�ɼ�Ƶ��
#define AC_INPUT_FREQ     50    //�����ź�Ƶ��
#define NO_LOOPL_UP_TABLE 0     //��NO_LOOPL_UP_TABLE=0ʱΪ���ģʽ��NO_LOOPL_UP_TABLE=1ʱΪ���Ǻ�������

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
*��������:void SOGI_Init(Second_order_Filter *Biquad_filter,u8 mode)    //���׹��������������ʼ��
*��ڲ���:Second_order_Filter    ����ָ���ַ
*         u8 mode      ��mode=1ʱ�����࣬mode=0ʱ����PI/2
*��������:���׹��������������ʼ��
*************************************************************************************************************/	
void SOGI_Init(Second_order_Filter* Biquad_filter, uint8_t mode)
{

	Biquad_filter->wc = (2 * PI * AC_INPUT_FREQ) / TS_FREQ;

	switch (mode)
	{
	case 1:  //������

		Biquad_filter->b0 = 2 * Biquad_filter->wc;
		Biquad_filter->b1 = 0;
		Biquad_filter->b2 = -2 * Biquad_filter->wc;
		Biquad_filter->a0 = 4 + 2 * Biquad_filter->wc + Biquad_filter->wc * Biquad_filter->wc;
		Biquad_filter->a1 = 2 * Biquad_filter->wc * Biquad_filter->wc - 8;
		Biquad_filter->a2 = 4 + Biquad_filter->wc * Biquad_filter->wc - 2 * Biquad_filter->wc;

		break;

	case 0:   //���� 

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
*��������:ClarkTransform(ST_AC_INPUT *stAc, ST_CLARK *stClack,u8 type)
*��ڲ���:ST_AC_INPUT *stAc    //������ָ��
*			  	ST_CLARK *stClack    //�����ָ��
*         u8 type              //��������ѡ��  Single_PhaseΪ���� Three_PhaseΪ����
*��������:������������
*************************************************************************************************************/																																					
void ClarkTransform(ST_AC* stAc, ST_CLARK* stClack, uint8_t type)
{
	static int ADC_V = 0;
	switch (type)
	{
		/*�ö��׹�����������������ź�   */
	case Single_Phase:  //����

//		ADC_V = (ADC_Value(0) - 2457) / 600;
//		stClack->Ualpha = Second_order_filter(&Sin, ADC_V);
//		stClack->Ubete = Second_order_filter(&Cos, ADC_V);
//		stClack->Ugamma = 0;

		break;


		/*���ھ�ֹ��������ϵ�����µ���תʸ��V��ģ���������λ�ͷ�ֵ��С*/
	case Three_Phase:    //����

		stClack->Ualpha = 0.3333f * (stAc->Ua + stAc->Ua - stAc->Ub - stAc->Uc);
		stClack->Ubete = 0.57735f * (stAc->Ub - stAc->Uc);
		stClack->Ugamma = 0;


		break;
	}

}


/*************************************************************************************************************
*��������:void ParkTransform(ST_CLARK *stClack, ST_PARK *stPark)  //park�任
*��ڲ���:ST_CLARK *stClack    //������ָ��
*			  	ST_PARK *stPark     //�����ָ��
*��������://park�任
*����˵�����ڦ�������ϵ�µĵ�ѹʸ��V��Ȼ����ʱ����λ�仯����Ҫ�õ�һ�������ھ�̬���������п��Ƹ��٣�
*�����������ת����ϵdq����ת����ϵʵʱ�����ѹʸ��V(�����Ƕ�)��ת����˵�����ѹʸ��V��d��
*�ϵķ���Ϊ���ķ�ֵ����q���ϵķ���Ϊ0������ֹ��������ϵ���µķ���Ͷ�䵽��ת����ϵdq�ϵĹ���
*��Ϊ2s/2r�任����park�任
*************************************************************************************************************/


void ParkTransform(ST_CLARK* stClack, ST_PARK* stPark)  //park�任
{
	stPark->Ud = stClack->Ualpha * stPark->Sinwt - stClack->Ubete * stPark->Coswt;
	stPark->Uq = stClack->Ualpha * stPark->Coswt + stClack->Ubete * stPark->Sinwt;
	stPark->U0 = 0;
}


void Uabc_to_dq(ST_AC* stAc, ST_PARK* stPark)  //dq�任
{
	stPark->Ud = 0.66666667f * (stAc->Ua * stPark->Sinwt + stAc->Ub * stPark->Sinwt_Lagging + stAc->Uc * stPark->Sinwt_Leading);
	stPark->Uq = 0.66666667f * (stAc->Ua * stPark->Coswt + stAc->Ub * stPark->Coswt_Lagging + stAc->Uc * stPark->Coswt_Leading);
	stPark->U0 = 0;
}


void dq_to_Uabc(ST_PARK* stPark, ST_AC* stAc)  //dq���任
{
	stAc->Ua = stPark->Uq * stPark->Coswt + stPark->Ud * stPark->Sinwt;
	stAc->Ub = stPark->Uq * stPark->Coswt_Lagging + stPark->Ud * stPark->Sinwt_Lagging;
	stAc->Uc = stPark->Uq * stPark->Coswt_Leading + stPark->Ud * stPark->Sinwt_Leading;
}



/*************************************************************************************************************
*��������:float pll(u8 type,u8 mode)            //���໷����
*��ڲ���:u8 type       //��������ѡ��  Single_PhaseΪ���� Three_PhaseΪ����
*����ֵ����Ϊ���ģʽʱ����ֵΪ�������ݵ�λ�ã���Ϊ���Ǻ�������ʱ����ֵΪ�Ƕ�
*��������:���໷����
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

	PLLLoopOut = Incremental_PID(&PllLoop, 0, stPark.Uq)��
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


	/* sin�Ƕȼ���������ֵ   */
	TempIndex = 63.662f * PLLSinAngle;// ���Ƕ�ת����index��2pi�ֳ�400���㣬������ 400/2pi

	if ((uint16_t)TempIndex < (uint16_t)(TempIndex + 0.5f))   TempIndex = (uint16_t)(TempIndex + 1);      //��������
	else  TempIndex = (uint16_t)TempIndex;

	SinIndex = TempIndex;
	CosIndex = TempIndex + SIN_TAB_90;
	SinIndex %= MAX_SIN_TAB;
	CosIndex %= MAX_SIN_TAB;

	// �����ұ��õ�g_f32SinWT��g_f32CosWT
//	stPark.Sinwt = SinTab[SinIndex];
//	stPark.Coswt = SinTab[CosIndex];
	return  SinIndex;


#endif


}


