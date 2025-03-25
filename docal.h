#ifndef __DOCAL_H__
#define __DOCAL_H__

#include "pid.h"

//У׼����
#define VIN_CALI 	0.5926
#define VSA_CALI 	0.5921
#define VSB_CALI  0.5933
#define VSC_CALI 	0.5985
#define ISA1_CALI 0.7231
#define ISB1_CALI 0.7572
#define ISC1_CALI 0.7159
#define ISA2_CALI 0.7276
#define ISB2_CALI 0.7266
#define ISC2_CALI 0.7299

typedef struct PLL_t
{
    fp32 coswt;
    fp32 sinwt;
    fp32 Valpha;
    fp32 Vbeta;
    fp32 Wt;//电网电压频率
    fp32 V_web;//电网电压
    fp32 VoltD;
    fp32 VoltQ;
    fp32 V_inject;//注入电压
    fp32 I_inject;//注入电流
    
    pid_parameter_t ppl_pid;

}pll_t;

typedef enum pll_type_t
{
   	Single_Phase,  //单相
	  Three_Phase,   //三相
	
} Pll_Manner_of_working; //锁相环类型选择  





typedef struct clark_t
{
    float Ualpha;
    float Ubete;
    float Ugamma;
}ST_CLARK;


typedef struct park_t
{
    float Ud;
    float Uq;
    float U0;
    float Sinwt;
    float Coswt;
    float Sinwt_Leading;   //Sin(wt + 2/3*PI)
    float Coswt_Leading;   //Cos(wt + 2/3*PI)
    float Sinwt_Lagging;   //Sin(wt - 2/3*PI)
    float Coswt_Lagging;   //Cos(wt - 2/3*PI)
}ST_PARK;

typedef struct   SOGI_t  //PID初始化参数，每次一个PID都要重新定义一个新名称。
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
	
} sogi_t;

void SOGI_Init(sogi_t *Biquad_filter,uint8_t mode);
void ClarkTransform(pll_t *pll,uint8_t type);
void ParkTransform(pll_t *pll);

void PPL(pll_t *pll,fp32 alpha,fp32 beta,fp32 V_web);
float Second_order_filter(sogi_t* Biquad_filter, int ADC_Value);

void V_for_current(detection_volume_t *Vout,detection_volume_t *cur,fp32 setVout, fp32 actual_Vout, fp32 actual_cur);
void AC_V_detect(detection_volume_t *AC_V);

#endif
