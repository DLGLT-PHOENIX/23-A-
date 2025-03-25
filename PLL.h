#ifndef _PLL_H_
#define _PLL_H_

#include "struct_typedef.h"
#include "filter.h"
#include "arm_math.h"
//#include "math.h" 
#include "pid.h"

typedef enum 
{
   	Single_Phase,  //单相
	  Three_Phase,   //三相
	
} Pll_Manner_of_working; //锁相环类型选择  


typedef struct
{
    float Ua;
    float Ub;
    float Uc;
}ST_AC;


typedef struct
{
    float Ualpha;
    float Ubete;
    float Ugamma;
}ST_CLARK;


typedef struct
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

extern ST_AC       stAcInputVolt;
extern ST_AC       Incoming_Current;   
extern ST_CLARK    stClark;
extern ST_PARK     stPark;
extern ST_PARK    Uabc_dq;
extern ST_AC      dq_Uabc;



void Voltage_to_read(ST_AC *stAc,float Ua,float Ub,float Uc);
void SOGI_Init(Second_order_Filter *Biquad_filter,uint8_t mode);
void ClarkTransform(ST_AC *stAc, ST_CLARK *stClack,uint8_t type);
void ParkTransform(ST_CLARK *stClack, ST_PARK *stPark);
void Uabc_to_dq(ST_AC* stAc, ST_PARK* stPark);  //dq变换
void dq_to_Uabc(ST_PARK* stPark,ST_AC* stAc);  //dq反变换
float PLL(uint8_t type);


#endif 

