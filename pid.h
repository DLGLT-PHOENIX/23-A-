///*************************** Dongguan-University of Technology -ACE**************************
// * @file    pid.h
// * @author  ֣���
// * @version V1.0.3
// * @date    2022/11/19
// * @brief
// ******************************************************************************
// * @verbatim
// *  �Ѿ��ѳ��õ�pidģʽ���ӽ�ȥ�ˣ������˲���һЩû��ʵ��
// *  ʹ�÷�����
// *      �ȴ���һ��pid�ṹ�塣
// *      ��ʼ����
// *            ��ʹ��PidInit��ע��ģʽ�����������ã��� | ����
// *            Ȼ��ʹ��PidInitMode��ÿ��ģʽ�����ֵ
// *      ʹ�ã�PidCalculate
// *            pid_clear
// *            User_Fun_Callback_Register
// *  demo��
// *       pid_parameter_t motor6020pid_s;//����һ��pid�ṹ��
// *       PidInit(&motor6020pid_s,20,0,0,Output_Limit | Integral_Limit);//ʹ������޷��ͻ����޷�ģʽ
// *       PidInitMode(&motor6020pid_s,Integral_Limit,1000,0);//�����޷�ģʽ����
// *	    PidInitMode(&motor6020pid_s,Output_Limit,30000,0);//����޷�ģʽ����
// *          while(1)
// *          {
// *             ActualValue = sersor();                     //��ȡ����
// *             setvalue  = setclaculate();                 //��ȡ����
// *             PidCalculate(&motor6020pid_s,setvalue,ActualValue);   //����
// *          }
// *
// * @attention
// *      ��ȷ��User_Fun_Callback_Registerע���˺�����ʹ�ýṹ���е��Զ��庯��
// *      ����Ĭ����ʹ�õģ������ʹ�ã�����������У�ʹ���˻������������pid��
// * @version
// * v1.0.1 �����˲���ʽPID�����й��ܴ���֤
// * V1.0.2 �����˺ܶ�ע�ͣ�Integral_Limit ��Output_Limit ��StepInģʽ����֤��Derivative_On_Measurement�ƺ��е�����
// * V1.0.3 �����˻��ֺ�����˲�������֤
// ************************** Dongguan-University of Technology -ACE***************************/
#ifndef __PID_H
#define __PID_H
#include "struct_typedef.h"

typedef enum
{
	NONE = 0X00,					  // 0000 0000 ��
	Deadzone = 0x01,				  // 0000 0001 ����
	Integral_Limit = 0x02,			  // 0000 0010 �����޷�
	Output_Limit = 0x04,			  // 0000 0100 ����޷�
	Derivative_On_Measurement = 0x08, // 0000 1000 ΢������ TODO:
	Separated_Integral = 0x10,		  // 0001 0000 ���ַ���
	ChangingIntegrationRate = 0x20,	  // 0010 0000 �����
	OutputFilter = 0x40,			  // 0100 0000 ����˲�
	DerivativeFilter = 0x80,		  // 1000 0000 ΢���˲�
	StepIn = 0x0100,				  // 0000 0001 0000 0000 ����ʽ
} PID_mode_e;

//һ�׵�ͨ�˲�����
typedef __packed struct
{
	fp32 input;		 //��������
	fp32 last_input; //�ϴ�����
	fp32 out;		 //�˲����������
	fp32 num;		 //�˲�����
} first_order_filter_t;

typedef __packed struct Pid_parameter_t // pid�ṹ�����
{
	fp32 Kp;
	fp32 Ki;
	fp32 Kd;

	fp32 SetValue;
	fp32 LastSetValue;
	fp32 LastActualValue;
	fp32 ActualValue;

	fp32 Ierror;
	fp32 Pout;
	fp32 Iout;
	fp32 Dout;
	fp32 out;

	fp32 Derror; //΢����
	fp32 LastDerror;
	fp32 LastLastDerror;
	fp32 error; //�����
	fp32 LastError;
	fp32 LastLastError;

	fp32 max_out; //������
	fp32 min_out;

	uint32_t mode; // pidģʽ

	/* �����޷� */
	fp32 max_Ierror; //���������
	/* ������� */
	fp32 deadband;
	/* ���ַ��� */
	fp32 threshold_max; //���ַ������ֵ
	fp32 threshold_min; //���ַ�����Сֵ
	/* �����ֱ��� */
	// fp32 maximum; //���ֵ
	// fp32 minimum; //��Сֵ
	/* ����� */
	fp32 errorabsmax; //ƫ�����ֵ���ֵ
	fp32 errorabsmin; //ƫ�����ֵ��Сֵ
	/* ΢������ */
	fp32 gama; //΢�������˲�ϵ��
	/* ΢���˲� */
	first_order_filter_t d_filter; //΢���˲��ṹ��
	/* ����˲� */
	first_order_filter_t out_filter; //����˲��ṹ��

	/* ������ */
	fp32 stepIn;

	void (*User_Fun)(struct Pid_parameter_t *);
} pid_parameter_t;

typedef struct Detection_volume_t //
{
  fp32 ACvoltage;
  fp32 ACcurrent;
  fp32     temp;
  fp32    Vref;
	fp32 curtemp;
	fp32 ave;				//一个周期的平均值
	fp32 sum;				//一个周期的代数和
	fp32 pf_sum;			//平方和
	fp32 ACvalue;	//瞬时交流分量（电压）
	fp32 DCvalue;		//瞬时直流量
	fp32 rms;				//一个周期的均方根值
	fp32 VIN;
  pid_parameter_t current_pid;
 pid_parameter_t VIN_pid;
	pid_parameter_t Vout_pid;
 pid_parameter_t Vboost_pid;
}detection_volume_t;



void PidInit(pid_parameter_t *pid, fp32 kp, fp32 ki, fp32 kd, uint32_t mode);
void PidInitMode(pid_parameter_t *pid, uint32_t mode, fp32 num1, fp32 num2);
void pid_clear(pid_parameter_t *pid);
void User_Fun_Callback_Register(pid_parameter_t *pid, void (*User_Fun)(struct Pid_parameter_t *));
fp32 PidCalculate(pid_parameter_t *pid, fp32 SetValue, fp32 ActualValue);
float Incremental_PID(pid_parameter_t *PID,float Measured,float Target);


fp32 single_loop(pid_parameter_t *spid, fp32 setSpeed, fp32 actualSpeed);
fp32 double_loop(pid_parameter_t *spid, pid_parameter_t *ppid, fp32 setPosition, fp32 actual_position, fp32 actual_speed);

//int PWM_cal(pid_parameter_t *cur_pid,pid_parameter_t *vbuck_pid,uint8_t mode, fp32 SetValue, fp32 cur_ActualValue,fp32 vbuck_ActualValue);

#endif


