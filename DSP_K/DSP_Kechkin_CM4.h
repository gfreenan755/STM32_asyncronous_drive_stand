#ifndef DSP_Kechkin_CM4_H
#define DSP_Kechkin_CM4_H

#include "stm32f4xx.h"
#include <arm_math.h>

//#ifndef ARM_MATH_CM4
//#define ARM_MATH_CM4
//#endif

//------------------------------------------------------------//
//---------------- DSP Kechkin Library ver 1 -----------------//
//------------------------------------------------------------//

/*
                             Структура  Регулятора
*/
typedef struct {
float  	Int;
float  	P;

float  	k_Int;									
float  	k_P;

float  	Out;

float  	In;
float  	Fb;
float  	d;

float  	UpLimit;
float  	DownLimit;
}DSP_K_Regulator_Structure;

typedef struct {
uint16_t  	cnt_s;

uint16_t  	angle_f[5];
uint16_t        angle_d[4];

float  	speed;
float  	speed_el;
}Speed_structure;

/*
       
*/
typedef struct {
float  	K_Intense;
}DSP_K_IntenseSetter_Structure;

/*
       
*/
typedef struct {
int  	Period;
float   Level;
float   Out;
int     Cnt;
}DSP_K_Generator_Structure;

// -----------------  FIR Filter -----------------------------//
#define DSP_K_Filter_Max_Resolution 			64
#define DSP_K_Filter_Max_signal_length 	                64
typedef struct {
uint16_t 						 Resolution;
uint32_t 						 output_signal_length;	
float	  		 				 firState[DSP_K_Filter_Max_Resolution+DSP_K_Filter_Max_signal_length -1];        //[Resolution + output_signal_length -1];
float  							 input_signal[DSP_K_Filter_Max_Resolution];
float  							 output_signal[DSP_K_Filter_Max_signal_length];
float  							 Fir1_coef[DSP_K_Filter_Max_Resolution];	
arm_fir_instance_f32                                     S;
float                                                    Out;
}DSP_K_Filter_Init_Structure;

// -----------------  Atan2 -----------------------------//
#define M_PI                    3.141592653589793f
#define M_PI12 (M_PI/12.F)
#define M_PI6 (M_PI/6.F)
#define M_PI2 (M_PI/2.F)
#define M_PI3 (M_PI/3.F)
/* square root of 3 */
#define SQRT3                   1.732050807569f

//################  Vector_PWM  ###################
#define sin_0    0.0f
#define cos_0    1.0f

#define sin_60   0.8660254037844f
#define cos_60   0.5f

#define sin_120  sin_60
#define cos_120 -cos_60

#define sin_180  sin_0
#define cos_180 -cos_0

#define sin_240 -sin_60
#define cos_240 -cos_60

#define sin_300 -sin_60
#define cos_300  cos_60 

void DSP_K_Polar_to_AB(float Amp, float Rad, float *Bufer_A_B);
float DSP_K_Atan2(float cos, float sin);

void DSP_K_FIR_Init(DSP_K_Filter_Init_Structure* Filter);
void DSP_K_FIR_Process(DSP_K_Filter_Init_Structure* Filter, float In_ADC);
void DSP_K_MeanFilter_Process(DSP_K_Filter_Init_Structure* Filter, float In_ADC);

void DSP_K_Get_Sin_5t_Cos_5t(float *Bufer_Sin_Cos, float *Bufer_Sin_5t_Cos_5t);
float DSP_K_Get_ElRadians(float Radians, uint8_t PolePairs);
float DSP_K_RadiansShifter(float Radians, float RadShift);


void DSP_K_PWM_Blocked_2VT(TIM_TypeDef *Tim, uint8_t Table, float Amp);
void DSP_K_PWM_Blocked_4VT(TIM_TypeDef *Tim, uint8_t Table, float Amp);
void DSP_K_SVPWM(TIM_TypeDef *Tim, float A, float B);

void DSP_K_Sin_Cos_Shift(float *Sin_Cos, float *Sin_Cos_Shift, float *Sin_Cos_Out);

void DSP_K_Conv_ABC_to_ab(float *A_B_C, float *a_b);
void DSP_K_Conv_Direct_PK(float *a_b, float *d_q, float *AngleES);
void DSP_K_Conv_Inverse_PK(float *d_q, float *a_b, float *AngleES);

void DSP_K_Regulator(DSP_K_Regulator_Structure *Reg);

void Speed_measure(Speed_structure *Spd);
void Current_measure(float *iABC, unsigned short *ADC_Buf);

#endif
