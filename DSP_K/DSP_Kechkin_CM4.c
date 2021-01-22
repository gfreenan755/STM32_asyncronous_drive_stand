

//------------------------------------------------------------//
//---------------- DSP Kechkin Library ver 1 -----------------//
//------------------------------------------------------------//

#include "stm32f4xx.h"                  // Device header
#include "DSP_Kechkin_CM4.h"
/*
Инициализация фильтра
Пример настройки FIR фильтра

//25 Гц  порядок 10 
float   h_25_10[] =  {0.016192f,0.037976f,0.093139f,0.15587f,0.19682f,0.19682f,0.15587f,0.093139f,0.037976f,0.016192f};  
//50 Гц  порядок 10 
float h_50_10[] ={0.016183f,0.037966f,0.093132f,0.15588f,0.19684f,0.19684f,0.15588f,0.093132f,0.037966f,0.016183f};    

DSP_Filter_DPR_Sin.Resolution=10;               //Order
for(int i = 0 ;i<(sizeof(h_25_10)/4) ;i++)      //coef
{                                              
DSP_Filter_DPR_Sin.Fir1_coef[i] = h_25_10[i];
DSP_Filter_DPR_Cos.Fir1_coef[i] = h_25_10[i];
DSP_Filter_DPR_Ref.Fir1_coef[i] = h_25_10[i];
}
DSP_Filter_DPR_Sin.output_signal_length = 1;    //out lenght
DSP_Filter_DPR_Cos.output_signal_length = 1;
DSP_Filter_DPR_Ref.output_signal_length = 1;
DSP_K_FIR_Init(&DSP_Filter_DPR_Sin);            //init
DSP_K_FIR_Init(&DSP_Filter_DPR_Cos);
DSP_K_FIR_Init(&DSP_Filter_DPR_Ref);

*/
void DSP_K_FIR_Init(DSP_K_Filter_Init_Structure* Filter)
{

arm_fir_init_f32(&Filter->S, Filter->Resolution, Filter->Fir1_coef, Filter->firState, Filter->output_signal_length);

}


/*
Функция предназнчачена для вызова из программы обработчика АЦП
*/
void DSP_K_FIR_Process(DSP_K_Filter_Init_Structure* Filter, float In_ADC)
{
	for(int i=0; i < Filter->Resolution-1; i++)
	{
		Filter->input_signal[ i ] = Filter->input_signal[ i + 1 ];
	}
	
Filter->input_signal[ Filter->Resolution-1 ] = In_ADC;
arm_fir_f32(&Filter->S,Filter->input_signal , Filter->output_signal, DSP_K_Filter_Max_signal_length);
Filter->Out = Filter->output_signal[Filter->Resolution-1];
}
/*
Функция предназнчачена для вызова из программы обработчика АЦП
*/
void DSP_K_MeanFilter_Process(DSP_K_Filter_Init_Structure* Filter, float In_ADC)
{
   Filter->input_signal[0] = (In_ADC +Filter->input_signal[0])/2.0f;
   for(int i=1; i < Filter->Resolution; i++)
   {
        Filter->input_signal[i] = (Filter->input_signal[i-1] +Filter->input_signal[i])/2.0f;

   }
Filter->Out = Filter->input_signal[Filter->Resolution-1];	
}
/*
DSP_K_Polar_to_AB
*/
void DSP_K_Polar_to_AB(float Amp, float Rad, float *Bufer_A_B)
{
*Bufer_A_B     = Amp * arm_cos_f32(Rad);
*(Bufer_A_B+1) = Amp * arm_sin_f32(Rad);
}
/*
DSP_K_Atan2
*/
float DSP_K_Atan2(float cos, float sin)
{

  float  x,x2,a;
	int sta=0,sp=0;
	x = sin/cos;
  /* check up the sign change */
  if(x<0.F) {x=-x;sta|=1;}
  /* check up the invertation */
  if(x>1.F) {x=1.F/x;sta|=2;}
  /* process shrinking the domain until x<PI/12 */
  while(x>M_PI12) {
    sp++; a=x+SQRT3;
		a=1.F/a;
		x*=SQRT3;
		x-=1.F; x*=a;}
  /* calculation core */
  x2=x*x;
	a=x2+1.4087812F;
	a=0.55913709F/a;
	a+=0.60310579F;
  a-=0.05160454F*x2;
	a*=x;
  /* process until sp=0 */
  while(sp>0) {a+=M_PI6;sp--;}
  /* invertation took place */
  if(sta&2) a=M_PI2-a;
  /* sign change took place */
  if(sta&1) a=-a;
	
		if(cos==0) {
		if(sin> 0){a =  PI/2;}
		if(sin <0){a = -PI/2;}}
		else if(cos<0) {
		if(sin>=0){a+=PI;}
		if(sin <0){a-=PI;}}
  return(a);
}
/*
DSP_K_Get_Sin_5t_Cos_5t
*/
void DSP_K_Get_Sin_5t_Cos_5t(float *Bufer_Sin_Cos, float *Bufer_Sin_5t_Cos_5t)
{
float Sin_buf_5[6];
float Cos_buf_5[6];

Sin_buf_5[0] = Bufer_Sin_Cos[0];
Sin_buf_5[1] = Sin_buf_5[0];
Sin_buf_5[2] = Sin_buf_5[1] * Sin_buf_5[1];
Sin_buf_5[3] = Sin_buf_5[2] * Sin_buf_5[1];
Sin_buf_5[4] = Sin_buf_5[3] * Sin_buf_5[1];
Sin_buf_5[5] = Sin_buf_5[4] * Sin_buf_5[1];

Bufer_Sin_5t_Cos_5t[0] = 5.0f * Sin_buf_5[1] - 20.0f * Sin_buf_5[3] + 16.0f * Sin_buf_5[5];

	
Cos_buf_5[0] = Bufer_Sin_Cos[1];
Cos_buf_5[1] = Cos_buf_5[0];
Cos_buf_5[2] = Cos_buf_5[1]* Cos_buf_5[1];
Cos_buf_5[3] = Cos_buf_5[2]* Cos_buf_5[1];
Cos_buf_5[4] = Cos_buf_5[3]* Cos_buf_5[1];
Cos_buf_5[5] = Cos_buf_5[4]* Cos_buf_5[1];

Bufer_Sin_5t_Cos_5t[1] = 5.0f * Cos_buf_5[1] - 20.0f * Cos_buf_5[3] + 16.0f * Cos_buf_5[5];

}

/*
DSP_K_Get_Sin_5t_Cos_5t
*/
float DSP_K_Get_ElRadians(float Radians, uint8_t PolePairs)
{

float Out;  
float tmp;
int kratnost;

tmp = (Radians+PI)*PolePairs;
kratnost = (int)(tmp/PI/2.0f);
if(kratnost<0)kratnost*=-1;
if(tmp>2.0f*PI)
{
  Out = tmp - 2*PI*kratnost-PI;
}
else 
{
  Out = tmp-PI;
}
return Out;
}

/*
DSP_K_Get_Sin_5t_Cos_5t
*/
float DSP_K_RadiansShifter(float Radians, float RadShift)
{

float Out;  

Out = Radians-RadShift;
if     (Out<-PI)     Out+=2.0f*PI;
else if(Out> PI)     Out-=2.0f*PI;
return Out;
}
/*
DSP_K_Sin_Cos_Shift

Angle Out = Angle - Shift;

*/
void DSP_K_Sin_Cos_Shift(float *Sin_Cos, float *Sin_Cos_Shift, float *Sin_Cos_Out)
{

Sin_Cos_Out[0] = (Sin_Cos[0])*(Sin_Cos_Shift[1])-(Sin_Cos[1])*(Sin_Cos_Shift[0]);
Sin_Cos_Out[1] = (Sin_Cos[1])*(Sin_Cos_Shift[1])+(Sin_Cos[0])*(Sin_Cos_Shift[0]);


}

/*
DSP_K_Conv_ABC_to_ab

*/
void DSP_K_Conv_ABC_to_ab(float *A_B_C, float *a_b)
{

a_b[0] = A_B_C[0]*1.0f;
a_b[1] = 1.0f/SQRT3*(A_B_C[1]-A_B_C[2])*1.0f;

}

/*
DSP_K_Conv_Direct_PK

*/
void DSP_K_Conv_Direct_PK(float *a_b, float *d_q, float *AngleES)
{
d_q[0] = a_b[1]*arm_sin_f32(*AngleES)+a_b[0]*arm_cos_f32(*AngleES);
d_q[1] = a_b[1]*arm_cos_f32(*AngleES)-a_b[0]*arm_sin_f32(*AngleES);
}
/*
DSP_K_Conv_Inverse_PK

*/
void DSP_K_Conv_Inverse_PK(float *d_q, float *a_b, float *AngleES)
{
a_b[0] = d_q[0]*arm_cos_f32(*AngleES)-d_q[1]*arm_sin_f32(*AngleES);
a_b[1] = d_q[0]*arm_sin_f32(*AngleES)+d_q[1]*arm_cos_f32(*AngleES);
}

/*
Encoder KUBLER speed measure
*/

void Speed_measure(Speed_structure *Spd){
    
  
 if(Spd->cnt_s < 19){
    Spd->cnt_s++;
  }
  else{   
  Spd->angle_f[4] = Spd->angle_f[3];
  Spd->angle_f[3] = Spd->angle_f[2];
  Spd->angle_f[2] = Spd->angle_f[1];
  Spd->angle_f[1] = Spd->angle_f[0];
    
  if(Spd->angle_f[0] != Spd->angle_f[1]){
    Spd->angle_d[0] = Spd->angle_f[0] - Spd->angle_f[1];
  }
  if(Spd->angle_f[1] != Spd->angle_f[2]){
    Spd->angle_d[1] = Spd->angle_f[1] - Spd->angle_f[2];
  }
  if(Spd->angle_f[2] != Spd->angle_f[3]){
    Spd->angle_d[2] = Spd->angle_f[2] - Spd->angle_f[3];
  }
  if(Spd->angle_f[3] != Spd->angle_f[4]){
    Spd->angle_d[3] = Spd->angle_f[3] - Spd->angle_f[4];
  }
  
  
  if((Spd->angle_d[0] - Spd->angle_d[1]) < 2){
    if((Spd->angle_d[1] - Spd->angle_d[2]) < 2){
      if((Spd->angle_d[2] - Spd->angle_d[3]) < 2){
        Spd->speed = Spd->angle_d[2]*0.0610426f;
      }
    }
  }

  Spd->speed_el = Spd->speed*3.0f;
  if(Spd->speed_el > 200){
    Spd->speed = 0;
    Spd->speed_el = 0;
  }
  Spd->cnt_s = 0;
  }

}

void Current_measure(float *iABC, unsigned short *ADC_Buf){
    
  iABC[0] = (float)((ADC_Buf[0]>>2) - 589) / 512;            //i_U
  iABC[1] = (float)((ADC_Buf[1]>>2) - 587) / 512;            //i_V
  iABC[2] = (float)((ADC_Buf[2]>>2) - 585) / 512;            //i_W
  
}


void DSP_K_Regulator(DSP_K_Regulator_Structure *Reg)
{

  //volatile static float IntOld =0.0f;
  

//Ошибка
Reg->d = Reg->In  - Reg->Fb;

//Интегральная часть
//Reg->Int = (Reg->d  * 0.00005f*Reg->k_Int+IntOld)*0.5f;
Reg->Int += Reg->d  * 0.00005f*Reg->k_Int;
//Reg->Int *= 0.5f;
//IntOld = Reg->Int;

////Ограничение              [от -1 до 1]
//if(IntOld<-1.0f) IntOld = -1.0f;
//if(IntOld> 1.0f) IntOld =  1.0f;
//Ограничение              [от -1 до 1]

if(Reg->Int< Reg->DownLimit) Reg->Int = Reg->DownLimit;
if(Reg->Int> Reg->UpLimit)   Reg->Int = Reg->UpLimit;

//Пропорциональная часть
Reg->P  = Reg->k_P *Reg->d;

//Ограничение              [от -1 до 1]
Reg->Out = Reg->Int + Reg->P;
     if(Reg->Out<Reg->DownLimit) Reg->Out =  Reg->DownLimit;
else if(Reg->Out>Reg->UpLimit)   Reg->Out =  Reg->UpLimit;
}

/*
DSP_K_PWM_TIM1_Blocked_2VT
*/
void DSP_K_PWM_Blocked_2VT(TIM_TypeDef *Tim, uint8_t Table, float Amp)
{
switch (Table)
{       // VT1 & VT6
	case 0:
        Tim->CCER   &= ~(TIM_CCER_CC1NE |  TIM_CCER_CC1E  |  TIM_CCER_CC2NE | 
                          TIM_CCER_CC2E  |  TIM_CCER_CC3NE |  TIM_CCER_CC3E);
        Tim->CCER    |= TIM_CCER_CC3NE  |  TIM_CCER_CC1E  |  TIM_CCER_CC2E;   //1 3n
        Tim->CCER    |= TIM_CCER_CC1P  |  TIM_CCER_CC2P  |  TIM_CCER_CC3P;   //1 3n
        Tim->CCER    |= TIM_CCER_CC1NP  |  TIM_CCER_CC2NP  |  TIM_CCER_CC3NP;   //1 3n
	Tim->CCR1 = (uint16_t)(Tim->ARR*1.0f);
	Tim->CCR3 = (uint16_t)(Tim->ARR*Amp);
	Tim->CCR2 = 0;
	break;
        // VT3 & VT6	
	case 1:
        Tim->CCER   &= ~(TIM_CCER_CC1NE |  TIM_CCER_CC1E  |  TIM_CCER_CC2NE | 
                          TIM_CCER_CC2E  |  TIM_CCER_CC3NE |  TIM_CCER_CC3E); 
	Tim->CCER  |= TIM_CCER_CC3NE  | TIM_CCER_CC2E |  TIM_CCER_CC1E;    //2 3n
        Tim->CCER    |= TIM_CCER_CC1P  |  TIM_CCER_CC2P  |  TIM_CCER_CC3P;   //1 3n
        Tim->CCER    |= TIM_CCER_CC1NP  |  TIM_CCER_CC2NP  |  TIM_CCER_CC3NP;   //1 3n
	Tim->CCR2 = (uint16_t)(Tim->ARR*1.0f);
	Tim->CCR3 = (uint16_t)(Tim->ARR*Amp);
	Tim->CCR1 = 0;
	break;
        // VT3 & VT2
	case 2:
        Tim->CCER   &= ~(TIM_CCER_CC1NE |  TIM_CCER_CC1E  |  TIM_CCER_CC2NE | 
                          TIM_CCER_CC2E  |  TIM_CCER_CC3NE |  TIM_CCER_CC3E);
	Tim->CCER  |= TIM_CCER_CC2E | TIM_CCER_CC1NE | TIM_CCER_CC3E ;    //2 1n
        Tim->CCER    |= TIM_CCER_CC1P  |  TIM_CCER_CC2P  |  TIM_CCER_CC3P;   //1 3n
        Tim->CCER    |= TIM_CCER_CC1NP  |  TIM_CCER_CC2NP  |  TIM_CCER_CC3NP;   //1 3n
	Tim->CCR2 = (uint16_t)(Tim->ARR*1.0f);
	Tim->CCR1 = (uint16_t)(Tim->ARR*Amp);
	Tim->CCR3 = 0;
	break;
        // VT5 & VT2	
	case 3:
        Tim->CCER   &= ~(TIM_CCER_CC1NE |  TIM_CCER_CC1E  |  TIM_CCER_CC2NE | 
                          TIM_CCER_CC2E  |  TIM_CCER_CC3NE |  TIM_CCER_CC3E);
	Tim->CCER  |= TIM_CCER_CC3E  | TIM_CCER_CC1NE | TIM_CCER_CC2E;   //3 1n
        Tim->CCER    |= TIM_CCER_CC1P  |  TIM_CCER_CC2P  |  TIM_CCER_CC3P;   //1 3n
        Tim->CCER    |= TIM_CCER_CC1NP  |  TIM_CCER_CC2NP  |  TIM_CCER_CC3NP;   //1 3n
	Tim->CCR3 = (uint16_t)(Tim->ARR*1.0f);
	Tim->CCR1 = (uint16_t)(Tim->ARR*Amp);
	Tim->CCR2 = 0;
	break;
        // VT5 & VT4	
	case 4:
        Tim->CCER   &= ~(TIM_CCER_CC1NE |  TIM_CCER_CC1E  |  TIM_CCER_CC2NE | 
                          TIM_CCER_CC2E  |  TIM_CCER_CC3NE |  TIM_CCER_CC3E);
	Tim->CCER  |= TIM_CCER_CC3E  | TIM_CCER_CC2NE | TIM_CCER_CC1E;     //3 2n
        Tim->CCER    |= TIM_CCER_CC1P  |  TIM_CCER_CC2P  |  TIM_CCER_CC3P;   //1 3n
        Tim->CCER    |= TIM_CCER_CC1NP  |  TIM_CCER_CC2NP  |  TIM_CCER_CC3NP;   //1 3n
	Tim->CCR3 = (uint16_t)(Tim->ARR*1.0f);
	Tim->CCR2 = (uint16_t)(Tim->ARR*Amp);
	Tim->CCR1 = 0;
	break;
        // VT1 & VT4
	case 5:
        Tim->CCER   &= ~(TIM_CCER_CC1NE |  TIM_CCER_CC1E  |  TIM_CCER_CC2NE | 
                          TIM_CCER_CC2E  |  TIM_CCER_CC3NE |  TIM_CCER_CC3E);
	Tim->CCER  |= TIM_CCER_CC1E  | TIM_CCER_CC2NE | TIM_CCER_CC3E;	     //1 2n
        Tim->CCER    |= TIM_CCER_CC1P  |  TIM_CCER_CC2P  |  TIM_CCER_CC3P;   //1 3n
        Tim->CCER    |= TIM_CCER_CC1NP  |  TIM_CCER_CC2NP  |  TIM_CCER_CC3NP;   //1 3n
	
	Tim->CCR1 = (uint16_t)(Tim->ARR*1.0f);
	Tim->CCR2 = (uint16_t)(Tim->ARR*Amp);
	Tim->CCR3 = 0;
	break;
	// All HiZ (floating input)
	default:
        TIM1->CCER   &= ~(TIM_CCER_CC1NE |  TIM_CCER_CC1E  |  TIM_CCER_CC2NE | 
                          TIM_CCER_CC2E  |  TIM_CCER_CC3NE |  TIM_CCER_CC3E);
        Tim->CCER    |= TIM_CCER_CC1P  |  TIM_CCER_CC2P  |  TIM_CCER_CC3P;   //1 3n
        Tim->CCER    |= TIM_CCER_CC1NP  |  TIM_CCER_CC2NP  |  TIM_CCER_CC3NP;   //1 3n
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	break;
}
}

/*
DSP_K_PWM_TIM1_Blocked_4VT
*/
 void DSP_K_PWM_Blocked_4VT(TIM_TypeDef *Tim, uint8_t Table, float Amp)
{
  uint16_t tmp;
    
switch (Table)
{       // VT1 & VT6  and  VT4 & VT2
	case 0:
        Tim->CCER   &= ~(TIM_CCER_CC1NE |  TIM_CCER_CC1E  |  TIM_CCER_CC2NE | 
                          TIM_CCER_CC2E  |  TIM_CCER_CC3NE |  TIM_CCER_CC3E);
        Tim->CCER    |= TIM_CCER_CC3NE  |  TIM_CCER_CC1E  |     //1 3n
                        TIM_CCER_CC1NE  |  TIM_CCER_CC3E  ;     //3 1n
        Tim->CCER    |= TIM_CCER_CC2E;
        tmp = (uint16_t)(0.5f*(float)(Tim->ARR)*(1.0f+Amp));
	Tim->CCR3 = tmp;
	Tim->CCR1 = (uint16_t)(Tim->ARR) - tmp;
        Tim->CCR2 = 0;
	break;
        // VT3 & VT6  and  VT4 & VT2	
	case 1:
        Tim->CCER   &= ~(TIM_CCER_CC1NE |  TIM_CCER_CC1E  |  TIM_CCER_CC2NE | 
                          TIM_CCER_CC2E  |  TIM_CCER_CC3NE |  TIM_CCER_CC3E);
        Tim->CCER    |= TIM_CCER_CC3NE  |  TIM_CCER_CC2E  |     //2 3n
                        TIM_CCER_CC2NE  |  TIM_CCER_CC3E  ;     //3 2n
        Tim->CCER    |= TIM_CCER_CC1E;
        tmp = (uint16_t)(0.5f*(float)(Tim->ARR)*(1.0f+Amp));
	Tim->CCR3 = tmp;
	Tim->CCR2 = (uint16_t)(Tim->ARR) - tmp;
        Tim->CCR1 = 0;
	break;
        // VT3 & VT2  and  VT4 & VT2
	case 2:
        Tim->CCER   &= ~(TIM_CCER_CC1NE |  TIM_CCER_CC1E  |  TIM_CCER_CC2NE | 
                          TIM_CCER_CC2E  |  TIM_CCER_CC3NE |  TIM_CCER_CC3E);
        Tim->CCER    |= TIM_CCER_CC1NE  |  TIM_CCER_CC2E  |     //2 1n
                        TIM_CCER_CC2NE  |  TIM_CCER_CC1E  ;     //1 2n
        Tim->CCER    |= TIM_CCER_CC3E;
        tmp = (uint16_t)(0.5f*(float)(Tim->ARR)*(1.0f+Amp));
	Tim->CCR1 = tmp;
	Tim->CCR2 = (uint16_t)(Tim->ARR) - tmp;
        Tim->CCR3 = 0;
	break;
        // VT5 & VT2  and  VT4 & VT2	
	case 3:
        Tim->CCER   &= ~(TIM_CCER_CC1NE |  TIM_CCER_CC1E  |  TIM_CCER_CC2NE | 
                          TIM_CCER_CC2E  |  TIM_CCER_CC3NE |  TIM_CCER_CC3E);
        Tim->CCER    |= TIM_CCER_CC1NE  |  TIM_CCER_CC3E  |     //3 1n
                        TIM_CCER_CC3NE  |  TIM_CCER_CC1E  ;     //1 3n
        Tim->CCER    |= TIM_CCER_CC2E;
        tmp = (uint16_t)(0.5f*(float)(Tim->ARR)*(1.0f+Amp));
	Tim->CCR1 = tmp;
	Tim->CCR3 = (uint16_t)(Tim->ARR) - tmp;
        Tim->CCR2 = 0;
	break;
        // VT5 & VT4  and  VT4 & VT2	
	case 4:
        Tim->CCER   &= ~(TIM_CCER_CC1NE |  TIM_CCER_CC1E  |  TIM_CCER_CC2NE | 
                          TIM_CCER_CC2E  |  TIM_CCER_CC3NE |  TIM_CCER_CC3E);
        Tim->CCER    |= TIM_CCER_CC2NE  |  TIM_CCER_CC3E  |     //3 2n
                        TIM_CCER_CC3NE  |  TIM_CCER_CC2E  ;     //2 3n
        Tim->CCER    |= TIM_CCER_CC1E;
        tmp = (uint16_t)(0.5f*(float)(Tim->ARR)*(1.0f+Amp));
	Tim->CCR2 = tmp;
	Tim->CCR3 = (uint16_t)(Tim->ARR) - tmp;
        Tim->CCR1 = 0;
	break;
        // VT1 & VT4  and  VT4 & VT2
	case 5:
        Tim->CCER   &= ~(TIM_CCER_CC1NE |  TIM_CCER_CC1E  |  TIM_CCER_CC2NE | 
                          TIM_CCER_CC2E  |  TIM_CCER_CC3NE |  TIM_CCER_CC3E);
        Tim->CCER    |= TIM_CCER_CC2NE  |  TIM_CCER_CC1E  |     //1 2n
                        TIM_CCER_CC1NE  |  TIM_CCER_CC2E  ;     //2 1n
        Tim->CCER    |= TIM_CCER_CC3E;
        tmp = (uint16_t)(0.5f*(float)(Tim->ARR)*(1.0f+Amp));
	Tim->CCR2 = tmp;
	Tim->CCR1 = (uint16_t)(Tim->ARR) - tmp;
        Tim->CCR3 = 0;
	break;
	// All HiZ (floating input)
	default:
        Tim->CCER  = 0;
	Tim->CCR1 = 0;
	Tim->CCR2 = 0;
	Tim->CCR3 = 0;
	break;
}
}

/*
DSP_K_PWM_TIM1_Blocked_4VT
*/
void DSP_K_SVPWM(TIM_TypeDef *Tim, float A, float B)
{
 
static float bufer[2];

float f_sin      = 0.0f;
float f_sin_0_60 = 0.0f;

float f_cos      = 0.0f;
float f_cos_0_60 = 0.0f;
 
static float module     = 0.0f;

float Tb1 = 0.0f;
float Tb2 = 0.0f;
float T0  = 0.0f;

float t1 = 0.0f;
float t2 = 0.0f;
float t3  = 0.0f;


bufer[0]=A;
bufer[1]=B;
arm_cmplx_mag_f32(bufer,&module,2);

f_sin = bufer[1] / module;
f_cos = bufer[0] / module;

if(module==0)
{
Tim->CCR1 = (int)Tim->ARR/2;
Tim->CCR2 = (int)Tim->ARR/2;
Tim->CCR3 = (int)Tim->ARR/2;
}
//########################################### 1
 else if((f_sin >= sin_0) && (f_sin < sin_60)  &&\
    (f_cos > cos_60) && (f_cos <= cos_0) )
 {


 
 f_sin_0_60 = f_sin;
 f_cos_0_60 = f_cos;
 
 Tb1 = module * (sin_60 * f_cos_0_60 - cos_60 * f_sin_0_60);
 Tb2 = module * f_sin_0_60;
 T0  = 0.5f*(1.0f-Tb1-Tb2);
 
 t1 = Tb1+Tb2+T0;
 t2 = Tb2+T0;
 t3 = Tb1+T0;

Tim->CCR1 = (int)(Tim->ARR*t1+0);
Tim->CCR2 = (int)(Tim->ARR*t2+0);
Tim->CCR3 = (int)(Tim->ARR*T0+0);


 
 }
 //###########################################2
  else if((f_sin >= sin_60)   &&\
    (f_cos > cos_120) && (f_cos <= cos_60) )
 {
 
 f_sin_0_60 = f_sin * cos_60 - sin_60 * f_cos;
 f_cos_0_60 = f_cos * cos_60 + f_sin * sin_60;
 
 Tb1 = module * (sin_60 * f_cos_0_60 - cos_60 * f_sin_0_60);
 Tb2 = module * f_sin_0_60;
 T0  = 0.5f*(1.0f-Tb1-Tb2);
 
 t1 = Tb1+Tb2+T0;
 t2 = Tb2+T0;
 t3 = Tb1+T0;
 
Tim->CCR1 = (int)(Tim->ARR*t3+0);
Tim->CCR2 = (int)(Tim->ARR*t1+0);
Tim->CCR3 = (int)(Tim->ARR*T0+0);
 }
 //###########################################3
 else if((f_sin > sin_180) && (f_sin <= sin_120)  &&\
    (f_cos > cos_180) && (f_cos <= cos_120) )
 {
 
 f_sin_0_60 = f_sin * cos_120 - sin_120 * f_cos;
 f_cos_0_60 = f_cos * cos_120 + f_sin * sin_120;
 
 Tb1 = module * (sin_60 * f_cos_0_60 - cos_60 * f_sin_0_60);
 Tb2 = module * f_sin_0_60;
 T0  = 0.5f*(1.0f-Tb1-Tb2);
 
 t1 = Tb1+Tb2+T0;
 t2 = Tb2+T0;
 t3 = Tb1+T0;
 
Tim->CCR1 = (int)(Tim->ARR*T0+0);
Tim->CCR2 = (int)(Tim->ARR*t1+0);
Tim->CCR3 = (int)(Tim->ARR*t2+0);
 }
 //###########################################4
  else if((f_sin > sin_240) && (f_sin <= sin_180)  &&\
    (f_cos >= cos_180) && (f_cos < cos_240) )
 {
 
 f_sin_0_60 = f_sin * cos_180 - sin_180 * f_cos;
 f_cos_0_60 = f_cos * cos_180 + f_sin * sin_180;
 
 Tb1 = module * (sin_60 * f_cos_0_60 - cos_60 * f_sin_0_60);
 Tb2 = module * f_sin_0_60;
 T0  = 0.5f*(1.0f-Tb1-Tb2);
 
 t1 = Tb1+Tb2+T0;
 t2 = Tb2+T0;
 t3 = Tb1+T0;
 
Tim->CCR1 = (int)(Tim->ARR*T0+0);
Tim->CCR2 = (int)(Tim->ARR*t3+0);
Tim->CCR3 = (int)(Tim->ARR*t1+0);
 }
 //###########################################5
  else if( (f_sin <=sin_240)  &&\
    (f_cos >= cos_240) && (f_cos < cos_300) )
 {
 f_sin_0_60 = f_sin * cos_240 - sin_240 * f_cos;
 f_cos_0_60 = f_cos * cos_240 + f_sin * sin_240;
 
 Tb1 = module * (sin_60 * f_cos_0_60 - cos_60 * f_sin_0_60);
 Tb2 = module * f_sin_0_60;
 T0  = 0.5f*(1.0f-Tb1-Tb2);
 
 t1 = Tb1+Tb2+T0;
 t2 = Tb2+T0;
 t3 = Tb1+T0;
 
Tim->CCR1 = (int)(Tim->ARR*t2+0);
Tim->CCR2 = (int)(Tim->ARR*T0+0);
Tim->CCR3 = (int)(Tim->ARR*t1+0);
 }
 //###########################################6
 else if((f_sin >= sin_300) && (f_sin < sin_0)  &&\
    (f_cos >= cos_300) && (f_cos < cos_0) )
 {
 
 f_sin_0_60 = f_sin * cos_300 - sin_300 * f_cos;
 f_cos_0_60 = f_cos * cos_300 + f_sin * sin_300;
 
 Tb1 = module * (sin_60 * f_cos_0_60 - cos_60 * f_sin_0_60);
 Tb2 = module * f_sin_0_60;
 T0  = 0.5f*(1.0f-Tb1-Tb2);
 
 t1 = Tb1+Tb2+T0;
 t2 = Tb2+T0;
 t3 = Tb1+T0;
 
Tim->CCR1 = (int)(Tim->ARR*t1+0);
Tim->CCR2 = (int)(Tim->ARR*T0+0);
Tim->CCR3 = (int)(Tim->ARR*t3+0);
 }
 
//DWT_CYCCNT = 0;  //if need to start from zero
//sin_data = arm_sin_f32(0.01f);
//Tick=DWT_CYCCNT;



}