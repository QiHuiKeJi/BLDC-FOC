#include "main.h"
#include "mylib.h"
#include "hall_speed_pos_fdbk.h"

float Vq;
float	Vd;
float Va, Vb, Vc, Va_1, Vb_1, Vc_1;
float theta, theta_elec_degrees;
float Vinv1,Vinv2, Vinv3;
float angle_init , error_angle_last, sine_init, cos_init;
extern float angle ;


void Set_ENx(void)
{
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_8|LL_GPIO_PIN_9|LL_GPIO_PIN_10); // EN1,2,3 to 1 enable all half-bridges
}


void FOC_InitPosition(void) // establishing zero position, d-axis directed to A winding, theta = 90
{
	/*
	if(CALIBRATION)
	{
	Vq=3;
	
	Va_1 = arm_cos_f32(0);//cos(theta         );     
	Vb_1 = arm_cos_f32(0 - 2.0943951023931954923084289221863);//cos(theta - 2.0943951023931954923084289221863 ); //2*Pi/3
	Vc_1 = arm_cos_f32(0 + 2.0943951023931954923084289221863);//cos(theta + 2.0943951023931954923084289221863);
	
	Va = Va_1 * Vq; // projection calculation of Vq into A phase
	Vb = Vb_1 * Vq; // projection calculation of Vq into B phase
	Vc = Vc_1 * Vq; // projection calculation of Vq into C phase
	
	Vinv1 = Va + 6; // Obtaining value for invertor, +6 because Vinv relates with V_phase as Vinv = Vphase + Vdc/2 in order to avoid negative values for invertor voltage.
	Vinv2 = Vb + 6; // should also be taken into account that Vphase(max) = Vdc/2 (with sine PWM) 
	Vinv3 = Vc + 6;
	
	TIM1->CCR1 = (uint32_t)(Vinv1*PWM_period/Vdc)  ; 
  TIM1->CCR2 = (uint32_t)(Vinv2*PWM_period/Vdc)  ;
  TIM1->CCR3 = (uint32_t)(Vinv3*PWM_period/Vdc)  ;
	
	myDelay_ms(1000);	
		
	angle_init = CQ_average_angle();
		
	}
	else
		
	{
	
	// init angle was calculated once. Now it is used like starting point for electrical angles and engine does not need position initialization
		
	angle_init = 238.066;
	
	} */

	angle_init = 238.066;
}


uint32_t vaa,vbb,vcc;
float error_in_proc, er_mem, angle_mem,integral;

void FOC(float angle, float error_angle, float K_p, float K_d, float K_I, uint32_t dt)
{
	HALL_CalcElAngle (&HALL_M1);
	
	theta_elec_degrees = ((angle - angle_init)*Pole_Pairs + 90 ); // 11 - pole pairs (22P). + 90 because at initial position theta = 90  
	theta = theta_elec_degrees*0.01745329251994329576923690768489 ;//Pi/180; // translating into radians
	 
	
	Vd = 0; // !
	
	if(error_angle > 180)
	{
		error_angle = 360 - error_angle;
		error_angle = - error_angle;
	}
	if(error_angle < -180)
	{
		error_angle = 360 + error_angle;
		//error_angle = - error_angle;
	}
	
	
	error_in_proc = error_angle;
	if ((error_angle > 100)||(error_angle< -100)) 
	{
		er_mem = error_angle;
		angle_mem = angle;
	}
	integral = dt*0.000001*error_angle_last+integral;
	Vq = K_p*error_angle ;//+ ((error_angle - error_angle_last)/(dt*0.000001))*K_d + integral*K_I; //Speed; //
	error_angle_last = error_angle;
	
	
	
	
	if(Vq < -6) Vq = -6; // 6V = Vdc/2 , voltage limitation
	if(Vq > 6) Vq = 6;
	
	
	
	Va_1 = arm_cos_f32(theta);//cos(theta         );     
	Vb_1 = arm_cos_f32(theta - 2.0943951023931954923084289221863);//cos(theta - 2.0943951023931954923084289221863 /* 2*Pi/3 */);
	Vc_1 = arm_cos_f32(theta + 2.0943951023931954923084289221863);//cos(theta + 2.0943951023931954923084289221863);
	
	
	Va = Va_1 * Vq; // projection calculation of Vq into A phase
	Vb = Vb_1 * Vq; // projection calculation of Vq into B phase
	Vc = Vc_1 * Vq; // projection calculation of Vq into C phase
	
	Vinv1 = Va + 6; // Obtaining value for invertor, +6 because Vinv relates with V_phase as Vinv = Vphase + Vdc/2 in order to avoid negative values for invertor voltage.
	Vinv2 = Vb + 6; // should also be taken into account that Vphase(max) = Vdc/2 (with sine PWM) 
	Vinv3 = Vc + 6;
	
	// Vinx_max = 12V, PWM = Vinv*PWM_period/Vinv_max
	TIM1->CCR1 = (uint32_t)(Vinv1*PWM_period/Vdc)  ; 
  TIM1->CCR2 = (uint32_t)(Vinv2*PWM_period/Vdc)  ;
  TIM1->CCR3 = (uint32_t)(Vinv3*PWM_period/Vdc)  ;	
	
	vaa = TIM1->CCR1;
	vbb = TIM1->CCR2;
	vcc = TIM1->CCR3;
	
}





void sinus_control(float des_val_)
{
	theta_elec_degrees = ((des_val_)*11 + 90 ); // 11 - pole pairs (22P). + 90 because at initial position theta = 90  
	theta = theta_elec_degrees*0.01745329251994329576923690768489 ;//Pi/180; // translating into radians
	
	Vq=6;
	
	
	Va_1 = arm_cos_f32(theta);//cos(theta         );     
	Vb_1 = arm_cos_f32(theta - 2.0943951023931954923084289221863);//cos(theta - 2.0943951023931954923084289221863 /* 2*Pi/3 */);
	Vc_1 = arm_cos_f32(theta + 2.0943951023931954923084289221863);//cos(theta + 2.0943951023931954923084289221863);
	
	
	Va = Va_1 * Vq; // projection calculation of Vq into A phase
	Vb = Vb_1 * Vq; // projection calculation of Vq into B phase
	Vc = Vc_1 * Vq; // projection calculation of Vq into C phase
	
	Vinv1 = Va + 6; // Obtaining value for invertor, +6 because Vinv relates with V_phase as Vinv = Vphase + Vdc/2 in order to avoid negative values for invertor voltage.
	Vinv2 = Vb + 6; // should also be taken into account that Vphase(max) = Vdc/2 (with sine PWM) 
	Vinv3 = Vc + 6;
	
	// Vinx_max = 12V, PWM = Vinv*PWM_period/Vinv_max
	TIM1->CCR1 = (uint32_t)(Vinv1*PWM_period/Vdc)  ; 
  TIM1->CCR2 = (uint32_t)(Vinv2*PWM_period/Vdc)  ;
  TIM1->CCR3 = (uint32_t)(Vinv3*PWM_period/Vdc)  ;
}



uint8_t started;
float step, step2;
void sinus_control_V2(float error_angle, float V, float K, float step_max)
{
	
	
	if(error_angle > 180)
	{
		error_angle = 360 - error_angle;
		error_angle = - error_angle;
	}
	if(error_angle < -180)
	{
		error_angle = 360 + error_angle;
		//error_angle = - error_angle;
	}
	
	
	error_in_proc = error_angle;
	if ((error_angle > 100)||(error_angle< -100)) 
	{
		er_mem = error_angle;
		angle_mem = angle;
	}
	
	
	if(!started)
	{
		theta = 0;
		Va_1 = arm_cos_f32(theta);//cos(theta         );     
		Vb_1 = arm_cos_f32(theta - 2.0943951023931954923084289221863);//cos(theta - 2.0943951023931954923084289221863 /* 2*Pi/3 */);
		Vc_1 = arm_cos_f32(theta + 2.0943951023931954923084289221863);//cos(theta + 2.0943951023931954923084289221863);
		
		
		Va = Va_1 * V; // projection calculation of Vq into A phase
		Vb = Vb_1 * V; // projection calculation of Vq into B phase
		Vc = Vc_1 * V; // projection calculation of Vq into C phase
		
		Vinv1 = Va + 6; // Obtaining value for invertor, +6 because Vinv relates with V_phase as Vinv = Vphase + Vdc/2 in order to avoid negative values for invertor voltage.
		Vinv2 = Vb + 6; // should also be taken into account that Vphase(max) = Vdc/2 (with sine PWM) 
		Vinv3 = Vc + 6;
		
		// Vinx_max = 12V, PWM = Vinv*PWM_period/Vinv_max
		TIM1->CCR1 = (uint32_t)(Vinv1*PWM_period/Vdc)  ; 
		TIM1->CCR2 = (uint32_t)(Vinv2*PWM_period/Vdc)  ;
		TIM1->CCR3 = (uint32_t)(Vinv3*PWM_period/Vdc)  ;
		started =1;
	}
	
	
	step = error_angle*K;
	
	if(step>step_max) { step=step_max;}
	if(step< -step_max) {step=-step_max; }
	
	theta = theta+step;
		
	
	Va_1 = arm_cos_f32(theta);//cos(theta         );     
	Vb_1 = arm_cos_f32(theta - 2.0943951023931954923084289221863);//cos(theta - 2.0943951023931954923084289221863 /* 2*Pi/3 */);
	Vc_1 = arm_cos_f32(theta + 2.0943951023931954923084289221863);//cos(theta + 2.0943951023931954923084289221863);
	
	
	Va = Va_1 * V; // projection calculation of Vq into A phase
	Vb = Vb_1 * V; // projection calculation of Vq into B phase
	Vc = Vc_1 * V; // projection calculation of Vq into C phase
	
	Vinv1 = Va + 6; // Obtaining value for invertor, +6 because Vinv relates with V_phase as Vinv = Vphase + Vdc/2 in order to avoid negative values for invertor voltage.
	Vinv2 = Vb + 6; // should also be taken into account that Vphase(max) = Vdc/2 (with sine PWM) 
	Vinv3 = Vc + 6;
	
	// Vinx_max = 12V, PWM = Vinv*PWM_period/Vinv_max
	TIM1->CCR1 = (uint32_t)(Vinv1*PWM_period/Vdc)  ; 
  TIM1->CCR2 = (uint32_t)(Vinv2*PWM_period/Vdc)  ;
  TIM1->CCR3 = (uint32_t)(Vinv3*PWM_period/Vdc)  ;	
}





extern float sin_x, cos_x, tv_g, t_g, t_d;
float thetta_vector;
void combined_control_V3(float angle, float error_angle, float V, float K, float step_max)
{
	
	
	if(error_angle > 180)
	{
		error_angle = 360 - error_angle;
		error_angle = - error_angle;
	}
	if(error_angle < -180)
	{
		error_angle = 360 + error_angle;
		//error_angle = - error_angle;
	}
	
	
	error_in_proc = error_angle;
	if ((error_angle > 100)||(error_angle< -100)) 
	{
		er_mem = error_angle;
		angle_mem = angle;
	}
	
	
	
	
	
	theta_elec_degrees = ((angle - angle_init)*Pole_Pairs + 90 ); // 11 - pole pairs (22P). + 90 because at initial position theta = 90  
	theta = theta_elec_degrees*0.01745329251994329576923690768489 ;//Pi/180; // translating into radians
	 
	
	if(!started)
	{
		thetta_vector = theta - PI/2;// aim voltage vector to d axis
		Va_1 = arm_cos_f32(thetta_vector);//cos(theta         );     
	Vb_1 = arm_cos_f32(thetta_vector - 2.0943951023931954923084289221863);//cos(theta - 2.0943951023931954923084289221863 ); //2*Pi/3
	Vc_1 = arm_cos_f32(thetta_vector + 2.0943951023931954923084289221863);//cos(theta + 2.0943951023931954923084289221863);
	
		
	
	
	Va = Va_1 * V; // projection calculation of Vq into A phase
	Vb = Vb_1 * V; // projection calculation of Vq into B phase
	Vc = Vc_1 * V; // projection calculation of Vq into C phase
	
	Vinv1 = Va + 6; // Obtaining value for invertor, +6 because Vinv relates with V_phase as Vinv = Vphase + Vdc/2 in order to avoid negative values for invertor voltage.
	Vinv2 = Vb + 6; // should also be taken into account that Vphase(max) = Vdc/2 (with sine PWM) 
	Vinv3 = Vc + 6;
		
		
	
	// Vinx_max = 12V, PWM = Vinv*PWM_period/Vinv_max
	TIM1->CCR1 = (uint32_t)(Vinv1*PWM_period/Vdc)  ; 
  TIM1->CCR2 = (uint32_t)(Vinv2*PWM_period/Vdc)  ;
  TIM1->CCR3 = (uint32_t)(Vinv3*PWM_period/Vdc)  ;
	started =1;
	
	}
	
	
	
	
	
	
	step = error_angle*K;
//	step = error_angle*0.02;
	//step2 = step;
	//theta_elec_degrees = ((err)*11 + 90 ); // 11 - pole pairs (22P). + 90 because at initial position theta = 90  
	
	
	if(step>step_max) { step=step_max;}
	if(step< -step_max) {step=-step_max; }
	
	thetta_vector = thetta_vector+step;
	
	
	
	/*
	// tranlating to unit circle for visual determination of angle of loosing synchronisation
	
	//sin_x = arm_sin_f32(thetta_vector);
	//cos_x = arm_cos_f32(thetta_vector); 
	tv_g = thetta_vector*57.295779513082320876798154814105 ;//atan2(sin_x, cos_x)*57.295779513082320876798154814105 ;
	
	//sin_x = arm_sin_f32(theta);
	//cos_x = arm_cos_f32(theta); 
	t_g = theta*57.295779513082320876798154814105 ;//atan2(sin_x, cos_x)*57.295779513082320876798154814105 ;
	
	t_d = tv_g - t_g;
	
	*/	

	
	if((thetta_vector - theta) < -PI )
	{
		thetta_vector = theta - PI ;//- 2*PI;
	}
	if((thetta_vector - theta)> 0 ) 
	{
		thetta_vector = theta;
	}
	
	
	//if(thetta_vector < - theta) thetta_vector = - theta;
		
	
	
	Va_1 = arm_cos_f32(thetta_vector);//cos(theta         );     
	Vb_1 = arm_cos_f32(thetta_vector - 2.0943951023931954923084289221863);//cos(theta - 2.0943951023931954923084289221863 );
	Vc_1 = arm_cos_f32(thetta_vector + 2.0943951023931954923084289221863);//cos(theta + 2.0943951023931954923084289221863);
	
	
	Va = Va_1 * V; // projection calculation of Vq into A phase
	Vb = Vb_1 * V; // projection calculation of Vq into B phase
	Vc = Vc_1 * V; // projection calculation of Vq into C phase
	
	Vinv1 = Va + 6; // Obtaining value for invertor, +6 because Vinv relates with V_phase as Vinv = Vphase + Vdc/2 in order to avoid negative values for invertor voltage.
	Vinv2 = Vb + 6; // should also be taken into account that Vphase(max) = Vdc/2 (with sine PWM) 
	Vinv3 = Vc + 6;
	
	// Vinx_max = 12V, PWM = Vinv*PWM_period/Vinv_max
	TIM1->CCR1 = (uint32_t)(Vinv1*PWM_period/Vdc)  ; 
  TIM1->CCR2 = (uint32_t)(Vinv2*PWM_period/Vdc)  ;
  TIM1->CCR3 = (uint32_t)(Vinv3*PWM_period/Vdc)  ;	
} 