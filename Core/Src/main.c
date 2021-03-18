/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "hall_speed_pos_fdbk.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t mode;
float angle, angle_error;
float des_val;
uint32_t t1, t2, dt1;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
uint32_t ccval = 500;
float da = 0.1;
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/

  /** DISABLE: JTAG-DP Disabled and SW-DP Disabled
  */
//  LL_GPIO_AF_DisableRemap_SWJ();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
//	NVIC_EnableIRQ(SysTick_IRQn);
  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
	MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	MX_TIM2_Init();
	Set_ENx();

	LL_TIM_EnableCounter(TIM1);
	LL_TIM_EnableAllOutputs(TIM1);
	
	HALL_Init (&HALL_M1);
	NVIC_EnableIRQ(TIM4_IRQn);
	NVIC_EnableIRQ(TIM1_UP_IRQn);
	des_val = 360000;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		Delay_ms(1);
//		ccval++;
//		if(ccval > PWM_period) ccval = 0;
//		LL_TIM_OC_SetCompareCH1(TIM1, ccval);
		
//		angle = angle + da;
//		
////	angle = CQ_average_angle();
////  des_val = ADC_average*360/4095;  
//	angle_error = des_val - angle;	
//	

//	if(mode==0)	FOC(angle, angle_error, 1.1,   0,  0.01,  dt1)	;		
// 
//	if(mode==1) sinus_control_V2(angle_error, 6, 0.001, 0.05);
//		
//	if(mode==2)combined_control_V3(angle, angle_error, 6, 0.001, 0.05);
		

	led_blink();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}


void control_poll(void)
{

		angle = angle + da;
		
//	angle = CQ_average_angle();
//  des_val = ADC_average*360/4095;  
	angle_error = des_val - angle;	
	

	if(mode==0)	FOC(angle, angle_error, 1.1,   0,  0.01,  dt1)	;		
 
	if(mode==1) sinus_control_V2(angle_error, 6, 0.001, 0.05);
		
	if(mode==2)combined_control_V3(angle, angle_error, 6, 0.001, 0.05);

}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
