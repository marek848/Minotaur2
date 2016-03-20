/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"

/* USER CODE BEGIN 0 */
#include "jazda.h"

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim4;

/******************************************************************************/
/*            Cortex-M3 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles DMA1 channel1 global interrupt.
*/
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
	uint8_t i;
//	counter++;

		for(i=0;i<6;i++)
		{
			test=adcData[i];
			if(test>Max3[i]) Max3[i]=test;
			if(test<Min3[i]) Min3[i]=test;
		}

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
* @brief This function handles TIM4 global interrupt.
*/
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
	static uint8_t count=0;
	static int32_t rate[3]={0,0,0},prev_rate=0,tmp_rate[2],prev_vel=0;

	count++;
	indexer=count%5;
	if(count%2==0)
		{
			HAL_GPIO_WritePin(Sensor2_GPIO_Port,Sensor2_Pin,0);
			HAL_GPIO_WritePin(Sensor1_GPIO_Port,Sensor1_Pin,1);
//			HAL_GPIO_WritePin(Sensor3_GPIO_Port,Sensor3_Pin,1);

			for(i=0;i<6;i++)
			{
				if(i==1||i==2||i==5) MinMax[i][0]=Max3[i];
				if(i==0||i==3||i==4) MinMax[i][1]=Min3[i];
				SensorTab[i][indexer]=(MinMax[i][0]-MinMax[i][1])/10;
				Min3[i]=4000;
				Max3[i]=0;
			}
		}
		else
		{
			HAL_GPIO_WritePin(Sensor1_GPIO_Port,Sensor1_Pin,0);
			HAL_GPIO_WritePin(Sensor2_GPIO_Port,Sensor2_Pin,1);
//			HAL_GPIO_WritePin(Sensor3_GPIO_Port,Sensor3_Pin,0);

			for(i=0;i<6;i++)
			{
				if(i==0||i==3||i==4) MinMax[i][0]=Max3[i];
				if(i==1||i==2||i==5) MinMax[i][1]=Min3[i];
				SensorTab[i][indexer]=(MinMax[i][0]-MinMax[i][1])/10;
				Min3[i]=4000;
				Max3[i]=0;
			}
		}
	test3[0]=SensorTab[4][indexer];

	/******************************************* Measurement rotational speed ***********************************/
	if(count%4==3)
	{
		tmp_rate[0]=TIM2->CNT;
		tmp_rate[1]=TIM3->CNT;
		rate[0]=( tmp_rate[0]-tmp_rate[1] )*16; //64/4
		lin_vel=((tmp_rate[1]-16384)+(tmp_rate[0]-16384))*8;// 17/2
		TIM3->CNT=16384;
		TIM2->CNT=16384;
	}
	rate[1]=((Read_AXIS(0x2C)-dryf)*175)/10000;
	rot_vel=(rate[1]+rate[1])/2;
	angle +=(prev_rate + rot_vel)/2;
	distance +=(prev_vel + lin_vel)/2; // [um]

	prev_rate = rot_vel;
	prev_vel = lin_vel;

	test3[4]=test3[1]-angle;
	if ((test3[4]<2000 && test3[4]>0)||(test3[4]>-2000 && test3[4]<0)) test3[2]++;

	/******************************************* Drive straight ***********************************/
	if(tryb==1)
	{
		if (SensorTab[2][indexer]-dys0[2]<50)
		{
			error=(SensorTab[0][indexer]-SensorTab[2][indexer])/2 - (SensorTab[2][indexer]-dys0[2]);
		}
		else if(SensorTab[3][indexer]-dys0[3]<50)
		{
			error= (SensorTab[3][indexer]-SensorTab[1][indexer])*3/4 + (SensorTab[3][indexer]-dys0[3]);
		}
		else error=-angle/100;

		propocjonal=error*K_drive;

		integral+=error*I_drive;
		if (integral>1000) integral=100;
		if (integral<-1000) integral=-100;

		derivative=(error-(SensorTab[3][(indexer+1)%5]-SensorTab[1][(indexer+1)%5])*3/4 + (SensorTab[3][(indexer+1)%5]-dys0[3]))*D_drive;

		regulator=propocjonal+integral+derivative;

		speed[0]=VEL-regulator;
		speed[1]=VEL+regulator;

		if(speed[0]>999) speed[0]=999;
		else if(speed[0]<0) speed[0]=0;
		if(speed[1]>999) speed[1]=999;
		else if(speed[1]<0) speed[1]=0;

		TIM1->CCR1=speed[0];
		TIM1->CCR2=speed[1];
	}

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
* @brief This function handles EXTI line[15:10] interrupts.
*/
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
	if(start==1) start=0;
	else start=1;


  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/