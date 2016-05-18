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

int32_t le_back, ri_back, le_fr, ri_fr;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim4;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart3;

/******************************************************************************/
/*            Cortex-M3 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
if(HAL_GetTick()%100==0 && Status!=STOP_STATUS && Transmit==1) HAL_UART_Transmit_DMA(&huart3,(uint8_t*)TxBuffer,34);
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
//	uint8_t i;
////	counter++;
//
//		for(i=0;i<6;i++)
//		{
//			test=adcData[i];
//			if(test>Max3[i]) Max3[i]=test;
//			if(test<Min3[i]) Min3[i]=test;
//		}

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel2 global interrupt.
*/
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */
	int32_t pom;

	TxBuffer[0]='#';
	for (i=0;i<8;i++)
	{
		if (i<6) pom=SensorTab[i]-dys0[i];
		if(i==6) pom=angle1/1000;
		if(i==7) pom=distance/1000;
		if (pom>0)
		{
			TxBuffer[i*4+1]='+';
			TxBuffer[i*4+2]=(uint8_t)( pom/100+48);
			TxBuffer[i*4+3]=(uint8_t)((pom%100)/10+48);
			TxBuffer[i*4+4]=(uint8_t)( pom%10+48);
		}
		else
		{
			TxBuffer[i*4+1]='-';
			TxBuffer[i*4+2]=(uint8_t)( (  pom*(-1) )/100+48);
			TxBuffer[i*4+3]=(uint8_t)( ( (pom*(-1) )%100)/10+48);
			TxBuffer[i*4+4]=(uint8_t)( (  pom*(-1))%10+48);
		}
	}
	TxBuffer[33]='~';
  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel3 global interrupt.
*/
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */
	static int8_t tmp_tryb=-1;

	if(RxBuffer[0]=='d')	//Drive
	{
		Status=DRIVE_STATUS;
		Transmit=1;
		tryb=tmp_tryb;
		tmp_tryb=-1;
	}
	if(RxBuffer[0]=='p')	//Pause
	{
		Status=PAUSE_STATUS;
		Transmit=1;
		tmp_tryb=tryb;
		tryb=0;
		TIM1->CCR1=0;
		TIM1->CCR2=0;
	}
	if(RxBuffer[0]=='r')	//Reset
	{
		NVIC_SystemReset();
	}
	if(RxBuffer[0]=='P')
	{
		tryb=1;
		drive(VEL);
		tryb=0;
	}
	if(RxBuffer[0]=='L') rotary(VELR,-95000);
	if(RxBuffer[0]=='R') rotary(VELR,90000);
  /* USER CODE END DMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
* @brief This function handles TIM4 global interrupt.
*/
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
	static uint8_t count=0;
	int i;
	static int32_t rate[3]={0,0,0},prev_rate=0,tmp_rate[2],prev_vel=0;

	indexer=count%4;
	switch (count%4)
	{
		case 0:
				HAL_GPIO_WritePin(Sensor2_GPIO_Port,Sensor2_Pin,1);
				HAL_GPIO_WritePin(Sensor1_GPIO_Port,Sensor1_Pin,1);
//				SensorTab[0]=adcDataOn[4]-adcDataOff[4];
//				SensorTab[1]=adcDataOn[5]-adcDataOff[5];
//				SensorTab[2]=adcDataOn[0]-adcDataOff[0];
//				SensorTab[3]=adcDataOn[1]-adcDataOff[1];
//				SensorTab[4]=adcDataOn[2]-adcDataOff[2];
//				SensorTab[5]=adcDataOn[3]-adcDataOff[3];
				for(i=0;i<6;i++) SensorTab[i]=(adcDataOn[i]-adcDataOff[i])/10;
				break;
		case 1:
				HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcDataOn, ADC_SIZE);
				break;
		case 2:
				HAL_GPIO_WritePin(Sensor1_GPIO_Port,Sensor1_Pin,0);
				HAL_GPIO_WritePin(Sensor2_GPIO_Port,Sensor2_Pin,0);
				break;
		case 3:
				HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcDataOff, ADC_SIZE);
				break;
	}
	count++;

	/******************************************* Measurement rotational speed ***********************************/
	if(count%4==3)
	{
		tmp_rate[0]=TIM2->CNT;
		tmp_rate[1]=TIM3->CNT;
		rate[0]=( tmp_rate[0]-tmp_rate[1] )*13; //64/4
		lin_vel=((tmp_rate[1]-16384)+(tmp_rate[0]-16384))*8;// 17/2
		TIM3->CNT=16384;
		TIM2->CNT=16384;
	}
	rate[1]=((Read_AXIS(0x2C)-dryf)*700)/10000;
	rot_vel=(rate[1]+rate[1])/2;
	angle +=(prev_rate + rot_vel)/2;
	angle1+=rate[0];
	distance +=(prev_vel + lin_vel)/2; // [um]

	prev_rate = rot_vel;
	prev_vel = lin_vel;

	test3[4]=test3[1]-angle;
	if ((test3[4]<1000 && test3[4]>0)||(test3[4]>-1000 && test3[4]<0)) test3[2]++;

	/******************************************* Drive straight ***********************************/
	if(tryb==1)
	{
		le_fr=SensorTab[2]-dys0[2];
		le_back=SensorTab[0]-dys0[0];
		ri_fr=SensorTab[3]-dys0[3];
		ri_back=SensorTab[1]-dys0[1];

		if (ri_back>SSR_Tresh &&  le_back>SSL_Tresh && abs(le_fr-le_back)<100 && abs(ri_fr-ri_back)<abs(SSR_Tresh))
		{
			error=(ri_back - le_back)*2/2 - angle1/90;
		}
		else if (le_fr>SSL_Tresh &&  le_back>SSL_Tresh && abs(le_fr-le_back)<abs(SSL_Tresh))
		{
			error=(le_back-le_fr)/4 - le_back*3/2 - angle1/90;
		}
		else if(ri_fr>SSR_Tresh && ri_back>SSR_Tresh && abs(ri_fr-ri_back)<abs(SSR_Tresh))
		{
			error= (ri_fr-ri_back)/4 + ri_back*3/2 - angle1/90;
		}
		else error=-angle1/30;

		propocjonal=error*K_drive;

		integral+=error*I_drive;
		if (integral>200) integral=100;
		if (integral<-200) integral=-100;

		derivative=(error-error2)*D_drive;
		error2=error;
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
* @brief This function handles USART3 global interrupt.
*/
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
* @brief This function handles EXTI line[15:10] interrupts.
*/
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
	if(Status==STOP_STATUS) Status=DRIVE_STATUS;
	else Status=STOP_STATUS;


  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
