/**
  ******************************************************************************
  * @file    GPIO/IOToggle/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "mpu6000.h"
#include "hw_config.h"
/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup GPIO_IOToggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */

/**
  * @}
  */
	int m1=0;
	int m2=0;
	extern 	int m2;
extern mpu6000_data_t mpu6000_data;
void EXTI3_IRQHandler() {
	
	
	if(EXTI_GetITStatus(EXTI_Line3) != RESET) //?????EXTI6???????????  
  //????????EXTI??????????
  {
       MPU6000_SPI_ReadSensor();
					mpu6000_data_t mpu6000_data_tmp;
			mpu6000_data_tmp=mpu6000_data;
			mpu6000_data_tmp.data.dummy=0x55;
		USB_TxWrite((uint8_t *)&mpu6000_data_tmp,sizeof(mpu6000_data));
  EXTI_ClearFlag(EXTI_Line3);    //???????(??)   ??????????,???????????
    
    EXTI_ClearITPendingBit(EXTI_Line3); //??EXTI?????
		m1++;
  }
}

int n=0;
void DMA1_Channel2_IRQHandler(){
	n++;

    DMA_ClearFlag((DMA1_FLAG_TC2 | DMA1_FLAG_TE2 | DMA1_FLAG_HT2 | DMA1_FLAG_GL2));

    /* Wait for the final bytes of the transfer to complete, including CRC byte(s). */
    while (!(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE))) {
        ;
    }

    /* Wait for the final bytes of the transfer to complete, including CRC byte(s). */
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY)) {
        ;
    }
	 

   
}
void DMA1_Channel3_IRQHandler(){
	n++;

    DMA_ClearFlag((DMA1_FLAG_TC3 | DMA1_FLAG_TE3 | DMA1_FLAG_HT3 | DMA1_FLAG_GL3));

    /* Wait for the final bytes of the transfer to complete, including CRC byte(s). */
    while (!(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE))) {
        ;
    }

    /* Wait for the final bytes of the transfer to complete, including CRC byte(s). */
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY)) {
        ;
    }
	
		

   
}
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
