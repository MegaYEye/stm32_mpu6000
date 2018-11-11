/**
  ******************************************************************************
  * @file    GPIO/IOToggle/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body.
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
#include "stm32f10x.h"
#include "mpu6000.h"
#include "delay.h"
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
GPIO_InitTypeDef GPIO_InitStructure;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int txbuf[100];int txlen=10;
int rxbuf[100];int rxlen=10;

void init() {
	int i;
		SPI_InitTypeDef spi;
		NVIC_InitTypeDef nvic;
		EXTI_InitTypeDef exti;
		DMA_InitTypeDef dma;
	
		SystemInit();
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    /* Enable GPIOA, GPIOB, GPIOC, GPIOD, GPIOE and AFIO clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE |
                           RCC_APB2Periph_AFIO, ENABLE);
		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 , ENABLE); 	

		
	
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		
		

		dma.DMA_PeripheralBaseAddr=(uint32_t)&(SPI1->DR);
		dma.DMA_DIR                           = DMA_DIR_PeripheralSRC,
		dma.DMA_MemoryBaseAddr=(uint32_t)rxbuf;
		dma.DMA_BufferSize=rxlen;
    dma.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    dma .DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma .DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    dma .DMA_Mode     = DMA_Mode_Normal;
    dma .DMA_Priority = DMA_Priority_Medium;
    dma .DMA_M2M                           = DMA_M2M_Disable;
		
		DMA_DeInit(DMA1_Channel2);
		DMA_Init(DMA1_Channel2,&dma);
		
		DMA_ITConfig(DMA1_Channel2,DMA_IT_TC, ENABLE);
		
		dma.DMA_PeripheralBaseAddr=(uint32_t)&(SPI1->DR);
		dma.DMA_MemoryBaseAddr=(uint32_t)txbuf;
		dma.DMA_BufferSize=txlen;
		dma.DMA_DIR                           = DMA_DIR_PeripheralDST;
    dma.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    dma .DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma .DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    dma .DMA_Mode     = DMA_Mode_Normal;
    dma .DMA_Priority = DMA_Priority_Medium;
    dma .DMA_M2M                           = DMA_M2M_Disable;
		
		DMA_DeInit(DMA1_Channel3);
		DMA_Init(DMA1_Channel3,&dma);
		
		DMA_ITConfig(DMA1_Channel3,DMA_IT_TC, DISABLE);
		
		spi.SPI_Mode=SPI_Mode_Master;
		spi.SPI_Direction=SPI_Direction_2Lines_FullDuplex;
		spi.SPI_DataSize=SPI_DataSize_8b;
		spi.SPI_NSS=SPI_NSS_Soft;
		spi.SPI_FirstBit=SPI_FirstBit_MSB;
		spi.SPI_CRCPolynomial=7;
		spi.SPI_CPOL=SPI_CPOL_High;
		spi.SPI_CPHA=SPI_CPHA_2Edge;
		spi.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_64;
		SPI_Init(SPI1,&spi);
		
		

		
		
					DMA_Cmd(DMA1_Channel2, ENABLE);
    DMA_Cmd(DMA1_Channel3, ENABLE);
				
		
		SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, ENABLE);
		SPI_Cmd(SPI1, ENABLE); 
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		
		
		nvic.NVIC_IRQChannel=EXTI3_IRQn;
		nvic.NVIC_IRQChannelPreemptionPriority=0;
		nvic.NVIC_IRQChannelSubPriority=0;
		nvic.NVIC_IRQChannelCmd=ENABLE;
		
		NVIC_Init(&nvic);
		
		nvic.NVIC_IRQChannel=DMA1_Channel2_IRQn;
		nvic.NVIC_IRQChannelPreemptionPriority=1;
		nvic.NVIC_IRQChannelSubPriority=0;
		nvic.NVIC_IRQChannelCmd=ENABLE;
		
		NVIC_Init(&nvic);
		
		
		exti.EXTI_Line=EXTI_Line3;
		exti.EXTI_Mode=EXTI_Mode_Interrupt;
		exti.EXTI_Trigger=EXTI_Trigger_Rising;
		exti.EXTI_LineCmd=ENABLE;
		
		/*while(1) {
			Who_Am_I();
		}*/

			delay_init();
	delay_ms(1);
	MPU6000_Config();
	MPU6000_Config();
			for(i=0;i<5;i++) {
			MPU6000_SetReg(PIOS_MPU6000_ACCEL_CFG_REG, PIOS_MPU6000_ACCEL_8G);
				MPU6000_SetReg(PIOS_MPU6000_ACCEL_CFG_REG,PIOS_MPU6000_SCALE_2000_DEG);
		}
	
	
		EXTI_Init(&exti);

}


 

 

int d=0;
int c=0;
extern 	int m1;
extern 	int m2;
extern mpu6000_data_t mpu6000_data;

uint8_t data[2]={'a','b'};
int i=0;
int main(void)
{
  init();
	//MPU6000_SPI_ReadSensor();
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4);

	USB_Config();
	
	
	while(1) {
		//Who_Am_I();
		//MPU6000_SPI_ReadSensor();
		if(m2<m1) {
			

			
		//USB_TxWrite((uint8_t *)&mpu6000_data_tmp,sizeof(mpu6000_data));
			m2++;

		}
	}
	
}


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
