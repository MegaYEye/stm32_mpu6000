
#include "stm32f10x.h"
#include "mpu6000.h"
#include "delay.h"



u8 tx_dummy_byte;
u8 rx_dummy_byte;


void SPI_Writebyte(u8 data)
{
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI1, data); 
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET); 
	SPI_I2S_ReceiveData(SPI1); 
}

u8 SPI_Readbyte(u8 data)
{ 
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI1, data); 
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET); 
	return SPI_I2S_ReceiveData(SPI1); 
}

int MPU6000_SetReg(u8 reg, u8 data)  {
			

GPIO_ResetBits(GPIOA,GPIO_Pin_4); 
		SPI_Writebyte(0x7f & reg);
		SPI_Writebyte(data);
	
GPIO_SetBits(GPIOA,GPIO_Pin_4); 
		

    return 0;
}
static int MPU6000_GetReg(u8 reg)
{
	
		u8 byte;
			GPIO_ResetBits(GPIOA,GPIO_Pin_4); 
		


		SPI_Writebyte((0x80 | reg));
		byte=SPI_Readbyte(0);
	

GPIO_SetBits(GPIOA,GPIO_Pin_4); 

    return byte;
   
}
int32_t PIOS_MPU6000_ConfigureRanges(
    enum pios_mpu6000_range gyroRange,
    enum pios_mpu6000_accel_range accelRange,
    enum pios_mpu6000_filter filterSetting)
{

    // update filter settings
    MPU6000_SetReg(PIOS_MPU6000_DLPF_CFG_REG, filterSetting);

    // Sample rate divider, chosen upon digital filtering settings
   // MPU6000_SetReg(PIOS_MPU6000_SMPLRT_DIV_REG,
    //                           filterSetting == PIOS_MPU6000_LOWPASS_256_HZ ?
    //                           7 : 0);
		MPU6000_SetReg(PIOS_MPU6000_SMPLRT_DIV_REG,1);
 

    // Gyro range
    MPU6000_SetReg(PIOS_MPU6000_GYRO_CFG_REG, gyroRange);
 
    MPU6000_SetReg(PIOS_MPU6000_ACCEL_CFG_REG, accelRange);

    return 0;
}
int MPU6000_TransferBlock(uint32_t spi_id, const uint8_t *send_buffer, uint8_t *receive_buffer, uint16_t len, void *callback)
{
//    struct pios_spi_dev *spi_dev = (struct pios_spi_dev *)spi_id;

 

    DMA_InitTypeDef dma_init;

    /* Exit if ongoing transfer */
    if (DMA_GetCurrDataCounter(DMA1_Channel2)) {
        return -3;
    }

    /* Disable the SPI peripheral */
    SPI_Cmd(SPI1, DISABLE);

    /* Disable the DMA channels */
    DMA_Cmd(DMA1_Channel2, DISABLE);
    DMA_Cmd(DMA1_Channel3, DISABLE);

    /* Set callback function */
    //spi_dev->callback = callback;

    /*
     * Configure Rx channel
     */

    /* Start with the default configuration for this peripheral */
    //dma_init = spi_dev->cfg->dma.rx.init;
			dma_init.DMA_PeripheralBaseAddr=(uint32_t)&(SPI1->DR);
		dma_init.DMA_DIR                           = DMA_DIR_PeripheralSRC,
		//dma_init.DMA_MemoryBaseAddr=(uint32_t)rxbuf;
		//dma_init.DMA_BufferSize=rxlen;
    dma_init.DMA_PeripheralInc      = DMA_PeripheralInc_Disable,
    dma_init.DMA_MemoryInc          = DMA_MemoryInc_Enable,
    dma_init .DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte,
    dma_init .DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte,
    dma_init .DMA_Mode     = DMA_Mode_Normal,
    dma_init .DMA_Priority = DMA_Priority_Medium,
    dma_init .DMA_M2M                           = DMA_M2M_Disable;

    if (receive_buffer != 0) {
        /* Enable memory addr. increment - bytes written into receive buffer */
        dma_init.DMA_MemoryBaseAddr = (uint32_t)receive_buffer;
        dma_init.DMA_MemoryInc = DMA_MemoryInc_Enable;
    } else {
        /* Disable memory addr. increment - bytes written into dummy buffer */
        rx_dummy_byte = 0xFF;
        dma_init.DMA_MemoryBaseAddr = (uint32_t)&rx_dummy_byte;
        dma_init.DMA_MemoryInc = DMA_MemoryInc_Disable;
    }
 

    dma_init.DMA_BufferSize = len;
    DMA_Init(DMA1_Channel2, &(dma_init));

    /*
     * Configure Tx channel
     */

    /* Start with the default configuration for this peripheral */
    //dma_init = spi_dev->cfg->dma.tx.init;
				dma_init.DMA_PeripheralBaseAddr=(uint32_t)&(SPI1->DR);
		//dma_init.DMA_MemoryBaseAddr=(uint32_t)txbuf;
		//dma_init.DMA_BufferSize=txlen;
		dma_init.DMA_DIR                           = DMA_DIR_PeripheralDST;
    dma_init.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    dma_init.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    dma_init .DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma_init .DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    dma_init .DMA_Mode     = DMA_Mode_Normal;
    dma_init .DMA_Priority = DMA_Priority_Medium;
    dma_init .DMA_M2M                           = DMA_M2M_Disable;

    if (send_buffer != 0) {
        /* Enable memory addr. increment - bytes written into receive buffer */
        dma_init.DMA_MemoryBaseAddr = (uint32_t)send_buffer;
        dma_init.DMA_MemoryInc = DMA_MemoryInc_Enable;
    } else {
        /* Disable memory addr. increment - bytes written into dummy buffer */
        tx_dummy_byte = 0xFF;
        dma_init.DMA_MemoryBaseAddr = (uint32_t)&tx_dummy_byte;
        dma_init.DMA_MemoryInc = DMA_MemoryInc_Disable;
    }


        dma_init.DMA_BufferSize = len;
    

    DMA_Init(DMA1_Channel3, &(dma_init));

    /* Enable DMA interrupt if callback function active */
    DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, (callback != 0) ? ENABLE : DISABLE);

    /* Flush out the CRC registers */
    SPI_CalculateCRC(SPI1, DISABLE);
    (void)SPI_GetCRC(SPI1, SPI_CRC_Rx);
    SPI_I2S_ClearFlag(SPI1, SPI_FLAG_CRCERR);

    /* Make sure to flush out the receive buffer */
    (void)SPI_I2S_ReceiveData(SPI1);

 

    /* Start DMA transfers */
    DMA_Cmd(DMA1_Channel2, ENABLE);
    DMA_Cmd(DMA1_Channel3, ENABLE);

    /* Reenable the SPI device */
    SPI_Cmd(SPI1, ENABLE);

    if (callback) {
        /* User has requested a callback, don't wait for the transfer to complete. */
        return 0;
    }

    /* Wait until all bytes have been transmitted/received */
    while (DMA_GetCurrDataCounter(DMA1_Channel2)) {
        ;
    }

    /* Wait for the final bytes of the transfer to complete, including CRC byte(s). */
    while (!(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE))) {
        ;
    }

    /* Wait for the final bytes of the transfer to complete, including CRC byte(s). */
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY)) {
        ;
    }

    /* Check the CRC on the transfer if enabled. */


    /* No error */
    return 0;
}
u8 tmp;
void Who_Am_I() {
tmp=MPU6000_GetReg(PIOS_MPU6000_WHOAMI);
}
void MPU6000_Config() 
{
	int i;
	MPU6000_SetReg(PIOS_MPU6000_PWR_MGMT_REG, PIOS_MPU6000_PWRMGMT_IMU_RST);
	delay_ms(100);
	// Reset chip and fifo
	MPU6000_SetReg(PIOS_MPU6000_USER_CTRL_REG, PIOS_MPU6000_USERCTL_GYRO_RST |
                               PIOS_MPU6000_USERCTL_SIG_COND |
                               PIOS_MPU6000_USERCTL_FIFO_RST);
	
	
	
	 

    // Wait for reset to finish
    while (MPU6000_GetReg(PIOS_MPU6000_USER_CTRL_REG) &
           (PIOS_MPU6000_USERCTL_GYRO_RST |
            PIOS_MPU6000_USERCTL_SIG_COND |
            PIOS_MPU6000_USERCTL_FIFO_RST)) {
        ;
    }
    delay_ms(100);
    // Power management configuration
   MPU6000_SetReg(PIOS_MPU6000_PWR_MGMT_REG, PIOS_MPU6000_PWRMGMT_PLL_X_CLK);

    // Interrupt configuration
    MPU6000_SetReg(PIOS_MPU6000_INT_CFG_REG, PIOS_MPU6000_INT_CLR_ANYRD);


    // Interrupt configuration
    MPU6000_SetReg(PIOS_MPU6000_INT_EN_REG, PIOS_MPU6000_INTEN_DATA_RDY);

    // FIFO storage
    MPU6000_SetReg(PIOS_MPU6000_FIFO_EN_REG, PIOS_MPU6000_FIFO_TEMP_OUT | PIOS_MPU6000_FIFO_GYRO_X_OUT | PIOS_MPU6000_FIFO_GYRO_Y_OUT | PIOS_MPU6000_FIFO_GYRO_Z_OUT);
		
    PIOS_MPU6000_ConfigureRanges(PIOS_MPU6000_SCALE_2000_DEG, PIOS_MPU6000_ACCEL_8G, PIOS_MPU6000_LOWPASS_188_HZ);
    // Interrupt configuration
    MPU6000_SetReg(PIOS_MPU6000_USER_CTRL_REG,PIOS_MPU6000_USERCTL_DIS_I2C);

    // Interrupt configuration
    MPU6000_SetReg(PIOS_MPU6000_PWR_MGMT_REG, PIOS_MPU6000_PWRMGMT_PLL_X_CLK);

    // Interrupt configuration
    //MPU6000_SetReg(PIOS_MPU6000_INT_CFG_REG, PIOS_MPU6000_INT_CLR_ANYRD) ;
		MPU6000_SetReg(PIOS_MPU6000_INT_CFG_REG, 0) ;

    // Interrupt configuration
    MPU6000_SetReg(PIOS_MPU6000_INT_EN_REG, PIOS_MPU6000_INTEN_DATA_RDY);
		
	
			
    if ((MPU6000_GetReg(PIOS_MPU6000_INT_EN_REG)) != PIOS_MPU6000_INTEN_DATA_RDY) {
        return;
    }
	
}
mpu6000_data_t mpu6000_data;
int MPU6000_SPI_ReadSensor(){
 const uint8_t mpu6000_send_buf[1 + PIOS_MPU6000_SAMPLES_BYTES] = { PIOS_MPU6000_SENSOR_FIRST_REG | 0x80 };

    GPIO_ResetBits(GPIOA,GPIO_Pin_4); 
    if (MPU6000_TransferBlock(0, &mpu6000_send_buf[0], &mpu6000_data.buffer[0], sizeof(mpu6000_data_t), 0) < 0) {
        GPIO_SetBits(GPIOA,GPIO_Pin_4); 
        return 0;
    }
    GPIO_SetBits(GPIOA,GPIO_Pin_4); 
    return 1;
}
void MPU6000_IRQHandler() {
 
	
	
 /* gyro_read_timestamp = PIOS_DELAY_GetRaw();
    bool woken = false;

    if (!mpu6000_configured) {
        return;
    }

    if (dev->driver->ReadSensor(&woken)) {
        woken |= PIOS_MPU6000_HandleData(gyro_read_timestamp);
    }
	

    return woken;*/
	 
}
