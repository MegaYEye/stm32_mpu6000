#include "stm32f10x.h"
#ifndef __MPU6000_H
#define __MPU6000_H


#define PIOS_MPU6000_SMPLRT_DIV_REG           0X19
#define PIOS_MPU6000_DLPF_CFG_REG             0X1A
#define PIOS_MPU6000_GYRO_CFG_REG             0X1B
#define PIOS_MPU6000_ACCEL_CFG_REG            0X1C
#define PIOS_MPU6000_FIFO_EN_REG              0x23
#define PIOS_MPU6000_INT_CFG_REG              0x37
#define PIOS_MPU6000_INT_EN_REG               0x38
#define PIOS_MPU6000_INT_STATUS_REG           0x3A
#define PIOS_MPU6000_ACCEL_X_OUT_MSB          0x3B
#define PIOS_MPU6000_ACCEL_X_OUT_LSB          0x3C
#define PIOS_MPU6000_ACCEL_Y_OUT_MSB          0x3D
#define PIOS_MPU6000_ACCEL_Y_OUT_LSB          0x3E
#define PIOS_MPU6000_ACCEL_Z_OUT_MSB          0x3F
#define PIOS_MPU6000_ACCEL_Z_OUT_LSB          0x40
#define PIOS_MPU6000_TEMP_OUT_MSB             0x41
#define PIOS_MPU6000_TEMP_OUT_LSB             0x42
#define PIOS_MPU6000_GYRO_X_OUT_MSB           0x43
#define PIOS_MPU6000_GYRO_X_OUT_LSB           0x44
#define PIOS_MPU6000_GYRO_Y_OUT_MSB           0x45
#define PIOS_MPU6000_GYRO_Y_OUT_LSB           0x46
#define PIOS_MPU6000_GYRO_Z_OUT_MSB           0x47
#define PIOS_MPU6000_GYRO_Z_OUT_LSB           0x48
#define PIOS_MPU6000_USER_CTRL_REG            0x6A
#define PIOS_MPU6000_PWR_MGMT_REG             0x6B
#define PIOS_MPU6000_FIFO_CNT_MSB             0x72
#define PIOS_MPU6000_FIFO_CNT_LSB             0x73
#define PIOS_MPU6000_FIFO_REG                 0x74
#define PIOS_MPU6000_WHOAMI                   0x75

/* FIFO enable for storing different values */
#define PIOS_MPU6000_FIFO_TEMP_OUT            0x80
#define PIOS_MPU6000_FIFO_GYRO_X_OUT          0x40
#define PIOS_MPU6000_FIFO_GYRO_Y_OUT          0x20
#define PIOS_MPU6000_FIFO_GYRO_Z_OUT          0x10
#define PIOS_MPU6000_ACCEL_OUT                0x08

/* Interrupt Configuration */
#define PIOS_MPU6000_INT_ACTL                 0x80
#define PIOS_MPU6000_INT_OPEN                 0x40
#define PIOS_MPU6000_INT_LATCH_EN             0x20
#define PIOS_MPU6000_INT_CLR_ANYRD            0x10

#define PIOS_MPU6000_INTEN_OVERFLOW           0x10
#define PIOS_MPU6000_INTEN_DATA_RDY           0x01

/* Interrupt status */
#define PIOS_MPU6000_INT_STATUS_FIFO_FULL     0x80
#define PIOS_MPU6000_INT_STATUS_FIFO_OVERFLOW 0x10
#define PIOS_MPU6000_INT_STATUS_IMU_RDY       0X04
#define PIOS_MPU6000_INT_STATUS_DATA_RDY      0X01

/* User control functionality */
#define PIOS_MPU6000_USERCTL_FIFO_EN          0X40
#define PIOS_MPU6000_USERCTL_I2C_MST_EN       0x20
#define PIOS_MPU6000_USERCTL_DIS_I2C          0X10
#define PIOS_MPU6000_USERCTL_FIFO_RST         0X04
#define PIOS_MPU6000_USERCTL_SIG_COND         0X02
#define PIOS_MPU6000_USERCTL_GYRO_RST         0X01

/* Power management and clock selection */
#define PIOS_MPU6000_PWRMGMT_IMU_RST          0X80
#define PIOS_MPU6000_PWRMGMT_INTERN_CLK       0X00
#define PIOS_MPU6000_PWRMGMT_PLL_X_CLK        0X01
#define PIOS_MPU6000_PWRMGMT_PLL_Y_CLK        0X02
#define PIOS_MPU6000_PWRMGMT_PLL_Z_CLK        0X03
#define PIOS_MPU6000_PWRMGMT_STOP_CLK         0X07

enum pios_mpu6000_range {
    PIOS_MPU6000_SCALE_250_DEG  = 0x00,
    PIOS_MPU6000_SCALE_500_DEG  = 0x08,
    PIOS_MPU6000_SCALE_1000_DEG = 0x10,
    PIOS_MPU6000_SCALE_2000_DEG = 0x18
};

enum pios_mpu6000_filter {
    PIOS_MPU6000_LOWPASS_256_HZ = 0x00,
    PIOS_MPU6000_LOWPASS_188_HZ = 0x01,
    PIOS_MPU6000_LOWPASS_98_HZ  = 0x02,
    PIOS_MPU6000_LOWPASS_42_HZ  = 0x03,
    PIOS_MPU6000_LOWPASS_20_HZ  = 0x04,
    PIOS_MPU6000_LOWPASS_10_HZ  = 0x05,
    PIOS_MPU6000_LOWPASS_5_HZ   = 0x06
};

enum pios_mpu6000_accel_range {
    PIOS_MPU6000_ACCEL_2G  = 0x00,
    PIOS_MPU6000_ACCEL_4G  = 0x08,
    PIOS_MPU6000_ACCEL_8G  = 0x10,
    PIOS_MPU6000_ACCEL_16G = 0x18
};


#define PIOS_MPU6000_LOCATION_TOP    0x00
#define PIOS_MPU6000_LOCATION_BOTTOM 0x10

#define PIOS_MPU6000_LOCATION_MASK   0xf0

enum pios_mpu6000_orientation { // clockwise rotation from board forward, when looking at sensor itself, which can be also on the bottom side
    PIOS_MPU6000_TOP_0DEG      = 0 | PIOS_MPU6000_LOCATION_TOP,
    PIOS_MPU6000_TOP_90DEG     = 1 | PIOS_MPU6000_LOCATION_TOP,
    PIOS_MPU6000_TOP_180DEG    = 2 | PIOS_MPU6000_LOCATION_TOP,
    PIOS_MPU6000_TOP_270DEG    = 3 | PIOS_MPU6000_LOCATION_TOP,
    PIOS_MPU6000_BOTTOM_0DEG   = 4 | PIOS_MPU6000_LOCATION_BOTTOM,
    PIOS_MPU6000_BOTTOM_90DEG  = 5 | PIOS_MPU6000_LOCATION_BOTTOM,
    PIOS_MPU6000_BOTTOM_180DEG = 6 | PIOS_MPU6000_LOCATION_BOTTOM,
    PIOS_MPU6000_BOTTOM_270DEG = 7 | PIOS_MPU6000_LOCATION_BOTTOM,
};

#define PIOS_MPU6000_SAMPLES_BYTES    14
#define PIOS_MPU6000_SENSOR_FIRST_REG PIOS_MPU6000_ACCEL_X_OUT_MSB

typedef union {
    uint8_t buffer[1 + PIOS_MPU6000_SAMPLES_BYTES];
    struct {
        uint8_t dummy;
        uint8_t Accel_X_h;
        uint8_t Accel_X_l;
        uint8_t Accel_Y_h;
        uint8_t Accel_Y_l;
        uint8_t Accel_Z_h;
        uint8_t Accel_Z_l;//for +-8g configuration, 4096LSB/g
        uint8_t Temperature_h;
        uint8_t Temperature_l;//Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53
        uint8_t Gyro_X_h;
        uint8_t Gyro_X_l;
        uint8_t Gyro_Y_h;
        uint8_t Gyro_Y_l;
        uint8_t Gyro_Z_h;
        uint8_t Gyro_Z_l;//for +-2000g configuration, it is 16.4 LSB/degree per sec
    } data;
} mpu6000_data_t;

#define GET_SENSOR_DATA(mpudataptr, sensor) (mpudataptr.data.sensor##_h << 8 | mpudataptr.data.sensor##_l)


void MPU6000_Config(void);
int MPU6000_SetReg(u8 reg, u8 data);
static int MPU6000_GetReg(u8 reg);
void MPU6000_IRQHandler(void);
int MPU6000_SPI_ReadSensor(void);
void Who_Am_I(void);

#endif
