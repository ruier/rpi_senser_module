#ifndef _MAG3110_H_
#define _MAG3110_H_

#include "stdlib.h"
#include "stdint.h"

/*! Macros for chip id */
#define ID_MAG3110	(0xC4)
//! An enum for MAG3110 registers
/*! Contains offset for hw registers */
enum {
	REG3110_DR_STATUS = 0x00,
	REG3110_OUT_X_MSB ,
	REG3110_OUT_X_LSB ,
	REG3110_OUT_Y_MSB ,
	REG3110_OUT_Y_LSB ,
	REG3110_OUT_Z_MSB ,
	REG3110_OUT_Z_LSB ,
	REG3110_WHO_AM_I ,
	REG3110_SYSMOD ,
	REG3110_OFF_X_MSB ,
	REG3110_OFF_X_LSB ,
	REG3110_OFF_Y_MSB ,
	REG3110_OFF_Y_LSB ,
	REG3110_OFF_Z_MSB ,
	REG3110_OFF_Z_LSB ,
	REG3110_DIE_TEMP ,
	REG3110_CTRL_REG1 ,
	REG3110_CTRL_REG2 ,
};

//! An enum 
/*! device states */
enum 
{
	STANDBY,
	WAKE,
};

//! An enum 
/*! data output modes */
enum 
{
	SCALED,
	RAW,
};

typedef struct {
  int16_t Xout;
  int16_t Yout;
  int16_t Zout;
}MAG3110_DATA;
/*! Bit definations of hardware registers*/
#define XYZ_DATA_OVF	(0x01 << 7)
#define Z_DATA_OVF	(0x01 << 6)
#define Y_DATA_OVF	(0x01 << 5)
#define X_DATA_OVF	(0x01 << 4)
#define XYZ_DATA_RDY	(0x01 << 3)
#define Z_DATA_RDY	(0x01 << 2)
#define Y_DATA_RDY	(0x01 << 1)
#define X_DATA_RDY	(0x01 << 0)
#define ALL_DATA_RDY	(0x07)

#define SYS_MODE_MASK		0x03
#define SYS_MODE_STDBY		0x00
#define SYS_MODE_ACTIVE_RAW	0x01
#define SYS_MODE_ACTIVE_SCALED	0x02

#define DATA_RATE_MASK		0xE0
#define OVERSAMP_MASK		0x18
#define FAST_READ		0x04
#define TRIG_MEASURE		0x02
#define ACTIVE_MODE		0x01
#define STDBY_MODE		0x00

#define RAW_DATA_MASK		(0x01 << 4)
#define MAG_SEN_RESET		(0x01 << 3)
#define Z_SELT_TEST		(0x01 << 2)
#define Y_SELT_TEST		(0x01 << 1)
#define X_SELT_TEST		(0x01 << 0)

#if 0
//  registers address
#define MAG3110_REGS        11
#define MAG3110_DR_STATUS   0x00                                           
#define MAG3110_OUT_X_MSB   0x01                                        
#define MAG3110_DIE_TEMP    0x0F

#define MAG3110_CTRL_REG1   0x10
#define MMA3110_WHOIAM      0x07
#define MMA3110_ID  0xC4
#endif
//  mask
#define BIT_ZYXDR_MASK 0x80
#define RES_14BIT   4
#define RES_12BIT  16
#define RES_10BIT  64

#define XYZ_SIZE  3

// GPIO0IRQ                         UG FIX change MA to MB when needed depending where TWRPI-ADPTSTB is connected
#define MA_GPIO0IRQ
#define MA_GPIO0IRQ_D
#define MA_GPIO0IRQ_INIT
#define MA_GPIO0IRQ_DEINIT

#define MB_GPIO0IRQ
#define MB_GPIO0IRQ_D
#define MB_GPIO0IRQ_INIT
#define MB_GPIO0IRQ_DEINIT

//  typedef

/****************************************************************************************
* Redefined IIC function for write and read
*****************************************************************************************/
/*! Macros for chip I2C address */
#define MAG3110_I2C_ADDRESS (0x0E)
      
uint8_t MAG3110_WRITE_REGISTER(char reg, char value);   
uint8_t MAG3110_READ_REGISTER(char reg);     
uint8_t MAG3110_BULK_READ(char startReg, char length, char *buf);
/****************************************************************************************
* MAG3110 API Functions
*****************************************************************************************/
extern uint8_t MAG3110_Init(void);
extern int16_t MAG3110_ReadRawData_x(void); 
extern int16_t MAG3110_ReadRawData_y(void); 
extern int16_t MAG3110_ReadRawData_z(void); 
extern uint8_t MAG3110_ReadAsInt(MAG3110_DATA * pdata);

#endif