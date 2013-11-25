/***********************************************************************************************\
* Freescale MPL3115A2 Driver
*
* Filename: mpl3115A2.h
*
*
* (c) Copyright 2009, Freescale, Inc.  All rights reserved.
*
* No part of this document must be reproduced in any form - including copied,
* transcribed, printed or by any electronic means - without specific written
* permission from Freescale Semiconductor.
*
\***********************************************************************************************/
#ifndef _MMA8491Q_INTERNAL_H_
#define _MMA8491Q_INTERNAL_H_

#include "stdint.h"
/****************************************************************************************
* MMA8491Q Sensor Internal Registers
*****************************************************************************************/
enum
{
  MMA8491Q_STATUS = 0,          // 0x00
  MMA8491Q_OUT_X_MSB,           // 0x01
  MMA8491Q_OUT_X_LSB,           // 0x02
  MMA8491Q_OUT_Y_MSB,           // 0x03
  MMA8491Q_OUT_Y_LSB,           // 0x04
  MMA8491Q_OUT_Z_MSB,           // 0x05
  MMA8491Q_OUT_Z_LSB,           // 0x06
};

/*
**  STATUS Registers
*/
#define ZYXDR_BIT               (1<<3)
#define ZDR_BIT                 (1<<2)
#define YDR_BIT                 (1<<1)
#define XDR_BIT                 (1<<0)
/****************************************************************************************
* Redefined GPIO for EN\XOUT\YOUT\ZOUT
*****************************************************************************************/
#define MMA8491Q_EN		(13)
#define MMA8491Q_EN_SHIFT	(1 << MMA8491Q_EN)

#define MMA8491Q_XOUT		(5)
#define MMA8491Q_XOUT_SHIFT	(1 << MMA8491Q_XOUT)

#define MMA8491Q_YOUT		(0)
#define MMA8491Q_YOUT_SHIFT	(1 << MMA8491Q_YOUT)

#define MMA8491Q_ZOUT		(2)
#define MMA8491Q_ZOUT_SHIFT	(1 << MMA8491Q_ZOUT)

uint8_t MMA8491Q_Enable();       
uint8_t MMA8491Q_DisEnable();    

/****************************************************************************************
* Redefined IIC Interface for write and read
*****************************************************************************************/
#define MMA8491Q_IIC_ADDRESS (0x55)
#define I2C_MMA8491Q  I2C1_BASE_PTR
uint8_t MMA8491Q_WRITE_REGISTER(char reg, char value);     
uint8_t MMA8491Q_READ_REGISTER(char reg);        
uint8_t MMA8491Q_BULK_READ(char startReg, char length, char *buf);   

typedef struct
{
  int16_t Xout;
  int16_t Yout;
  int16_t Zout;
} MMA8491Q_DATA;

/***********************************************************************************************\
* Public memory declarations
\***********************************************************************************************/

/***********************************************************************************************\
* Public prototypes
\***********************************************************************************************/
extern uint8_t MMA8491Q_Init (void);
extern uint8_t MMA8491_Read(MMA8491Q_DATA *pdata);

#endif  /* MMA8491Q */