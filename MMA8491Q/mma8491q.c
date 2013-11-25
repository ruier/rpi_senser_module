/***********************************************************************************************\
* Freescale MMA8491Q Driver
*
* Filename: MMA8491Q.c
*
*
* (c) Copyright 2009, Freescale, Inc.  All rights reserved.
*
* No part of this document must be reproduced in any form - including copied,
* transcribed, printed or by any electronic means - without specific written
* permission from Freescale Semiconductor.
*
\***********************************************************************************************/
#include "MMA8491Q_Internal.h"
#include "stdlib.h"
#include "../bcm2835.h"

/***********************************************************************************************\
* Private macros
\***********************************************************************************************/
#ifdef	FALSE
#undef	FALSE
#endif
#define FALSE	(0)

#ifdef	TRUE
#undef	TRUE
#endif
#define	TRUE	(1)

#ifdef	NULL
#undef	NULL
#endif
#define NULL	(0)

#ifdef  ON
#undef  ON
#endif
#define ON      (1)

#ifdef  OFF
#undef  OFF
#endif
#define OFF     (0)
/***********************************************************************************************\
* Private type definitions
\***********************************************************************************************/
#define PIN RPI_GPIO_P1_22
/***********************************************************************************************\
* Private prototypes
\***********************************************************************************************/

/***********************************************************************************************\
* Private memory declarations
\***********************************************************************************************/

/***********************************************************************************************\
* Public memory declarations
\***********************************************************************************************/

/***********************************************************************************************\
* Public functions
\***********************************************************************************************/

uint8_t MMA8491Q_Enable()
{
    if (!bcm2835_init())
        return 1;
    // Set the pin to be an output
    bcm2835_gpio_fsel(PIN, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_write(PIN, HIGH);	
    return 0;
}   
   
uint8_t MMA8491Q_DisEnable()
{
    if (!bcm2835_init())
        return 1;
    // Set the pin to be an output
    bcm2835_gpio_fsel(PIN, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_write(PIN, LOW);	
    return 0;
}

uint8_t MMA8491Q_WRITE_REGISTER(char reg, char value)
{
	/* care the end of string '\n' here */
	char data[2];
	data[0] = reg;
	data[1] = value;
    if (!bcm2835_init())
        return 1;
        
    bcm2835_i2c_begin();
    bcm2835_i2c_setClockDivider(BCM2835_I2C_CLOCK_DIVIDER_2500); // The default	
    bcm2835_i2c_setSlaveAddress(MMA8491Q_IIC_ADDRESS);
    bcm2835_i2c_write(data, 2); 
    
}

uint8_t MMA8491Q_READ_REGISTER(char reg)
{
	char *buf = malloc(sizeof(char));
    if (!bcm2835_init())
        return 1;
        
    bcm2835_i2c_begin();
    bcm2835_i2c_setClockDivider(BCM2835_I2C_CLOCK_DIVIDER_2500); // The default	
    bcm2835_i2c_setSlaveAddress(MMA8491Q_IIC_ADDRESS);
    bcm2835_i2c_read_register_rs(&reg, buf, 1);
    return *buf;	
}

uint8_t MMA8491Q_BULK_READ(char startReg, char length, char *buf)
{
    if (!bcm2835_init())
        return 1;
        
    bcm2835_i2c_begin();
    bcm2835_i2c_setClockDivider(BCM2835_I2C_CLOCK_DIVIDER_2500); // The default	
    bcm2835_i2c_setSlaveAddress(MMA8491Q_IIC_ADDRESS);
    bcm2835_i2c_read_register_rs(&startReg, buf, length);
    return *buf;		
}

/*********************************************************\
* Initialize MMA8491Q
\*********************************************************/
uint8_t MMA8491Q_Init (void)
{

  return TRUE;
}


/*********************************************************\
* Read raw data from MMA8491Q
\*********************************************************/
uint8_t MMA8491_ReadRaw(uint8_t * pdata)
{
    uint8_t Status = 0;

    MMA8491Q_Enable();
    
    MMA8491Q_BULK_READ(MMA8491Q_OUT_X_MSB, 6, pdata);
    MMA8491Q_DisEnable();
  
  return TRUE;
}

/*********************************************************\
* Read X data from MMA8491Q
\*********************************************************/
uint8_t MMA8491_Read(MMA8491Q_DATA *pdata)
{
  uint8_t RawData[8] = {0};
  
  if (MMA8491_ReadRaw(RawData) == TRUE){
    pdata->Xout = RawData[0];
    pdata->Xout <<= 8;
    pdata->Xout += RawData[1];
    pdata->Xout >>= 2;
    
    pdata->Yout = RawData[2];
    pdata->Yout <<= 8;
    pdata->Yout += RawData[3];
    pdata->Yout >>= 2;

    pdata->Zout = RawData[4];
    pdata->Zout <<= 8;
    pdata->Zout += RawData[5];
    pdata->Zout >>= 2;

  }
  return TRUE;
}

