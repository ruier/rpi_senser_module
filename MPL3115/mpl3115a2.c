/***********************************************************************************************\
* Freescale MPL3115A2 Driver
*
* Filename: mma845x.c
*
*
* (c) Copyright 2009, Freescale, Inc.  All rights reserved.
*
* No part of this document must be reproduced in any form - including copied,
* transcribed, printed or by any electronic means - without specific written
* permission from Freescale Semiconductor.
*
\***********************************************************************************************/

#include "mpl3115a2.h"
#include "../bcm2835.h"

/***********************************************************************************************\
* Private macros
\***********************************************************************************************/
/*
 * Misc. Defines
 */
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

/***********************************************************************************************\
* Private prototypes
\***********************************************************************************************/

/***********************************************************************************************\
* Private memory declarations
\***********************************************************************************************/
uint8_t MPL3115A2_enabled = FALSE;
/***********************************************************************************************\
* Public memory declarations
\***********************************************************************************************/

extern byte SlaveAddressIIC;

/***********************************************************************************************\
* Public functions
\***********************************************************************************************/

uint8_t MPL3115A2_WRITE_REGISTER(char reg, char value)
{
	/* care the end of string '\n' here */
	char data[2];
	data[0] = reg;
	data[1] = value;

    bcm2835_i2c_setSlaveAddress(MPL3115A2_IIC_ADDRESS);
    bcm2835_i2c_write(data, 2); 
    
}

uint8_t MPL3115A2_READ_REGISTER(char reg)
{
	char *buf = malloc(sizeof(char));

    bcm2835_i2c_setSlaveAddress(MPL3115A2_IIC_ADDRESS);
    bcm2835_i2c_read_register_rs(&reg, buf, 1);
    return *buf;	
}

/*********************************************************\
* Put MPL3115A2 into Active Mode
\*********************************************************/
void MPL3115A2_Active (void)
{
  MPL3115A2_WRITE_REGISTER( CTRL_REG1, (MPL3115A2_READ_REGISTER( CTRL_REG1) | ACTIVE_MASK));
}                 


/*********************************************************\
* Put MPL3115A2 into Standby Mode
\*********************************************************/
byte MPL3115A2_Standby (void)
{
  byte n;  
  /*
  **  Read current value of System Control 1 Register.
  **  Put sensor into Standby Mode.
  **  Return with previous value of System Control 1 Register.
  */
  n = MPL3115A2_READ_REGISTER( CTRL_REG1);
  MPL3115A2_WRITE_REGISTER( CTRL_REG1, n & STANDBY_SBYB_0);

  return(n);
}          


/*********************************************************\
* Initialize MPL3115A2
\*********************************************************/
uint8_t MPL3115A2_Init_Alt (void)
{
  MPL3115A2_enabled = FALSE;
  
  bcm2835_i2c_begin();
  bcm2835_i2c_setClockDivider(BCM2835_I2C_CLOCK_DIVIDER_2500); // The default	
  	
  if(MPL3115A2_READ_REGISTER(MPL3115A2_WHO_AM_I) == MPL3115A2_ID)
  {
    MPL3115A2_enabled = TRUE;

    /*
    **  Configure sensor for:
    **    - 128 Oversampling
    **    - Altitude Mode
    **    - Set Interrupts to Active Low/Open Drain
    **    - Set Data Event Flags for Pressure/Altitude and Temperature
    **    - Set Generate Data Event Flag
    */
    MPL3115A2_WRITE_REGISTER( CTRL_REG1, (ALT_MASK | OS2_MASK | OS1_MASK | OS0_MASK));  
    MPL3115A2_WRITE_REGISTER( CTRL_REG2, CLEAR_CTRLREG2);    
    MPL3115A2_WRITE_REGISTER( CTRL_REG3, (PP_OD1_MASK | PP_OD2_MASK));  
    MPL3115A2_WRITE_REGISTER( CTRL_REG4, INT_EN_CLEAR);
    MPL3115A2_WRITE_REGISTER( CTRL_REG5, INT_CFG_CLEAR);    
    MPL3115A2_WRITE_REGISTER( PT_DATA_CFG_REG, (DREM_MASK | PDEFE_MASK | TDEFE_MASK));    
  }

  return MPL3115A2_enabled;
}

/*********************************************************\
* Initialize MPL3115A2
\*********************************************************/
uint8_t MPL3115A2_Init_Bar (void)
{
  
  MPL3115A2_enabled = FALSE;

  bcm2835_i2c_begin();
  bcm2835_i2c_setClockDivider(BCM2835_I2C_CLOCK_DIVIDER_2500); // The default	
    
  if(MPL3115A2_READ_REGISTER(MPL3115A2_WHO_AM_I) == MPL3115A2_ID)
  {
    MPL3115A2_enabled = TRUE;
    /*
    **  Configure sensor for:
    **    - 128 Oversampling
    **    - Barometer Mode
    **    - Set Interrupts to Active Low/Open Drain
    **    - Set Data Event Flags for Pressure/Altitude and Temperature
    **    - Set Generate Data Event Flag
    */
    MPL3115A2_WRITE_REGISTER( CTRL_REG1, (OS2_MASK | OS1_MASK | OS0_MASK));
    MPL3115A2_WRITE_REGISTER( CTRL_REG2, CLEAR_CTRLREG2);    
    MPL3115A2_WRITE_REGISTER( CTRL_REG3, (PP_OD1_MASK | PP_OD2_MASK));  
    MPL3115A2_WRITE_REGISTER( CTRL_REG4, INT_EN_CLEAR);
    MPL3115A2_WRITE_REGISTER( CTRL_REG5, INT_CFG_CLEAR);    
    MPL3115A2_WRITE_REGISTER( PT_DATA_CFG_REG, (DREM_MASK | PDEFE_MASK | TDEFE_MASK));    
  }

  return MPL3115A2_enabled;

}

/*********************************************************\
* Read Alt data from MPL3115A2
\*********************************************************/
uint32_t MPL3115A2_Read_Alt (void)
{
  uint32_t val = 0;
  if (MPL3115A2_enabled == TRUE)
  {
    val = MPL3115A2_READ_REGISTER( OUT_P_MSB_REG);
    val <<= 8;
    val += MPL3115A2_READ_REGISTER( OUT_P_CSB_REG);
    val <<= 8;
    val += MPL3115A2_READ_REGISTER( OUT_P_LSB_REG);
  }
  return val;
}

/*********************************************************\
* Read Bar data from MPL3115A2
\*********************************************************/
//void MPL3115A2_Read_Bar (tbar_18* bar_value, tword* dec_value)
//{
//  bar_value->Byte.tbar_msb =   MPL3115A2_WRITE_REGISTER( OUT_P_MSB_REG);
//  bar_value->Byte.tbar_csb = MPL3115A2_WRITE_REGISTER( OUT_P_CSB_REG);
//  bar_value->Byte.tbar_lsb = MPL3115A2_WRITE_REGISTER( OUT_P_LSB_REG);
//
//  dec_value->Byte.hi = MPL3115A2_WRITE_REGISTER( OUT_P_LSB_REG);
//  dec_value->Byte.lo = 0x0;
//}

/*********************************************************\
* Read Temperature  data from MPL3115A2
\*********************************************************/
uint32_t MPL3115A2_Read_Temp (void)
{
  uint32_t val = 0;

  if (MPL3115A2_enabled == TRUE)
  {
    val = MPL3115A2_READ_REGISTER( OUT_T_MSB_REG);
    val <<= 8;
    val += MPL3115A2_READ_REGISTER( OUT_T_LSB_REG); 
  }
  return val;
}

/*********************************************************\
* SetOSR
\*********************************************************/
uint8_t MPL3115A2_SetOSR (uint8_t osr)
{
  if (MPL3115A2_enabled != TRUE) return FALSE;

  if (osr < 8) {
    osr <<= 3;                       

    osr |= MPL3115A2_Standby() & CLEAR_OSR;
    MPL3115A2_WRITE_REGISTER(CTRL_REG1, osr);
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}                 

/*********************************************************\
* Set time of step
\*********************************************************/
uint8_t MPL3115A2_SetStepTime (uint8_t step)
{
  if (MPL3115A2_enabled != TRUE) return FALSE;

  if (step < 0x0f) {

    MPL3115A2_Standby();
    step |= MPL3115A2_READ_REGISTER(CTRL_REG2) & CLEAR_ST_MASK;
    MPL3115A2_WRITE_REGISTER(CTRL_REG2, step);
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}                 

/***********************************************************************************************\
* Private functions
\***********************************************************************************************/
