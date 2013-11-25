#include "mag3110.h"
#include "stdint.h"
#include "../bcm2835.h"

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
typedef unsigned char byte;
static uint8_t MAG3110_enabled = FALSE;
int _f3110InitializeChip(void);

uint8_t MAG3110_WRITE_REGISTER(char reg, char value)
{
	/* care the end of string '\n' here */
	char data[2];
	data[0] = reg;
	data[1] = value;

    bcm2835_i2c_setSlaveAddress(MAG3110_I2C_ADDRESS);
    bcm2835_i2c_write(data, 2); 
    
}

uint8_t MAG3110_READ_REGISTER(char reg)
{
	char *buf = malloc(sizeof(char));

    bcm2835_i2c_setSlaveAddress(MAG3110_I2C_ADDRESS);
    bcm2835_i2c_read_register_rs(&reg, buf, 1);
    return *buf;	
}

uint8_t MAG3110_BULK_READ(char startReg, char length, char *buf)
{
    bcm2835_i2c_setSlaveAddress(MAG3110_I2C_ADDRESS);
    bcm2835_i2c_read_register_rs(&startReg, buf, length);
    return *buf;		
}

/**************************************************************************//*!
* @brief    The IIC Initialization for MMA7660
*
* @param    mod - current used module for conection
******************************************************************************/
uint8_t MAG3110_Init(void)
{
  byte who_i_am = 0;

  bcm2835_i2c_begin();
  bcm2835_i2c_setClockDivider(BCM2835_I2C_CLOCK_DIVIDER_2500); // The default	 
  
  MAG3110_WRITE_REGISTER(REG3110_CTRL_REG1, 0x00);

  who_i_am = MAG3110_READ_REGISTER(REG3110_WHO_AM_I);
	
  if(who_i_am == ID_MAG3110) 
  {
    MAG3110_WRITE_REGISTER(REG3110_CTRL_REG2, 0x80);
    MAG3110_enabled = TRUE;
  }
  
  return MAG3110_enabled;
}  

/**************************************************************************//*!
* @brief    Function reads current data from sensor's register 
*
* @param    *mag3110 - pointer to value buffer 
******************************************************************************/
uint8_t MAG3110_ReadRawData(uint8_t * pdata) 
{
  byte ix = 0;
  int16_t z_val = 0;
  
  if(MAG3110_enabled == TRUE)
  {
    MAG3110_WRITE_REGISTER(REG3110_CTRL_REG2, 0x80);
    MAG3110_WRITE_REGISTER(REG3110_CTRL_REG1, 0x02);
    
    do {
        ix = MAG3110_READ_REGISTER(REG3110_DR_STATUS);

    }while((ix & ALL_DATA_RDY) != ALL_DATA_RDY);
    
    MAG3110_BULK_READ(REG3110_OUT_X_MSB, 6, pdata);
  }
  return TRUE;
}

/**************************************************************************//*!
* @brief    Function reads current data from sensor's register 
*
* @param    *mag3110 - pointer to value buffer 
******************************************************************************/
uint8_t MAG3110_ReadAsInt(MAG3110_DATA * pdata) 
{
  //uint8_t RawData[6] = 0;
  
    if(MAG3110_enabled == TRUE)
    {
        pdata->Xout = MAG3110_ReadRawData_x() / 40;
        pdata->Yout = MAG3110_ReadRawData_y() / 40;
        pdata->Zout = MAG3110_ReadRawData_z() / 40;

    //    if (MAG3110_ReadRawData(RawData) == TRUE){
    //      pdata->Xout = RawData[0];
    //      pdata->Xout <<= 8;
    //      pdata->Xout += RawData[1];
    //      pdata->Xout /= 40;
    //      
    //      pdata->Yout = RawData[2];
    //      pdata->Yout <<= 8;
    //      pdata->Yout += RawData[3];
    //      pdata->Yout /= 40;
    //
    //      pdata->Zout = RawData[4];
    //      pdata->Zout <<= 8;
    //      pdata->Zout += RawData[5];
    //      pdata->Zout /= 40;
    //    }
    }
    else
    {
        pdata->Xout = 0;
        pdata->Yout = 0;
        pdata->Zout = 0;
    }
  
  return TRUE;
}

/**************************************************************************//*!
* @brief    Function reads current data from sensor's register 
*
* @param    *mag3110 - pointer to value buffer 
******************************************************************************/
int16_t MAG3110_ReadRawData_x(void) 
{
  byte ix = 0;
  int16_t x_val = 0;
  

  if(MAG3110_enabled == TRUE)
  {
    x_val = MAG3110_READ_REGISTER(REG3110_OUT_X_MSB);
    x_val = MAG3110_READ_REGISTER(REG3110_OUT_X_LSB);

    MAG3110_WRITE_REGISTER(REG3110_CTRL_REG2, 0x80);
    MAG3110_WRITE_REGISTER(REG3110_CTRL_REG1, 0x02);
    
    do {
      ix = MAG3110_READ_REGISTER(REG3110_DR_STATUS);
    }while((ix & X_DATA_RDY) != X_DATA_RDY);

    x_val = MAG3110_READ_REGISTER(REG3110_OUT_X_MSB);
    x_val <<= 8;
    x_val += MAG3110_READ_REGISTER(REG3110_OUT_X_LSB);
  }
  
  return x_val;
}

/**************************************************************************//*!
* @brief    Function reads current data from sensor's register 
*
* @param    *mag3110 - pointer to value buffer 
******************************************************************************/
int16_t MAG3110_ReadRawData_y(void) 
{
  byte ix = 0;
  int16_t y_val = 0;
  
  if(MAG3110_enabled == TRUE)
  {
    y_val = MAG3110_READ_REGISTER(REG3110_OUT_Y_MSB);
    y_val = MAG3110_READ_REGISTER(REG3110_OUT_Y_LSB);

    MAG3110_WRITE_REGISTER(REG3110_CTRL_REG2, 0x80);
    MAG3110_WRITE_REGISTER(REG3110_CTRL_REG1, 0x02);
   
    do {
      ix = MAG3110_READ_REGISTER(REG3110_DR_STATUS);
    }while((ix & Y_DATA_RDY) != Y_DATA_RDY);

    y_val = MAG3110_READ_REGISTER(REG3110_OUT_Y_MSB);
    y_val <<= 8;
    y_val += MAG3110_READ_REGISTER(REG3110_OUT_Y_LSB);
  }
  
  return y_val;
}

/**************************************************************************//*!
* @brief    Function reads current data from sensor's register 
*
* @param    *mag3110 - pointer to value buffer 
******************************************************************************/
int16_t MAG3110_ReadRawData_z(void) 
{
  byte ix = 0;
  int16_t z_val = 0;
  
  if(MAG3110_enabled == TRUE)
  {
    z_val = MAG3110_READ_REGISTER(REG3110_OUT_Z_MSB);
    z_val = MAG3110_READ_REGISTER(REG3110_OUT_Z_LSB);

    MAG3110_WRITE_REGISTER(REG3110_CTRL_REG2, 0x80);
    MAG3110_WRITE_REGISTER(REG3110_CTRL_REG1, 0x02);
    
    do {
      ix = MAG3110_READ_REGISTER(REG3110_DR_STATUS);
    }while((ix & Z_DATA_RDY) != Z_DATA_RDY);

    z_val = MAG3110_READ_REGISTER(REG3110_OUT_Z_MSB);
    z_val <<= 8;
    z_val += MAG3110_READ_REGISTER(REG3110_OUT_Z_LSB);
  }
  
  return z_val;
}

