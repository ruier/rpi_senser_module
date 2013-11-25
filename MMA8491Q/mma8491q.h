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
#ifndef _MMA8491Q_H_
#define _MMA8491Q_H_

#include "common.h"


/***********************************************************************************************\
* Public type definitions
\***********************************************************************************************/
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

#endif  /* _MPL3115_H */
