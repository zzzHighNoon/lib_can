/******************************************************************************
  * @file    lib_can.h
  * @author  
  * @version V1.0.0
  * @date    
  * @brief    
  *          
*******************************************************************************/    
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LIB_CAN
#define __LIB_CAN

/*******************************************************************************/
/*------------------------------- REG MAP -------------------------------------*/
/*******************************************************************************/
#define CAN_CNTRL_BITS   0
#define CAN_RPM_CMD      1               
#define CAN_D_POSN       2
#define CAN_MOT_CLIM  	 3
#define CAN_RGN_CLIM  	 4
#define CAN_ACCDEC       5
#define CAN_MOPTHRESH    6
#define CAN_MOT_CFG      7
#define CAN_RPM_FB       8
#define CAN_IFB          9
#define CAN_BUSMVDC      10
#define CAN_MOT_TEMP     11
#define CAN_DRV_TEMP     12
#define CAN_STAT_BITS    13

//#define CAN_TOTAL_REGS   14

#define CAN_MAX_NODES    16



/* Includes ------------------------------------------------------------------*/
#include "globals.h"
#include "defines.h"

/* canFrmType is defined in can_map.h which is included by each layer header */
#include "can_map.h"
#include "can_test.h"
#include "canL1_hw.h"
#include "canL2_link.h"
#include "canL3_proto.h"
#include "canL4_app.h"

#endif

