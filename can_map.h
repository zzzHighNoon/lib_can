/******************************************************************************
 * @file    can_map.h
 * @brief   Shared ROM parameter tables for CAN stack.
 *
 *          Two tables, both const, both compiled into hub and drive:
 *
 *          canMap[]   - application-level definition: min/max/vep/dflt/type
 *                       vep is used hub-side only (EIP parameter index)
 *                       0xFF = not backed by an EIP parameter object
 *
 *          canL2map[] - link-level scheduling definition: minTO/maxTO/bit/type
 *                       minTO = minimum update interval (ms) - rate limiter
 *                       maxTO = maximum update interval (ms) - heartbeat
 *                       bit   = PARAM_MAP bit position in 29-bit CAN ID
 *
 *          Parameter direction (type field):
 *          AO = Analog Output = Hub -> Drive
 *          AI = Analog Input  = Drive -> Hub
 *
 *          CAN_TOTAL_REGS = 14  (indices 0-13)
 ******************************************************************************/
#ifndef __CAN_MAP_H
#define __CAN_MAP_H

#include "globals.h"

/*---------------------------------------------------------------------------*/
/* Core CAN frame type - defined here so all layers see it via can_map.h     */
/* without any layer needing to include lib_can.h (the umbrella header).     */
/*---------------------------------------------------------------------------*/
typedef struct
{
    u32 id;         /* 29-bit extended ID (DIR|ADDR|PARAM_MAP encoded)       */
    u8  dlc;        /* Data length code 0-8                                  */
    u8  data[8];    /* Payload bytes                                         */
} canFrmType;

/*---------------------------------------------------------------------------*/
/* CAN frame routing header - decoded from 29-bit extended ID by L2.        */
/* Defined here so L4 can reference it without including canL2_link.h.      */
/*---------------------------------------------------------------------------*/
typedef struct
{
    u8   dir;       /* 0 = Hub->Drive (AO),  1 = Drive->Hub (AI)            */
    u8   addr;      /* Source/destination node address                       */
    u16  paramMap;  /* 14-bit PARAM_MAP bitmask                              */
} canL2hdrType;

/*---------------------------------------------------------------------------*/
/* Constants                                                                  */
/*---------------------------------------------------------------------------*/
#define CAN_TOTAL_REGS      14U     /* Total number of parameters            */
#define CAN_NUM_DRIVES      16U     /* Maximum drive nodes on network        */

#define CAN_HUB_ADDR        16U     /* Hub fixed node address                */
#define CAN_BCAST_ADDR      31U     /* Broadcast: hub -> all drives          */
#define CAN_DRIVE_ADDR_MAX  15U     /* Highest valid drive address           */

#define AO                  0U      /* Hub -> Drive                          */
#define AI                  1U      /* Drive -> Hub                          */

#define VEP_NONE            0xFFU   /* Not backed by an EIP parameter        */

/*---------------------------------------------------------------------------*/
/* canMap - application parameter definitions                                 */
/* vep: hub-side EIP parameter index (VEP_NONE = not EIP-sourced)            */
/*---------------------------------------------------------------------------*/
typedef struct
{
    u8   type;      /* AO or AI                                              */
    u16  min;       /* Minimum valid value                                   */
    u16  max;       /* Maximum valid value                                   */
    u8   vep;       /* EIP param index (hub only) - VEP_NONE if not mapped   */
    u16  dflt;      /* Default value on init                                 */
} canMapType;

extern const canMapType canMap[CAN_TOTAL_REGS];

/*---------------------------------------------------------------------------*/
/* canL2map - link layer scheduling definitions                               */
/*---------------------------------------------------------------------------*/
typedef struct
{
    u8   type;      /* AO or AI - must match canMap entry                    */
    u16  minTO;     /* Min update interval ms - rate limiter                 */
    u16  maxTO;     /* Max update interval ms - heartbeat                    */
    u8   bit;       /* PARAM_MAP bit position in CAN extended ID (0-13)      */
} canL2mapType;

extern const canL2mapType canL2map[CAN_TOTAL_REGS];

#endif /* __CAN_MAP_H */
