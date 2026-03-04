/******************************************************************************
 * @file    can_map.c
 * @brief   ROM parameter table definitions - shared between hub and drive.
 *          Compiled once, linked into both targets.
 ******************************************************************************/
#include "can_map.h"

/*---------------------------------------------------------------------------*/
/* canMap - application parameter definitions                                 */
/*---------------------------------------------------------------------------*/
const canMapType canMap[CAN_TOTAL_REGS] =
{
  {.type=AO, .min=0,    .max=4000,  .vep=VEP_NONE, .dflt=0     }, /* 0  CNTRL_BITS        */
  {.type=AO, .min=0,    .max=3,     .vep=VEP_NONE, .dflt=0     }, /* 1  RPM_CMD           */
  {.type=AO, .min=3000, .max=30000, .vep=0,        .dflt=27000 }, /* 2  Delta Posn        */
  {.type=AO, .min=3000, .max=10000, .vep=1,        .dflt=5000  }, /* 3  Mot Current Limit */
  {.type=AO, .min=0,    .max=16000, .vep=2,        .dflt=1000  }, /* 4  Rgn Current Limit */
  {.type=AO, .min=0,    .max=4000,  .vep=3,        .dflt=0     }, /* 5  Accel/Decel       */
  {.type=AO, .min=0,    .max=1,     .vep=4,        .dflt=0     }, /* 6  MOP Thresh        */
  {.type=AI, .min=0,    .max=65535, .vep=VEP_NONE, .dflt=0     }, /* 7  Mot Config        */
  {.type=AI, .min=0,    .max=65535, .vep=VEP_NONE, .dflt=0     }, /* 8  RPM FB            */
  {.type=AI, .min=0,    .max=2,     .vep=VEP_NONE, .dflt=0     }, /* 9  IFB               */
  {.type=AI, .min=0,    .max=250,   .vep=VEP_NONE, .dflt=0     }, /* 10 Bus mVDC          */
  {.type=AI, .min=0,    .max=250,   .vep=VEP_NONE, .dflt=0     }, /* 11 Motor Temp C      */
  {.type=AI, .min=0,    .max=65535, .vep=VEP_NONE, .dflt=0     }, /* 12 Drive Temp C      */
  {.type=AI, .min=0,    .max=65535, .vep=VEP_NONE, .dflt=24    }, /* 13 Status Bits       */
};

/*---------------------------------------------------------------------------*/
/* canL2map - link layer scheduling definitions                               */
/*---------------------------------------------------------------------------*/
const canL2mapType canL2map[CAN_TOTAL_REGS] =
{
  {.type=AO, .minTO=5,  .maxTO=10,  .bit=0  }, /* 0  CNTRL_BITS        */
  {.type=AO, .minTO=5,  .maxTO=10,  .bit=1  }, /* 1  RPM_CMD           */
  {.type=AO, .minTO=5,  .maxTO=10,  .bit=2  }, /* 2  Delta Posn        */
  {.type=AO, .minTO=25, .maxTO=100, .bit=3  }, /* 3  Mot Current Limit */
  {.type=AO, .minTO=25, .maxTO=100, .bit=4  }, /* 4  Rgn Current Limit */
  {.type=AO, .minTO=25, .maxTO=100, .bit=5  }, /* 5  Accel/Decel       */
  {.type=AO, .minTO=25, .maxTO=100, .bit=6  }, /* 6  MOP Thresh        */
  {.type=AI, .minTO=25, .maxTO=100, .bit=7  }, /* 7  Mot Config        */
  {.type=AI, .minTO=5,  .maxTO=25,  .bit=8  }, /* 8  RPM FB            */
  {.type=AI, .minTO=5,  .maxTO=25,  .bit=9  }, /* 9  IFB               */
  {.type=AI, .minTO=10, .maxTO=100, .bit=10 }, /* 10 Bus mVDC          */
  {.type=AI, .minTO=25, .maxTO=100, .bit=11 }, /* 11 Motor Temp C      */
  {.type=AI, .minTO=25, .maxTO=100, .bit=12 }, /* 12 Drive Temp C      */
  {.type=AI, .minTO=5,  .maxTO=10,  .bit=13 }, /* 13 Status Bits       */
};
