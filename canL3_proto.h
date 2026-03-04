/******************************************************************************
 * @file    canL3_proto.h
 * @brief   CAN Layer 3 - Protocol layer.
 *
 *          Responsibilities:
 *            - Encode a DIR/ADDR/PARAM_MAP combination into a 29-bit ExtId
 *            - Build a complete canFrmType from a parameter set:
 *                * Scan PARAM_MAP bits 13->0
 *                * Pack up to 4 values MSB-first into payload
 *                * Set DLC = paramCount * 2
 *            - Unpack a received frame back into individual param values
 *              indexed by their PARAM_MAP bit position
 *
 *          Frame fill rule (enforced here):
 *            ready >= 1: if only 1 param ready, caller supplies 1 fill param
 *            ready 2-4:  send exactly what is ready, no padding
 *            ready > 4:  caller selects top 4 before calling build
 *
 *          Payload byte order:
 *            Highest set bit in PARAM_MAP -> bytes [0..1]  (big-endian u16)
 *            Next set bit                 -> bytes [2..3]
 *            ...
 *            Lowest set bit               -> bytes [n-2..n-1]
 ******************************************************************************/
#ifndef __CANL3_PROTO_H
#define __CANL3_PROTO_H

#include "globals.h"
#include "can_map.h"
#include "canL1_hw.h"

/*---------------------------------------------------------------------------*/
/* Constants                                                                  */
/*---------------------------------------------------------------------------*/
#define CANL3_MAX_PARAMS_PER_FRAME  4U      /* Max params packed in one frame */
#define CANL3_MIN_PARAMS_PER_FRAME  2U      /* Min params - fill rule applies */
#define CANL3_BYTES_PER_PARAM       2U      /* Each param is one u16          */

/*---------------------------------------------------------------------------*/
/* Types                                                                      */
/*---------------------------------------------------------------------------*/

/**
 * @brief  Input descriptor for building one CAN frame.
 *         Caller populates this, passes to canL3buildFrame().
 */
typedef struct
{
    u8   dir;                                   /* AO=0 or AI=1              */
    u8   addr;                                  /* Destination/source node   */
    u8   count;                                 /* Number of params (1-4)    */
    u8   paramIdx[CANL3_MAX_PARAMS_PER_FRAME];  /* canMap index for each     */
    u16  values[CANL3_MAX_PARAMS_PER_FRAME];    /* Corresponding u16 values  */
} canL3frameDescType;

/**
 * @brief  Output of canL3unpackFrame() - one entry per param found.
 */
typedef struct
{
    u8   count;                                 /* Number of params decoded  */
    u8   paramIdx[CANL3_MAX_PARAMS_PER_FRAME];  /* canMap index of each      */
    u16  values[CANL3_MAX_PARAMS_PER_FRAME];    /* Decoded u16 values        */
} canL3unpackType;

/*---------------------------------------------------------------------------*/
/* Function prototypes                                                        */
/*---------------------------------------------------------------------------*/

/**
 * @brief  Encode DIR, ADDR, PARAM_MAP into a 29-bit extended ID.
 * @param  dir       0=Hub->Drive  1=Drive->Hub
 * @param  addr      Node address (0-31)
 * @param  paramMap  14-bit bitmask of params present in frame
 * @return 29-bit extended ID value for canFrmType.id
 */
u32 canL3encodeId(u8 dir, u8 addr, u16 paramMap);

/**
 * @brief  Build a complete CAN frame ready for L2/L1 transmission.
 *         Sets ExtId, packs payload MSB-first by PARAM_MAP bit order,
 *         sets DLC = desc->count * 2.
 * @param  desc  Populated frame descriptor (1-4 params).
 * @param  frm   Output: complete frame ready to send.
 */
void canL3buildFrame(const canL3frameDescType* desc, canFrmType* frm);

/**
 * @brief  Unpack a received CAN frame into individual param values.
 *         Scans PARAM_MAP bits 13->0, reads 2 bytes per set bit.
 *         Validates DLC against bit count.
 * @param  frm      Received frame from L2.
 * @param  paramMap PARAM_MAP extracted by L2 decode.
 * @param  out      Output: decoded params and values.
 * @return 0 on success, -1 if DLC/bitcount mismatch.
 */
s8 canL3unpackFrame(const canFrmType* frm, u16 paramMap, canL3unpackType* out);

/**
 * @brief  Count the number of set bits in a 16-bit value.
 *         Used to validate DLC against PARAM_MAP popcount.
 */
u8 canL3popcount16(u16 val);

#endif /* __CANL3_PROTO_H */
