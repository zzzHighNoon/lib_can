/******************************************************************************
 * @file    canL3_proto.c
 * @brief   CAN Layer 3 - Protocol layer implementation.
 ******************************************************************************/
#include "canL3_proto.h"
#include <string.h>

/*******************************************************************************
 * Function : canL3popcount16
 * Brief    : Count set bits in a 16-bit value (no compiler intrinsic needed).
 *******************************************************************************/
u8 canL3popcount16(u16 val)
{
    u8 count = 0U;
    while (val)
    {
        count += (u8)(val & 1U);
        val >>= 1;
    }
    return count;
}

/*******************************************************************************
 * Function : canL3encodeId
 * Brief    : Pack DIR(1) | ADDR(5) | PARAM_MAP(14) into 29-bit ExtId.
 *******************************************************************************/
u32 canL3encodeId(u8 dir, u8 addr, u16 paramMap)
{
    return  ((u32)(dir     & 0x01U)  << CANL1_ID_DIR_SHIFT)  |
            ((u32)(addr    & 0x1FU)  << CANL1_ID_ADDR_SHIFT) |
            ((u32)(paramMap & 0x3FFFU) << CANL1_ID_PMAP_SHIFT);
}

/*******************************************************************************
 * Function : canL3buildFrame
 * Brief    : Build a complete CAN frame from a frame descriptor.
 *
 *            Params are packed MSB-first by bit position in PARAM_MAP:
 *            highest set bit -> bytes [0..1], next -> [2..3], etc.
 *
 *            Byte order within each u16: big-endian (high byte first)
 *            so that the MSB of PARAM_MAP maps to byte 0 of payload.
 *******************************************************************************/
void canL3buildFrame(const canL3frameDescType* desc, canFrmType* frm)
{
    u16  paramMap = 0U;
    u8   payIdx   = 0U;

    memset(frm->data, 0, sizeof(frm->data));

    /* Build PARAM_MAP bitmask and pack payload simultaneously               */
    for (u8 i = 0U; i < desc->count && i < CANL3_MAX_PARAMS_PER_FRAME; i++)
    {
        u8 bit = canL2map[desc->paramIdx[i]].bit;   /* Bit pos from L2 map   */

        paramMap |= (u16)(1U << bit);

        /* Big-endian u16: high byte first                                   */
        frm->data[payIdx++] = (u8)(desc->values[i] >> 8);
        frm->data[payIdx++] = (u8)(desc->values[i] & 0xFFU);
    }

    frm->id  = canL3encodeId(desc->dir, desc->addr, paramMap);
    frm->dlc = (u8)(desc->count * CANL3_BYTES_PER_PARAM);
}

/*******************************************************************************
 * Function : canL3unpackFrame
 * Brief    : Decode a received frame into individual param index + value pairs.
 *
 *            Scans PARAM_MAP bits 13 down to 0. Each set bit corresponds to
 *            the next 2 bytes in the payload (MSB-first order).
 *
 *            Validates: DLC must equal popcount(paramMap) * 2.
 *******************************************************************************/
s8 canL3unpackFrame(const canFrmType* frm, u16 paramMap, canL3unpackType* out)
{
    u8 bitCount = canL3popcount16(paramMap);

    /* Integrity check: DLC must match bit count */
    if (frm->dlc != (u8)(bitCount * CANL3_BYTES_PER_PARAM))
        return -1;

    if (bitCount == 0U || bitCount > CANL3_MAX_PARAMS_PER_FRAME)
        return -1;

    out->count  = 0U;
    u8 payIdx   = 0U;

    /* Scan bits 13 down to 0 - same order as build */
    for (s8 bit = (s8)(CAN_TOTAL_REGS - 1); bit >= 0; bit--)
    {
        if (paramMap & (u16)(1U << bit))
        {
            u8 idx = out->count;

            /* Find which canMap entry owns this bit position                */
            for (u8 p = 0U; p < CAN_TOTAL_REGS; p++)
            {
                if (canL2map[p].bit == (u8)bit)
                {
                    out->paramIdx[idx] = p;
                    break;
                }
            }

            /* Reconstruct u16 big-endian from payload                      */
            out->values[idx] = ((u16)frm->data[payIdx] << 8) |
                                 (u16)frm->data[payIdx + 1U];

            payIdx += CANL3_BYTES_PER_PARAM;
            out->count++;
        }
    }

    return 0;
}
