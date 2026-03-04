/******************************************************************************
 * @file    canL2_link.c
 * @brief   CAN Layer 2 - Link layer implementation.
 ******************************************************************************/
#include "canL2_link.h"
#include "canL4_app.h"      /* For canL4rcvFrmL2() upward call               */

/*******************************************************************************
 * Function : canL2init
 *******************************************************************************/
void canL2init(void)
{
    /* No L2 state to initialise - L1 holds all HW state */
}

/*******************************************************************************
 * Function : canL2decodeId
 * Brief    : Extract DIR, ADDR, PARAM_MAP from 29-bit extended ID.
 *******************************************************************************/
void canL2decodeId(u32 extId, canL2hdrType* hdr)
{
    hdr->dir      = (u8) ((extId & CANL1_ID_DIR_MASK)  >> CANL1_ID_DIR_SHIFT);
    hdr->addr     = (u8) ((extId & CANL1_ID_ADDR_MASK) >> CANL1_ID_ADDR_SHIFT);
    hdr->paramMap = (u16)((extId & CANL1_ID_PMAP_MASK) >> CANL1_ID_PMAP_SHIFT);
}

/*******************************************************************************
 * Function : canL2sendFrmL4
 * Brief    : Pass a frame from L4 down to L1.
 *            BUSY propagates back to L4 - scanner stalls in main loop.
 *******************************************************************************/
canL2TxResultType canL2sendFrmL4(const canFrmType* frm)
{
    canL1TxResultType res = canL1addTxMsg(frm);

    switch (res)
    {
        case CANL1_TX_OK:    return CANL2_TX_OK;
        case CANL1_TX_BUSY:  return CANL2_TX_BUSY;
        default:             return CANL2_TX_ERROR;
    }
}

/*******************************************************************************
 * Function : canL2rcvFrmL1
 * Brief    : Called from L1 RX ISR. Decode the frame header and pass up
 *            to L4. Hardware filter ensures only correctly-directed frames
 *            reach here, but we do a lightweight sanity check on ADDR.
 *******************************************************************************/
void canL2rcvFrmL1(const canFrmType* frm)
{
    canL2hdrType hdr;
    canL2decodeId(frm->id, &hdr);

    /* Sanity: PARAM_MAP must have at least 1 bit set and DLC must be even   */
    if (hdr.paramMap == 0U)        return;
    if ((frm->dlc == 0U) ||
        (frm->dlc % 2U != 0U) ||
        (frm->dlc > 8U))           return;

    /* Route to L4 receive handler                                           */
    canL4rcvFrmL2(frm, &hdr);
}
