/******************************************************************************
 * @file    canL2_link.h
 * @brief   CAN Layer 2 - Link layer.
 *
 *          Responsibilities:
 *            TX: Receive frame from L4, pass to L1. Return BUSY upstream
 *                so L4 scanner stalls naturally in the main loop.
 *            RX: Entry point from L1 ISR. Decode DIR and ADDR from the
 *                29-bit extended ID, route to the correct L4 handler.
 *
 *          canL2map[] lives here - scheduling constants (minTO/maxTO/bit)
 *          referenced by both L2 routing and L4 scheduler.
 ******************************************************************************/
#ifndef __CANL2_LINK_H
#define __CANL2_LINK_H

#include "globals.h"
#include "can_map.h"
#include "canL1_hw.h"

/*---------------------------------------------------------------------------*/
/* Types                                                                      */
/*---------------------------------------------------------------------------*/

/* L2 TX result mirrors L1 result - BUSY propagates up to L4 scanner        */
typedef enum
{
    CANL2_TX_OK = 0,
    CANL2_TX_BUSY,
    CANL2_TX_ERROR
} canL2TxResultType;

/* canL2hdrType is defined in can_map.h (included above)                    */

/*---------------------------------------------------------------------------*/
/* Function prototypes                                                        */
/*---------------------------------------------------------------------------*/

/**
 * @brief  Initialise L2. No state beyond what L1 holds.
 */
void canL2init(void);

/**
 * @brief  TX entry point called by L4 scheduler.
 *         Passes frame directly to L1. Returns BUSY if mailboxes full.
 *         L4 caller stalls in main loop until OK.
 * @param  frm  Frame with fully encoded 29-bit ExtId and packed payload.
 * @return CANL2_TX_OK / CANL2_TX_BUSY / CANL2_TX_ERROR
 */
canL2TxResultType canL2sendFrmL4(const canFrmType* frm);

/**
 * @brief  RX entry point called from L1 ISR (canL1rcvRxMsg).
 *         Decodes DIR and ADDR from ExtId, routes to L4 receive handler.
 * @param  frm  Raw frame from L1 with 29-bit id in frm->id.
 */
void canL2rcvFrmL1(const canFrmType* frm);

/**
 * @brief  Decode a 29-bit extended ID into its DIR/ADDR/PARAM_MAP fields.
 * @param  extId  Raw 29-bit value from canFrmType.id
 * @param  hdr    Output: populated routing header
 */
void canL2decodeId(u32 extId, canL2hdrType* hdr);

#endif /* __CANL2_LINK_H */
