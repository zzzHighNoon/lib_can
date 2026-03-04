/******************************************************************************
 * @file    canL1_hw.c
 * @brief   CAN Layer 1 - Hardware/HAL layer for STM32F103
 *          Extended 29-bit ID rewrite.
 *
 *          RX path:  FIFO0 pending ISR -> canL1rcvRxMsg() -> canL2rcvFrmL1()
 *          TX path:  canL1addTxMsg() called from L4 scanner (main loop)
 *                    Returns BUSY when all 3 mailboxes full - caller stalls
 *          Errors:   HAL_CAN_ErrorCallback() categorises into canL1ErrType
 *          Watchdog: tx.clk reset to 0 in every TX complete callback
 ******************************************************************************/
#include "globals.h"
#include "lib_can.h"
#include "canL1_hw.h"
#include "canL2_link.h"     /* For canL2rcvFrmL1() upward call               */

/*---------------------------------------------------------------------------*/
/* Module instance and filter bank counter                                    */
/*---------------------------------------------------------------------------*/
canL1type canL1;
static u32 s_filtBank = 0;  /* Incremented each time a filter is configured  */

/*******************************************************************************
 * Function : canL1init
 * Brief    : Store HAL handle, start peripheral, activate interrupts.
 *            Timing (1 Mbps) is set in CubeMX - not repeated here.
 *******************************************************************************/
canL1type* canL1init(CAN_HandleTypeDef* hcan)
{
    canL1.hcan = hcan;

    HAL_CAN_Start(canL1.hcan);

    HAL_CAN_ActivateNotification(canL1.hcan,
        CAN_IT_RX_FIFO0_MSG_PENDING |   /* Consolidated to FIFO0             */
        CAN_IT_TX_MAILBOX_EMPTY     |
        CAN_IT_ERROR                |
        CAN_IT_BUSOFF               |
        CAN_IT_LAST_ERROR_CODE);

    return &canL1;
}

/*******************************************************************************
 * Function : canL1addTxMsg
 * Brief    : Non-blocking TX. Places one frame into a free HW mailbox.
 *            Called from L4 scanner in main loop. Returns BUSY if all
 *            three mailboxes are occupied - L4 stalls and retries next pass.
 *******************************************************************************/
canL1TxResultType canL1addTxMsg(const canFrmType* frm)
{
    if (HAL_CAN_GetTxMailboxesFreeLevel(canL1.hcan) == 0)
        return CANL1_TX_BUSY;

    /* Extended 29-bit ID - ExtId carries the full DIR|ADDR|PARAM_MAP word   */
    canL1.tx.hdr.ExtId               = frm->id & 0x1FFFFFFFUL;
    canL1.tx.hdr.StdId               = 0;
    canL1.tx.hdr.IDE                 = CAN_ID_EXT;
    canL1.tx.hdr.RTR                 = CAN_RTR_DATA;
    canL1.tx.hdr.DLC                 = (frm->dlc <= 8U) ? frm->dlc : 8U;
    canL1.tx.hdr.TransmitGlobalTime  = DISABLE;

    HAL_StatusTypeDef st =
        HAL_CAN_AddTxMessage(canL1.hcan,
                              &canL1.tx.hdr,
                              (u8*)frm->data,
                              &canL1.tx.mb);

    if (st == HAL_OK)   return CANL1_TX_OK;
    if (st == HAL_BUSY) return CANL1_TX_BUSY;

    return CANL1_TX_ERROR;
}

/*******************************************************************************
 * Function : canL1rcvRxMsg  (called from FIFO0 pending ISR)
 * Brief    : Read one frame from FIFO0, decode ExtId, pass up to L2.
 *******************************************************************************/
static void canL1rcvRxMsg(void)
{
    if (HAL_CAN_GetRxMessage(canL1.hcan,
                              CAN_RX_FIFO0,
                              &canL1.rx.hdr,
                              canL1.rx.frm.data) != HAL_OK)
    {
        canL1.rx.err++;
        return;
    }

    /* Extended ID - full 29-bit word passed up as-is for L3 to decode      */
    canL1.rx.frm.id  = canL1.rx.hdr.ExtId & 0x1FFFFFFFUL;
    canL1.rx.frm.dlc = (canL1.rx.hdr.DLC <= 8U)
                       ? (u8)canL1.rx.hdr.DLC : 8U;

    canL2rcvFrmL1(&canL1.rx.frm);  /* Pass frame up to L2                   */
    canL1.rx.cnt++;
}

/*******************************************************************************
 * Function : canL1filtCfgDrive
 * Brief    : Hardware filter for a DRIVE node.
 *            Accept only: DIR=0 (bit 28) AND ADDR=nodeAddr (bits 27..23).
 *            All other frames rejected in hardware before reaching FIFO0.
 *
 *            bxCAN 32-bit mask mode for extended ID:
 *            FilterIdHigh/Low   = acceptance code, bits [31..16] / [15..0]
 *            FilterMaskIdHigh/Low = mask, bit=1 means that bit must match
 *
 *            The HAL shifts the 29-bit ExtId into bits [31..3] of the
 *            32-bit filter register (bit 2 = IDE flag must also be set).
 *******************************************************************************/
void canL1filtCfgDrive(u8 nodeAddr)
{
    /* Build the 29-bit acceptance ID: DIR=0, ADDR=nodeAddr, PMAP=don't care */
    u32 filtId   = ((u32)(nodeAddr & 0x1FU) << CANL1_ID_ADDR_SHIFT);

    /* Mask: DIR bit and all 5 ADDR bits must match exactly                  */
    u32 filtMask = CANL1_ID_DIR_MASK | CANL1_ID_ADDR_MASK;

    /* HAL format: shift left 3, set IDE bit (bit 2) for extended frame      */
    canL1.filt.FilterIdHigh       = (u16)((filtId   << 3) >> 16);
    canL1.filt.FilterIdLow        = (u16)((filtId   << 3) & 0xFFFFU) | 0x0004U;
    canL1.filt.FilterMaskIdHigh   = (u16)((filtMask << 3) >> 16);
    canL1.filt.FilterMaskIdLow    = (u16)((filtMask << 3) & 0xFFFFU) | 0x0004U;

    canL1.filt.FilterBank         = s_filtBank++;
    canL1.filt.FilterMode         = CAN_FILTERMODE_IDMASK;
    canL1.filt.FilterScale        = CAN_FILTERSCALE_32BIT;
    canL1.filt.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    canL1.filt.FilterActivation   = ENABLE;
    canL1.filt.SlaveStartFilterBank = 14U;

    HAL_CAN_ConfigFilter(canL1.hcan, &canL1.filt);
}

/*******************************************************************************
 * Function : canL1filtCfgHub
 * Brief    : Hardware filter for the HUB.
 *            Accept only: DIR=1 (bit 28 set). Any ADDR, any PARAM_MAP.
 *            Rejects any frame sent in the Hub->Drive direction (DIR=0),
 *            including the hub's own TX frames if loopback is active.
 *******************************************************************************/
void canL1filtCfgHub(void)
{
    /* Acceptance ID: DIR=1, everything else don't care                      */
    u32 filtId   = CANL1_ID_DIR_MASK;

    /* Mask: only the DIR bit must match                                     */
    u32 filtMask = CANL1_ID_DIR_MASK;

    canL1.filt.FilterIdHigh       = (u16)((filtId   << 3) >> 16);
    canL1.filt.FilterIdLow        = (u16)((filtId   << 3) & 0xFFFFU) | 0x0004U;
    canL1.filt.FilterMaskIdHigh   = (u16)((filtMask << 3) >> 16);
    canL1.filt.FilterMaskIdLow    = (u16)((filtMask << 3) & 0xFFFFU) | 0x0004U;

    canL1.filt.FilterBank         = s_filtBank++;
    canL1.filt.FilterMode         = CAN_FILTERMODE_IDMASK;
    canL1.filt.FilterScale        = CAN_FILTERSCALE_32BIT;
    canL1.filt.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    canL1.filt.FilterActivation   = ENABLE;
    canL1.filt.SlaveStartFilterBank = 14U;

    HAL_CAN_ConfigFilter(canL1.hcan, &canL1.filt);
}

/*---------------------------------------------------------------------------*/
/* ISR Callbacks                                                              */
/*---------------------------------------------------------------------------*/

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    canL1rcvRxMsg();
}

/* tx.clk watchdog reset - preserved from original, all three mailboxes      */
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
    canL1.tx.clk = 0;
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
    canL1.tx.clk = 0;
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
    canL1.tx.clk = 0;
}

/*******************************************************************************
 * Function : HAL_CAN_ErrorCallback
 * Brief    : Categorise error flags into canL1ErrType fields.
 *            Preserved exactly from original including HAL_CAN_ResetError().
 *******************************************************************************/
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    u32 err = HAL_CAN_GetError(hcan);

    canL1.err.bus    |= (err & CAN_ERR_BUS_MASK)    ? 1U : 0U;
    canL1.err.rxOVRN |= (err & CAN_ERR_RX_OVR_MASK) ? 1U : 0U;
    canL1.err.txMB   |= (err & CAN_ERR_TX_MB_MASK)  ? 1U : 0U;
    canL1.err.hal    |= (err & CAN_ERR_HAL_MASK)    ? 1U : 0U;
    canL1.err.bof    |= (err & HAL_CAN_ERROR_BOF)   ? 1U : 0U;

    HAL_CAN_ResetError(canL1.hcan);
}

/*******************************************************************************
 * Function : canL1initLoopback
 * Brief    : Switch peripheral to CAN_MODE_SILENT_LOOPBACK for self-test.
 *            TX frames are looped back internally - nothing goes on the wire.
 *            The CAN handle's Mode field is patched directly so CubeMX init
 *            settings are not permanently changed.
 *******************************************************************************/
void canL1initLoopback(CAN_HandleTypeDef* hcan)
{
    canL1.hcan = hcan;

    HAL_CAN_Stop(canL1.hcan);
    HAL_CAN_DeInit(canL1.hcan);

    /* Patch mode - all other timing fields left as CubeMX configured them   */
    canL1.hcan->Init.Mode = CAN_MODE_SILENT_LOOPBACK;

    HAL_CAN_Init(canL1.hcan);
    HAL_CAN_Start(canL1.hcan);

    /* Same interrupts as normal init */
    HAL_CAN_ActivateNotification(canL1.hcan,
        CAN_IT_RX_FIFO0_MSG_PENDING |
        CAN_IT_TX_MAILBOX_EMPTY     |
        CAN_IT_ERROR                |
        CAN_IT_BUSOFF               |
        CAN_IT_LAST_ERROR_CODE);

    s_filtBank = 0;     /* Reset filter bank counter for fresh configuration  */
}

/*******************************************************************************
 * Function : canL1filtCfgLoopback
 * Brief    : Accept-all filter for loopback test.
 *            Passes any extended frame to FIFO0 regardless of ID content.
 *******************************************************************************/
void canL1filtCfgLoopback(void)
{
    canL1.filt.FilterIdHigh         = 0x0000U;
    canL1.filt.FilterIdLow          = 0x0000U;
    canL1.filt.FilterMaskIdHigh     = 0x0000U;
    canL1.filt.FilterMaskIdLow      = 0x0000U;
    canL1.filt.FilterBank           = s_filtBank++;
    canL1.filt.FilterMode           = CAN_FILTERMODE_IDMASK;
    canL1.filt.FilterScale          = CAN_FILTERSCALE_32BIT;
    canL1.filt.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    canL1.filt.FilterActivation     = ENABLE;
    canL1.filt.SlaveStartFilterBank = 14U;

    HAL_CAN_ConfigFilter(canL1.hcan, &canL1.filt);
}
