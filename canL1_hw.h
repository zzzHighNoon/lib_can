/******************************************************************************
 * @file    canL1_hw.h
 * @brief   CAN Layer 1 - Hardware/HAL layer for STM32F103
 *          Rewritten from standard 11-bit to extended 29-bit ID.
 *
 *          Preserves from original:
 *            - canL1ErrType error struct and categorised error masks
 *            - CANL1_TX_OK / BUSY / ERROR result codes
 *            - tx.clk watchdog pattern (reset in TX complete callbacks)
 *            - HAL_CAN_ResetError() at end of error callback
 *            - canL1filtBank incremental filter allocation
 *
 *          Changes from original:
 *            - CAN_ID_STD -> CAN_ID_EXT throughout
 *            - StdId -> ExtId in TX header
 *            - Consolidated to FIFO0 for RX
 *            - Filter helpers rewritten for 29-bit extended ID mask format
 *            - canL1filtCfgDrive() and canL1filtCfgHub() replace generic fns
 ******************************************************************************/
#ifndef __CANL1_HW_H
#define __CANL1_HW_H

#include "globals.h"

/*---------------------------------------------------------------------------*/
/* Error category masks - preserved from original                             */
/*---------------------------------------------------------------------------*/

/* Bus / protocol-level errors */
#define CAN_ERR_BUS_MASK    ( \
    HAL_CAN_ERROR_STF  |      \
    HAL_CAN_ERROR_FOR  |      \
    HAL_CAN_ERROR_ACK  |      \
    HAL_CAN_ERROR_BR   |      \
    HAL_CAN_ERROR_BD   |      \
    HAL_CAN_ERROR_CRC  )

/* RX FIFO overruns */
#define CAN_ERR_RX_OVR_MASK ( \
    HAL_CAN_ERROR_RX_FOV0 |   \
    HAL_CAN_ERROR_RX_FOV1 )

/* TX mailbox errors (arbitration lost excluded intentionally) */
#define CAN_ERR_TX_MB_MASK  ( \
    HAL_CAN_ERROR_TX_TERR0 |  \
    HAL_CAN_ERROR_TX_TERR1 |  \
    HAL_CAN_ERROR_TX_TERR2 )

/* HAL / software misuse errors */
#define CAN_ERR_HAL_MASK    (          \
    HAL_CAN_ERROR_TIMEOUT         |    \
    HAL_CAN_ERROR_NOT_INITIALIZED |    \
    HAL_CAN_ERROR_NOT_READY       |    \
    HAL_CAN_ERROR_NOT_STARTED     |    \
    HAL_CAN_ERROR_PARAM           )

/*---------------------------------------------------------------------------*/
/* 29-bit Extended ID field layout                                            */
/*                                                                            */
/*  Bit  28       27..23    22..9          8..0                               */
/*       ───────  ────────  ─────────────  ────────                           */
/*       DIR      ADDR      PARAM_MAP      reserved                           */
/*       1 bit    5 bits    14 bits        9 bits                             */
/*                                                                            */
/*  DIR:       0 = Hub->Drive (AO)   1 = Drive->Hub (AI)                     */
/*  ADDR:      0-15 = drive node     16 = hub    31 = broadcast               */
/*  PARAM_MAP: bit N set = param N present in this frame (bits 0-13 used)     */
/*---------------------------------------------------------------------------*/
#define CANL1_ID_DIR_SHIFT      28U
#define CANL1_ID_ADDR_SHIFT     23U
#define CANL1_ID_PMAP_SHIFT     9U

#define CANL1_ID_DIR_MASK       (0x1UL  << CANL1_ID_DIR_SHIFT)
#define CANL1_ID_ADDR_MASK      (0x1FUL << CANL1_ID_ADDR_SHIFT)
#define CANL1_ID_PMAP_MASK      (0x3FFFUL<< CANL1_ID_PMAP_SHIFT)

/*---------------------------------------------------------------------------*/
/* Types - preserved and extended from original                               */
/*---------------------------------------------------------------------------*/

/* TX result - preserved exactly from original */
typedef enum
{
    CANL1_TX_OK = 0,    /* Accepted into a TX mailbox                        */
    CANL1_TX_BUSY,      /* No mailbox free - caller stalls and retries        */
    CANL1_TX_ERROR      /* HAL_ERROR or bad state - log and recover           */
} canL1TxResultType;

/* Categorised error flags - preserved exactly from original */
typedef struct {
    u8 bus;             /* Bus/protocol error (stuff, form, ACK, CRC...)      */
    u8 rxOVRN;          /* RX FIFO overrun                                    */
    u8 txMB;            /* TX mailbox error                                   */
    u8 hal;             /* HAL/software misuse error                          */
    u8 bof;             /* Bus-off condition                                  */
} canL1ErrType;

/* RX frame container */
typedef struct
{
    CAN_RxHeaderTypeDef hdr;    /* HAL RX header (ExtId lives here)           */
    canFrmType          frm;    /* Decoded frame passed to L2                 */
    u32                 clk;    /* Timestamp of last received frame           */
    u32                 err;    /* RX error counter                           */
    u32                 cnt;    /* Received frame counter                     */
} canL1rxType;

/* TX frame container */
typedef struct
{
    CAN_TxHeaderTypeDef hdr;    /* HAL TX header                              */
    u32                 clk;    /* Watchdog: reset to 0 in TX complete ISR    */
    u32                 mb;     /* Mailbox used for last transmission         */
} canL1txType;

/* Top-level L1 instance */
typedef struct
{
    CAN_FilterTypeDef   filt;
    CAN_HandleTypeDef*  hcan;
    canL1txType         tx;
    canL1rxType         rx;
    canL1ErrType        err;
} canL1type;

/*---------------------------------------------------------------------------*/
/* Function prototypes                                                        */
/*---------------------------------------------------------------------------*/

/**
 * @brief  Initialise L1: store handle, start CAN, activate interrupts.
 * @param  hcan  Pointer to CubeMX-generated CAN handle.
 * @return Pointer to the L1 instance.
 */
canL1type* canL1init(CAN_HandleTypeDef* hcan);

/**
 * @brief  Attempt to place one frame into a TX mailbox.
 *         Non-blocking. Returns BUSY if all three mailboxes are occupied.
 * @param  frm  Frame to transmit (extended ID in frm->id, 29-bit).
 * @return CANL1_TX_OK / CANL1_TX_BUSY / CANL1_TX_ERROR
 */
canL1TxResultType canL1addTxMsg(const canFrmType* frm);

/**
 * @brief  Configure hardware acceptance filter for a DRIVE node.
 *         Accepts only frames where DIR=0 (Hub->Drive) AND ADDR=nodeAddr.
 *         All other frames are rejected in hardware.
 * @param  nodeAddr  This drive's node address (0-15).
 */
void canL1filtCfgDrive(u8 nodeAddr);

/**
 * @brief  Configure hardware acceptance filter for the HUB.
 *         Accepts all frames where DIR=1 (Drive->Hub), any ADDR.
 */
void canL1filtCfgHub(void);

/**
 * @brief  Re-initialise the peripheral in silent loopback mode for self-test.
 *         TX frames are received internally without driving the bus wire.
 *         Call canL1filtCfgLoopback() after this to open the RX filter.
 *         Restore normal mode by calling canL1init() again after testing.
 * @param  hcan  Same CAN handle used in canL1init().
 */
void canL1initLoopback(CAN_HandleTypeDef* hcan);

/**
 * @brief  Open accept-all filter for loopback test.
 *         Accepts any extended frame regardless of DIR, ADDR or PARAM_MAP.
 */
void canL1filtCfgLoopback(void);

#endif /* __CANL1_HW_H */
