/******************************************************************************
 * @file    canL4_app.h
 * @brief   CAN Layer 4 - Application layer.
 *
 *          Hub build:
 *            hubState[CAN_NUM_DRIVES][CAN_TOTAL_REGS]
 *            Round-robin scanner across all active drives.
 *            AO params (0-6): hub holds current value, sends to drives.
 *            AI params (7-13): hub receives from drives, stores locally.
 *
 *          Drive build:
 *            driveState[CAN_TOTAL_REGS]
 *            Scanner iterates AI params (7-13), sends to hub.
 *            AO params (0-6): written by hub, stored locally on receive.
 *
 *          Scanner behaviour (main loop / lowest priority IRQ):
 *            - Stateful cursor: s_curDrive, s_curParam persist across calls
 *            - On CANL2_TX_BUSY: return immediately, resume at cursor
 *            - On frame sent: advance cursor round-robin
 *            - Frame fill rule:
 *                ready==1: pull 1 fill param (closest to maxTO, same DIR)
 *                ready 2-4: send exactly what is ready
 *                ready >4:  send top 4 (lower param index = higher priority)
 *
 *          Send condition per param:
 *            elapsed = sysTick - state.lastSentTick
 *            send if: (dirty && elapsed >= minTO) || (elapsed >= maxTO)
 *
 *          Compile-time target selection:
 *            Define CAN_TARGET_HUB   for hub build
 *            Define CAN_TARGET_DRIVE for drive build
 ******************************************************************************/
#ifndef __CANL4_APP_H
#define __CANL4_APP_H

#include "globals.h"
#include "can_map.h"
#include "canL3_proto.h"

/* canL2hdrType is defined in canL2_link.h - visible at all call sites
 * through lib_can.h which includes canL2_link.h before canL4_app.h.        */


/*---------------------------------------------------------------------------*/
/* Build target validation                                                    */
/*---------------------------------------------------------------------------*/
#define CAN_TARGET_HUB
#if !defined(CAN_TARGET_HUB) && !defined(CAN_TARGET_DRIVE)
  #error "Define either CAN_TARGET_HUB or CAN_TARGET_DRIVE in your build"
#endif
#if defined(CAN_TARGET_HUB) && defined(CAN_TARGET_DRIVE)
  #error "CAN_TARGET_HUB and CAN_TARGET_DRIVE cannot both be defined"
#endif

/*---------------------------------------------------------------------------*/
/* Per-parameter runtime state                                                */
/*---------------------------------------------------------------------------*/
typedef struct
{
    u16  value;             /* Current live value                            */
    u16  lastSent;          /* Value at time of last transmission            */
    u32  lastSentTick;      /* sysTick value at last transmission            */
    u8   dirty;             /* 1 = value changed since last send             */
} canL4paramStateType;

/*---------------------------------------------------------------------------*/
/* Per-drive state block (hub only: array of CAN_NUM_DRIVES)                 */
/*---------------------------------------------------------------------------*/
typedef struct
{
    canL4paramStateType param[CAN_TOTAL_REGS];
    u8  active;             /* 1 = this drive is on the network              */
} canL4driveStateType;

/*---------------------------------------------------------------------------*/
/* Module state (extern for diagnostic access)                               */
/*---------------------------------------------------------------------------*/
#ifdef CAN_TARGET_HUB
extern canL4driveStateType canL4hub[CAN_NUM_DRIVES];
#endif

#ifdef CAN_TARGET_DRIVE
extern canL4paramStateType canL4drive[CAN_TOTAL_REGS];
extern u8                  canL4driveAddr;  /* This drive's node address     */
#endif

/*---------------------------------------------------------------------------*/
/* Function prototypes                                                        */
/*---------------------------------------------------------------------------*/

/**
 * @brief  Initialise L4 state. Load defaults from canMap[].dflt.
 *         Hub: initialise all 16 drive state blocks, mark all inactive.
 *         Drive: initialise single state array, set node address.
 * @param  nodeAddr  Drive build only: this node's address (0-15).
 *                   Hub build: pass 0, ignored.
 */
void canL4init(u8 nodeAddr);

/**
 * @brief  Main scanner - call from while(1) or lowest priority IRQ.
 *
 *         Hub: iterates active drives round-robin. For each drive, scans
 *              AO params (0-6) for send condition. Builds and sends frame.
 *              On BUSY: returns immediately. Cursor preserved for next call.
 *
 *         Drive: scans AI params (7-13) for send condition. Builds and
 *                sends frame to hub. On BUSY: returns immediately.
 *
 * @param  sysTick  Current 32-bit system tick (milliseconds).
 */
void canL4scan(u32 sysTick);

/**
 * @brief  RX entry point called from L2 on receipt of a valid frame.
 *         Hub: receives AI params (7-13) from a drive, stores in hubState.
 *         Drive: receives AO params (0-6) from hub, stores in driveState.
 * @param  frm  Received frame.
 * @param  hdr  Decoded L2 header (dir, addr, paramMap).
 */
void canL4rcvFrmL2(const canFrmType* frm, const canL2hdrType* hdr);

/**
 * @brief  Write a parameter value (hub AO or drive AI).
 *         Sets dirty flag if value has changed. Thread-safe: disable IRQ
 *         around the write if called from both main loop and IRQ context.
 *
 *         Hub:   paramIdx 0-6  (AO - to be sent to drives)
 *                driveIdx 0-15 selects which drive's table to update.
 *         Drive: paramIdx 7-13 (AI - to be sent to hub)
 *                driveIdx ignored.
 *
 * @param  driveIdx  Hub only: target drive (0-15). Drive build: pass 0.
 * @param  paramIdx  Parameter index (0-13).
 * @param  value     New u16 value.
 */
void canL4setParam(u8 driveIdx, u8 paramIdx, u16 value);

/**
 * @brief  Read a parameter value.
 *         Hub:   paramIdx 7-13 (AI received from drive driveIdx)
 *                paramIdx 0-6  (AO currently queued for drive driveIdx)
 *         Drive: paramIdx 0-6  (AO received from hub)
 *                paramIdx 7-13 (AI current value before sending)
 * @param  driveIdx  Hub only: source drive (0-15). Drive build: pass 0.
 * @param  paramIdx  Parameter index (0-13).
 * @return Current value, or 0 if index out of range.
 */
u16 canL4getParam(u8 driveIdx, u8 paramIdx);

/**
 * @brief  Mark a drive as active (hub only). Inactive drives are skipped
 *         by the scanner. Call after node discovery or commissioning.
 * @param  driveIdx  Drive index (0-15).
 * @param  active    1 = active, 0 = inactive.
 */
#ifdef CAN_TARGET_HUB
void canL4setDriveActive(u8 driveIdx, u8 active);
#endif

#endif /* __CANL4_APP_H */
