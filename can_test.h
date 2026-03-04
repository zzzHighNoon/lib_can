/******************************************************************************
 * @file    can_test.h
 * @brief   CAN stack loopback self-test.
 *
 *          Tests the full stack path in silent loopback mode:
 *            canL4setParam() -> canL4scan() -> canL3buildFrame()
 *            -> canL2sendFrmL4() -> canL1addTxMsg()
 *            -> [HW loopback] -> canL1 ISR -> canL2rcvFrmL1()
 *            -> canL3unpackFrame() -> canL4rcvFrmL2() -> canL4getParam()
 *
 *          Usage:
 *            1. In CubeMX: configure CAN1 timing for 1 Mbps but leave
 *               Mode as Normal - canL1initLoopback() overrides it at runtime.
 *            2. Call can_testRun(&hcan1) once from main() before the main loop.
 *            3. Check can_testGetResult() or set a breakpoint on the result.
 *            4. After testing call can_testRestore(&hcan1) to return to
 *               normal mode, then configure your production filter.
 *
 *          No hardware connection required. Nothing transmitted on the bus.
 ******************************************************************************/
#ifndef __CAN_TEST_H
#define __CAN_TEST_H

#include "globals.h"
#include "lib_can.h"

/*---------------------------------------------------------------------------*/
/* Test result                                                                */
/*---------------------------------------------------------------------------*/
typedef enum
{
    CAN_TEST_PENDING  = 0,  /* Test not yet complete                         */
    CAN_TEST_PASS     = 1,  /* All cases passed                              */
    CAN_TEST_FAIL     = 2   /* One or more cases failed - check detail flags  */
} canTestResultType;

/*---------------------------------------------------------------------------*/
/* Per-test-case pass/fail detail                                             */
/*---------------------------------------------------------------------------*/
typedef struct
{
    /* ID encode/decode round-trip */
    u8 idEncodeDecode;      /* canL3encodeId -> canL2decodeId matches         */

    /* Frame build / unpack round-trip */
    u8 frameBuildUnpack;    /* canL3buildFrame -> canL3unpackFrame matches     */

    /* L1 TX accepted into mailbox */
    u8 l1TxAccepted;        /* canL1addTxMsg returned CANL1_TX_OK             */

    /* L1 RX received via loopback ISR */
    u8 l1RxReceived;        /* FIFO0 ISR fired and frame was passed to L2     */

    /* Full stack: setParam -> scan -> loopback -> getParam */
    u8 fullStackAO;         /* AO param set on hub, received on drive side    */
    u8 fullStackAI;         /* AI param set on drive side, received on hub    */

    /* Value integrity */
    u8 valueIntegrity;      /* All received values match transmitted values   */

    /* DLC matches param count */
    u8 dlcCorrect;          /* DLC = paramCount * 2 verified on receive       */

} canTestDetailType;

/*---------------------------------------------------------------------------*/
/* Function prototypes                                                        */
/*---------------------------------------------------------------------------*/

/**
 * @brief  Run all loopback self-tests.
 *         Initialises the peripheral in silent loopback mode, runs tests,
 *         stores results. Blocking - polls for RX up to a timeout.
 * @param  hcan  Pointer to CubeMX CAN handle (e.g. &hcan1).
 */
void can_testRun(CAN_HandleTypeDef* hcan);

/**
 * @brief  Return the overall test result.
 */
canTestResultType can_testGetResult(void);

/**
 * @brief  Return pointer to per-case detail flags for inspection.
 */
const canTestDetailType* can_testGetDetail(void);

/**
 * @brief  Restore the peripheral to normal operating mode after testing.
 *         Stops CAN, reinitialises with CAN_MODE_NORMAL, restarts.
 *         Call this before configuring production filters.
 * @param  hcan  Same handle passed to can_testRun().
 */
void can_testRestore(CAN_HandleTypeDef* hcan);

#endif /* __CAN_TEST_H */
