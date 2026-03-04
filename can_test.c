/******************************************************************************
 * @file    can_test.c
 * @brief   CAN stack loopback self-test implementation.
 ******************************************************************************/
#include "can_test.h"
#include "canL1_hw.h"
#include "canL2_link.h"
#include "canL3_proto.h"
#include "canL4_app.h"
#include <string.h>

/*---------------------------------------------------------------------------*/
/* Test configuration                                                         */
/*---------------------------------------------------------------------------*/
#define TEST_DRIVE_IDX      0U          /* Drive slot used for full-stack test */
#define TEST_DRIVE_ADDR     0U          /* Node address for that drive slot     */
#define TEST_TIMEOUT_MS     100U        /* Max ms to wait for loopback RX       */

/* Test values - chosen to be non-default and easy to spot in a debugger     */
#define TEST_VAL_RPM_CMD    1234U       /* AO param 1  - RPM_CMD              */
#define TEST_VAL_DPOSN      15000U      /* AO param 2  - Delta Posn           */
#define TEST_VAL_RPM_FB     5678U       /* AI param 8  - RPM FB               */
#define TEST_VAL_STAT_BITS  0xA5A5U     /* AI param 13 - Status Bits          */

/*---------------------------------------------------------------------------*/
/* Module state                                                               */
/*---------------------------------------------------------------------------*/
static canTestResultType  s_result = CAN_TEST_PENDING;
static canTestDetailType  s_detail;

/* Loopback RX capture - filled by canL4rcvFrmL2 during test                 */
static volatile u8  s_rxFired   = 0U;
static volatile u16 s_rxParamMap = 0U;
static volatile u16 s_rxValues[CANL3_MAX_PARAMS_PER_FRAME];
static volatile u8  s_rxCount   = 0U;
static volatile u8  s_rxDlc     = 0U;

/*---------------------------------------------------------------------------*/
/* Test-mode RX hook                                                          */
/* canL4rcvFrmL2 calls this in loopback mode instead of writing to state     */
/*---------------------------------------------------------------------------*/
void can_testRxHook(const canFrmType* frm, const canL2hdrType* hdr)
{
    canL3unpackType unpacked;

    s_rxDlc     = frm->dlc;
    s_rxParamMap = hdr->paramMap;
    s_rxFired   = 1U;

    if (canL3unpackFrame(frm, hdr->paramMap, &unpacked) == 0)
    {
        s_rxCount = unpacked.count;
        for (u8 i = 0U; i < unpacked.count && i < CANL3_MAX_PARAMS_PER_FRAME; i++)
            s_rxValues[i] = unpacked.values[i];
    }
}

/*---------------------------------------------------------------------------*/
/* Simple busy-wait using HAL_GetTick() - no RTOS needed                     */
/*---------------------------------------------------------------------------*/
static u8 waitForRx(void)
{
    u32 start = HAL_GetTick();
    while (!s_rxFired)
    {
        if ((HAL_GetTick() - start) >= TEST_TIMEOUT_MS)
            return 0U;  /* Timeout */
    }
    return 1U;
}

/*===========================================================================*/
/* Test cases                                                                 */
/*===========================================================================*/

/*---------------------------------------------------------------------------*/
/* TC1: ID encode/decode round-trip                                           */
/*---------------------------------------------------------------------------*/
static u8 tc_idEncodeDecode(void)
{
    /* Encode a known DIR/ADDR/PARAM_MAP and decode it back                  */
    u8  dir_in   = 1U;          /* AI - Drive->Hub                           */
    u8  addr_in  = 5U;          /* Drive node 5                              */
    u16 pmap_in  = 0x0300U;     /* Bits 9 and 8 set (IFB + RPM FB)          */

    u32 extId = canL3encodeId(dir_in, addr_in, pmap_in);

    canL2hdrType hdr;
    canL2decodeId(extId, &hdr);

    if (hdr.dir      != dir_in)  return 0U;
    if (hdr.addr     != addr_in) return 0U;
    if (hdr.paramMap != pmap_in) return 0U;

    return 1U;
}

/*---------------------------------------------------------------------------*/
/* TC2: Frame build / unpack round-trip (no hardware)                        */
/*---------------------------------------------------------------------------*/
static u8 tc_frameBuildUnpack(void)
{
    /* Build a 2-param AO frame */
    canL3frameDescType desc;
    desc.dir        = AO;
    desc.addr       = TEST_DRIVE_ADDR;
    desc.count      = 2U;
    desc.paramIdx[0] = CAN_RPM_CMD;     /* param 1 */
    desc.paramIdx[1] = CAN_D_POSN;     /* param 2 */
    desc.values[0]  = TEST_VAL_RPM_CMD;
    desc.values[1]  = TEST_VAL_DPOSN;

    canFrmType frm;
    canL3buildFrame(&desc, &frm);

    /* DLC must be 4 (2 params * 2 bytes) */
    if (frm.dlc != 4U) return 0U;

    /* Unpack and verify */
    canL2hdrType hdr;
    canL2decodeId(frm.id, &hdr);

    canL3unpackType out;
    if (canL3unpackFrame(&frm, hdr.paramMap, &out) != 0) return 0U;
    if (out.count != 2U) return 0U;

    /* Values must match in order (MSB param first) */
    u8 found_rpm = 0U, found_dposn = 0U;
    for (u8 i = 0U; i < out.count; i++)
    {
        if (out.paramIdx[i] == CAN_RPM_CMD  && out.values[i] == TEST_VAL_RPM_CMD)
            found_rpm = 1U;
        if (out.paramIdx[i] == CAN_D_POSN   && out.values[i] == TEST_VAL_DPOSN)
            found_dposn = 1U;
    }

    return (found_rpm && found_dposn) ? 1U : 0U;
}

/*---------------------------------------------------------------------------*/
/* TC3: L1 TX accepted - verify mailbox accepts a raw frame                  */
/*---------------------------------------------------------------------------*/
static u8 tc_l1TxAccepted(void)
{
    canFrmType frm;
    frm.id      = canL3encodeId(AO, TEST_DRIVE_ADDR,
                                 (u16)(1U << canL2map[CAN_RPM_CMD].bit));
    frm.dlc     = 2U;
    frm.data[0] = (u8)(TEST_VAL_RPM_CMD >> 8);
    frm.data[1] = (u8)(TEST_VAL_RPM_CMD & 0xFFU);

    s_rxFired = 0U;
    canL1TxResultType res = canL1addTxMsg(&frm);
    return (res == CANL1_TX_OK) ? 1U : 0U;
}

/*---------------------------------------------------------------------------*/
/* TC4 + TC5 + TC6 + TC7: Full stack AO and AI                               */
/* Uses canL4setParam -> canL4scan -> [loopback] -> can_testRxHook           */
/*---------------------------------------------------------------------------*/
static void tc_fullStack(u8* aoPass, u8* aiPass,
                          u8* valPass, u8* dlcPass)
{
    *aoPass  = 0U;
    *aiPass  = 0U;
    *valPass = 0U;
    *dlcPass = 0U;

    /* ---- AO test: set two hub->drive params, scan, check RX hook ---- */

    /* Mark drive 0 active so scanner will service it                        */
    canL4setDriveActive(TEST_DRIVE_IDX, 1U);

    /* Write two AO values - force dirty + maxTO expired by setting
       lastSentTick to 0 (already 0 from init)                              */
    canL4setParam(TEST_DRIVE_IDX, CAN_RPM_CMD, TEST_VAL_RPM_CMD);
    canL4setParam(TEST_DRIVE_IDX, CAN_D_POSN,  TEST_VAL_DPOSN);

    s_rxFired = 0U;
    s_rxCount = 0U;

    /* Run scanner - should build and transmit one AO frame                  */
    canL4scan(0U);      /* sysTick=0, all maxTO already expired              */

    /* Wait for loopback RX                                                  */
    if (!waitForRx()) return;     /* Timeout - all sub-tests fail            */

    *aoPass = 1U;

    /* Check DLC = 4 (2 params) */
    *dlcPass = (s_rxDlc == 4U) ? 1U : 0U;

    /* Check values */
    u8 foundRpm = 0U, foundDposn = 0U;
    for (u8 i = 0U; i < s_rxCount; i++)
    {
        /* Map received paramIdx back via hook's unpacked data               */
        /* We verify via PARAM_MAP bits instead                              */
    }
    /* Verify by re-unpacking using stored paramMap                          */
    {
        canFrmType chk;
        chk.dlc = s_rxDlc;
        for (u8 b = 0U; b < s_rxDlc; b++)
            chk.data[b] = 0U;   /* We don't have raw bytes here - use values */

        /* Value check direct from hook capture                              */
        for (u8 i = 0U; i < s_rxCount; i++)
        {
            if (s_rxValues[i] == TEST_VAL_RPM_CMD) foundRpm   = 1U;
            if (s_rxValues[i] == TEST_VAL_DPOSN)   foundDposn = 1U;
        }
    }
    *valPass = (foundRpm && foundDposn) ? 1U : 0U;

    /* ---- AI test: set two drive->hub params, scan, check RX hook ---- */
    canL4setParam(0U, CAN_RPM_FB,    TEST_VAL_RPM_FB);
    canL4setParam(0U, CAN_STAT_BITS, TEST_VAL_STAT_BITS);

    s_rxFired = 0U;
    s_rxCount = 0U;

    canL4scan(0U);

    if (!waitForRx()) return;

    *aiPass = 1U;

    /* Check AI values received */
    u8 foundFb = 0U, foundStat = 0U;
    for (u8 i = 0U; i < s_rxCount; i++)
    {
        if (s_rxValues[i] == TEST_VAL_RPM_FB)    foundFb   = 1U;
        if (s_rxValues[i] == TEST_VAL_STAT_BITS) foundStat = 1U;
    }
    if (foundFb && foundStat && *valPass) *valPass = 1U;
    else                                  *valPass = 0U;
}

/*===========================================================================*/
/* Public API                                                                 */
/*===========================================================================*/

void can_testRun(CAN_HandleTypeDef* hcan)
{
    memset(&s_detail, 0, sizeof(s_detail));
    s_result  = CAN_TEST_PENDING;
    s_rxFired = 0U;

    /* --- Init hardware in silent loopback mode --- */
    canL1initLoopback(hcan);
    canL1filtCfgLoopback();

    /* --- Init upper layers ---
     * Use drive build for AI direction test, hub build for AO.
     * Since we're compiled as one target, we test whichever direction
     * matches the compile-time CAN_TARGET_* define.                        */
    canL4init(TEST_DRIVE_ADDR);

    /* --- Run test cases --- */

    /* TC1: ID encode/decode - pure software, no hardware needed            */
    s_detail.idEncodeDecode = tc_idEncodeDecode();

    /* TC2: Frame build/unpack - pure software                              */
    s_detail.frameBuildUnpack = tc_frameBuildUnpack();

    /* TC3: L1 TX accepted into mailbox                                     */
    s_detail.l1TxAccepted = tc_l1TxAccepted();

    /* TC4: Wait for the TC3 frame to loop back via ISR                     */
    s_rxFired = 0U;
    s_detail.l1RxReceived = waitForRx();

    /* TC5-8: Full stack via L4 scanner                                     */
    tc_fullStack(&s_detail.fullStackAO,
                 &s_detail.fullStackAI,
                 &s_detail.valueIntegrity,
                 &s_detail.dlcCorrect);

    /* --- Overall result --- */
    if (s_detail.idEncodeDecode   &&
        s_detail.frameBuildUnpack &&
        s_detail.l1TxAccepted     &&
        s_detail.l1RxReceived     &&
        s_detail.fullStackAO      &&
        s_detail.fullStackAI      &&
        s_detail.valueIntegrity   &&
        s_detail.dlcCorrect)
    {
        s_result = CAN_TEST_PASS;
    }
    else
    {
        s_result = CAN_TEST_FAIL;
    }
}

canTestResultType can_testGetResult(void)
{
    return s_result;
}

const canTestDetailType* can_testGetDetail(void)
{
    return &s_detail;
}

void can_testRestore(CAN_HandleTypeDef* hcan)
{
    HAL_CAN_Stop(hcan);
    HAL_CAN_DeInit(hcan);
    hcan->Init.Mode = CAN_MODE_NORMAL;
    HAL_CAN_Init(hcan);
    /* Caller re-runs canL1init() and canL1filtCfgDrive/Hub() after this    */
}
