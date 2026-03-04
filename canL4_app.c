/******************************************************************************
 * @file    canL4_app.c
 * @brief   CAN Layer 4 - Application layer implementation.
 *          Compiled for hub or drive depending on CAN_TARGET_HUB /
 *          CAN_TARGET_DRIVE define in the build system.
 ******************************************************************************/
#include "canL4_app.h"
#include "canL2_link.h"     /* canL2TxResultType, canL2sendFrmL4, CANL2_TX_* */
#include <string.h>

/* Forward declaration: test hook defined in can_test.c.
 * Only referenced when CAN_LOOPBACK_TEST is defined in the build.          */
#ifdef CAN_LOOPBACK_TEST
extern void can_testRxHook(const canFrmType* frm, const canL2hdrType* hdr);
#endif

/*---------------------------------------------------------------------------*/
/* Module state                                                               */
/*---------------------------------------------------------------------------*/
#ifdef CAN_TARGET_HUB
canL4driveStateType canL4hub[CAN_NUM_DRIVES];
#endif

#ifdef CAN_TARGET_DRIVE
canL4paramStateType canL4drive[CAN_TOTAL_REGS];
u8                  canL4driveAddr = 0U;
#endif

/* Stateful scanner cursor - persists across canL4scan() calls              */
#ifdef CAN_TARGET_HUB
static u8  s_curDrive = 0U;
#endif
static u8  s_curParam = 0U;

/*---------------------------------------------------------------------------*/
/* Internal helpers                                                           */
/*---------------------------------------------------------------------------*/

/**
 * @brief  Evaluate send condition for one parameter state entry.
 * @param  state    Pointer to the param state.
 * @param  mapIdx   Index into canL2map[] for minTO/maxTO.
 * @param  sysTick  Current tick.
 * @return 1 if this param should be sent, 0 otherwise.
 */
static u8 shouldSend(const canL4paramStateType* state,
                      u8 mapIdx,
                      u32 sysTick)
{
    u32 elapsed = sysTick - state->lastSentTick;
    u16 minTO   = canL2map[mapIdx].minTO;
    u16 maxTO   = canL2map[mapIdx].maxTO;

    if (state->dirty && elapsed >= (u32)minTO) return 1U;
    if (elapsed >= (u32)maxTO)                 return 1U;
    return 0U;
}

/**
 * @brief  Find the best fill param from the same DIR pool.
 *         "Best" = closest to maxTO expiry (largest elapsed/maxTO ratio).
 *         Must not already be in the ready list.
 *
 * @param  readyBits  Bitmask of param indices already selected (bit N = idx N)
 * @param  dir        AO or AI - only params of matching type considered
 * @param  sysTick    Current tick
 * @param  pState     Param state array for this drive (or drive state)
 * @return Param index of best fill, or 0xFF if none available
 */
static u8 findFillParam(u16 readyBits,
                         u8 dir,
                         u32 sysTick,
                         const canL4paramStateType* pState)
{
    u8  bestIdx   = 0xFFU;
    u32 bestScore = 0U;

    for (u8 p = 0U; p < CAN_TOTAL_REGS; p++)
    {
        /* Must match direction and not already selected                     */
        if (canL2map[p].type != dir)         continue;
        if (readyBits & (u16)(1U << p))      continue;

        /* Score = elapsed * 100 / maxTO  (0-100+, higher = more overdue)   */
        u32 elapsed = sysTick - pState[p].lastSentTick;
        u32 score   = (elapsed * 100UL) / (u32)canL2map[p].maxTO;

        if (score > bestScore)
        {
            bestScore = score;
            bestIdx   = p;
        }
    }
    return bestIdx;
}

/**
 * @brief  Build and send one CAN frame for a given drive and set of params.
 *         Updates lastSentTick and clears dirty for all transmitted params.
 *
 * @param  dir       AO or AI
 * @param  addr      Destination/source node address
 * @param  indices   Array of param indices to send (1-4 entries)
 * @param  count     Number of entries in indices[]
 * @param  pState    Param state array (values sourced from here)
 * @param  sysTick   Current tick (written to lastSentTick on success)
 * @return CANL2_TX_OK / CANL2_TX_BUSY / CANL2_TX_ERROR
 */
static canL2TxResultType buildAndSend(u8 dir,
                                       u8 addr,
                                       const u8* indices,
                                       u8 count,
                                       canL4paramStateType* pState,
                                       u32 sysTick)
{
    canL3frameDescType desc;
    canFrmType         frm;

    desc.dir   = dir;
    desc.addr  = addr;
    desc.count = count;

    for (u8 i = 0U; i < count; i++)
    {
        desc.paramIdx[i] = indices[i];
        desc.values[i]   = pState[indices[i]].value;
    }

    canL3buildFrame(&desc, &frm);

    canL2TxResultType res = canL2sendFrmL4(&frm);

    if (res == CANL2_TX_OK)
    {
        /* Update state for all transmitted params                           */
        for (u8 i = 0U; i < count; i++)
        {
            pState[indices[i]].lastSentTick = sysTick;
            pState[indices[i]].lastSent     = pState[indices[i]].value;
            pState[indices[i]].dirty        = 0U;
        }
    }

    return res;
}

/*---------------------------------------------------------------------------*/
/* Shared scanner core                                                        */
/* Scans params of a given direction for one node, builds one frame if ready */
/* Returns CANL2_TX_BUSY if mailboxes full (caller stops scanning)           */
/* Returns CANL2_TX_OK if frame sent or nothing to send for this pass        */
/*---------------------------------------------------------------------------*/
static canL2TxResultType scanOneNode(u8 dir,
                                      u8 addr,
                                      canL4paramStateType* pState,
                                      u32 sysTick)
{
    u8  readyIdx[CANL3_MAX_PARAMS_PER_FRAME];
    u8  readyCount = 0U;
    u16 readyBits  = 0U;

    /* First pass: collect params that pass send condition, priority = index */
    for (u8 p = s_curParam; p < CAN_TOTAL_REGS; p++)
    {
        if (canL2map[p].type != dir)          continue;
        if (!shouldSend(&pState[p], p, sysTick)) continue;

        readyIdx[readyCount++] = p;
        readyBits |= (u16)(1U << p);

        if (readyCount >= CANL3_MAX_PARAMS_PER_FRAME) break;
    }

    /* Nothing ready for this node on this pass                              */
    if (readyCount == 0U) return CANL2_TX_OK;

    /* Fill rule: if only 1 param ready, add the best fill to make 2        */
    if (readyCount == 1U)
    {
        u8 fillIdx = findFillParam(readyBits, dir, sysTick, pState);
        if (fillIdx != 0xFFU)
        {
            readyIdx[readyCount++] = fillIdx;
        }
        /* If no fill found (e.g. only 1 param exists for this dir),
           send the single param alone - DLC=2 is acceptable                */
    }

    return buildAndSend(dir, addr, readyIdx, readyCount, pState, sysTick);
}

/*===========================================================================*/
/* Public API                                                                 */
/*===========================================================================*/

/*******************************************************************************
 * Function : canL4init
 *******************************************************************************/
void canL4init(u8 nodeAddr)
{
#ifdef CAN_TARGET_HUB
    (void)nodeAddr;
    memset(canL4hub, 0, sizeof(canL4hub));

    for (u8 d = 0U; d < CAN_NUM_DRIVES; d++)
    {
        for (u8 p = 0U; p < CAN_TOTAL_REGS; p++)
        {
            canL4hub[d].param[p].value        = canMap[p].dflt;
            canL4hub[d].param[p].lastSent     = canMap[p].dflt;
            canL4hub[d].param[p].lastSentTick = 0U;
            canL4hub[d].param[p].dirty        = 0U;
        }
        canL4hub[d].active = 0U;    /* All drives inactive until discovered  */
    }
    s_curDrive = 0U;
#endif

#ifdef CAN_TARGET_DRIVE
    canL4driveAddr = nodeAddr;
    memset(canL4drive, 0, sizeof(canL4drive));

    for (u8 p = 0U; p < CAN_TOTAL_REGS; p++)
    {
        canL4drive[p].value        = canMap[p].dflt;
        canL4drive[p].lastSent     = canMap[p].dflt;
        canL4drive[p].lastSentTick = 0U;
        canL4drive[p].dirty        = 0U;
    }
#endif

    s_curParam = 0U;
}

/*******************************************************************************
 * Function : canL4scan
 * Brief    : Main scanner entry point. Call from while(1) or low-prio IRQ.
 *
 *            Hub: round-robin across active drives, scan AO params per drive.
 *            Drive: scan AI params, send to hub.
 *
 *            Stateful cursor (s_curDrive, s_curParam) ensures no drive or
 *            param is starved when TX mailboxes are frequently busy.
 *******************************************************************************/
void canL4scan(u32 sysTick)
{
#ifdef CAN_TARGET_HUB

    u8 drivesChecked = 0U;

    while (drivesChecked < CAN_NUM_DRIVES)
    {
        /* Skip inactive drives */
        if (!canL4hub[s_curDrive].active)
        {
            s_curDrive = (s_curDrive + 1U) % CAN_NUM_DRIVES;
            s_curParam = 0U;
            drivesChecked++;
            continue;
        }

        canL2TxResultType res = scanOneNode(
            AO,
            s_curDrive,         /* addr = drive index (0-15) */
            canL4hub[s_curDrive].param,
            sysTick);

        if (res == CANL2_TX_BUSY)
        {
            /* All mailboxes full - stall here, resume at same cursor        */
            return;
        }

        /* Advance to next drive after servicing current one                */
        s_curDrive = (s_curDrive + 1U) % CAN_NUM_DRIVES;
        s_curParam = 0U;
        drivesChecked++;
    }

#endif /* CAN_TARGET_HUB */

#ifdef CAN_TARGET_DRIVE

    canL2TxResultType res = scanOneNode(
        AI,
        CAN_HUB_ADDR,           /* Drive always sends to hub                */
        canL4drive,
        sysTick);

    if (res == CANL2_TX_BUSY)
    {
        /* Stall - resume at s_curParam on next call                        */
        return;
    }

    /* Reset param cursor for next scan pass                                */
    s_curParam = 0U;

#endif /* CAN_TARGET_DRIVE */
}

/*******************************************************************************
 * Function : canL4rcvFrmL2
 * Brief    : Called from L2 RX path (ISR context).
 *            Unpacks received params and writes to state table.
 *            In loopback test mode, routes to can_testRxHook() instead.
 *******************************************************************************/
void canL4rcvFrmL2(const canFrmType* frm, const canL2hdrType* hdr)
{
    canL3unpackType unpacked;

    if (canL3unpackFrame(frm, hdr->paramMap, &unpacked) != 0)
        return;     /* DLC/bitcount mismatch - discard                       */

#ifdef CAN_LOOPBACK_TEST
    /* In test mode: route received frame to test capture hook instead       */
    can_testRxHook(frm, hdr);
    return;
#endif

#ifdef CAN_TARGET_HUB
    /* Hub receives AI params (dir=1) from a drive                          */
    if (hdr->dir != AI)                         return;
    if (hdr->addr >= CAN_NUM_DRIVES)            return;

    canL4driveStateType* drv = &canL4hub[hdr->addr];

    for (u8 i = 0U; i < unpacked.count; i++)
    {
        u8 p = unpacked.paramIdx[i];
        if (p >= CAN_TOTAL_REGS)                continue;
        if (canMap[p].type != AI)               continue;  /* Sanity check  */

        drv->param[p].value = unpacked.values[i];
        /* AI params received by hub: mark active, no dirty flag needed     */
        drv->active = 1U;
    }
#endif

#ifdef CAN_TARGET_DRIVE
    /* Drive receives AO params (dir=0) from hub                            */
    if (hdr->dir != AO)                         return;

    for (u8 i = 0U; i < unpacked.count; i++)
    {
        u8 p = unpacked.paramIdx[i];
        if (p >= CAN_TOTAL_REGS)                continue;
        if (canMap[p].type != AO)               continue;  /* Sanity check  */

        /* Clamp to min/max before storing                                  */
        u16 val = unpacked.values[i];
        if (val < canMap[p].min) val = canMap[p].min;
        if (val > canMap[p].max) val = canMap[p].max;

        canL4drive[p].value = val;
        canL4drive[p].dirty = 0U;   /* Just received from hub - not dirty   */
    }
#endif
}

/*******************************************************************************
 * Function : canL4setParam
 *******************************************************************************/
void canL4setParam(u8 driveIdx, u8 paramIdx, u16 value)
{
    if (paramIdx >= CAN_TOTAL_REGS) return;

#ifdef CAN_TARGET_HUB
    if (driveIdx >= CAN_NUM_DRIVES) return;
    canL4paramStateType* s = &canL4hub[driveIdx].param[paramIdx];
    if (s->value != value)
    {
        s->value = value;
        s->dirty = 1U;
    }
#endif

#ifdef CAN_TARGET_DRIVE
    (void)driveIdx;
    canL4paramStateType* s = &canL4drive[paramIdx];
    if (s->value != value)
    {
        s->value = value;
        s->dirty = 1U;
    }
#endif
}

/*******************************************************************************
 * Function : canL4getParam
 *******************************************************************************/
u16 canL4getParam(u8 driveIdx, u8 paramIdx)
{
    if (paramIdx >= CAN_TOTAL_REGS) return 0U;

#ifdef CAN_TARGET_HUB
    if (driveIdx >= CAN_NUM_DRIVES) return 0U;
    return canL4hub[driveIdx].param[paramIdx].value;
#endif

#ifdef CAN_TARGET_DRIVE
    (void)driveIdx;
    return canL4drive[paramIdx].value;
#endif
}

/*******************************************************************************
 * Function : canL4setDriveActive  (hub only)
 *******************************************************************************/
#ifdef CAN_TARGET_HUB
void canL4setDriveActive(u8 driveIdx, u8 active)
{
    if (driveIdx >= CAN_NUM_DRIVES) return;
    canL4hub[driveIdx].active = active;
}
#endif
