/*************************************************************************************************/
/*!
 *  \file   wsf_timer.c
 *
 *  \brief  Timer service.
 *
 *  Copyright (c) 2009-2019 Arm Ltd.
 *
 *  Copyright (c) 2019 Packetcraft, Inc.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
/*************************************************************************************************/
#include "am_mcu_apollo.h"

#include "wsf_types.h"
#include "wsf_queue.h"
#include "wsf_timer.h"
#include "wsf_assert.h"
#include "wsf_cs.h"
#include "wsf_trace.h"

#include "ble_config.h"

#define CLOCK_PERIOD      WSF_OS_CLOCK_PERIOD
#define CLOCK_SOURCE      WSF_OS_CLOCK_SOURCE

/* convert seconds to timer ticks */
#define WSF_TIMER_SEC_TO_TICKS(sec)         ((1000 / WSF_MS_PER_TICK) * (sec))

/* convert milliseconds to timer ticks */
#define WSF_TIMER_MS_TO_TICKS(ms)           ((ms) / WSF_MS_PER_TICK)

#define CLK_TICKS_PER_WSF_TICKS             (WSF_MS_PER_TICK*CLOCK_PERIOD / 1000)

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

wsfQueue_t  wsfTimerTimerQueue;     /*!< Timer queue */

/*! \brief  Last RTC value read. */
static uint32_t wsfTimerRtcLastTicks = 0;

void am_stimer_cmpr4_isr(void)
{
  am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREE);

  am_hal_stimer_int_disable(AM_HAL_STIMER_INT_COMPAREF);
  am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREF);

  WsfTaskSetReady(0, WSF_TIMER_EVENT);
}

void am_stimer_cmpr5_isr(void)
{
  am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREF);

  WsfTaskSetReady(0, WSF_TIMER_EVENT);
}

/*************************************************************************************************/
/*!
 *  \brief  Remove a timer from queue.  Note this function does not lock task scheduling.
 *
 *  \param  pTimer  Pointer to timer.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void wsfTimerRemove(wsfTimer_t *pTimer)
{
  wsfTimer_t  *pElem;
  wsfTimer_t  *pPrev = NULL;

  pElem = (wsfTimer_t *) wsfTimerTimerQueue.pHead;

  /* find timer in queue */
  while (pElem != NULL)
  {
    if (pElem == pTimer)
    {
      break;
    }
    pPrev = pElem;
    pElem = pElem->pNext;
  }

  /* if timer found remove from queue */
  if (pElem != NULL)
  {
    WsfQueueRemove(&wsfTimerTimerQueue, pTimer, pPrev);

    pTimer->isStarted = FALSE;
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Insert a timer into the queue sorted by the timer expiration.
 *
 *  \param  pTimer  Pointer to timer.
 *  \param  ticks   Timer ticks until expiration.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void wsfTimerInsert(wsfTimer_t *pTimer, wsfTimerTicks_t ticks)
{
  wsfTimer_t  *pElem;
  wsfTimer_t  *pPrev = NULL;

  /* task schedule lock */
  WsfTaskLock();

  /* if timer is already running stop it first */
  if (pTimer->isStarted)
  {
    wsfTimerRemove(pTimer);
  }

  pTimer->isStarted = TRUE;
  pTimer->ticks = ticks;

  pElem = (wsfTimer_t *) wsfTimerTimerQueue.pHead;

  /* find insertion point in queue */
  while (pElem != NULL)
  {
    if (pTimer->ticks < pElem->ticks)
    {
      break;
    }
    pPrev = pElem;
    pElem = pElem->pNext;
  }

  /* insert timer into queue */
  WsfQueueInsert(&wsfTimerTimerQueue, pTimer, pPrev);

  /* task schedule unlock */
  WsfTaskUnlock();
}

/*************************************************************************************************/
/*!
 *  \brief  Initialize the timer service.  This function should only be called once
 *          upon system initialization.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WsfTimerInit(void)
{
  WSF_QUEUE_INIT(&wsfTimerTimerQueue);

  am_hal_stimer_int_enable(AM_HAL_STIMER_INT_COMPAREE);
  am_hal_stimer_int_enable(AM_HAL_STIMER_INT_COMPAREF);
  NVIC_EnableIRQ(STIMER_CMPR4_IRQn);
  NVIC_EnableIRQ(STIMER_CMPR5_IRQn);

  uint32_t oldCfg = am_hal_stimer_config(AM_HAL_STIMER_CFG_FREEZE);

  am_hal_stimer_config(
      (oldCfg & ~(AM_HAL_STIMER_CFG_FREEZE | CTIMER_STCFG_CLKSEL_Msk))
      | CLOCK_SOURCE
      | AM_HAL_STIMER_CFG_COMPARE_E_ENABLE
      | AM_HAL_STIMER_CFG_COMPARE_F_ENABLE
      );


  wsfTimerRtcLastTicks = am_hal_stimer_counter_get();
}

/*************************************************************************************************/
/*!
 *  \brief  Start a timer in units of seconds.
 *
 *  \param  pTimer  Pointer to timer.
 *  \param  sec     Seconds until expiration.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WsfTimerStartSec(wsfTimer_t *pTimer, wsfTimerTicks_t sec)
{
  WSF_TRACE_INFO2("WsfTimerStartSec pTimer:0x%x ticks:%u", (uint32_t)pTimer, WSF_TIMER_SEC_TO_TICKS(sec));

  /* insert timer into queue */
  wsfTimerInsert(pTimer, WSF_TIMER_SEC_TO_TICKS(sec));
}

/*************************************************************************************************/
/*!
 *  \brief  Start a timer in units of milliseconds.
 *
 *  \param  pTimer  Pointer to timer.
 *  \param  ms     Milliseconds until expiration.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WsfTimerStartMs(wsfTimer_t *pTimer, wsfTimerTicks_t ms)
{
  WSF_TRACE_INFO2("WsfTimerStartMs pTimer:0x%x ticks:%u", (uint32_t)pTimer, WSF_TIMER_MS_TO_TICKS(ms));

  /* insert timer into queue */
  wsfTimerInsert(pTimer, WSF_TIMER_MS_TO_TICKS(ms));
}

/*************************************************************************************************/
/*!
 *  \brief  Stop a timer.
 *
 *  \param  pTimer  Pointer to timer.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WsfTimerStop(wsfTimer_t *pTimer)
{
  WSF_TRACE_INFO1("WsfTimerStop pTimer:0x%x", pTimer);

  /* task schedule lock */
  WsfTaskLock();

  wsfTimerRemove(pTimer);

  /* task schedule unlock */
  WsfTaskUnlock();
}

/*************************************************************************************************/
/*!
 *  \brief  Update the timer service with the number of elapsed ticks.
 *
 *  \param  ticks  Number of ticks since last update.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WsfTimerUpdate(wsfTimerTicks_t ticks)
{
  wsfTimer_t  *pElem;

  /* task schedule lock */
  WsfTaskLock();

  pElem = (wsfTimer_t *) wsfTimerTimerQueue.pHead;

  /* iterate over timer queue */
  while (pElem != NULL)
  {
    /* decrement ticks while preventing underflow */
    if (pElem->ticks > ticks)
    {
      pElem->ticks -= ticks;
    }
    else
    {
      pElem->ticks = 0;

      /* timer expired; set task for this timer as ready */
      WsfTaskSetReady(pElem->handlerId, WSF_TIMER_EVENT);
    }

    pElem = pElem->pNext;
  }

  /* task schedule unlock */
  WsfTaskUnlock();
}

/*************************************************************************************************/
/*!
 *  \brief  Return the number of ticks until the next timer expiration.  Note that this
 *          function can return zero even if a timer is running, indicating a timer
 *          has expired but has not yet been serviced.
 *
 *  \param  pTimerRunning   Returns TRUE if a timer is running, FALSE if no timers running.
 *
 *  \return The number of ticks until the next timer expiration.
 */
/*************************************************************************************************/
wsfTimerTicks_t WsfTimerNextExpiration(bool_t *pTimerRunning)
{
  wsfTimerTicks_t ticks;

  /* task schedule lock */
  WsfTaskLock();

  if (wsfTimerTimerQueue.pHead == NULL)
  {
    *pTimerRunning = FALSE;
    ticks = 0;
  }
  else
  {
    *pTimerRunning = TRUE;
    ticks = ((wsfTimer_t *) wsfTimerTimerQueue.pHead)->ticks;
  }

  /* task schedule unlock */
  WsfTaskUnlock();

  return ticks;
}

/*************************************************************************************************/
/*!
 *  \brief  Service expired timers for the given task.
 *
 *  \param  taskId      Task ID.
 *
 *  \return Pointer to timer or NULL.
 */
/*************************************************************************************************/
wsfTimer_t *WsfTimerServiceExpired(wsfTaskId_t taskId)
{
  wsfTimer_t  *pElem;
  wsfTimer_t  *pPrev = NULL;

  /* Unused parameters */
  (void)taskId;

  /* task schedule lock */
  WsfTaskLock();

  /* find expired timers in queue */
  if (((pElem = (wsfTimer_t *) wsfTimerTimerQueue.pHead) != NULL) &&
      (pElem->ticks == 0))
  {
    /* remove timer from queue */
    WsfQueueRemove(&wsfTimerTimerQueue, pElem, pPrev);

    pElem->isStarted = FALSE;

    /* task schedule unlock */
    WsfTaskUnlock();

    WSF_TRACE_INFO1("Timer expired pTimer:0x%x", pElem);

    /* return timer */
    return pElem;
  }

  /* task schedule unlock */
  WsfTaskUnlock();

  return NULL;
}

/*************************************************************************************************/
/*!
 *  \brief  Function for checking if there is an active timer and if there is enough time to
 *          go to sleep and going to sleep.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WsfTimerSleep(void)
{
  wsfTimerTicks_t nextExpiration;
  bool_t bTimerRunning;

  nextExpiration = WsfTimerNextExpiration(&bTimerRunning);

  if (nextExpiration > 0)
  {
    uint32_t sleep_ticks = nextExpiration * CLK_TICKS_PER_WSF_TICKS;
    am_hal_stimer_compare_delta_set(4, sleep_ticks);
    am_hal_stimer_compare_delta_set(5, sleep_ticks+1);

    /* enable RTC interrupt */
    am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREE);
    am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREF);

    am_hal_stimer_int_enable(AM_HAL_STIMER_INT_COMPAREE);
    am_hal_stimer_int_enable(AM_HAL_STIMER_INT_COMPAREF);
  }
  else
  {
    am_hal_stimer_int_disable(AM_HAL_STIMER_INT_COMPAREE);
    am_hal_stimer_int_disable(AM_HAL_STIMER_INT_COMPAREF);

    am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREE);
    am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREF);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Function for updating WSF timer based on elapsed RTC ticks.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WsfTimerSleepUpdate(void)
{
  uint32_t        elapsed;
  wsfTimerTicks_t wsfElapsed = 0;

  /* Get current RTC tick count. */
  uint32_t current_ticks = am_hal_stimer_counter_get();

  if (current_ticks != wsfTimerRtcLastTicks)
  {
    elapsed = current_ticks - wsfTimerRtcLastTicks;

    wsfElapsed = elapsed / CLK_TICKS_PER_WSF_TICKS;

    if (wsfElapsed)
    {
      /* update last ticks */
      wsfTimerRtcLastTicks = current_ticks;

      /* update wsf timers */
      WsfTimerUpdate(wsfElapsed);
    }
  }
}
