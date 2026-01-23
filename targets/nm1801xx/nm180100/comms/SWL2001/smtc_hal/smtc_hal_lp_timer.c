/*!
 * \file      smtc_hal_lp_timer.c
 *
 * \brief     Implements Low Power Timer utilities functions.
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type
#include <stddef.h>

#include <am_mcu_apollo.h>

#include "smtc_hal_lp_timer.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define HAL_LP_TIMER_NB 1  //!< Number of supported low power timers

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */
static hal_lp_timer_irq_t hal_lp_timer_irq;
static float hal_lp_ticks_in_ms;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */
static float rtc_get_clock_tick_in_ms();

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_lp_timer_init( hal_lp_timer_id_t id )
{
    uint32_t oldCfg = am_hal_stimer_config(AM_HAL_STIMER_CFG_FREEZE);

    am_hal_stimer_config(
        (oldCfg & ~(AM_HAL_STIMER_CFG_FREEZE))
        | AM_HAL_STIMER_CFG_COMPARE_C_ENABLE
        | AM_HAL_STIMER_CFG_COMPARE_D_ENABLE
        );

    am_hal_stimer_int_enable(AM_HAL_STIMER_INT_COMPAREC);
    am_hal_stimer_int_enable(AM_HAL_STIMER_INT_COMPARED);
    NVIC_EnableIRQ(STIMER_CMPR2_IRQn);
    NVIC_EnableIRQ(STIMER_CMPR3_IRQn);

    hal_lp_timer_irq.callback = NULL;
    hal_lp_ticks_in_ms = rtc_get_clock_tick_in_ms();
}

void hal_lp_timer_start( hal_lp_timer_id_t id, const uint32_t milliseconds, const hal_lp_timer_irq_t* tmr_irq )
{
    uint32_t ui32DelayInTicks;

    hal_lp_timer_irq.callback = tmr_irq->callback;
    hal_lp_timer_irq.context = tmr_irq->context;

    ui32DelayInTicks = milliseconds * hal_lp_ticks_in_ms;

    am_hal_stimer_compare_delta_set(2, ui32DelayInTicks);
    am_hal_stimer_compare_delta_set(3, ui32DelayInTicks+1);

    am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREC);
    am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPARED);
    am_hal_stimer_int_enable(AM_HAL_STIMER_INT_COMPAREC);
    am_hal_stimer_int_enable(AM_HAL_STIMER_INT_COMPARED);
}

void hal_lp_timer_stop( hal_lp_timer_id_t id )
{
    am_hal_stimer_int_disable(AM_HAL_STIMER_INT_COMPAREC);
    am_hal_stimer_int_disable(AM_HAL_STIMER_INT_COMPARED);
    am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREC);
    am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPARED);
}

void hal_lp_timer_irq_enable( hal_lp_timer_id_t id )
{
    NVIC_EnableIRQ(STIMER_CMPR2_IRQn);
    NVIC_EnableIRQ(STIMER_CMPR3_IRQn);
}

void hal_lp_timer_irq_disable( hal_lp_timer_id_t id )
{
    NVIC_DisableIRQ(STIMER_CMPR2_IRQn);
    NVIC_DisableIRQ(STIMER_CMPR3_IRQn);
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */
static float rtc_get_clock_tick_in_ms()
{
    uint32_t ui32CurrConfig = CTIMER->STCFG;
    uint32_t ui32Clksel = _FLD2VAL(CTIMER_STCFG_CLKSEL, ui32CurrConfig);
    float ui32Ticks = 0;

    switch(ui32Clksel)
    {
    case CTIMER_STCFG_CLKSEL_HFRC_DIV16:
        ui32Ticks = 3000.0f;
        break;

    case CTIMER_STCFG_CLKSEL_HFRC_DIV256:
        ui32Ticks = 187.5f;
        break;

    case CTIMER_STCFG_CLKSEL_XTAL_DIV1:
        ui32Ticks = 32.768f;
        break;

    case CTIMER_STCFG_CLKSEL_XTAL_DIV2:
        ui32Ticks = 16.384f;
        break;

    case CTIMER_STCFG_CLKSEL_XTAL_DIV32:
        ui32Ticks = 1.024f;
        break;

    case CTIMER_STCFG_CLKSEL_LFRC_DIV1:
        ui32Ticks = 1.024f;
        break;

    default:
        break;
    }

    return ui32Ticks;
}

void am_stimer_cmpr2_isr(void)
{
    am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREC);

    am_hal_stimer_int_disable(AM_HAL_STIMER_INT_COMPARED);
    am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPARED);

    if (hal_lp_timer_irq.callback)
    {
        hal_lp_timer_irq.callback(hal_lp_timer_irq.context);
        hal_lp_timer_irq.callback = NULL;
    }
}

void am_stimer_cmpr3_isr(void)
{
    am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPARED);
    am_hal_stimer_int_disable(AM_HAL_STIMER_INT_COMPARED);

    am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREC);
    am_hal_stimer_int_disable(AM_HAL_STIMER_INT_COMPAREC);

    if (hal_lp_timer_irq.callback)
    {
        hal_lp_timer_irq.callback(hal_lp_timer_irq.context);
        hal_lp_timer_irq.callback = NULL;
    }
}
/* --- EOF ------------------------------------------------------------------ */
