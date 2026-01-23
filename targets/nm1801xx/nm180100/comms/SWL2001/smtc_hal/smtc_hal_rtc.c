/*!
 * \file      smtc_hal_rtc.c
 *
 * \brief     RTC Hardware Abstraction Layer implementation
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
#include <limits.h>
#include <time.h>

#include <am_mcu_apollo.h>
#include <am_util.h>

#include "smtc_hal_rtc.h"
#include "smtc_hal_mcu.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*!
 * Calculates ceiling( X / N )
 */
#define DIVC( X, N ) ( ( ( X ) + ( N ) -1 ) / ( N ) )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

// clang-format off

/*!
 * Days, Hours, Minutes and seconds
 */
#define DAYS_IN_LEAP_YEAR              ( ( uint32_t ) 366U )
#define DAYS_IN_YEAR                   ( ( uint32_t ) 365U )
#define SECONDS_IN_1DAY                ( ( uint32_t ) 86400U )
#define SECONDS_IN_1HOUR               ( ( uint32_t ) 3600U )
#define SECONDS_IN_1MINUTE             ( ( uint32_t ) 60U )
#define MINUTES_IN_1HOUR               ( ( uint32_t ) 60U )
#define HOURS_IN_1DAY                  ( ( uint32_t ) 24U )

/*!
 * Correction factors
 */
#define DAYS_IN_MONTH_CORRECTION_NORM  ( ( uint32_t ) 0x99AAA0 )
#define DAYS_IN_MONTH_CORRECTION_LEAP  ( ( uint32_t ) 0x445550 )

static const char * const g_pcMonth[] =
{
    "January",
    "February",
    "March",
    "April",
    "May",
    "June",
    "July",
    "August",
    "September",
    "October",
    "November",
    "December",
    "Invalid month"
};

// clang-format on

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */
static float hal_rtc_tick_2_100us_multiplier;
static uint32_t hal_rtc_tick_reference;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * Get the elapsed time in seconds and milliseconds since the PostgreSQL epoch
 * January 1, 2000.
 *
 * \param [OUT] milliseconds Number of milliseconds elapsed since RTC
 *                           initialization
 * \retval seconds           Number of seconds elapsed since RTC initialization
 */
static uint32_t rtc_get_calendar_time( uint32_t* milliseconds );

static float rtc_get_clock_tick_in_100us();

static uint32_t rtc_str_to_int(const char *str);

static uint32_t rtc_mth_to_int(const char *str);
/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_rtc_init( void )
{
    /*
    am_hal_rtc_time_t hal_time;
    am_hal_rtc_time_t hal_alarm;

    hal_time.ui32Hour = rtc_str_to_int(&__TIME__[0]);
    hal_time.ui32Minute = rtc_str_to_int(&__TIME__[3]);
    hal_time.ui32Second = rtc_str_to_int(&__TIME__[6]);
    hal_time.ui32Hundredths = 0;
    hal_time.ui32Weekday = am_util_time_computeDayofWeek(
                            2000 + rtc_str_to_int(&__DATE__[9]),
                            rtc_mth_to_int(&__DATE__[0]) + 1,
                            rtc_str_to_int(&__DATE__[4]) );
    hal_time.ui32DayOfMonth = rtc_str_to_int(&__DATE__[4]);
    hal_time.ui32Month = rtc_mth_to_int(&__DATE__[0]);
    hal_time.ui32Year = rtc_str_to_int(&__DATE__[9]);
    hal_time.ui32Century = 1;

    hal_alarm.ui32Hour = 0;
    hal_alarm.ui32Minute = 0;
    hal_alarm.ui32Second = 0;
    hal_alarm.ui32Hundredths = 0;

    hal_rtc_tick_2_100us_multiplier = rtc_get_clock_tick_in_100us();
    hal_rtc_tick_reference = am_hal_stimer_counter_get();

    am_hal_rtc_osc_select(AM_HAL_RTC_OSC_XT);
    am_hal_rtc_osc_enable();

    // set tick reference relative to the start of the day
    hal_rtc_tick_reference = ULONG_MAX - (( ( uint32_t ) hal_time.ui32Second + ( ( uint32_t ) hal_time.ui32Minute * SECONDS_IN_1MINUTE ) +
                 ( ( uint32_t ) hal_time.ui32Hour * SECONDS_IN_1HOUR ) ) * hal_rtc_tick_2_100us_multiplier * 10000);

    am_hal_rtc_time_set(&hal_time);
    am_hal_rtc_alarm_set(&hal_alarm, AM_HAL_RTC_ALM_RPT_DAY);
    am_hal_rtc_int_clear(AM_HAL_RTC_INT_ALM);
    am_hal_rtc_int_enable(AM_HAL_RTC_INT_ALM);
    NVIC_EnableIRQ(RTC_IRQn);
    */

    hal_rtc_tick_2_100us_multiplier = rtc_get_clock_tick_in_100us();
    hal_rtc_tick_reference = am_hal_stimer_counter_get();
}

uint32_t hal_rtc_get_time_s( void )
{
    uint32_t milliseconds = 0;
    return rtc_get_calendar_time( &milliseconds );
}

uint32_t hal_rtc_get_time_100us( void )
{
    uint32_t seconds             = 0;
    uint32_t milliseconds_div_10 = 0;

    seconds = rtc_get_calendar_time( &milliseconds_div_10 );

    return seconds * 10000 + milliseconds_div_10;
}

uint32_t hal_rtc_get_time_ms( void )
{
    uint32_t seconds             = 0;
    uint32_t milliseconds_div_10 = 0;

    seconds = rtc_get_calendar_time( &milliseconds_div_10 );

    return seconds * 1000 + ( milliseconds_div_10 / 10 );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static uint32_t rtc_get_calendar_time( uint32_t* milliseconds_div_10 )
{
    /*
    am_hal_rtc_time_t hal_time;
    uint32_t correction;
    uint32_t elapsed_seconds_since_epoch;
    uint32_t elapsed_seconds_since_day;
    uint32_t elapsed_ticks_since_day;
    uint32_t current_ticks;
    uint32_t remaining_ticks;

    am_hal_rtc_time_get(&hal_time);

    elapsed_seconds_since_day = ( ( uint32_t ) hal_time.ui32Second + ( ( uint32_t ) hal_time.ui32Minute * SECONDS_IN_1MINUTE ) +
                 ( ( uint32_t ) hal_time.ui32Hour * SECONDS_IN_1HOUR ) );

    // Calculate amount of elapsed days since 01/01/2000
    elapsed_seconds_since_epoch = DIVC( ( DAYS_IN_YEAR * 3 + DAYS_IN_LEAP_YEAR ) * hal_time.ui32Year, 4 );

    correction = ( ( hal_time.ui32Year % 4 ) == 0 ) ? DAYS_IN_MONTH_CORRECTION_LEAP : DAYS_IN_MONTH_CORRECTION_NORM;

    elapsed_seconds_since_epoch +=
        ( DIVC( ( hal_time.ui32Month ) * ( 30 + 31 ), 2 ) - ( ( ( correction >> ( ( hal_time.ui32Month ) * 2 ) ) & 0x03 ) ) );

    elapsed_seconds_since_epoch += ( hal_time.ui32DayOfMonth - 1 );

    // Convert from days to seconds
    elapsed_seconds_since_epoch *= SECONDS_IN_1DAY;

    elapsed_seconds_since_epoch += elapsed_seconds_since_day;

    // current_ticks = am_hal_stimer_counter_get();
    // if (current_ticks >= hal_rtc_tick_reference)
    // {
    //     elapsed_ticks_since_day = current_ticks - hal_rtc_tick_reference;
    // }
    // else
    // {
    //     elapsed_ticks_since_day = current_ticks + (ULONG_MAX - hal_rtc_tick_reference);
    // }

    // remaining_ticks  = elapsed_ticks_since_day;
    // remaining_ticks -= (elapsed_seconds_since_day * 10000 * hal_rtc_tick_2_100us_multiplier);
    // remaining_ticks -= (hal_time.ui32Hundredths * 100 * hal_rtc_tick_2_100us_multiplier);

    // *milliseconds_div_10 = hal_time.ui32Hundredths * 100 + remaining_ticks * hal_rtc_tick_2_100us_multiplier;

    *milliseconds_div_10 = hal_time.ui32Hundredths * 100;

    return elapsed_seconds_since_epoch;
*/

    uint32_t current_ticks = am_hal_stimer_counter_get();
    uint32_t elapsed_ticks = current_ticks - hal_rtc_tick_reference;
    uint32_t elapsed_seconds = elapsed_ticks / (hal_rtc_tick_2_100us_multiplier * 10000.0f);
    float remaining_ticks = elapsed_ticks - elapsed_seconds * (hal_rtc_tick_2_100us_multiplier * 10000.0f);

    float remaining_100us = remaining_ticks / hal_rtc_tick_2_100us_multiplier;
    *milliseconds_div_10 = (uint32_t)(remaining_100us);

    return elapsed_seconds;
}

static float rtc_get_clock_tick_in_100us()
{
    uint32_t ui32CurrConfig = CTIMER->STCFG;
    uint32_t ui32Clksel = _FLD2VAL(CTIMER_STCFG_CLKSEL, ui32CurrConfig);
    float ui32Ticks = 0;

    switch(ui32Clksel)
    {
    case CTIMER_STCFG_CLKSEL_HFRC_DIV16:
        ui32Ticks = 300.0f;
        break;

    case CTIMER_STCFG_CLKSEL_HFRC_DIV256:
        ui32Ticks = 18.75f;
        break;

    case CTIMER_STCFG_CLKSEL_XTAL_DIV1:
        ui32Ticks = 3.2768f;
        break;

    case CTIMER_STCFG_CLKSEL_XTAL_DIV2:
        ui32Ticks = 1.6384f;
        break;

    case CTIMER_STCFG_CLKSEL_XTAL_DIV32:
        ui32Ticks = 0.1024f;
        break;

    case CTIMER_STCFG_CLKSEL_LFRC_DIV1:
        ui32Ticks = 0.1024f;
        break;

    default:
        break;
    }

    return ui32Ticks;
}

static uint32_t rtc_str_to_int(const char *str)
{
    uint32_t ui32RetVal = 0;
    ui32RetVal += str[1] - '0';
    ui32RetVal += str[0] == ' ' ? 0 : (str[0] - '0') * 10;
    return ui32RetVal;
}

static uint32_t rtc_mth_to_int(const char *str)
{
    uint32_t idx;
    for (idx = 0; idx < 12; idx++)
    {
        if ( am_util_string_strnicmp(g_pcMonth[idx], str, 3) == 0 )
        {
            return idx;
        }
    }

    return 12;
}

void am_rtc_isr()
{
    hal_rtc_tick_reference = am_hal_stimer_counter_get();
}

/* --- EOF ------------------------------------------------------------------ */
