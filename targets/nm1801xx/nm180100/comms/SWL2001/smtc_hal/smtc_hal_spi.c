/*!
 * \file      smtc_hal_spi.c
 *
 * \brief     SPI Hardware Abstraction Layer implementation
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

#include <am_mcu_apollo.h>

#include "smtc_hal_spi.h"

#include "modem_pinout.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */
typedef struct hal_iom_pindef_s
{
    uint32_t             pin;
    am_hal_gpio_pincfg_t pincfg;
} hal_iom_pindef_t;

typedef struct hal_iom_def_s
{
    hal_iom_pindef_t mosi;
    hal_iom_pindef_t miso;
    hal_iom_pindef_t sclk;
} hal_iom_def_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */
static void* hal_iom_handler[AM_REG_IOM_NUM_MODULES];
static am_hal_iom_config_t hal_iom_config[AM_REG_IOM_NUM_MODULES];
static hal_iom_def_t hal_iom_definition[AM_REG_IOM_NUM_MODULES];

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_spi_init( const uint32_t id )
{
    if (id >= AM_REG_IOM_NUM_MODULES)
    {
        return;
    }

    hal_iom_config[id].eInterfaceMode = AM_HAL_IOM_SPI_MODE;
    hal_iom_config[id].ui32ClockFreq  = AM_HAL_IOM_4MHZ;
    hal_iom_config[id].eSpiMode = AM_HAL_IOM_SPI_MODE_0;

    am_hal_iom_initialize(id, &hal_iom_handler[id]);
    am_hal_iom_power_ctrl(hal_iom_handler[id], AM_HAL_SYSCTRL_WAKE, false);
    am_hal_iom_configure(hal_iom_handler[id], &hal_iom_config[id]);
    am_hal_iom_enable(hal_iom_handler[id]);

    hal_iom_definition[id].mosi.pincfg.eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA;
    hal_iom_definition[id].miso.pincfg.eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA;
    hal_iom_definition[id].sclk.pincfg.eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA;
    hal_iom_definition[id].mosi.pincfg.uIOMnum = id;
    hal_iom_definition[id].miso.pincfg.uIOMnum = id;
    hal_iom_definition[id].sclk.pincfg.uIOMnum = id;
    switch (id)
    {
    case 0:
        hal_iom_definition[id].mosi.pincfg.uFuncSel = AM_HAL_PIN_7_M0MOSI;
        hal_iom_definition[id].miso.pincfg.uFuncSel = AM_HAL_PIN_6_M0MISO;
        hal_iom_definition[id].sclk.pincfg.uFuncSel = AM_HAL_PIN_5_M0SCK;
        hal_iom_definition[id].mosi.pin = 7;
        hal_iom_definition[id].miso.pin = 6;
        hal_iom_definition[id].sclk.pin = 5;
        break;

    case 1:
        hal_iom_definition[id].mosi.pincfg.uFuncSel = AM_HAL_PIN_10_M1MOSI;
        hal_iom_definition[id].miso.pincfg.uFuncSel = AM_HAL_PIN_9_M1MISO;
        hal_iom_definition[id].sclk.pincfg.uFuncSel = AM_HAL_PIN_8_M1SCK;
        hal_iom_definition[id].mosi.pin = 10;
        hal_iom_definition[id].miso.pin = 9;
        hal_iom_definition[id].sclk.pin = 8;
        break;

    case 2:
        hal_iom_definition[id].mosi.pincfg.uFuncSel = AM_HAL_PIN_28_M2MOSI;
        hal_iom_definition[id].miso.pincfg.uFuncSel = AM_HAL_PIN_25_M2MISO;
        hal_iom_definition[id].sclk.pincfg.uFuncSel = AM_HAL_PIN_27_M2SCK;
        hal_iom_definition[id].mosi.pin = 28;
        hal_iom_definition[id].miso.pin = 25;
        hal_iom_definition[id].sclk.pin = 27;
        break;

    case 3:
        hal_iom_definition[id].mosi.pincfg.uFuncSel = AM_HAL_PIN_38_M3MOSI;
        hal_iom_definition[id].miso.pincfg.uFuncSel = AM_HAL_PIN_43_M3MISO;
        hal_iom_definition[id].sclk.pincfg.uFuncSel = AM_HAL_PIN_42_M3SCK;
        hal_iom_definition[id].mosi.pin = 38;
        hal_iom_definition[id].miso.pin = 43;
        hal_iom_definition[id].sclk.pin = 42;
        break;

    case 4:
        hal_iom_definition[id].mosi.pincfg.uFuncSel = AM_HAL_PIN_44_M4MOSI;
        hal_iom_definition[id].miso.pincfg.uFuncSel = AM_HAL_PIN_40_M4MISO;
        hal_iom_definition[id].sclk.pincfg.uFuncSel = AM_HAL_PIN_39_M4SCK;
        hal_iom_definition[id].mosi.pin = 44;
        hal_iom_definition[id].miso.pin = 40;
        hal_iom_definition[id].sclk.pin = 39;
        break;

    case 5:
        hal_iom_definition[id].mosi.pincfg.uFuncSel = AM_HAL_PIN_47_M5MOSI;
        hal_iom_definition[id].miso.pincfg.uFuncSel = AM_HAL_PIN_49_M5MISO;
        hal_iom_definition[id].sclk.pincfg.uFuncSel = AM_HAL_PIN_48_M5SCK;
        hal_iom_definition[id].mosi.pin = 47;
        hal_iom_definition[id].miso.pin = 49;
        hal_iom_definition[id].sclk.pin = 48;
        break;

    default:
        break;
    }

    am_hal_gpio_pinconfig(hal_iom_definition[id].mosi.pin, hal_iom_definition[id].mosi.pincfg);
    am_hal_gpio_pinconfig(hal_iom_definition[id].miso.pin, hal_iom_definition[id].miso.pincfg);
    am_hal_gpio_pinconfig(hal_iom_definition[id].sclk.pin, hal_iom_definition[id].sclk.pincfg);
}

void hal_spi_de_init( const uint32_t id )
{
    if (id >= AM_REG_IOM_NUM_MODULES)
    {
        return;
    }
    am_hal_gpio_pinconfig(hal_iom_definition[id].mosi.pin, g_AM_HAL_GPIO_DISABLE);
    am_hal_gpio_pinconfig(hal_iom_definition[id].miso.pin, g_AM_HAL_GPIO_DISABLE);
    am_hal_gpio_pinconfig(hal_iom_definition[id].sclk.pin, g_AM_HAL_GPIO_DISABLE);

    am_hal_iom_disable(hal_iom_handler[id]);
    am_hal_iom_power_ctrl(hal_iom_handler[id], AM_HAL_SYSCTRL_DEEPSLEEP, false);
    am_hal_iom_uninitialize(hal_iom_handler[id]);
    hal_iom_handler[id] = NULL;
}

uint8_t hal_spi_in_out( const uint32_t id, const uint8_t out_data )
{
    if (id >= AM_REG_IOM_NUM_MODULES)
    {
        return 0;
    }

    am_hal_iom_transfer_t transfer;
    uint32_t tx = out_data;
    uint32_t rx = 0;

    transfer.ui32InstrLen = 0;
    transfer.ui32Instr    = 0;
    transfer.eDirection   = AM_HAL_IOM_FULLDUPLEX;
    transfer.ui32NumBytes = 1;
    transfer.pui32TxBuffer = &tx;
    transfer.pui32RxBuffer = &rx;
    transfer.bContinue      = false;
    transfer.ui8RepeatCount = 0;
    transfer.ui32PauseCondition = 0;
    transfer.ui32StatusSetClr   = 0;
    transfer.uPeerInfo.ui32SpiChipSelect = AM_HAL_IOM_MAX_CS_SPI;

    am_hal_iom_spi_blocking_fullduplex(hal_iom_handler[id], &transfer);

    uint8_t in_data = (uint8_t)(rx & 0xFF);

    return in_data;
}

/* --- EOF ------------------------------------------------------------------ */
