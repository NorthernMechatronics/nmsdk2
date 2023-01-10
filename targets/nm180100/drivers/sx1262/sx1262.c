/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2022, Northern Mechatronics, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <machine/endian.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <am_mcu_apollo.h>
#include <am_util.h>

#include "sx1262.h"

#define RADIO_IOM_MODULE 3
#define RADIO_NRESET      44
#define RADIO_BUSY        39
#define RADIO_DIO1        40
#define RADIO_DIO3        47

#define RADIO_MISO     43
#define RADIO_MOSI     38
#define RADIO_CLK      42
#define RADIO_NSS      36
#define RADIO_NSS_CHNL 1

static const am_hal_gpio_pincfg_t s_RADIO_DIO1 = {
    .uFuncSel       = AM_HAL_PIN_40_GPIO,
    .eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA,
    .eGPInput       = AM_HAL_GPIO_PIN_INPUT_ENABLE,
    .eIntDir        = AM_HAL_GPIO_PIN_INTDIR_LO2HI};

static const am_hal_gpio_pincfg_t s_RADIO_CLK = {
    .uFuncSel       = AM_HAL_PIN_42_M3SCK,
    .eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA,
    .uIOMnum        = 3};

static const am_hal_gpio_pincfg_t s_RADIO_MISO = {
    .uFuncSel = AM_HAL_PIN_43_M3MISO, .uIOMnum = 3};

static const am_hal_gpio_pincfg_t s_RADIO_MOSI = {
    .uFuncSel       = AM_HAL_PIN_38_M3MOSI,
    .eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA,
    .uIOMnum        = 3};

static const am_hal_gpio_pincfg_t s_RADIO_NSS = {
    .uFuncSel       = AM_HAL_PIN_36_NCE36,
    .eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA,
    .eGPOutcfg      = AM_HAL_GPIO_PIN_OUTCFG_PUSHPULL,
    .eGPInput       = AM_HAL_GPIO_PIN_INPUT_NONE,
    .eIntDir        = AM_HAL_GPIO_PIN_INTDIR_LO2HI,
    .uIOMnum        = 3,
    .uNCE           = 1,
    .eCEpol         = AM_HAL_GPIO_PIN_CEPOL_ACTIVELOW};

static am_hal_iom_config_t SX126xSpi;
void *SX126xHandle;

void SX126xIoInit(void)
{
    am_hal_gpio_pinconfig(RADIO_NRESET, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_pinconfig(RADIO_BUSY, g_AM_HAL_GPIO_INPUT);
    am_hal_gpio_pinconfig(RADIO_DIO1, s_RADIO_DIO1);

    am_hal_gpio_pinconfig(RADIO_CLK, s_RADIO_CLK);
    am_hal_gpio_pinconfig(RADIO_MISO, s_RADIO_MISO);
    am_hal_gpio_pinconfig(RADIO_MOSI, s_RADIO_MOSI);
    am_hal_gpio_pinconfig(RADIO_NSS, s_RADIO_NSS);

    SX126xSpi.eInterfaceMode = AM_HAL_IOM_SPI_MODE;
    SX126xSpi.ui32ClockFreq  = AM_HAL_IOM_4MHZ;
    SX126xSpi.eSpiMode       = AM_HAL_IOM_SPI_MODE_0;

    am_hal_iom_initialize(RADIO_IOM_MODULE, &SX126xHandle);
    am_hal_iom_power_ctrl(SX126xHandle, AM_HAL_SYSCTRL_WAKE, false);
    am_hal_iom_configure(SX126xHandle, &SX126xSpi);
    am_hal_iom_enable(SX126xHandle);
}

