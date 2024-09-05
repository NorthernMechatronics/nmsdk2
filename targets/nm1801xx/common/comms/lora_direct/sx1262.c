/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2023, Northern Mechatronics, Inc.
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
#include <stdint.h>

#include <am_mcu_apollo.h>
#include <am_util.h>

#include "sx1262.h"

#define SX1262_IOM_MODULE 3

#define LORA_RADIO_NRESET 44
#define LORA_RADIO_BUSY   39
#define LORA_RADIO_DIO1   40
#define LORA_RADIO_DIO3   47

#define LORA_RADIO_MISO     43
#define LORA_RADIO_MOSI     38
#define LORA_RADIO_CLK      42
#define LORA_RADIO_NSS      36
#define LORA_RADIO_NSS_CHNL 1

static const am_hal_gpio_pincfg_t gs_LORA_RADIO_CLK = {.uFuncSel = AM_HAL_PIN_42_M3SCK,
                                                       .eDriveStrength =
                                                           AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA,
                                                       .uIOMnum = SX1262_IOM_MODULE};

static const am_hal_gpio_pincfg_t gs_LORA_RADIO_MISO = {.uFuncSel = AM_HAL_PIN_43_M3MISO,
                                                        .uIOMnum = SX1262_IOM_MODULE};

static const am_hal_gpio_pincfg_t gs_LORA_RADIO_MOSI = {.uFuncSel = AM_HAL_PIN_38_M3MOSI,
                                                        .eDriveStrength =
                                                            AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA,
                                                        .uIOMnum = SX1262_IOM_MODULE};

static const am_hal_gpio_pincfg_t gs_LORA_RADIO_NSS = {.uFuncSel = AM_HAL_PIN_36_NCE36,
                                                       .eDriveStrength =
                                                           AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA,
                                                       .eGPOutcfg = AM_HAL_GPIO_PIN_OUTCFG_PUSHPULL,
                                                       .eGPInput = AM_HAL_GPIO_PIN_INPUT_NONE,
                                                       .eIntDir = AM_HAL_GPIO_PIN_INTDIR_LO2HI,
                                                       .uIOMnum = SX1262_IOM_MODULE,
#ifdef AM_PART_APOLLO3P                                                       
                                                       .bIomMSPIn = 1,
#endif
                                                       .uNCE = LORA_RADIO_NSS_CHNL,
                                                       .eCEpol = AM_HAL_GPIO_PIN_CEPOL_ACTIVELOW};

static void *psLoRaRadio;
static sx1262_operating_mode_e sx1262_operating_mode = SX1262_OP_MODE_SLEEP;

static void sx1262_isr_handler(void);
static am_hal_gpio_handler_t driver_layer_isr_handler;

static void sx1262_block_on_busy(void)
{
    uint32_t state = 1;

    if (sx1262_operating_mode != SX1262_OP_MODE_SLEEP)
    {
        while (state)
        {
            am_hal_gpio_state_read(LORA_RADIO_BUSY, AM_HAL_GPIO_INPUT_READ, &state);
        }
    }
}

void sx1262_write_command(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    am_hal_iom_transfer_t transaction;

    transaction.ui32InstrLen = 1;
    transaction.ui32Instr = cmd;
    transaction.eDirection = AM_HAL_IOM_TX;
    transaction.ui32NumBytes = len;
    transaction.pui32TxBuffer = (uint32_t *)data;
    transaction.bContinue = false;
    transaction.ui8RepeatCount = 0;
    transaction.ui32PauseCondition = 0;
    transaction.ui32StatusSetClr = 0;
    transaction.uPeerInfo.ui32SpiChipSelect = LORA_RADIO_NSS_CHNL;

    sx1262_block_on_busy();
    am_hal_iom_blocking_transfer(psLoRaRadio, &transaction);
}

void sx1262_write_registers(uint16_t addr, const uint8_t *data, uint8_t len)
{
    uint32_t instruction = (CMD_WRITEREGISTER << 16) | addr;

    am_hal_iom_transfer_t transaction;

    transaction.ui32InstrLen = 3;
    transaction.ui32Instr = instruction;
    transaction.eDirection = AM_HAL_IOM_TX;
    transaction.ui32NumBytes = len;
    transaction.pui32TxBuffer = (uint32_t *)data;
    transaction.bContinue = false;
    transaction.ui8RepeatCount = 0;
    transaction.ui32PauseCondition = 0;
    transaction.ui32StatusSetClr = 0;
    transaction.uPeerInfo.ui32SpiChipSelect = LORA_RADIO_NSS_CHNL;

    sx1262_block_on_busy();
    am_hal_iom_blocking_transfer(psLoRaRadio, &transaction);
}

void sx1262_write_buffer(uint8_t off, const uint8_t *data, uint8_t len)
{
    am_hal_iom_transfer_t transaction;

    transaction.ui32InstrLen = 2;
    transaction.ui32Instr = (CMD_WRITEBUFFER << 8) | off;
    transaction.eDirection = AM_HAL_IOM_TX;
    transaction.ui32NumBytes = len;
    transaction.pui32TxBuffer = (uint32_t *)data;
    transaction.bContinue = false;
    transaction.ui8RepeatCount = 0;
    transaction.ui32PauseCondition = 0;
    transaction.ui32StatusSetClr = 0;
    transaction.uPeerInfo.ui32SpiChipSelect = LORA_RADIO_NSS_CHNL;

    sx1262_block_on_busy();
    am_hal_iom_blocking_transfer(psLoRaRadio, &transaction);
}

void sx1262_write_fifo(uint8_t *buf, uint8_t len)
{
    static const uint8_t ui8FifoOffsets[] = {0, 0};
    sx1262_write_command(CMD_SETBUFFERBASEADDRESS, ui8FifoOffsets, 2);

    sx1262_write_buffer(0, buf, len);
}

uint8_t sx1262_read_command(uint8_t cmd, uint8_t *data, uint8_t len)
{
    am_hal_iom_transfer_t transaction;

    transaction.ui32InstrLen = 1;
    transaction.ui32Instr = cmd;
    transaction.eDirection = AM_HAL_IOM_RX;
    transaction.ui32NumBytes = len;
    transaction.pui32RxBuffer = (uint32_t *)data;
    transaction.bContinue = false;
    transaction.ui8RepeatCount = 0;
    transaction.ui32PauseCondition = 0;
    transaction.ui32StatusSetClr = 0;
    transaction.uPeerInfo.ui32SpiChipSelect = LORA_RADIO_NSS_CHNL;

    sx1262_block_on_busy();
    am_hal_iom_blocking_transfer(psLoRaRadio, &transaction);
}

void sx1262_read_registers(uint16_t addr, uint8_t *data, uint8_t len)
{
    uint32_t instruction = (CMD_READREGISTER << 16) | addr;

    am_hal_iom_transfer_t transaction;

    transaction.ui32InstrLen = 3;
    transaction.ui32Instr = instruction;
    transaction.eDirection = AM_HAL_IOM_RX;
    transaction.ui32NumBytes = 0;
    transaction.pui32TxBuffer = (uint32_t *)NULL;
    transaction.bContinue = true;
    transaction.ui8RepeatCount = 0;
    transaction.ui32PauseCondition = 0;
    transaction.ui32StatusSetClr = 0;
    transaction.uPeerInfo.ui32SpiChipSelect = LORA_RADIO_NSS_CHNL;

    sx1262_block_on_busy();
    am_hal_iom_blocking_transfer(psLoRaRadio, &transaction);

    transaction.ui32InstrLen = 1;
    transaction.ui32Instr = 0;
    transaction.eDirection = AM_HAL_IOM_RX;
    transaction.ui32NumBytes = len;
    transaction.pui32RxBuffer = (uint32_t *)data;
    transaction.bContinue = false;
    transaction.ui8RepeatCount = 0;
    transaction.ui32PauseCondition = 0;
    transaction.ui32StatusSetClr = 0;
    transaction.uPeerInfo.ui32SpiChipSelect = LORA_RADIO_NSS_CHNL;

    am_hal_iom_blocking_transfer(psLoRaRadio, &transaction);
}

void sx1262_read_buffer(uint8_t off, uint8_t *data, uint8_t len)
{
    uint32_t instruction = (CMD_READBUFFER << 16) | (off << 8);

    am_hal_iom_transfer_t transaction;

    transaction.ui32InstrLen = 3;
    transaction.ui32Instr = instruction;
    transaction.eDirection = AM_HAL_IOM_RX;
    transaction.ui32NumBytes = len;
    transaction.pui32RxBuffer = (uint32_t *)data;
    transaction.bContinue = false;
    transaction.ui8RepeatCount = 0;
    transaction.ui32PauseCondition = 0;
    transaction.ui32StatusSetClr = 0;
    transaction.uPeerInfo.ui32SpiChipSelect = LORA_RADIO_NSS_CHNL;

    sx1262_block_on_busy();
    am_hal_iom_blocking_transfer(psLoRaRadio, &transaction);
}

uint8_t sx1262_read_fifo(uint8_t *buf)
{
    // get buffer status
    uint8_t status[4];
    sx1262_read_command(CMD_GETRXBUFFERSTATUS, status, 4);

    // read buffer
    uint8_t len = status[1];
    uint8_t off = status[2];
    sx1262_read_buffer(off, buf, len);

    // return length
    return len;
}

uint8_t sx1262_io_init(void)
{
    driver_layer_isr_handler = NULL;

    am_hal_gpio_pinconfig(LORA_RADIO_NRESET, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(LORA_RADIO_NRESET, AM_HAL_GPIO_OUTPUT_TRISTATE_DISABLE);
    am_hal_gpio_state_write(LORA_RADIO_NRESET, AM_HAL_GPIO_OUTPUT_SET);

    am_hal_gpio_pinconfig(LORA_RADIO_BUSY, g_AM_HAL_GPIO_INPUT);
    am_hal_gpio_pinconfig(LORA_RADIO_DIO1, g_AM_HAL_GPIO_INPUT);
    am_hal_gpio_pinconfig(LORA_RADIO_DIO3, g_AM_HAL_GPIO_INPUT);

    am_hal_gpio_pinconfig(LORA_RADIO_CLK, gs_LORA_RADIO_CLK);
    am_hal_gpio_pinconfig(LORA_RADIO_MISO, gs_LORA_RADIO_MISO);
    am_hal_gpio_pinconfig(LORA_RADIO_MOSI, gs_LORA_RADIO_MOSI);
    am_hal_gpio_pinconfig(LORA_RADIO_NSS, gs_LORA_RADIO_NSS);

    am_hal_gpio_interrupt_register(LORA_RADIO_DIO1, sx1262_isr_handler);
    am_hal_gpio_interrupt_register(LORA_RADIO_DIO3, sx1262_isr_handler);

    am_hal_iom_config_t lora_radio_config;
    lora_radio_config.eInterfaceMode = AM_HAL_IOM_SPI_MODE;
    lora_radio_config.ui32ClockFreq = AM_HAL_IOM_4MHZ;
    lora_radio_config.eSpiMode = AM_HAL_IOM_SPI_MODE_0;

    if (am_hal_iom_initialize(SX1262_IOM_MODULE, &psLoRaRadio) != AM_HAL_STATUS_SUCCESS)
    {
        return 1;
    }

    if (am_hal_iom_power_ctrl(psLoRaRadio, AM_HAL_SYSCTRL_WAKE, false) != AM_HAL_STATUS_SUCCESS)
    {
        return 1;
    }

    if (am_hal_iom_configure(psLoRaRadio, &lora_radio_config) != AM_HAL_STATUS_SUCCESS)
    {
        return 1;
    }

    if (am_hal_iom_enable(psLoRaRadio) != AM_HAL_STATUS_SUCCESS)
    {
        return 1;
    }

    // am_hal_gpio_interrupt_clear(AM_HAL_GPIO_BIT(LORA_RADIO_DIO1));
    // am_hal_gpio_interrupt_clear(AM_HAL_GPIO_BIT(LORA_RADIO_DIO3));
    // am_hal_gpio_interrupt_enable(AM_HAL_GPIO_BIT(LORA_RADIO_DIO1));
    // am_hal_gpio_interrupt_enable(AM_HAL_GPIO_BIT(LORA_RADIO_DIO3));

    AM_HAL_GPIO_MASKCREATE(radio_dio_interrupts);
    AM_HAL_GPIO_MASKBITSMULT(pradio_dio_interrupts, LORA_RADIO_DIO1);
    AM_HAL_GPIO_MASKBITSMULT(pradio_dio_interrupts, LORA_RADIO_DIO3);
    am_hal_gpio_interrupt_clear(pradio_dio_interrupts);
    am_hal_gpio_interrupt_enable(pradio_dio_interrupts);

    return 0;
}

void sx1262_io_deinit(void)
{
    am_hal_iom_disable(psLoRaRadio);
    am_hal_iom_power_ctrl(psLoRaRadio, AM_HAL_SYSCTRL_DEEPSLEEP, false);
    am_hal_iom_uninitialize(psLoRaRadio);

    am_hal_gpio_pinconfig(LORA_RADIO_CLK, g_AM_HAL_GPIO_DISABLE);
    am_hal_gpio_pinconfig(LORA_RADIO_MISO, g_AM_HAL_GPIO_DISABLE);
    am_hal_gpio_pinconfig(LORA_RADIO_MOSI, g_AM_HAL_GPIO_DISABLE);
    am_hal_gpio_pinconfig(LORA_RADIO_NSS, g_AM_HAL_GPIO_DISABLE);

    am_hal_gpio_pinconfig(LORA_RADIO_NRESET, g_AM_HAL_GPIO_DISABLE);
    am_hal_gpio_pinconfig(LORA_RADIO_BUSY, g_AM_HAL_GPIO_DISABLE);
    am_hal_gpio_pinconfig(LORA_RADIO_DIO1, g_AM_HAL_GPIO_DISABLE);
    am_hal_gpio_pinconfig(LORA_RADIO_DIO3, g_AM_HAL_GPIO_DISABLE);
}

void sx1262_reset()
{
    am_hal_gpio_state_write(LORA_RADIO_NRESET, AM_HAL_GPIO_OUTPUT_CLEAR);
    am_util_delay_us(100);
    am_hal_gpio_state_write(LORA_RADIO_NRESET, AM_HAL_GPIO_OUTPUT_SET);
    am_util_delay_us(100);
}

void sx1262_dio_interrupt_register(am_hal_gpio_handler_t pfnHandler)
{
    driver_layer_isr_handler = pfnHandler;
}

void sx1262_dio_interrupt_enable(void)
{
    // am_hal_gpio_interrupt_clear(AM_HAL_GPIO_BIT(LORA_RADIO_DIO1));
    // am_hal_gpio_interrupt_clear(AM_HAL_GPIO_BIT(LORA_RADIO_DIO3));
    // am_hal_gpio_interrupt_enable(AM_HAL_GPIO_BIT(LORA_RADIO_DIO1));
    // am_hal_gpio_interrupt_enable(AM_HAL_GPIO_BIT(LORA_RADIO_DIO3));

    AM_HAL_GPIO_MASKCREATE(radio_dio_interrupts);
    AM_HAL_GPIO_MASKBITSMULT(pradio_dio_interrupts, LORA_RADIO_DIO1);
    AM_HAL_GPIO_MASKBITSMULT(pradio_dio_interrupts, LORA_RADIO_DIO3);
    am_hal_gpio_interrupt_clear(pradio_dio_interrupts);
    am_hal_gpio_interrupt_enable(pradio_dio_interrupts);

    NVIC_EnableIRQ(GPIO_IRQn);
}

void sx1262_dio_interrupt_disable(void)
{
    // am_hal_gpio_interrupt_clear(AM_HAL_GPIO_BIT(LORA_RADIO_DIO1));
    // am_hal_gpio_interrupt_clear(AM_HAL_GPIO_BIT(LORA_RADIO_DIO3));
    // am_hal_gpio_interrupt_disable(AM_HAL_GPIO_BIT(LORA_RADIO_DIO1));
    // am_hal_gpio_interrupt_disable(AM_HAL_GPIO_BIT(LORA_RADIO_DIO3));

    AM_HAL_GPIO_MASKCREATE(radio_dio_interrupts);
    AM_HAL_GPIO_MASKBITSMULT(pradio_dio_interrupts, LORA_RADIO_DIO1);
    AM_HAL_GPIO_MASKBITSMULT(pradio_dio_interrupts, LORA_RADIO_DIO3);
    am_hal_gpio_interrupt_clear(pradio_dio_interrupts);
    am_hal_gpio_interrupt_disable(pradio_dio_interrupts);
}

uint32_t sx1262_dio_get_pin1_state()
{
    uint32_t value;
    am_hal_gpio_state_read(LORA_RADIO_DIO1, AM_HAL_GPIO_INPUT_READ, &value);

    return value;
}

sx1262_operating_mode_e sx1262_get_mode()
{
    return sx1262_operating_mode;
}

void sx1262_set_mode(sx1262_operating_mode_e eMode, uint32_t ui32Parameter)
{
    uint8_t ui8Mode;

    switch (eMode)
    {
    case SX1262_OP_MODE_SLEEP:
        ui8Mode = CMD_SETSLEEP;
        sx1262_write_command(ui8Mode, (uint8_t *)&ui32Parameter, 1);
        break;

    case SX1262_OP_MODE_STANDBY:
        ui8Mode = CMD_SETSTANDBY;
        sx1262_write_command(ui8Mode, (uint8_t *)&ui32Parameter, 1);
        break;
    
    case SX1262_OP_MODE_FS:
        ui8Mode = CMD_SETFS;
        sx1262_write_command(ui8Mode, NULL, 0);
        break;

    case SX1262_OP_MODE_TXCW:
        ui8Mode = CMD_SETTXCONTINUOUSWAVE;
        sx1262_write_command(ui8Mode, NULL, 0);
        break;

    case SX1262_OP_MODE_CAD:
        ui8Mode = CMD_SETCAD;
        sx1262_write_command(ui8Mode, NULL, 0);
        break;

    case SX1262_OP_MODE_TX:
    {
        ui8Mode = CMD_SETTX;
        uint8_t timeout[3] = {
            (ui32Parameter >> 16) & 0xFF, (ui32Parameter >> 8) & 0xFF, ui32Parameter & 0xFF};
        sx1262_write_command(ui8Mode, timeout, 3);
    }
        break;
    case SX1262_OP_MODE_RX:
    {
        ui8Mode = CMD_SETRX;
        uint8_t timeout[3] = {
            (ui32Parameter >> 16) & 0xFF, (ui32Parameter >> 8) & 0xFF, ui32Parameter & 0xFF};
        sx1262_write_command(ui8Mode, timeout, 3);
    }
        break;
    }

    sx1262_operating_mode = eMode;
}

void sx1262_config_regulator(void)
{
    uint8_t mode = REGMODE_DCDC;
    sx1262_write_command(CMD_SETREGULATORMODE, &mode, 1);
}

// use DIO2 to drive antenna rf switch
void sx1262_set_dio2_rf_switch_ctrl(uint8_t ui8Enable)
{
    sx1262_write_command(CMD_SETDIO2ASRFSWITCHCTRL, &ui8Enable, 1);
}

// set radio to PACKET_TYPE_LORA or PACKET_TYPE_FSK mode
void sx1262_init_packet_type(void)
{
    uint8_t type = PACKET_TYPE_LORA;
    sx1262_write_command(CMD_SETPACKETTYPE, &type, 1);
}

void sx1262_calibrate_image(uint32_t ui32Frequency)
{
    static const struct
    {
        uint32_t min;
        uint32_t max;
        uint8_t f[2];
    } bands[] = {
        {430000000, 440000000, (uint8_t[]){0x6B, 0x6F}},
        {470000000, 510000000, (uint8_t[]){0x75, 0x81}},
        {779000000, 787000000, (uint8_t[]){0xC1, 0xC5}},
        {863000000, 870000000, (uint8_t[]){0xD7, 0xDB}},
        {902000000, 928000000, (uint8_t[]){0xE1, 0xE9}},
    };

    for (int i = 0; i < sizeof(bands) / sizeof(bands[0]); i++)
    {
        if ((ui32Frequency >= bands[i].min) && (ui32Frequency <= bands[i].max))
        {
            sx1262_write_command(CMD_CALIBRATEIMAGE, bands[i].f, 2);
        }
    }
}

void sx1262_set_frequency(uint32_t ui32Frequency)
{
    sx1262_calibrate_image(ui32Frequency);

    uint32_t v = (uint32_t)(((uint64_t)ui32Frequency << 25) / 32000000);
    uint32_t f = __bswap32(v);

    sx1262_write_command(CMD_SETRFFREQUENCY, (uint8_t *)&f, 4);
}

void sx1262_config_modulation_parameters(uint8_t ui8SF,
                                         uint8_t ui8BW,
                                         uint8_t ui8CR,
                                         uint8_t ui8LDR)
{
    uint8_t param[4];
    param[0] = ui8SF;
    param[1] = ui8BW;
    param[2] = ui8CR;
    param[3] = ui8LDR;

    sx1262_write_command(CMD_SETMODULATIONPARAMS, param, 4);
}

void sx1262_config_packet_parameters(uint16_t ui16PreambleLength,
                                     uint8_t ui8PacketType,
                                     uint8_t ui8PayloadLength,
                                     uint8_t ui8CRC,
                                     uint8_t ui8IQ)
{
    uint8_t param[6];
    param[0] = (uint8_t)(ui16PreambleLength >> 8);
    param[1] = (uint8_t)(ui16PreambleLength & 0xFF);
    param[2] = ui8PacketType;
    param[3] = ui8PayloadLength;
    param[4] = ui8CRC;
    param[5] = ui8IQ;

    sx1262_write_command(CMD_SETPACKETPARAMS, param, 6);
}

void sx1262_interrupt_clear(uint16_t ui16Mask)
{
    uint8_t buf[2] = {ui16Mask >> 8, ui16Mask & 0xFF};
    sx1262_write_command(CMD_CLEARIRQSTATUS, buf, 2);
}

void sx1262_stop_timer_on_preamble(uint8_t ui8Enable)
{
    sx1262_write_command(CMD_STOPTIMERONPREAMBLE, &ui8Enable, 1);
}

void sx1262_set_symbol_timeout(uint8_t ui8Symbols)
{
    sx1262_write_command(CMD_SETLORASYMBNUMTIMEOUT, &ui8Symbols, 1);
}

uint16_t sx1262_interrupt_status_get(void)
{
    uint8_t buf[3];
    sx1262_read_command(CMD_GETIRQSTATUS, buf, sizeof(buf));
    return ((uint16_t)(buf[1] << 8) | buf[2]);
}

void sx1262_get_packet_status(int8_t *pi8RSSI, int8_t *pi8SNR, int8_t *pi8RSCP)
{
    uint8_t buf[4];
    sx1262_read_command(CMD_GETPACKETSTATUS, buf, sizeof(buf));
    *pi8RSSI = -buf[1] / 2;
    *pi8SNR = buf[2] / 4;
    *pi8RSCP = -buf[3] / 2;
}

uint16_t sx1262_get_error_status(void)
{
    uint8_t buf[3];
    sx1262_read_command(CMD_GETDEVICEERRORS, buf, 3);
    return ((uint16_t)(buf[1] << 8) | buf[2]);
}

uint16_t sx1262_get_device_status(void)
{
    uint8_t buf;
    sx1262_read_command(CMD_GETSTATUS, &buf, 1);
    return buf;
}

void sx1262_interrupt_enable(uint16_t ui16Mask,
                             uint16_t ui16Dio1Mask,
                             uint16_t ui16Dio2Mask,
                             uint16_t ui16Dio3Mask)
{
    uint8_t param[] = {
        (uint8_t)(ui16Mask >> 8), (uint8_t)(ui16Mask & 0xFF),
        (uint8_t)(ui16Dio1Mask >> 8), (uint8_t)(ui16Dio1Mask & 0xFF),
        (uint8_t)(ui16Dio2Mask >> 8), (uint8_t)(ui16Dio2Mask & 0xFF),
        (uint8_t)(ui16Dio3Mask >> 8), (uint8_t)(ui16Dio3Mask & 0xFF),
    };

    sx1262_write_command(CMD_SETDIOIRQPARAMS, param, 8);
}

void sx1262_set_power(int8_t i8Power)
{
    // high power PA: -9 ... +22 dBm
    // low power PA: -17 ... +14 dBm
    if (i8Power > 22)
    {
        i8Power = 22;
    }
    else if (i8Power < -17)
    {
        i8Power = -17;
    }

    // set PA config (and reset OCP to 140mA)
    uint8_t pa_config[] = {0x00, 0x00, DEVICE_SEL_SX1262, PA_LUT_1};
    if (i8Power <= 14)
    {
        pa_config[0] = 0x02; pa_config[1] = 0x02;
    }
    else if (i8Power <= 17)
    {
        pa_config[0] = 0x02; pa_config[1] = 0x03;
    }
    else if (i8Power <= 20)
    {
        pa_config[0] = 0x03; pa_config[1] = 0x05;
    }
    else if (i8Power <= 22)
    {
        pa_config[0] = 0x04; pa_config[1] = 0x07;
    }
    sx1262_write_command(CMD_SETPACONFIG, pa_config, 4);

    uint8_t ui8TransmitParameters[2];
    ui8TransmitParameters[0] = (uint8_t)i8Power;
    ui8TransmitParameters[1] = SET_RAMP_200U; // ramp time 200us
    sx1262_write_command(CMD_SETTXPARAMS, ui8TransmitParameters, 2);
}

void sx1262_set_syncword(uint16_t ui16SyncWord)
{
    uint8_t buf[2] = {ui16SyncWord >> 8, ui16SyncWord & 0xFF};
    sx1262_write_registers(REG_LORASYNCWORDMSB, buf, 2);
}

void sx1262_isr_handler(void)
{
    if (driver_layer_isr_handler)
    {
        driver_layer_isr_handler();
    }
}

int8_t sx1262_get_rssi(void)
{
    uint8_t buf[2];
    sx1262_read_command(CMD_GETRSSIINST, buf, sizeof(buf));
    return -buf[1] / 2;
}