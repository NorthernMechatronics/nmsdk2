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
#include <am_mcu_apollo.h>
#include <am_util.h>
#include <stdint.h>
#include <string.h>

#include "lora_radio.h"
#include "sx1262.h"

#define LORA_RADIO_MAX_IRQ (10)

static lora_radio_callback_t lora_radio_irq_callback_table[LORA_RADIO_MAX_IRQ];

static volatile bool lora_radio_irq_fired = false;

static lora_radio_callback_t lora_radio_isr_handler;

// Modulation parameters
static lora_radio_sf_e lora_radio_param_sf;
static lora_radio_bw_e lora_radio_param_bw;
static lora_radio_cr_e lora_radio_param_cr;
static lora_radio_ldr_e lora_radio_param_ldr;

// Packet parameters
static uint16_t lora_radio_param_preamble_length;
static lora_radio_crc_e lora_radio_param_crc;
static lora_radio_iq_e lora_radio_param_iq;

// RF parameters
static lora_radio_sw_e lora_radio_param_syncword;

static lora_radio_packet_t lora_radio_rx_packet;
static uint8_t lora_radio_rx_buffer[LORA_RADIO_MAX_PHYSICAL_PACKET_LEN];

static void lora_radio_isr(void);

static int32_t fast_log2(int32_t v)
{
    int r = 0xFFFF - v >> 31 & 0x10;
    v >>= r;
    int shift = 0xFF - v >> 31 & 0x8;
    v >>= shift;
    r |= shift;
    shift = 0xF - v >> 31 & 0x4;
    v >>= shift;
    r |= shift;
    shift = 0x3 - v >> 31 & 0x2;
    v >>= shift;
    r |= shift;
    r |= (v >> 1);
    return r;
}

void lora_radio_config_reset()
{
    memset(&lora_radio_irq_callback_table, 0, LORA_RADIO_MAX_IRQ * sizeof(lora_radio_callback_t));
    memset(&lora_radio_rx_packet, 0, sizeof(lora_radio_packet_t));

    lora_radio_param_sf = LORA_RADIO_SF5;
    lora_radio_param_bw = LORA_RADIO_BW7;
    lora_radio_param_cr = LORA_RADIO_CR45;
    lora_radio_param_ldr = LORA_RADIO_LDR_ON;
    lora_radio_param_preamble_length = 8;
    lora_radio_param_crc = LORA_RADIO_CRC_OFF;
    lora_radio_param_iq = LORA_RADIO_IQ_STANDARD;
    lora_radio_param_syncword = LORA_RADIO_SW_PRIVATE;
}

void lora_radio_init()
{
    sx1262_io_init();
    sx1262_dio_interrupt_register(lora_radio_isr);
    sx1262_reset();
    sx1262_dio_interrupt_enable();
}

void lora_radio_deinit()
{
    sx1262_dio_interrupt_disable();
    sx1262_io_deinit();
}

void lora_radio_reset()
{
    sx1262_reset();
    sx1262_config_regulator();
    sx1262_set_mode(SX1262_OP_MODE_STANDBY, STDBY_RC);
    sx1262_set_dio2_rf_switch_ctrl(1);
    sx1262_init_packet_type();
}

void lora_radio_config_get(lora_radio_parameter_e eParam, uint32_t *value)
{
    if (eParam == LORA_RADIO_PARAM_SF)
    {
        *value = lora_radio_param_sf;
    }
    else if (eParam == LORA_RADIO_PARAM_BW)
    {
        *value = lora_radio_param_bw;
    }
    else if (eParam == LORA_RADIO_PARAM_CR)
    {
        *value = lora_radio_param_cr;
    }
    else if (eParam == LORA_RADIO_PARAM_PREAMBLE_LEN)
    {
        *value = lora_radio_param_preamble_length;
    }
    else if (eParam == LORA_RADIO_PARAM_CRC)
    {
        *value = lora_radio_param_crc;
    }
    else if (eParam == LORA_RADIO_PARAM_IQ)
    {
        *value = lora_radio_param_iq;
    }
    else if (eParam == LORA_RADIO_PARAM_SYNCWORD)
    {
        *value = lora_radio_param_syncword;
    }
    else if (eParam == LORA_RADIO_PARAM_LDR)
    {
        *value = lora_radio_param_ldr;
    }
}

static void lora_radio_config_set_ldr()
{
    switch (lora_radio_param_bw)
    {
    case LORA_RADIO_BW7:
        lora_radio_param_ldr = LORA_RADIO_LDR_ON;
        break;
    case LORA_RADIO_BW10:
        lora_radio_param_ldr = LORA_RADIO_LDR_ON;
        break;
    case LORA_RADIO_BW15:
        lora_radio_param_ldr = LORA_RADIO_LDR_ON;
        break;
    case LORA_RADIO_BW20:
        lora_radio_param_ldr = LORA_RADIO_LDR_ON;
        break;
    case LORA_RADIO_BW31:
        lora_radio_param_ldr = LORA_RADIO_LDR_ON;
        break;
    case LORA_RADIO_BW41:
        lora_radio_param_ldr = lora_radio_param_sf >= LORA_RADIO_SF9 ? LORA_RADIO_LDR_ON : LORA_RADIO_LDR_OFF;
        break;
    case LORA_RADIO_BW62:
        lora_radio_param_ldr = lora_radio_param_sf >= LORA_RADIO_SF10 ? LORA_RADIO_LDR_ON : LORA_RADIO_LDR_OFF;
        break;
    case LORA_RADIO_BW125:
        lora_radio_param_ldr = lora_radio_param_sf >= LORA_RADIO_SF11 ? LORA_RADIO_LDR_ON : LORA_RADIO_LDR_OFF;
        break;
    case LORA_RADIO_BW250:
        lora_radio_param_ldr = lora_radio_param_sf >= LORA_RADIO_SF12 ? LORA_RADIO_LDR_ON : LORA_RADIO_LDR_OFF;
        break;
    case LORA_RADIO_BW500:
        lora_radio_param_ldr = LORA_RADIO_LDR_OFF;
        break;
    }
}

void lora_radio_config_set(lora_radio_parameter_e eParam, uint32_t value)
{
    if (eParam == LORA_RADIO_PARAM_SF)
    {
        lora_radio_param_sf = value & 0xFF;
        lora_radio_config_set_ldr();
    }
    else if (eParam == LORA_RADIO_PARAM_BW)
    {
        lora_radio_param_bw = value & 0xFF;
        lora_radio_config_set_ldr();
    }
    else if (eParam == LORA_RADIO_PARAM_CR)
    {
        lora_radio_param_cr = value & 0xFF;
    }
    else if (eParam == LORA_RADIO_PARAM_PREAMBLE_LEN)
    {
        lora_radio_param_preamble_length = value & 0xFFFF;
    }
    else if (eParam == LORA_RADIO_PARAM_CRC)
    {
        lora_radio_param_crc = value & 0xFF;
    }
    else if (eParam == LORA_RADIO_PARAM_IQ)
    {
        lora_radio_param_iq = value & 0xFF;
    }
    else if (eParam == LORA_RADIO_PARAM_SYNCWORD)
    {
        lora_radio_param_syncword = value & 0xFFFF;
    }
}

void lora_radio_state_set(lora_radio_state_e eState)
{
    switch (eState)
    {
    case LORA_RADIO_STATE_SLEEP:
        sx1262_set_mode(SX1262_OP_MODE_SLEEP, SLEEP_WARM);
        break;
    case LORA_RADIO_STATE_DEEPSLEEP:
        sx1262_set_mode(SX1262_OP_MODE_SLEEP, SLEEP_COLD);
        break;
    case LORA_RADIO_STATE_STANDBY:
        sx1262_set_mode(SX1262_OP_MODE_STANDBY, STDBY_RC);
        break;
    default:
        break;
    }
}

void lora_radio_transmit(lora_radio_ht_e eHeaderType, lora_radio_packet_t *psPacket)
{
    sx1262_set_mode(SX1262_OP_MODE_STANDBY, STDBY_RC);
    sx1262_config_regulator();
    sx1262_set_dio2_rf_switch_ctrl(1);
    sx1262_init_packet_type();

    sx1262_config_packet_parameters(lora_radio_param_preamble_length,
                                    eHeaderType,
                                    psPacket->ui8PayloadLength,
                                    lora_radio_param_crc,
                                    lora_radio_param_iq);

    sx1262_config_modulation_parameters(
        lora_radio_param_sf, lora_radio_param_bw, lora_radio_param_cr, lora_radio_param_ldr);

    // WORKAROUND - Modulation Quality with 500 kHz LoRa Bandwidth, see DS_SX1261-2_V1.2 datasheet chapter 15.1
    uint8_t value;
    sx1262_read_registers(REG_TX_MODULATION, &value, 1);
    if (lora_radio_param_bw == LORA_RADIO_BW500)
    {
        value &= ~0x04;
    }
    else
    {
        value |= 0x04;
    }
    sx1262_write_registers(REG_TX_MODULATION, &value, 1);
    // WORKAROUND END

    sx1262_set_frequency(psPacket->ui32Frequency);
    sx1262_set_power(psPacket->i8TransmitPower);
    sx1262_set_syncword(lora_radio_param_syncword);

    sx1262_write_fifo(psPacket->pui8Payload, psPacket->ui8PayloadLength);

    sx1262_interrupt_clear(LORA_RADIO_IRQ_ALL);
    sx1262_interrupt_enable(LORA_RADIO_IRQ_TXDONE | LORA_RADIO_IRQ_TIMEOUT | LORA_RADIO_IRQ_RXDONE | LORA_RADIO_IRQ_PREAMBLEDETECTED | LORA_RADIO_IRQ_HEADERVALID | LORA_RADIO_IRQ_HEADERERR | LORA_RADIO_IRQ_CRCERR,
                            LORA_RADIO_IRQ_TXDONE | LORA_RADIO_IRQ_TIMEOUT | LORA_RADIO_IRQ_RXDONE | LORA_RADIO_IRQ_PREAMBLEDETECTED | LORA_RADIO_IRQ_HEADERVALID | LORA_RADIO_IRQ_HEADERERR | LORA_RADIO_IRQ_CRCERR,
                            LORA_RADIO_IRQ_NONE,
                            LORA_RADIO_IRQ_NONE);

    sx1262_set_mode(SX1262_OP_MODE_TX, (psPacket->ui32Timeout) & 0xFFFFFF);
}

void lora_set_frequency(uint32_t frequency)
{
    sx1262_set_frequency(frequency);
}

void lora_radio_receive(lora_radio_ht_e eHeaderType,
                        uint8_t ui8MaxPayloadLength,
                        uint32_t ui32Frequency,
                        uint32_t ui32Timeout)
{
    sx1262_set_mode(SX1262_OP_MODE_STANDBY, STDBY_RC);
    sx1262_config_regulator();
    sx1262_set_dio2_rf_switch_ctrl(1);
    sx1262_init_packet_type();

    sx1262_config_packet_parameters(lora_radio_param_preamble_length,
                                    eHeaderType,
                                    ui8MaxPayloadLength,
                                    lora_radio_param_crc,
                                    lora_radio_param_iq);

    sx1262_config_modulation_parameters(
        lora_radio_param_sf, lora_radio_param_bw, lora_radio_param_cr, lora_radio_param_ldr);

    // WORKAROUND - Optimizing the Inverted IQ Operation, see DS_SX1261-2_V1.2 datasheet chapter 15.4
    uint8_t value;
    sx1262_read_registers(REG_IQ_POLARITY, &value, 1);
    if (lora_radio_param_iq == LORA_RADIO_IQ_INVERTED)
    {
        value &= ~0x04;
    }
    else
    {
        value |= 0x04;
    }
    sx1262_write_registers(REG_IQ_POLARITY, &value, 1);
    // WORKAROUND END

    sx1262_set_frequency(ui32Frequency);
    sx1262_set_syncword(lora_radio_param_syncword);

    sx1262_interrupt_clear(LORA_RADIO_IRQ_ALL);
    sx1262_interrupt_enable(LORA_RADIO_IRQ_RXDONE | LORA_RADIO_IRQ_TIMEOUT | LORA_RADIO_IRQ_TXDONE | LORA_RADIO_IRQ_PREAMBLEDETECTED | LORA_RADIO_IRQ_HEADERVALID | LORA_RADIO_IRQ_HEADERERR | LORA_RADIO_IRQ_CRCERR,
                            LORA_RADIO_IRQ_RXDONE | LORA_RADIO_IRQ_TIMEOUT | LORA_RADIO_IRQ_TXDONE | LORA_RADIO_IRQ_PREAMBLEDETECTED | LORA_RADIO_IRQ_HEADERVALID | LORA_RADIO_IRQ_HEADERERR | LORA_RADIO_IRQ_CRCERR,
                            LORA_RADIO_IRQ_NONE,
                            LORA_RADIO_IRQ_NONE);

    sx1262_set_mode(SX1262_OP_MODE_RX, (ui32Timeout)&0xFFFFFF);
}

void lora_radio_transmit_carrier(uint32_t ui32Frequency, int8_t i8TransmitPower)
{
    sx1262_set_mode(SX1262_OP_MODE_STANDBY, STDBY_RC);
    sx1262_config_regulator();
    sx1262_set_dio2_rf_switch_ctrl(1);
    sx1262_init_packet_type();

    sx1262_set_frequency(ui32Frequency);
    sx1262_set_power(i8TransmitPower);

    sx1262_interrupt_clear(LORA_RADIO_IRQ_ALL);

    sx1262_set_mode(SX1262_OP_MODE_TXCW, 0);
}

void lora_radio_cad_start(uint32_t ui32Frequency, uint32_t ui32Timeout)
{
    uint8_t cadDetMin, cadDetPeak, cadSymbolNum, cadExitMode;
    uint32_t cadTimeout = ui32Timeout;

    cadDetMin = 10;
    cadExitMode = 1;
    switch (lora_radio_param_sf)
    {
    case LORA_RADIO_SF7:
        cadDetPeak = 22;
        cadSymbolNum = 1;
        break;
    case LORA_RADIO_SF8:
        cadDetPeak = 22;
        cadSymbolNum = 1;
        break;
    case LORA_RADIO_SF9:
        cadDetPeak = 23;
        cadSymbolNum = 2;
        break;
    case LORA_RADIO_SF10:
        cadDetPeak = 24;
        cadSymbolNum = 2;
        break;
    case LORA_RADIO_SF11:
        cadDetPeak = 25;
        cadSymbolNum = 2;
        break;
    case LORA_RADIO_SF12:
        cadDetPeak = 28;
        cadSymbolNum = 2;
        break;
    default:
        return;
    }

    sx1262_set_mode(SX1262_OP_MODE_STANDBY, STDBY_RC);
    sx1262_set_dio2_rf_switch_ctrl(1);
//    sx1262_config_regulator();
    sx1262_init_packet_type();

    sx1262_config_modulation_parameters(
        lora_radio_param_sf, lora_radio_param_bw, lora_radio_param_cr, lora_radio_param_ldr);

    sx1262_set_frequency(ui32Frequency);
    sx1262_set_syncword(lora_radio_param_syncword);

    uint8_t buf[7];
    buf[0] = cadSymbolNum;
    buf[1] = cadDetPeak;
    buf[2] = cadDetMin;
    buf[3] = cadExitMode;
    buf[4] = ( uint8_t )( ( cadTimeout >> 16 ) & 0xFF );
    buf[5] = ( uint8_t )( ( cadTimeout >> 8 ) & 0xFF );
    buf[6] = ( uint8_t )( cadTimeout & 0xFF );
    sx1262_write_command( CMD_SETCADPARAMS, buf, 7 );

    sx1262_interrupt_enable(LORA_RADIO_IRQ_CADDONE | LORA_RADIO_IRQ_CADDETECTED,
                            LORA_RADIO_IRQ_CADDONE | LORA_RADIO_IRQ_CADDETECTED,
                            LORA_RADIO_IRQ_NONE,
                            LORA_RADIO_IRQ_NONE);
    sx1262_interrupt_clear(LORA_RADIO_IRQ_ALL);

    sx1262_set_mode(SX1262_OP_MODE_CAD, 0);
}

void lora_radio_irq_handler_set(lora_radio_callback_t pfnHandler)
{
    lora_radio_isr_handler = pfnHandler;
}

void lora_radio_irq_register(lora_radio_irq_e eIRQ, lora_radio_callback_t pfnHandler)
{
    int index = fast_log2(eIRQ);
    lora_radio_irq_callback_table[index] = pfnHandler;
}

void lora_radio_irq_unregister(lora_radio_irq_e eIRQ, lora_radio_callback_t pfnHandler)
{
    int index = fast_log2(eIRQ);
    lora_radio_irq_callback_table[index] = NULL;
}

bool lora_radio_irq_is_fired(void)
{
    return lora_radio_irq_fired;
}

void lora_radio_irq_process(void)
{
    bool irq_fired;

    AM_CRITICAL_BEGIN
    irq_fired = lora_radio_irq_fired;
    lora_radio_irq_fired = false;
    AM_CRITICAL_END

    if (irq_fired)
    {
        uint16_t ui16IrqStatus = sx1262_interrupt_status_get();
        sx1262_interrupt_clear(ui16IrqStatus);

        AM_CRITICAL_BEGIN
        if (sx1262_dio_get_pin1_state())
        {
            lora_radio_irq_fired = true;
        }
        AM_CRITICAL_END

        int32_t index;
        if (ui16IrqStatus & LORA_RADIO_IRQ_TXDONE)
        {
            index = fast_log2(LORA_RADIO_IRQ_TXDONE);
            if (lora_radio_irq_callback_table[index])
            {
                lora_radio_irq_callback_table[index](NULL);
            }
        }
        if (ui16IrqStatus & LORA_RADIO_IRQ_RXDONE)
        {
            if (ui16IrqStatus & LORA_RADIO_IRQ_CRCERR)
            {
                index = fast_log2(LORA_RADIO_IRQ_CRCERR);
                if (lora_radio_irq_callback_table[index])
                {
                    lora_radio_irq_callback_table[index](NULL);
                }
            }
            else
            {
                index = fast_log2(LORA_RADIO_IRQ_RXDONE);
                if (lora_radio_irq_callback_table[index])
                {
                    memset(&lora_radio_rx_buffer, 0, LORA_RADIO_MAX_PHYSICAL_PACKET_LEN);
                    lora_radio_rx_packet.ui8PayloadLength = sx1262_read_fifo(lora_radio_rx_buffer);
                    sx1262_get_packet_status(&lora_radio_rx_packet.i8Rssi,
                                                &lora_radio_rx_packet.i8Snr,
                                                &lora_radio_rx_packet.i8Rscp);
                    lora_radio_rx_packet.pui8Payload = lora_radio_rx_buffer;
                    lora_radio_irq_callback_table[index](&lora_radio_rx_packet);
                }
            }
        }
        if (ui16IrqStatus & LORA_RADIO_IRQ_HEADERERR)
        {
            index = fast_log2(LORA_RADIO_IRQ_HEADERERR);
            if (lora_radio_irq_callback_table[index])
            {
                lora_radio_irq_callback_table[index](NULL);
            }
        }
        if (ui16IrqStatus & LORA_RADIO_IRQ_TIMEOUT)
        {
            index = fast_log2(LORA_RADIO_IRQ_TIMEOUT);
            if (lora_radio_irq_callback_table[index])
            {
                lora_radio_irq_callback_table[index](NULL);
            }
        }
        if (ui16IrqStatus & LORA_RADIO_IRQ_CADDONE)
        {
            index = fast_log2(LORA_RADIO_IRQ_CADDONE);
            bool bCadDetected = (ui16IrqStatus & LORA_RADIO_IRQ_CADDETECTED ? 1 : 0);
            if (lora_radio_irq_callback_table[index])
            {
                lora_radio_irq_callback_table[index]((void *)bCadDetected);
            }
        }
        if (ui16IrqStatus & LORA_RADIO_IRQ_PREAMBLEDETECTED)
        {
            index = fast_log2(LORA_RADIO_IRQ_PREAMBLEDETECTED);
            if (lora_radio_irq_callback_table[index])
            {
                lora_radio_irq_callback_table[index](NULL);
            }
        }
        if (ui16IrqStatus & LORA_RADIO_IRQ_HEADERVALID)
        {
            index = fast_log2(LORA_RADIO_IRQ_HEADERVALID);
            if (lora_radio_irq_callback_table[index])
            {
                lora_radio_irq_callback_table[index](NULL);
            }
        }
    }
}

void lora_radio_isr()
{
    lora_radio_irq_fired = true;
    if (lora_radio_isr_handler)
    {
        lora_radio_isr_handler(NULL);
    }
}

int8_t lora_radio_rssi()
{
    return sx1262_get_rssi();
}

lora_radio_state_e lora_radio_state_get()
{
    sx1262_operating_mode_e mode = sx1262_get_mode();

    switch(mode)
    {
    case SX1262_OP_MODE_SLEEP:
        return LORA_RADIO_STATE_SLEEP;

    case SX1262_OP_MODE_STANDBY:
        return LORA_RADIO_STATE_STANDBY;

    case SX1262_OP_MODE_TX:
        return LORA_RADIO_STATE_TX;

    case SX1262_OP_MODE_RX:
        return LORA_RADIO_STATE_RX;

    case SX1262_OP_MODE_CAD:
        return LORA_RADIO_STATE_CAD;

    default:
        break;
    }
}

void lora_radio_get_entropy(uint8_t *buf, size_t buflen)
{
    sx1262_operating_mode_e mode = sx1262_get_mode();

    // The sx1262 random number generator only works with the radio 'on'.
    // If it's not 'on', turn on the receiver temporarily.
    switch (mode)
    {
    case SX1262_OP_MODE_SLEEP:
    case SX1262_OP_MODE_STANDBY:
        lora_radio_receive(
            LORA_RADIO_HT_VARIABLE_LENGTH,
            LORA_RADIO_MAX_PHYSICAL_PACKET_LEN,
            915000000,
            0xFFFFFF);
        break;
    default:
        break;
    }

    uint8_t *end = buf + buflen / 4 * 4;
    for (; buf < end; buf += 4)
    {
        sx1262_read_registers(REG_RANDOMNUMBERGEN0, buf, 4);
    }
    if (buflen % 4)
    {
        sx1262_read_registers(REG_RANDOMNUMBERGEN0, buf, buflen % 4);
    }

    // Restore the radio mode
    switch (mode)
    {
    case SX1262_OP_MODE_SLEEP:
        sx1262_set_mode(SX1262_OP_MODE_SLEEP, SLEEP_WARM);
        break;
    case SX1262_OP_MODE_STANDBY:
        sx1262_set_mode(SX1262_OP_MODE_STANDBY, STDBY_RC);
        break;
    default:
        break;
    }
}
