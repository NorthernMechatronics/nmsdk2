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
#ifndef LORA_RADIO_H_
#define LORA_RADIO_H_

#ifdef __cplusplus
extern "C" {
#endif

#define LORA_RADIO_MAX_PHYSICAL_PACKET_LEN  (255)

typedef enum {
    LORA_RADIO_STATUS_SUCCESS,
    LORA_RADIO_STATUS_FAIL,
    LORA_RADIO_STATUS_INVALID_HANDLE,
    LORA_RADIO_STATUS_IN_USE,
    LORA_RADIO_STATUS_TIMEOUT,
    LORA_RADIO_STATUS_OUT_OF_RANGE,
    LORA_RADIO_STATUS_INVALID_ARG,
    LORA_RADIO_STATUS_INVALID_OPERATION,
    LORA_RADIO_STATUS_HW_ERR,
} lora_radio_status_e;


typedef enum lora_radio_sf_e
{
    LORA_RADIO_SF5  = 0x05,
    LORA_RADIO_SF6  = 0x06,
    LORA_RADIO_SF7  = 0x07,
    LORA_RADIO_SF8  = 0x08,
    LORA_RADIO_SF9  = 0x09,
    LORA_RADIO_SF10 = 0x0A,
    LORA_RADIO_SF11 = 0x0B,
    LORA_RADIO_SF12 = 0x0C,
} lora_radio_sf_e;

typedef enum lora_radio_bw_e
{
    LORA_RADIO_BW7      = 0x00,
    LORA_RADIO_BW10     = 0x08,
    LORA_RADIO_BW15     = 0x01,
    LORA_RADIO_BW20     = 0x09,
    LORA_RADIO_BW31     = 0x02,
    LORA_RADIO_BW41     = 0x0A,
    LORA_RADIO_BW62     = 0x03,
    LORA_RADIO_BW125    = 0x04,
    LORA_RADIO_BW250    = 0x05,
    LORA_RADIO_BW500    = 0x06,
} lora_radio_bw_e;

typedef enum lora_radio_cr_e
{
    LORA_RADIO_CR45 = 0x01,
    LORA_RADIO_CR46 = 0x02,
    LORA_RADIO_CR47 = 0x03,
    LORA_RADIO_CR48 = 0x04,
} lora_radio_cr_e;

typedef enum lora_radio_ldr_e
{
    LORA_RADIO_LDR_OFF  = 0x00,
    LORA_RADIO_LDR_ON   = 0x01,
} lora_radio_ldr_e;

typedef enum lora_radio_ht_e
{
    LORA_RADIO_HT_VARIABLE_LENGTH   = 0x00,
    LORA_RADIO_HT_FIXED_LENGTH      = 0x01,
} lora_radio_ht_e;

typedef enum lora_radio_crc_e
{
    LORA_RADIO_CRC_OFF  = 0x00,
    LORA_RADIO_CRC_ON   = 0x01,
} lora_radio_crc_e;

typedef enum lora_radio_iq_e
{
    LORA_RADIO_IQ_STANDARD  = 0x00,
    LORA_RADIO_IQ_INVERTED  = 0x01,
} lora_radio_iq_e;

typedef enum lora_radio_sw_e
{
    LORA_RADIO_SW_PRIVATE   = 0x1424,
    LORA_RADIO_SW_PUBLIC    = 0x3444,
} lora_radio_sw_e;

typedef struct lora_radio_packet_s
{
    uint32_t    ui32Frequency;
    uint32_t    ui32Timeout;
    int8_t      i8TransmitPower;
    uint8_t    *pui8Payload;
    uint8_t     ui8PayloadLength;
    int8_t      i8Rssi;
    int8_t      i8Snr;
    int8_t      i8Rscp;
} lora_radio_packet_t;

typedef enum lora_radio_state_e
{
    LORA_RADIO_STATE_SLEEP,
    LORA_RADIO_STATE_DEEPSLEEP,
    LORA_RADIO_STATE_STANDBY,
    LORA_RADIO_STATE_TX,
    LORA_RADIO_STATE_RX,
    LORA_RADIO_STATE_CAD
} lora_radio_state_e;

typedef enum lora_radio_irq_e {
    LORA_RADIO_IRQ_NONE   = 0x0000,
    LORA_RADIO_IRQ_TXDONE = 0x0001,
    LORA_RADIO_IRQ_RXDONE = 0x0002,
    LORA_RADIO_IRQ_PREAMBLEDETECTED = 0x0004,
    LORA_RADIO_IRQ_SYNCWORDVALID = 0x0008,
    LORA_RADIO_IRQ_HEADERVALID = 0x0010,
    LORA_RADIO_IRQ_HEADERERR = 0x0020,
    LORA_RADIO_IRQ_CRCERR = 0x0040,
    LORA_RADIO_IRQ_CADDONE = 0x0080,
    LORA_RADIO_IRQ_CADDETECTED = 0x0100,
    LORA_RADIO_IRQ_TIMEOUT = 0x0200,
    LORA_RADIO_IRQ_ALL = 0x03FF
} lora_radio_irq_e;

typedef enum lora_radio_parameter_e {
    LORA_RADIO_PARAM_SF,
    LORA_RADIO_PARAM_BW,
    LORA_RADIO_PARAM_CR,
    LORA_RADIO_PARAM_PREAMBLE_LEN,
    LORA_RADIO_PARAM_CRC,
    LORA_RADIO_PARAM_IQ,
    LORA_RADIO_PARAM_SYNCWORD,
} lora_radio_parameter_e;

typedef void (*lora_radio_callback_t)(void *);


void lora_radio_init();
void lora_radio_deinit();
void lora_radio_reset();

void lora_radio_config_reset();
void lora_radio_config_get(lora_radio_parameter_e eParam, uint32_t *value);
void lora_radio_config_set(lora_radio_parameter_e eParam, uint32_t value);

void lora_radio_state_set(lora_radio_state_e eState);
lora_radio_state_e lora_radio_state_get();

void lora_set_frequency(uint32_t frequency);

void lora_radio_transmit(
    lora_radio_ht_e eHeaderType,
    lora_radio_packet_t *psPacket);

/**
 * @brief 
 * 
 * @param eHeaderType
 * @param ui8MaxPayloadLength 
 * @param ui32Frequency 
 * @param ui32Timeout  Timeout duration = ui32Timeout * 15.625us
 */
void lora_radio_receive (
    lora_radio_ht_e eHeaderType,
    uint8_t  ui8MaxPayloadLength,
    uint32_t ui32Frequency,
    uint32_t ui32Timeout);

void lora_radio_transmit_carrier(uint32_t ui32Frequency, int8_t i8TransmitPower);

void lora_radio_cad_start(uint32_t ui32Frequency, uint32_t ui32Timeout);

void lora_radio_irq_handler_set(lora_radio_callback_t pfnHandler);
void lora_radio_irq_register(lora_radio_irq_e eIRQ, lora_radio_callback_t pfnHandler);
void lora_radio_irq_unregister(lora_radio_irq_e eIRQ, lora_radio_callback_t pfnHandler);
bool lora_radio_irq_is_fired(void);
void lora_radio_irq_process(void);

int8_t lora_radio_rssi();

void lora_radio_get_entropy(uint8_t *buf, size_t buflen);

#ifdef __cplusplus
}
#endif

#endif // LORA_RADIO_H_