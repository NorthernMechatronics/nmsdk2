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
#ifndef SX1262_H_
#define SX1262_H_

#include <am_mcu_apollo.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RSSI_OFF    64
#define SNR_SCALEUP 4

#define CMD_SETSLEEP              0x84
#define CMD_SETSTANDBY            0x80
#define CMD_SETFS                 0xC1
#define CMD_SETTX                 0x83
#define CMD_SETRX                 0x82
#define CMD_STOPTIMERONPREAMBLE   0x9F
#define CMD_SETRXDUTYCYCLE        0x94
#define CMD_SETCAD                0xC5
#define CMD_SETTXCONTINUOUSWAVE   0xD1
#define CMD_SETTXINFINITEPREAMBLE 0xD2
#define CMD_SETREGULATORMODE      0x96
#define CMD_CALIBRATE             0x89
#define CMD_CALIBRATEIMAGE        0x98
#define CMD_SETPACONFIG           0x95
#define CMD_SETRXTXFALLBACKMODE   0x93

// Commands to Access the Radio Registers and FIFO Buffer
#define CMD_WRITEREGISTER 0x0D
#define CMD_READREGISTER  0x1D
#define CMD_WRITEBUFFER   0x0E
#define CMD_READBUFFER    0x1E

// Commands Controlling the Radio IRQs and DIOs
#define CMD_SETDIOIRQPARAMS       0x08
#define CMD_GETIRQSTATUS          0x12
#define CMD_CLEARIRQSTATUS        0x02
#define CMD_SETDIO2ASRFSWITCHCTRL 0x9D
#define CMD_SETDIO3ASTCXOCTRL     0x97

// Commands Controlling the RF and Packets Settings
#define CMD_SETRFFREQUENCY        0x86
#define CMD_SETPACKETTYPE         0x8A
#define CMD_GETPACKETTYPE         0x11
#define CMD_SETTXPARAMS           0x8E
#define CMD_SETMODULATIONPARAMS   0x8B
#define CMD_SETPACKETPARAMS       0x8C
#define CMD_SETCADPARAMS          0x88
#define CMD_SETBUFFERBASEADDRESS  0x8F
#define CMD_SETLORASYMBNUMTIMEOUT 0xA0

// Commands Returning the Radio Status
#define CMD_GETSTATUS         0xC0
#define CMD_GETRSSIINST       0x15
#define CMD_GETRXBUFFERSTATUS 0x13
#define CMD_GETPACKETSTATUS   0x14
#define CMD_GETDEVICEERRORS   0x17
#define CMD_CLEARDEVICEERRORS 0x07
#define CMD_GETSTATS          0x10
#define CMD_RESETSTATS        0x00

// ----------------------------------------
// List of Registers

#define REG_WHITENINGMSB     0x06B8
#define REG_WHITENINGLSB     0x06B9
#define REG_CRCINITVALMSB    0x06BC
#define REG_CRCINITVALLSB    0x06BD
#define REG_CRCPOLYVALMSB    0x06BE
#define REG_CRCPOLYVALLSB    0x06BF
#define REG_SYNCWORD0        0x06C0
#define REG_SYNCWORD1        0x06C1
#define REG_SYNCWORD2        0x06C2
#define REG_SYNCWORD3        0x06C3
#define REG_SYNCWORD4        0x06C4
#define REG_SYNCWORD5        0x06C5
#define REG_SYNCWORD6        0x06C6
#define REG_SYNCWORD7        0x06C7
#define REG_NODEADDRESS      0x06CD
#define REG_BROADCASTADDR    0x06CE
#define REG_IQ_POLARITY      0x0736
#define REG_LORASYNCWORDMSB  0x0740
#define REG_LORASYNCWORDLSB  0x0741
#define REG_RANDOMNUMBERGEN0 0x0819
#define REG_RANDOMNUMBERGEN1 0x081A
#define REG_RANDOMNUMBERGEN2 0x081B
#define REG_RANDOMNUMBERGEN3 0x081C
#define REG_TX_MODULATION    0x0889
#define REG_RXGAIN           0x08AC
#define REG_OCPCONFIG        0x08E7
#define REG_XTATRIM          0x0911
#define REG_XTBTRIM          0x0912

// sleep modes
#define SLEEP_COLD 0x00 // (no rtc timeout)
#define SLEEP_WARM 0x04 // (no rtc timeout)

// standby modes
#define STDBY_RC   0x00
#define STDBY_XOSC 0x01

// regulator modes
#define REGMODE_LDO  0x00
#define REGMODE_DCDC 0x01

// packet types
#define PACKET_TYPE_FSK  0x00
#define PACKET_TYPE_LORA 0x01

// crc types
#define CRC_OFF        0x01
#define CRC_1_BYTE     0x00
#define CRC_2_BYTE     0x02
#define CRC_1_BYTE_INV 0x04
#define CRC_2_BYTE_INV 0x06

// tx ramp times
#define SET_RAMP_10U    0x00
#define SET_RAMP_20U    0x01
#define SET_RAMP_40U    0x02
#define SET_RAMP_80U    0x03
#define SET_RAMP_200U   0x04
#define SET_RAMP_800U   0x05
#define SET_RAMP_1700U  0x06
#define SET_RAMP_3400U  0x07

// device selection
#define DEVICE_SEL_SX1262   0x00
#define DEVICE_SEL_SX1261   0x01

#define PA_LUT_1    0x01

typedef enum sx1262_operating_mode_e
{
    SX1262_OP_MODE_SLEEP,
    SX1262_OP_MODE_STANDBY,
    SX1262_OP_MODE_FS,
    SX1262_OP_MODE_TXCW,
    SX1262_OP_MODE_TX,
    SX1262_OP_MODE_RX,
    SX1262_OP_MODE_CAD,
} sx1262_operating_mode_e;

uint8_t sx1262_io_init(void);
void sx1262_io_deinit(void);
void sx1262_reset(void);
void sx1262_dio_interrupt_register(am_hal_gpio_handler_t pfnHandler);
void sx1262_dio_interrupt_enable(void);
void sx1262_dio_interrupt_disable(void);
uint32_t sx1262_dio_get_pin1_state();
void sx1262_set_mode(sx1262_operating_mode_e eMode, uint32_t ui32Parameter);
sx1262_operating_mode_e sx1262_get_mode();
void sx1262_config_regulator(void);
void sx1262_set_dio2_rf_switch_ctrl(uint8_t ui8Enable);
void sx1262_init_packet_type(void);
void sx1262_calibrate_image(uint32_t ui32Frequency);
void sx1262_set_frequency(uint32_t ui32Frequency);
void sx1262_config_modulation_parameters(uint8_t ui8SF,
                                         uint8_t ui8BW,
                                         uint8_t ui8CR,
                                         uint8_t ui8LDR);
void sx1262_config_packet_parameters(uint16_t ui16PreambleLength,
                                     uint8_t ui8PacketType,
                                     uint8_t ui8PayloadLength,
                                     uint8_t ui8CRC,
                                     uint8_t ui8IQ);
void sx1262_interrupt_clear(uint16_t ui16Mask);
void sx1262_stop_timer_on_preamble(uint8_t ui8Enable);
void sx1262_set_symbol_timeout(uint8_t ui8Symbols);
uint16_t sx1262_interrupt_status_get(void);
void sx1262_get_packet_status(int8_t *pi8RSSI, int8_t *pi8SNR, int8_t *pi8RSCP);
uint16_t sx1262_get_error_status(void);
uint16_t sx1262_get_device_status(void);
void sx1262_interrupt_enable(uint16_t ui16Mask,
                             uint16_t ui16Dio1Mask,
                             uint16_t ui16Dio2Mask,
                             uint16_t ui16Dio3Mask);
void sx1262_set_power(int8_t i8Power);
void sx1262_set_syncword(uint16_t ui16SyncWord);

void sx1262_write_fifo(uint8_t *buf, uint8_t len);
uint8_t sx1262_read_fifo(uint8_t *buf);

void sx1262_write_command(uint8_t cmd, const uint8_t *data, uint8_t len);

void sx1262_write_registers(uint16_t addr, const uint8_t *data, uint8_t len);
void sx1262_read_registers(uint16_t addr, uint8_t *data, uint8_t len);

int8_t sx1262_get_rssi(void);

#ifdef __cplusplus
}
#endif

#endif