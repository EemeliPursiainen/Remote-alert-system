/*
 * This file has been ported to C from the Arduino LoRa library.
 * The creator of the original library is Sandeep Mistry
 *
 * MIT License
 *
 * Copyright (c) 2016 Sandeep Mistry
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef INC_LORA_H_
#define INC_LORA_H_

#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define _ss GPIO_PIN_4
#define _reset GPIO_PIN_3
#define _dio0 GPIO_PIN_1
#define _sf 7
#define _freq 868E6
#define _bw 31.25E3



// registers
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_OCP                  0x0b
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_RSSI_VALUE           0x1b
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_FREQ_ERROR_MSB       0x28
#define REG_FREQ_ERROR_MID       0x29
#define REG_FREQ_ERROR_LSB       0x2a
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_INVERTIQ             0x33
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_INVERTIQ2            0x3b
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PA_DAC               0x4d

// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06
#define MODE_CAD                 0x07

// PA config
#define PA_BOOST                 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40
#define IRQ_CAD_DONE_MASK          0x04
#define IRQ_CAD_DETECTED_MASK      0x01

#define RF_MID_BAND_THRESHOLD    525E6
#define RSSI_OFFSET_HF_PORT      157
#define RSSI_OFFSET_LF_PORT      164

#define MAX_PKT_LENGTH           255

bool loraBegin();
void loraWriteByte(uint8_t byte);
void loraWriteBuf(const uint8_t *buffer, size_t size);
uint8_t loraRead();
uint8_t loraReadRegister(uint8_t address);
uint8_t loraSingleTransfer(uint8_t address, uint8_t value);
void loraSleep(void);
uint8_t loraWriteRegister(uint8_t address, uint8_t value);
void loraSetFrequency(long frequency);
void loraSetTxPower(int level);
void loraIdle(void);
void loraSetOCP(uint8_t mA);
void loraSetSpreadingFactor(int sf);
void loraSetSignalBandwidth(long sbw);
void loraSetLdoFlag();
void loraSetCodingRate4(int denominator);
void loraSetPreambleLength(long length);
void loraSetSyncWord(int sw);
void loraEnableCrc();
void loraDisableCrc();
void loraEnableInvertIQ();
void loraDisableInvertIQ();
int loraBeginPacket();
int loraBeginPacketHeader(int implicitHeader);
void loraEndPacket(bool async);
void loraHandleDio0Rise();
bool loraIsTransmitting();
void loraExplicitHeaderMode();
void loraImplicitHeaderMode();
int loraParsePacket();
int loraAvailable();



#endif /* INC_LORA_H_ */
