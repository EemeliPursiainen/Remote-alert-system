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

#include "main.h"
#include "LoRa.h"

uint8_t _packetIndex;
bool _implicitHeaderMode;
void (*_onReceive)(int);
void (*_onCadDone)(bool);
void (*_onTxDone)();

bool loraBegin() {
	LL_GPIO_SetOutputPin(GPIOA, _ss);
	// perform reset
	LL_GPIO_ResetOutputPin(GPIOA, _reset);
	LL_mDelay(10);
	LL_GPIO_SetOutputPin(GPIOA, _reset);
	LL_mDelay(10);
	// check version
	uint8_t version = loraReadRegister(REG_VERSION);
	if (version != 0x12) {
		return 0;
	}
	// put in sleep mode
	loraSleep();

	// set frequency
	//loraSetFrequency(frequency);
	uint64_t frf = ((uint64_t) _freq << 19) / 32000000;
	loraWriteRegister(REG_FRF_MSB, (uint8_t) (frf >> 16));
	loraWriteRegister(REG_FRF_MID, (uint8_t) (frf >> 8));
	loraWriteRegister(REG_FRF_LSB, (uint8_t) (frf >> 0));

	// set base addresses
	loraWriteRegister(REG_FIFO_TX_BASE_ADDR, 0);
	loraWriteRegister(REG_FIFO_RX_BASE_ADDR, 0);
	// set LNA boost
	loraWriteRegister(REG_LNA, loraReadRegister(REG_LNA) | 0x03);
	// set auto AGC
	loraWriteRegister(REG_MODEM_CONFIG_3, 0x04);
	// set output power to 14 dBm
	loraSetTxPower(14);
	// put in standby mode
	loraIdle();

	return 1;
}

uint8_t loraReadRegister(uint8_t address) {
	return loraSingleTransfer(address & 0x7f, 0x00);
}

uint8_t loraSingleTransfer(uint8_t address, uint8_t value) {
	uint8_t response;

//	HAL_SPI_Init(&hspi1);
	LL_GPIO_ResetOutputPin(GPIOA, _ss);

	HAL_SPI_Transmit(&hspi1, (uint8_t*) &address, 1, 100);
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) &value, (uint8_t*) &response, 1,
			100);

	LL_GPIO_SetOutputPin(GPIOA, _ss);
	//HAL_SPI_DeInit(&hspi1);

	return response;
}

void loraSleep() {
	loraWriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

uint8_t loraWriteRegister(uint8_t address, uint8_t value) {
	return loraSingleTransfer(address | 0x80, value);
}

void loraSetFrequency(long frequency) {

	uint64_t frf = ((uint64_t) frequency << 19) / 32000000;

	loraWriteRegister(REG_FRF_MSB, (uint8_t) (frf >> 16));
	loraWriteRegister(REG_FRF_MID, (uint8_t) (frf >> 8));
	loraWriteRegister(REG_FRF_LSB, (uint8_t) (frf >> 0));
}

void loraSetTxPower(int level) {

	// PA BOOST
	if (level > 17) {
		if (level > 20) {
			level = 20;
		}

		// subtract 3 from level, so 18 - 20 maps to 15 - 17
		level -= 3;

		// High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
		loraWriteRegister(REG_PA_DAC, 0x87);
		loraSetOCP(140);
	} else {
		if (level < 2) {
			level = 2;
		}
		//Default value PA_HF/LF or +17dBm
		loraWriteRegister(REG_PA_DAC, 0x84);
		loraSetOCP(100);
	}

	loraWriteRegister(REG_PA_CONFIG, PA_BOOST | (level - 2));
}

void loraIdle(void) {
	loraWriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void loraSetOCP(uint8_t mA) {
	uint8_t ocpTrim = 27;

	if (mA <= 120) {
		ocpTrim = (mA - 45) / 5;
	} else if (mA <= 240) {
		ocpTrim = (mA + 30) / 10;
	}

	loraWriteRegister(REG_OCP, 0x20 | (0x1F & ocpTrim));
}

/*int loraGetSpreadingFactor() {
 return loraReadRegister(REG_MODEM_CONFIG_2) >> 4;
 }*/

void loraSetSpreadingFactor(int sf) {
	if (sf < 6) {
		sf = 6;
	} else if (sf > 12) {
		sf = 12;
	}

	if (sf == 6) {
		loraWriteRegister(REG_DETECTION_OPTIMIZE, 0xc5);
		loraWriteRegister(REG_DETECTION_THRESHOLD, 0x0c);
	} else {
		loraWriteRegister(REG_DETECTION_OPTIMIZE, 0xc3);
		loraWriteRegister(REG_DETECTION_THRESHOLD, 0x0a);
	}

	loraWriteRegister(REG_MODEM_CONFIG_2,
			(loraReadRegister(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
	loraSetLdoFlag();
}

/*long loraGetSignalBandwidth() {
 uint8_t bw = (loraReadRegister(REG_MODEM_CONFIG_1) >> 4);

 switch (bw) {
 case 0:
 return 7.8E3;
 case 1:
 return 10.4E3;
 case 2:
 return 15.6E3;
 case 3:
 return 20.8E3;
 case 4:
 return 31.25E3;
 case 5:
 return 41.7E3;
 case 6:
 return 62.5E3;
 case 7:
 return 125E3;
 case 8:
 return 250E3;
 case 9:
 return 500E3;
 }

 return -1;
 }*/

void loraSetSignalBandwidth(long sbw) {
	int bw;

	if (sbw <= 7.8E3) {
		bw = 0;
	} else if (sbw <= 10.4E3) {
		bw = 1;
	} else if (sbw <= 15.6E3) {
		bw = 2;
	} else if (sbw <= 20.8E3) {
		bw = 3;
	} else if (sbw <= 31.25E3) {
		bw = 4;
	} else if (sbw <= 41.7E3) {
		bw = 5;
	} else if (sbw <= 62.5E3) {
		bw = 6;
	} else if (sbw <= 125E3) {
		bw = 7;
	} else if (sbw <= 250E3) {
		bw = 8;
	} else /*if (sbw <= 250E3)*/{
		bw = 9;
	}

	loraWriteRegister(REG_MODEM_CONFIG_1,
			(loraReadRegister(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
	loraSetLdoFlag();
}

void loraSetLdoFlag() {
	// Section 4.1.1.5
//	long symbolDuration = 1000/ (loraGetSignalBandwidth() / (1L << loraGetSpreadingFactor()));
	long symbolDuration = 1000 / (_bw / (1L << _sf));

	// Section 4.1.1.6
	bool ldoOn = symbolDuration > 16;

	uint8_t config3 = loraReadRegister(REG_MODEM_CONFIG_3);

	config3 = 1 << 3 & ldoOn << 3;
	//bitWrite(config3, 3, ldoOn);
	loraWriteRegister(REG_MODEM_CONFIG_3, config3);
}

void loraSetCodingRate4(int denominator) {
	if (denominator < 5) {
		denominator = 5;
	} else if (denominator > 8) {
		denominator = 8;
	}

	int cr = denominator - 4;

	loraWriteRegister(REG_MODEM_CONFIG_1,
			(loraReadRegister(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void loraSetPreambleLength(long length) {
	loraWriteRegister(REG_PREAMBLE_MSB, (uint8_t) (length >> 8));
	loraWriteRegister(REG_PREAMBLE_LSB, (uint8_t) (length >> 0));
}

void loraSetSyncWord(int sw) {
	loraWriteRegister(REG_SYNC_WORD, sw);
}

void loraEnableCrc() {
	loraWriteRegister(REG_MODEM_CONFIG_2,
			loraReadRegister(REG_MODEM_CONFIG_2) | 0x04);
}

void loraDisableCrc() {
	loraWriteRegister(REG_MODEM_CONFIG_2,
			loraReadRegister(REG_MODEM_CONFIG_2) & 0xfb);
}

void loraEnableInvertIQ() {
	loraWriteRegister(REG_INVERTIQ, 0x66);
	loraWriteRegister(REG_INVERTIQ2, 0x19);
}

void loraDisableInvertIQ() {
	loraWriteRegister(REG_INVERTIQ, 0x27);
	loraWriteRegister(REG_INVERTIQ2, 0x1d);
}

int loraBeginPacket() {
	return loraBeginPacketHeader(false);
}

int loraBeginPacketHeader(int implicitHeader) {
	if (loraIsTransmitting()) {
		return 0;
	}

	// put in standby mode
	loraIdle();

	if (implicitHeader) {
		loraImplicitHeaderMode();
	} else {
		loraExplicitHeaderMode();
	}

	// reset FIFO address and payload length
	loraWriteRegister(REG_FIFO_ADDR_PTR, 0);
	loraWriteRegister(REG_PAYLOAD_LENGTH, 0);
	return 1;
}

void loraEndPacket(bool async) {
	if ((async) && (_onTxDone))
		loraWriteRegister(REG_DIO_MAPPING_1, 0x40); // DIO0 => TXDONE

	// put in TX mode
	loraWriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

	if (!async) {
		// wait for TX done
		int i = 0;
		while ((loraReadRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) {
			i++;
		}
		// clear IRQ's
		loraWriteRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
	}
}

bool loraIsTransmitting() {
	if ((loraReadRegister(REG_OP_MODE) & MODE_TX) == MODE_TX) {
		return true;
	}

	if (loraReadRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) {
		// clear IRQ's
		loraWriteRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
	}

	return false;
}

int loraParsePacket(int size) {
	size = 0;
	int packetLength = 0;
	int irqFlags = loraReadRegister(REG_IRQ_FLAGS);

	if (size > 0) {
		loraImplicitHeaderMode();

		loraWriteRegister(REG_PAYLOAD_LENGTH, size & 0xff);
	} else {
		loraExplicitHeaderMode();
	}

	// clear IRQ's
	loraWriteRegister(REG_IRQ_FLAGS, irqFlags);

	if ((irqFlags & IRQ_RX_DONE_MASK)
			&& (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
		// received a packet
		_packetIndex = 0;

		// read packet length
		if (_implicitHeaderMode) {
			packetLength = loraReadRegister(REG_PAYLOAD_LENGTH);
		} else {
			packetLength = loraReadRegister(REG_RX_NB_BYTES);
		}

		// set FIFO address to current RX address
		loraWriteRegister(REG_FIFO_ADDR_PTR,
				loraReadRegister(REG_FIFO_RX_CURRENT_ADDR));

		// put in standby mode
		loraIdle();
	} else if (loraReadRegister(REG_OP_MODE)
			!= (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)) {
		// not currently in RX mode

		// reset FIFO address
		loraWriteRegister(REG_FIFO_ADDR_PTR, 0);

		// put in single RX mode
		loraWriteRegister(REG_OP_MODE,
		MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
	}

	return packetLength;
}

int loraPacketRssi() {
	return (loraReadRegister(REG_PKT_RSSI_VALUE)
			- (_freq < RF_MID_BAND_THRESHOLD ?
			RSSI_OFFSET_LF_PORT :
												RSSI_OFFSET_HF_PORT));
}

float loraPacketSnr() {
	return ((int8_t) loraReadRegister(REG_PKT_SNR_VALUE)) * 0.25;
}

long loraPacketFrequencyError() {
	int32_t freqError = 0;
	freqError = (int32_t) (loraReadRegister(REG_FREQ_ERROR_MSB) & 0b111);
	freqError <<= 8L;
	freqError += (int32_t) (loraReadRegister(REG_FREQ_ERROR_MID));
	freqError <<= 8L;
	freqError += (int32_t) (loraReadRegister(REG_FREQ_ERROR_LSB));

	if (loraReadRegister(REG_FREQ_ERROR_MSB) & 0b1000) { // Sign bit is on
		freqError -= 524288; // 0b1000'0000'0000'0000'0000
	}

	const float fXtal = 32E6; // FXOSC: crystal oscillator (XTAL) frequency (2.5. Chip Specification, p. 14)
//	const float fError = ((((float) (freqError)) * (1L << 24)) / fXtal)* (loraGetSignalBandwidth() / 500000.0f); // p. 37
	const float fError = ((((float) (freqError)) * (1L << 24)) / fXtal)
			* (_bw / 500000.0f); // p. 37

	return ((long) fError);
}

int loraRssi() {
	return (loraReadRegister(REG_RSSI_VALUE)
			- (_freq < RF_MID_BAND_THRESHOLD ?
			RSSI_OFFSET_LF_PORT :
												RSSI_OFFSET_HF_PORT));
}

void loraWriteByte(uint8_t byte) {
	loraWriteBuf(&byte, sizeof(byte));
}

void loraWriteBuf(const uint8_t *loraBuffer, size_t size) {
	int currentLength = loraReadRegister(REG_PAYLOAD_LENGTH);

	// check size
	if ((currentLength + size) > MAX_PKT_LENGTH) {
		size = MAX_PKT_LENGTH - currentLength;
	}

	// write data
	for (size_t i = 0; i < size; i++) {
		loraWriteRegister(REG_FIFO, loraBuffer[i]);
	}

	// update length
	loraWriteRegister(REG_PAYLOAD_LENGTH, currentLength + size);
}

int loraAvailable() {
	return (loraReadRegister(REG_RX_NB_BYTES) - _packetIndex);
}

uint8_t loraRead() {
	if (!loraAvailable()) {
		return -1;
	}
	_packetIndex++;
	return loraReadRegister(REG_FIFO);
}

int loraPeek() {
	if (!loraAvailable()) {
		return -1;
	}

	// store current FIFO address
	int currentAddress = loraReadRegister(REG_FIFO_ADDR_PTR);

	// read
	uint8_t b = loraReadRegister(REG_FIFO);

	// restore FIFO address
	loraWriteRegister(REG_FIFO_ADDR_PTR, currentAddress);

	return b;
}
/*
 void loraFlush() {
 }

 void loraOnReceive(void (*callback)(int)) {
 _onReceive = callback;

 if (callback) {
 //pinMode(_dio0, INPUT);
 //SPI.usingInterrupt(digitalPinToInterrupt(_dio0));
 //attachInterrupt(digitalPinToInterrupt(_dio0), LoRaClass::onDio0Rise, RISING);
 } else {
 //detachInterrupt(digitalPinToInterrupt(_dio0));
 //SPI.notUsingInterrupt(digitalPinToInterrupt(_dio0));
 }
 }*/

void loraExplicitHeaderMode() {
	_implicitHeaderMode = 0;

	loraWriteRegister(REG_MODEM_CONFIG_1,
			loraReadRegister(REG_MODEM_CONFIG_1) & 0xfe);
}

void loraImplicitHeaderMode() {
	_implicitHeaderMode = 1;

	loraWriteRegister(REG_MODEM_CONFIG_1,
			loraReadRegister(REG_MODEM_CONFIG_1) | 0x01);
}


