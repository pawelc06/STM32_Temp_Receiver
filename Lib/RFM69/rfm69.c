/*
 * rfm69.c
 *
 *  Created on: 17 paź 2020
 *      Author: Pawel
 */
#include "stm32f1xx_hal.h"
#include "rfm69registers.h"
#include <stdbool.h>
#include "rfm69.h"
//#include "systimer.h"
#include <stdio.h>
#include <string.h>

GPIO_TypeDef* _csGPIO;
  uint16_t _csPin;
  GPIO_TypeDef* _resetGPIO;
  uint16_t _resetPin;
  GPIO_TypeDef* _dataGPIO;
  uint16_t _dataPin;
  bool _init2;
  volatile RFM69Mode _mode;
  bool _highPowerDevice;
  uint8_t _powerLevel;
  int _rssi;
  bool _autoReadRSSI;
  bool _ookEnabled;
  RFM69DataMode _dataMode;
  bool _highPowerSettings;
  bool _csmaEnabled;
  volatile char _rxBuffer[RFM69_MAX_PAYLOAD];
  volatile unsigned int _rxBufferLength;

void chipSelect(){
	HAL_GPIO_WritePin(RFM69_SPI_CS_PORT, RFM69_SPI_CS_PIN, GPIO_PIN_RESET);
}

void chipUnselect(){
	HAL_GPIO_WritePin(RFM69_SPI_CS_PORT, RFM69_SPI_CS_PIN, GPIO_PIN_SET);
}

void RFM69_reset()
{



  // generate reset impulse
	HAL_GPIO_WritePin(RFM69_RESET_PORT, RFM69_RESET_PIN, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(RFM69_RESET_PORT, RFM69_RESET_PIN, GPIO_PIN_RESET);

  // wait until module is ready
  HAL_Delay(10);


  _mode = RFM69_MODE_STANDBY;
}

/*
uint8_t readRegister(uint8_t address)
{
	uint8_t value[2];
	uint8_t reg[2];
	// sanity check

  if (address > 0x7f)
    return 0;

  // read value from register
  chipSelect();



  reg[0] = address & 0x7F;
  reg[1] = 0;
  HAL_SPI_TransmitReceive(&RFM69_SPI_PORT, (uint8_t *)reg, (uint8_t *)value, 2, 1000);


  chipUnselect();

  return value[1];


}
*/

uint8_t readRegister(uint8_t reg)
{
  // sanity check
  if (reg > 0x7f)
    return 0;

  // read value from register
  chipSelect();

  uint8_t value;
  HAL_SPI_Transmit(&RFM69_SPI_PORT, &reg , 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(&RFM69_SPI_PORT, &value, 1, HAL_MAX_DELAY);

  chipUnselect();

  return value;
}


/**
 * Write a RFM69 register value.
 *
 * @param reg The register to be written
 * @param value The value of the register to be set
 */
/*
void writeRegister(uint8_t reg, uint8_t value)
{
  // sanity check
	uint8_t packet[2];
  if (reg > 0x7f)
    return;

  // transfer value to register and set the write flag
  chipSelect();

  //spi_transfer(reg | 0x80);
  packet[0] = reg | 0x80;
  packet[1] = value;
  HAL_SPI_Transmit(&RFM69_SPI_PORT, (uint8_t *)&packet, 2, HAL_MAX_DELAY);




  chipUnselect();

}
*/

void writeRegister(uint8_t reg, uint8_t value)
{
  // sanity check
  if (reg > 0x7f)
    return;

  // transfer value to register and set the write flag
  chipSelect();

  uint8_t wreg = (reg | 0x80);
  HAL_SPI_Transmit(&RFM69_SPI_PORT, &wreg , 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(&RFM69_SPI_PORT, &value, 1, HAL_MAX_DELAY);

  //HAL_Delay(1);
  chipUnselect();
}


void RFM69_init(uint8_t freqBand,uint8_t networkID){

	chipUnselect();
	HAL_Delay(10);

	const uint8_t CONFIG[][2] =
	  {
	    /* 0x01 */ { REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },
	    /* 0x02 */ { REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00 }, // no shaping
	    /* 0x03 */ { REG_BITRATEMSB, RF_BITRATEMSB_55555}, // default: 4.8 KBPS
	    /* 0x04 */ { REG_BITRATELSB, RF_BITRATELSB_55555},
	    /* 0x05 */ { REG_FDEVMSB, RF_FDEVMSB_50000}, // default: 5KHz, (FDEV + BitRate / 2 <= 500KHz)
	    /* 0x06 */ { REG_FDEVLSB, RF_FDEVLSB_50000},

	    /* 0x07 */ { REG_FRFMSB, (uint8_t) (freqBand==RF69_315MHZ ? RF_FRFMSB_315 : (freqBand==RF69_433MHZ ? RF_FRFMSB_433 : (freqBand==RF69_868MHZ ? RF_FRFMSB_868 : RF_FRFMSB_915))) },
	    /* 0x08 */ { REG_FRFMID, (uint8_t) (freqBand==RF69_315MHZ ? RF_FRFMID_315 : (freqBand==RF69_433MHZ ? RF_FRFMID_433 : (freqBand==RF69_868MHZ ? RF_FRFMID_868 : RF_FRFMID_915))) },
	    /* 0x09 */ { REG_FRFLSB, (uint8_t) (freqBand==RF69_315MHZ ? RF_FRFLSB_315 : (freqBand==RF69_433MHZ ? RF_FRFLSB_433 : (freqBand==RF69_868MHZ ? RF_FRFLSB_868 : RF_FRFLSB_915))) },

	    // looks like PA1 and PA2 are not implemented on RFM69W, hence the max output power is 13dBm
	    // +17dBm and +20dBm are possible on RFM69HW
	    // +13dBm formula: Pout = -18 + OutputPower (with PA0 or PA1**)
	    // +17dBm formula: Pout = -14 + OutputPower (with PA1 and PA2)**
	    // +20dBm formula: Pout = -11 + OutputPower (with PA1 and PA2)** and high power PA settings (section 3.3.7 in datasheet)
	    ///* 0x11 */ { REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111},
	    ///* 0x13 */ { REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95 }, // over current protection (default is 95mA)

	    // RXBW defaults are { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_5} (RxBw: 10.4KHz)
	    /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2 }, // (BitRate < 2 * RxBw)
	    //for BR-19200: /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_3 },
	    /* 0x25 */ { REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 }, // DIO0 is the only IRQ we're using
	    /* 0x26 */ { REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF }, // DIO5 ClkOut disable for power saving
	    /* 0x28 */ { REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN }, // writing to this bit ensures that the FIFO & status flags are reset
	    /* 0x29 */ { REG_RSSITHRESH, 220 }, // must be set to dBm = (-Sensitivity / 2), default is 0xE4 = 228 so -114dBm
	    ///* 0x2D */ { REG_PREAMBLELSB, RF_PREAMBLESIZE_LSB_VALUE } // default 3 preamble bytes 0xAAAAAA
	    /* 0x2E */ { REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0 },
	    /* 0x2F */ { REG_SYNCVALUE1, 0x2D },      // attempt to make this compatible with sync1 byte of RFM12B lib
	    /* 0x30 */ { REG_SYNCVALUE2, networkID }, // NETWORK ID
	    /* 0x37 */ { REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF },
	    /* 0x38 */ { REG_PAYLOADLENGTH, 66 }, // in variable length mode: the max frame size, not used in TX
	    ///* 0x39 */ { REG_NODEADRS, nodeID }, // turned off because we're not using address filtering
	    /* 0x3C */ { REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE }, // TX on FIFO not empty
	    /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
	    //for BR-19200: /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_NONE | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
	    /* 0x6F */ { REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 }, // run DAGC continuously in RX mode for Fading Margin Improvement, recommended default for AfcLowBetaOn=0
	    {255, 0}
	  };

	writeRegister(0, 0);
	RFM69_reset();

	  for (uint8_t i = 0; CONFIG[i][0] != 255; i++)
		  writeRegister(CONFIG[i][0], CONFIG[i][1]);

	  // set PA and OCP settings according to RF module (normal/high power)
	    //setPASettings(0,false);

	    // clear FIFO and flags
	    //clearFIFO();

#ifdef USE_IRQ
	    //ExtInt_Config();
#endif
}

void RFM69_dumpRegisters(void)
{
	uint8_t rVal;
	char buf[10];
  for (uint8_t i = 1; i <= 0x71; i++)
  {
	  UART_Printf("[0x%X]: 0x%X\n", i, readRegister(i));
  }

}
