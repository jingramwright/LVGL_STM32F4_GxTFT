// created by Jean-Marc Zingg to be the GxIO_STM32F4_FSMC io class for the GxTFT library
//
// License: GNU GENERAL PUBLIC LICENSE V3, see LICENSE
//
// this is the io class for STM32F407V, STM32F407Z boards with FMSC TFT connector, e.g. for
// https://www.aliexpress.com/item/Free-shipping-STM32F407VET6-development-board-Cortex-M4-STM32-minimum-system-learning-board-ARM-core-board/32618222721.html
//
// e.g. for display with matching connector
// https://www.aliexpress.com/item/3-2-inch-TFT-LCD-screen-with-resistive-touch-screens-ILI9341-display-module/32662835059.html
//
// for pin information see the backside of the TFT, for the data pin to port pin mapping see FSMC pin table STM32F407V doc.
//
// this io class can be used with or adapted to other STM32F407V/Z processor boards with FSMC TFT connector.
//
// this version is for use with Arduino package "STM32 Boards (select from submenu)" (STMicroelectronics), 
// board "Generic STM32F4 series" part "BLACK F407VE", or "BLACK F407VG", or "BLACK F407ZE", or "BLACK F407ZG".
// preferences Additional Boards Manager URLs https://raw.githubusercontent.com/stm32duino/BoardManagerFiles/master/STM32/package_stm_index.json

#if defined(ARDUINO_ARCH_STM32) && defined(STM32F407xx) && !defined(STM32GENERIC) // "STM32 Boards (select from submenu)"
#if defined(ARDUINO_BLACK_F407VE) || defined(ARDUINO_BLACK_F407VG) || defined(ARDUINO_BLACK_F407ZE) || defined(ARDUINO_BLACK_F407ZG)

#include "GxIO_STM32F4_FSMC.h"

// TFT connector uses FSMC pins
// D0   D1   D2  D3  D4  D5  D6  D7   D8   D9   D10  D11  D12  D13 D14 D15
// PD14 PD15 PD0 PD1 PE7 PE8 PE9 PE10 PE11 PE12 PE13 PE14 PE15 PD8 PD9 PD10

// connector pins
// 01  02  03  04  05  06  07  08  09  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32
// GND RST D15 D14 D13 D12 D11 D10 D9  D8  D7  D6  D5  D4  D3  D2  D1  D0  RD  WR  RS  CS  SCK SCS SI  SO  INT BLK SET GND 3.3 GND
//         D10 D9  D8  E15 E14 E13 E12 E11 E10 E9  E8  E7  D1  D0  D15 D14 D4  D5  D13 D7                      B1

// D used : 15,14,13,..,..,10, 9, 8, 7,.., 5, 4,..,.., 1, 0, // 11
// D data : 15,14,..,..,..,10, 9, 8,..,..,..,..,..,.., 1, 0, // 7
//         |           |           |           |           |
// E used : 15,14,13,12,11,10, 9, 8, 7,..,..,..,..,..,..,.., // 9
// E data : 15,14,13,12,11,10, 9, 8, 7,..,..,..,..,..,..,.., // 9

//                    |       |        | // Ruler on 8 & 16
#define PD_DATA_BITS 0b1100011100000011
#define PD_CTRL_BITS 0b0010000010110000  // PD13(FSMC_A18),PD7(FSMC_NE1),PD5(FSMC_NWE),PD4(FSMC_NOE)
#define PD_FSMC_BITS 0b1110011110110011
#define PD_AFR0_MASK         0xF0FF00FF
#define PD_AFR0_FSMC         0xC0CC00CC
#define PD_AFR1_MASK 0xFFF00FFF
#define PD_AFR1_FSMC 0xCCC00CCC
//                    |       |        |
#define PD_MODE_MASK 0xFC3FCF0F // all FMSC MODE bits
#define PD_MODE_FSMC 0xA82A8A0A // FMSC MODE values 10 alternate function
#define PD_OSPD_FSMC 0x54154505 // FMSC OSPEED values 01 10MHz
//                    |       |        |
#define PE_DATA_BITS 0b1111111110000000
#define PE_AFR0_MASK         0xF0000000
#define PE_AFR0_FSMC         0xC0000000
#define PE_AFR1_MASK 0xFFFFFFFF
#define PE_AFR1_FSMC 0xCCCCCCCC
//                    |       |        |
#define PE_MODE_MASK 0xFFFFC000 // all FMSC MODE bits
#define PE_MODE_FSMC 0xAAAA8000 // FMSC MODE values 10 alternate function
#define PE_OSPD_FSMC 0x55554000 // FMSC OSPEED values 01 10MHz

#define ADDSET 15 // (ADDSET+1)*6ns = CS to RW
#define DATAST 30 // (DATAST+1)*6ns = RW length

// these are defined for STM32F103V in
// ...\Arduino\hardware\STM32GENERIC\STM32\system\STM32F1\CMSIS_Inc\stm32f103xe.h
// but not for STM32F407V/Z, maybe because relocatable
#define FSMC_BASE             ((uint32_t)0x60000000) /*!< FSMC base address */
#define FSMC_BANK1            (FSMC_BASE)               /*!< FSMC Bank1 base address */

#define CommandAccess FSMC_BANK1
#define DataAccess (FSMC_BANK1 + 0x80000)

GxIO_STM32F4_FSMC::GxIO_STM32F4_FSMC(bool bl_active_high)
{
  _cs   = PD7;  // FSMC_NE1
  _rs   = PD13; // FSMC_A18
  _rst  = 0;    // not available, driven from NRST
  _wr   = PD5;  // FSMC_NWE
  _rd   = PD4;  // FSMC_NOE
  _bl   = PB1;
  _bl_active_high = bl_active_high;
}

void GxIO_STM32F4_FSMC::reset()
{
  // _rst pin not available
}

void GxIO_STM32F4_FSMC::init()
{
  /* START INIT ROBBED FROM https://github.com/OutOfTheBots/ili9341_16bit_touch/blob/master/ili9341.c FOR 100MHz FSMC */
  //enable RCC for FSMC and both GPIO ports
	// RCC->AHB3ENR |= RCC_AHB3ENR_FSMCEN;
	// RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN;

	// //setup pins as AF
	// GPIOD->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1 | GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1 | GPIO_MODER_MODER7_1 | GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1 | GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1;
	// GPIOE->MODER |= GPIO_MODER_MODER7_1 | GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1  | GPIO_MODER_MODER11_1 | GPIO_MODER_MODER12_1 | GPIO_MODER_MODER13_1 | GPIO_MODER_MODER14_1 |  GPIO_MODER_MODER15_1;
	// //setup pins as PP
	// GPIOD->OTYPER &= 0b0011000001001100;
	// GPIOE->OTYPER &= 0b0000000001111111;
	// //set speed to 100MHz
	// GPIOD->OSPEEDR = ( GPIOD->OSPEEDR & ~PD_MODE_MASK) | PD_OSPD_FSMC; //|= (0b11<<(2*0)) | (0b11<<(2*1)) | (0b11<<(2*4)) | (0b11<<(2*5)) | (0b11<<(2*7)) | (0b11<<(2*8)) | (0b11<<(2*9)) | (0b11<<(2*10)) | (0b11<<(2*11)) | (0b11<<(2*14)) | (0b11<<(2*15));
	// GPIOE->OSPEEDR = (GPIOE->OSPEEDR & ~PE_MODE_MASK) | PE_OSPD_FSMC;//|= (0b11<<(2*7)) | (0b11<<(2*8)) | (0b11<<(2*9)) | (0b11<<(2*10)) | (0b11<<(2*11)) | (0b11<<(2*12)) | (0b11<<(2*13)) | (0b11<<(2*14)) | (0b11<<(2*15));
	// //set NO pull-up or pull-down
	// GPIOD->PUPDR &= ((0b11<<(2*2)) | (0b11<<(2*3)) | (0b11<<(2*6)) | (0b11<<(2*12)) | (0b11<<(2*13)));
	// GPIOE->PUPDR &= ((0b11<<(2*0)) | (0b11<<(2*1)) | (0b11<<(2*2)) | (0b11<<(2*3)) | (0b11<<(2*4)) | (0b11<<(2*5)) | (0b11<<(2*6)));
	// //set other pins to AF12 i.e as FSMC pins
	// GPIOD->AFR[0] = (GPIOD->AFR[0] & (0b1111<<(4*2) | 0b1111<<(4*3) | 0b1111<<(4*6))) | 0b1100<<(4*0) | 0b1100<<(4*1) | 0b1100<<(4*4) | 0b1100<<(4*5) | 0b1100<<(4*7);
	// GPIOD->AFR[1] = (GPIOD->AFR[1] & (0b1111<<(4*(13-8)) | 0b1111<<(4*(12-8)))) | 0b1100<<(4*(8-8)) | 0b1100<<(4*(9-8)) | 0b1100<<(4*(10-8)) | 0b1100<<(4*(11-8)) | 0b1100<<(4*(14-8)) | 0b1100<<(4*(15-8));
	// GPIOE->AFR[0] = (GPIOE->AFR[0] & ~(0b1111<<7)) | 0b1100<<(4*7);
	// GPIOE->AFR[1] = 0xCCCCCCCC;
  /* END INIT ROBBED FROM https://github.com/OutOfTheBots/ili9341_16bit_touch/blob/master/ili9341.c FOR 100MHz FSMC */

	//setup FSMC on Bank1 NORSRAM1
	//setup timings of FSCM
	//FSMC_Bank1->BTCR[1] = FSMC_BTR1_ADDSET_1 | FSMC_BTR1_DATAST_1;
	// Bank1 NOR/SRAM control register configuration
	//FSMC_Bank1->BTCR[0] = FSMC_BCR1_MWID_0 | FSMC_BCR1_WREN | FSMC_BCR1_MBKEN;


  RCC->AHB1ENR   |= 0x00000078;
  volatile uint32_t t = RCC->AHB1ENR; // delay

  GPIOD->AFR[0]  = ( GPIOD->AFR[0] & ~PD_AFR0_MASK) | PD_AFR0_FSMC;
  GPIOD->AFR[1]  = ( GPIOD->AFR[1] & ~PD_AFR1_MASK) | PD_AFR1_FSMC;
  GPIOD->MODER   = ( GPIOD->MODER & ~PD_MODE_MASK) | PD_MODE_FSMC;
  GPIOD->OSPEEDR = ( GPIOD->OSPEEDR & ~PD_MODE_MASK) | PD_OSPD_FSMC;
  GPIOD->OTYPER &= ~PD_MODE_MASK;
  GPIOD->PUPDR  &= ~PD_MODE_MASK;

  GPIOE->AFR[0]  = (GPIOE->AFR[0] & ~PE_AFR0_MASK) | PE_AFR0_FSMC;
  GPIOE->AFR[1]  = (GPIOE->AFR[1] & ~PE_AFR1_MASK) | PE_AFR1_FSMC;
  GPIOE->MODER   = (GPIOE->MODER & ~PE_MODE_MASK) | PE_MODE_FSMC;
  GPIOE->OSPEEDR = (GPIOE->OSPEEDR & ~PE_MODE_MASK) | PE_OSPD_FSMC;
  GPIOE->OTYPER &= ~PE_MODE_MASK;
  GPIOE->PUPDR  &= ~PE_MODE_MASK;

  RCC->AHB3ENR         |= 0x00000001;
  t = RCC->AHB1ENR; // delay
  (void)(t);

   FSMC_Bank1->BTCR[0] = FSMC_BCR1_MWID_0 | FSMC_BCR1_WREN | FSMC_BCR1_MBKEN;//0x000010D9;
   FSMC_Bank1->BTCR[1] = FSMC_BTR1_ADDSET_1 | FSMC_BTR1_DATAST_1;//(DATAST << 8) | ADDSET;

  digitalWrite(_bl, LOW);
  pinMode(_bl, OUTPUT);
}

uint8_t GxIO_STM32F4_FSMC::readDataTransaction()
{
  return *(uint8_t*)DataAccess;
}

uint16_t GxIO_STM32F4_FSMC::readData16Transaction()
{
  return *(uint16_t*)DataAccess;
}

uint8_t GxIO_STM32F4_FSMC::readData()
{
  return *(uint8_t*)DataAccess;
}

uint16_t GxIO_STM32F4_FSMC::readData16()
{
  return *(uint16_t*)DataAccess;
}

uint32_t GxIO_STM32F4_FSMC::readRawData32(uint8_t part)
{
  return *(uint16_t*)DataAccess;
}

void GxIO_STM32F4_FSMC::writeCommandTransaction(uint8_t c)
{
  *(uint8_t*)CommandAccess = c;
}

void GxIO_STM32F4_FSMC::writeCommand16Transaction(uint16_t c)
{
  *(uint16_t*)CommandAccess = c;
}

void GxIO_STM32F4_FSMC::writeDataTransaction(uint8_t d)
{
  *(uint8_t*)DataAccess = d;
}

void GxIO_STM32F4_FSMC::writeData16Transaction(uint16_t d, uint32_t num)
{
  while (num > 0)
  {
    *(uint16_t*)DataAccess = d;
    num--;
  }
}

void GxIO_STM32F4_FSMC::writeCommand(uint8_t c)
{
  *(uint8_t*)CommandAccess = c;
}

void GxIO_STM32F4_FSMC::writeCommand16(uint16_t c)
{
  *(uint16_t*)CommandAccess = c;
}

void GxIO_STM32F4_FSMC::writeData(uint8_t d)
{
  *(uint8_t*)DataAccess = d;
}

void GxIO_STM32F4_FSMC::writeData(uint8_t* d, uint32_t num)
{
  while (num > 0)
  {
    *(uint8_t*)DataAccess = *d;
    d++;
    num--;
  }
}

void GxIO_STM32F4_FSMC::writeData16(uint16_t d, uint32_t num)
{
  while (num > 0)
  {
    *(uint16_t*)DataAccess = d;
    num--;
  }
}

void GxIO_STM32F4_FSMC::writeAddrMSBfirst(uint16_t d)
{
  writeData16(d >> 8);
  writeData16(d & 0xFF);
}

void GxIO_STM32F4_FSMC::startTransaction()
{
}

void GxIO_STM32F4_FSMC::endTransaction()
{
}

void GxIO_STM32F4_FSMC::selectRegister(bool rs_low)
{
}

void GxIO_STM32F4_FSMC::setBackLight(bool lit)
{
  digitalWrite(_bl, (lit == _bl_active_high));
}

#endif
#endif