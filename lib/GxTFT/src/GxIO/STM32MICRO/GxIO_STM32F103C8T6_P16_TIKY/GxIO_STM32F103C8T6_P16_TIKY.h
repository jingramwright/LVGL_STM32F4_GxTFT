// created by Jean-Marc Zingg to be the GxIO_STM32F103C8T6_P16_TIKY io class for the GxTFT library
//
// License: GNU GENERAL PUBLIC LICENSE V3, see LICENSE
//
// this is a special wiring for
// https://www.aliexpress.com/item/5-0-inch-HD-IPS-TFT-LCD-module-resistance-touch-with-PCB-adapter-board-854-480/32666829945.html
//
// for STM32F103C8T6 Minimum System (not a BluePill, but only slightly different)
// https://www.aliexpress.com/item/1pcs-STM32F103C8T6-ARM-STM32-Minimum-System-Development-Board-Module-For-arduino/32653883227.html
//
// this version is for use with Arduino package "STM32 Boards (select from submenu)" (STMicroelectronics), board "Generic STM32F1 series" part "BluePill F103C8".
// preferences Additional Boards Manager URLs https://raw.githubusercontent.com/stm32duino/BoardManagerFiles/master/STM32/package_stm_index.json

#ifndef _GxIO_STM32F103C8T6_P16_TIKY_H_
#define _GxIO_STM32F103C8T6_P16_TIKY_H_

#include "../../GxIO.h"

#if (defined(ARDUINO_ARCH_STM32) && defined(STM32F1xx))

class GxIO_STM32F103C8T6_P16_TIKY : public GxIO
{
  public:
    GxIO_STM32F103C8T6_P16_TIKY();
    const char* name = "GxIO_STM32F103C8T6_P16_TIKY";
    void reset();
    void init();
    uint8_t readDataTransaction();
    uint16_t readData16Transaction();
    uint8_t readData();
    uint16_t readData16();
    uint32_t readRawData32(uint8_t part); // debug purpose
    void writeCommandTransaction(uint8_t c);
    void writeCommand16Transaction(uint16_t c);
    void writeDataTransaction(uint8_t d);
    void writeData16Transaction(uint16_t d, uint32_t num = 1);
    void writeCommand(uint8_t c);
    void writeCommand16(uint16_t c);
    void writeData(uint8_t d);
    void writeData(uint8_t* d, uint32_t num);
    void writeData16(uint16_t d, uint32_t num = 1);
    void writeAddrMSBfirst(uint16_t addr);
    void startTransaction();
    void endTransaction();
    void selectRegister(bool rs_low); // for generalized readData & writeData (RA8875)
    void setBackLight(bool lit);
  private:
    void setDataPinsOutput();
    void setDataPinsInput();
    int8_t _cs, _rs, _rst, _wr, _rd, _bl; // Control lines
};

#define GxIO_Class GxIO_STM32F103C8T6_P16_TIKY

#endif

#endif
