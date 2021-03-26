// created by Jake Wright to be the GxCTRL_R61529 class for the GxTFT library
// code extracts taken from https://github.com/Bodmer/TFT_HX8357
// code extracts taken from https://github.com/daumemo/ESP32_LCD_MIPI_DBI_TYPE_B_TEST
//
// License: GNU GENERAL PUBLIC LICENSE V3, see LICENSE

#ifndef _GxCTRL_R61529_H_
#define _GxCTRL_R61529_H_

#include "../GxCTRL.h"

class GxCTRL_R61529 : public GxCTRL
{
  public:
    GxCTRL_R61529(GxIO& io) : GxCTRL(io) {};
    const char* name = "GxCTRL_R61529";
    const uint32_t ID = 0x00; // R61529 does not have a readable Idenitifer.
    uint32_t readID();
    uint32_t readRegister(uint8_t nr, uint8_t index = 0, uint8_t bytes = 1);
    uint16_t readPixel(uint16_t x, uint16_t y);
    void     readRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t* data);
    void init();
    void setWindowAddress(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
    void setRotation(uint8_t r);
};


#define GxCTRL_Class GxCTRL_R61529

#endif