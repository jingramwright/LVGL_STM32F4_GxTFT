// created by Jake Wright to be the GxCTRL_R61529 class for the GxTFT library
// code extracts taken from https://github.com/Bodmer/TFT_HX8357
// code extracts taken from https://github.com/daumemo/ESP32_LCD_MIPI_DBI_TYPE_B_TEST
//
// License: GNU GENERAL PUBLIC LICENSE V3, see LICENSE

#include "GxCTRL_R61529.h"

#define CASET 0x2A
#define PASET 0x2B
#define RAMWR 0x2C
#define RAMRD 0x2E
#define MADCTL 0x36
#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_SS  0x02
#define MADCTL_GS  0x01

uint32_t GxCTRL_R61529::readID()
{
  return 0x00;
}

uint32_t GxCTRL_R61529::readRegister(uint8_t nr, uint8_t index, uint8_t bytes)
{
  uint32_t rv = 0;
  bytes = min(bytes, 4);
  IO.startTransaction();
  IO.writeCommand(nr);
  IO.readData(); // dummy
  for (uint8_t i = 0; i < index; i++)
  {
    IO.readData(); // skip
  }
  for (; bytes > 0; bytes--)
  {
    rv <<= 8;
    rv |= IO.readData();
  }
  IO.endTransaction();
  return rv;
}

uint16_t GxCTRL_R61529::readPixel(uint16_t x, uint16_t y)
{
  uint16_t rv;
  readRect(x, y, 1, 1, &rv);
  return rv;
}

void GxCTRL_R61529::readRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t* data)
{
  uint16_t xe = x + w - 1;
  uint16_t ye = y + h - 1;
  uint32_t num = uint32_t(w) * uint32_t(h);
  IO.startTransaction();
  IO.writeCommand(CASET);  // Column addr set
  IO.writeData(x >> 8);
  IO.writeData(x & 0xFF);  // XSTART
  IO.writeData(xe >> 8);
  IO.writeData(xe & 0xFF); // XEND
  IO.writeCommand(PASET);  // Row addr set
  IO.writeData(y >> 8);
  IO.writeData(y);         // YSTART
  IO.writeData(ye >> 8);
  IO.writeData(ye);        // YEND
  IO.writeCommand(RAMRD);  // read from RAM
  IO.readData(); // dummy
  for (; num > 0; num--)
  {
    uint16_t d = IO.readData16();
    *data++ = ((d & 0x001F) << 11) | (d & 0x07E0) | ((d & 0xF800) >> 11); // r,b swapped
  }
  IO.endTransaction();
}

void GxCTRL_R61529::init()
{
  // Configure R61529 display
  IO.writeCommandTransaction(0x11); //exit sleep mode;
  delay(200);
  IO.writeCommandTransaction(0xB0); //manufacturer command access protect
  IO.writeDataTransaction(0x04); //allow access to additional manufacturer's commands
  delay(1);

  IO.writeCommandTransaction(0xB3); //Frame Memory Access and Interface Setting
  IO.writeDataTransaction(0x02); // reset start position of a window area address...
  IO.writeDataTransaction(0x00); //TE pin is used. TE signal is output every frame.
  IO.writeDataTransaction(0x00); // empty according to the datasheet - does nothing;
  IO.writeDataTransaction(0x00); // convert 16/18 bits to 24bits data by writing zeroes to LSBs. Sets image data write/read format(?)
  IO.writeDataTransaction(0x00);  // ???? (not needed?)
  delay(1);

  IO.writeCommandTransaction(0xB4); //Display Mode
  IO.writeDataTransaction(0x00); //Uses internal oscillator
  delay(1);

  IO.writeCommandTransaction(0xC0); // Panel Driving Setting;
  IO.writeDataTransaction(0x03); // Output polarity is inverted. Left/right interchanging scan. Forward scan. BGR mode (depends on other settings). S960 → S1 (depends)
  IO.writeDataTransaction(0xDF); // Number of lines for driver to drive - 480.
  IO.writeDataTransaction(0x40); // Scan start position - Gate1. (depend on other param);
  IO.writeDataTransaction(0x10); // Dot inversion. Dot inversion in not-lit display area. If 0x13 - both will be set to 'column inversion'.
  IO.writeDataTransaction(0x00); // settings for non-lit display area...
  IO.writeDataTransaction(0x01); // 3 frame scan interval in non-display area...
  IO.writeDataTransaction(0x00); // Source output level in retrace period...
  IO.writeDataTransaction(0x55);//54 . Internal clock divider = 5 (low and high periods).

  IO.writeCommandTransaction(0xC1); //Display Timing Setting for Normal Mode
  IO.writeDataTransaction(0x07); // Clock devider = 12. 14MHz/12. Used by display circuit and step-up circuit.
  IO.writeDataTransaction(0x27); // These bits set the number of clocks in 1 line period. 0x27 - 39 clocks.
  IO.writeDataTransaction(0x08); // Number of back porch lines. 0x08 - 8 lines.
  IO.writeDataTransaction(0x08); // Number of front porch lines. 0x08 - 8lines.
  IO.writeDataTransaction(0x00); // Spacial configuriation mode 1 (?). 1 line inversion mode (?).

  IO.writeCommandTransaction(0xC4); // Source/Gate Driving Timing Setting
  IO.writeDataTransaction(0x57); // falling position (stop) of gate driver - 4 clocks... gate start position - 8 clocks...
  IO.writeDataTransaction(0x00); // nothing to set up according to the datasheet
  IO.writeDataTransaction(0x05); // Source precharge period (GND) - 5 clocks.
  IO.writeDataTransaction(0x03); // source precharge period (VCI) - 3 clocks.

  IO.writeCommandTransaction(0xC6); //DPI polarity control
  IO.writeDataTransaction(0x04); // VSYNC -Active Low. HSYNC - Active Low. DE pin enable data write in when DE=1. Reads data on the rising edge of the PCLK signal.

  //----Gamma setting start-----
  IO.writeCommandTransaction(0xC8);
  IO.writeDataTransaction(0x03);
  IO.writeDataTransaction(0x12);
  IO.writeDataTransaction(0x1A);
  IO.writeDataTransaction(0x24);
  IO.writeDataTransaction(0x32);
  IO.writeDataTransaction(0x4B);
  IO.writeDataTransaction(0x3B);
  IO.writeDataTransaction(0x29);
  IO.writeDataTransaction(0x1F);
  IO.writeDataTransaction(0x18);
  IO.writeDataTransaction(0x12);
  IO.writeDataTransaction(0x04);

  IO.writeDataTransaction(0x03);
  IO.writeDataTransaction(0x12);
  IO.writeDataTransaction(0x1A);
  IO.writeDataTransaction(0x24);
  IO.writeDataTransaction(0x32);
  IO.writeDataTransaction(0x4B);
  IO.writeDataTransaction(0x3B);
  IO.writeDataTransaction(0x29);
  IO.writeDataTransaction(0x1F);
  IO.writeDataTransaction(0x18);
  IO.writeDataTransaction(0x12);
  IO.writeDataTransaction(0x04);

  IO.writeCommandTransaction(0xC9);
  IO.writeDataTransaction(0x03);
  IO.writeDataTransaction(0x12);
  IO.writeDataTransaction(0x1A);
  IO.writeDataTransaction(0x24);
  IO.writeDataTransaction(0x32);
  IO.writeDataTransaction(0x4B);
  IO.writeDataTransaction(0x3B);
  IO.writeDataTransaction(0x29);
  IO.writeDataTransaction(0x1F);
  IO.writeDataTransaction(0x18);
  IO.writeDataTransaction(0x12);
  IO.writeDataTransaction(0x04);

  IO.writeDataTransaction(0x03);
  IO.writeDataTransaction(0x12);
  IO.writeDataTransaction(0x1A);
  IO.writeDataTransaction(0x24);
  IO.writeDataTransaction(0x32);
  IO.writeDataTransaction(0x4B);
  IO.writeDataTransaction(0x3B);
  IO.writeDataTransaction(0x29);
  IO.writeDataTransaction(0x1F);
  IO.writeDataTransaction(0x18);
  IO.writeDataTransaction(0x12);
  IO.writeDataTransaction(0x04);

  IO.writeCommandTransaction(0xCA);
  IO.writeDataTransaction(0x03);
  IO.writeDataTransaction(0x12);
  IO.writeDataTransaction(0x1A);
  IO.writeDataTransaction(0x24);
  IO.writeDataTransaction(0x32);
  IO.writeDataTransaction(0x4B);
  IO.writeDataTransaction(0x3B);
  IO.writeDataTransaction(0x29);
  IO.writeDataTransaction(0x1F);
  IO.writeDataTransaction(0x18);
  IO.writeDataTransaction(0x12);
  IO.writeDataTransaction(0x04);

  IO.writeDataTransaction(0x03);
  IO.writeDataTransaction(0x12);
  IO.writeDataTransaction(0x1A);
  IO.writeDataTransaction(0x24);
  IO.writeDataTransaction(0x32);
  IO.writeDataTransaction(0x4B);
  IO.writeDataTransaction(0x3B);
  IO.writeDataTransaction(0x29);
  IO.writeDataTransaction(0x1F);
  IO.writeDataTransaction(0x18);
  IO.writeDataTransaction(0x12);
  IO.writeDataTransaction(0x04);
//---Gamma setting end--------

  IO.writeCommandTransaction(0xD0); // Power (charge pump) settings
  IO.writeDataTransaction(0x99);//DC4~1//A5. Set up clock cycle of the internal step up controller.
  IO.writeDataTransaction(0x06);//BT // Set Voltage step up factor.
  IO.writeDataTransaction(0x08);// default according to the datasheet - does nothing.
  IO.writeDataTransaction(0x20);// VCN step up cycles.
  IO.writeDataTransaction(0x29);//VC1, VC2// VCI3 voltage = 2.70V;  VCI2 voltage = 3.8V.
  IO.writeDataTransaction(0x04);// default 
  IO.writeDataTransaction(0x01);// default 
  IO.writeDataTransaction(0x00);// default 
  IO.writeDataTransaction(0x08);// default
  IO.writeDataTransaction(0x01);// default
  IO.writeDataTransaction(0x00);// default
  IO.writeDataTransaction(0x06);// default
  IO.writeDataTransaction(0x01);// default
  IO.writeDataTransaction(0x00);// default
  IO.writeDataTransaction(0x00);// default
  IO.writeDataTransaction(0x20);// default

  IO.writeCommandTransaction(0xD1);//VCOM setting
  IO.writeDataTransaction(0x00);//disable write to VDC[7:0].
  IO.writeDataTransaction(0x20);//45 38 VPLVL// voltage of γ correction registers for positive polarity
  IO.writeDataTransaction(0x20);//45 38 VNLVL// voltage of γ correction registers for negative polarity
  IO.writeDataTransaction(0x15);//32 2A VCOMDC// VNLVL x 0.063

  IO.writeCommandTransaction(0xE0);//NVM Access Control
  IO.writeDataTransaction(0x00);//NVM access is disabled
  IO.writeDataTransaction(0x00);//Erase operation (disabled).
  IO.writeDataTransaction(0x00);//TE pin works as tearing effect pin. 
  // should be one more IO.writeDataTransaction(0x00); according to the datasheet.

  IO.writeCommandTransaction(0xE1); //set_DDB_write_control
  IO.writeDataTransaction(0x00); 
  IO.writeDataTransaction(0x00);
  IO.writeDataTransaction(0x00);
  IO.writeDataTransaction(0x00);
  IO.writeDataTransaction(0x00);
  IO.writeDataTransaction(0x00);

  IO.writeCommandTransaction(0xE2); //NVM Load Control
  IO.writeDataTransaction(0x00); // does not execute data load from the NVM to each command

  IO.writeCommandTransaction(0x36); //set_address_mode
  IO.writeDataTransaction(0x00); // data is not flipped in any way?

  IO.writeCommandTransaction(0x3A); // set_pixel_format
  IO.writeDataTransaction(0x55);// 16-Bit/pixel = 55h, 24-bit/pixel = 77h

  IO.writeCommandTransaction(0x2A); //set_column_address
  IO.writeDataTransaction(0x00); // starts from 0th frame buffer address
  IO.writeDataTransaction(0x00);
  IO.writeDataTransaction(0x01);
  IO.writeDataTransaction(0x3F);//320 - uses all columns

  IO.writeCommandTransaction(0x2B); //set_page_address
  IO.writeDataTransaction(0x00); // starts from 0th frame buffer address
  IO.writeDataTransaction(0x00);
  IO.writeDataTransaction(0x01);
  IO.writeDataTransaction(0xDF);//480 - uses all lines in the frame buffer

  IO.writeCommandTransaction(0x29); //set_display_on - This command causes the display module to start displaying the image data on the display device.
  delay(20);
  // End of R61529 display configuration
}

void GxCTRL_R61529::setWindowAddress(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
  IO.writeCommand(CASET);  // Column addr set
  IO.writeData(x0 >> 8);
  IO.writeData(x0 & 0xFF); // XSTART
  IO.writeData(x1 >> 8);
  IO.writeData(x1 & 0xFF); // XEND
  IO.writeCommand(PASET);  // Row addr set
  IO.writeData(y0 >> 8);
  IO.writeData(y0);        // YSTART
  IO.writeData(y1 >> 8);
  IO.writeData(y1);        // YEND
  IO.writeCommand(RAMWR);  // write to RAM
}

void GxCTRL_R61529::setRotation(uint8_t r)
{
  IO.startTransaction();
  IO.writeCommand(MADCTL);
  switch (r & 3)
  {
    case 0: // Portrait
      IO.writeData(MADCTL_RGB);
      break;
    case 1: // Landscape (Portrait + 90)
      IO.writeData(MADCTL_MV | MADCTL_RGB);
      break;
    case 2: // Inverter portrait
      IO.writeData(MADCTL_RGB | MADCTL_GS);
      break;
    case 3: // Inverted landscape
      IO.writeData(MADCTL_MV | MADCTL_RGB | MADCTL_GS);
      break;
  }
  IO.endTransaction();
}

