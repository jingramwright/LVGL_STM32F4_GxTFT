#include "Wire.h"

#define TOUCH_I2C_ADD 0x38
#define TOUCH_REG_XL 0x04
#define TOUCH_REG_XH 0x03
#define TOUCH_REG_YL 0x06
#define TOUCH_REG_YH 0x05

#define TOUCH_INT_PIN PE1

int getTouchInterruptPinValue()
{
    return digitalRead(TOUCH_INT_PIN);
}

int readTouchReg(int reg)
{
    int data = 0;

    Wire.beginTransmission(TOUCH_I2C_ADD);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(TOUCH_I2C_ADD, 1);
    if (Wire.available())
    {
        data = Wire.read();
    }

    return data;
}

int getTouchPointX()
{
    uint8_t XL = 0;
    uint8_t XH = 0;

    XH = readTouchReg(TOUCH_REG_XH);
    XL = readTouchReg(TOUCH_REG_XL);

    return ((XH & 0x0F) << 8) | XL;
}

int getTouchPointY()
{
	uint8_t YL = 0;
	uint8_t YH = 0;

    YH = readTouchReg(TOUCH_REG_YH);
    YL = readTouchReg(TOUCH_REG_YL);

    return ((YH & 0x0F) << 8) | YL;
}

void touchInit()
{
    Wire.setSDA(PB11);
    Wire.setSCL(PB10);
    Wire.setClock(40000);
    Wire.begin();
    pinMode(TOUCH_INT_PIN, INPUT);

    Wire.beginTransmission(0x38);
    Wire.write(0xA4);
    Wire.write(0x00); //turn on interrupt
    Wire.endTransmission();
}