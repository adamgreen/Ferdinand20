// Fast ST7789 IPS 240x240 SPI display library
// (c) 2019 by Pawel A. Hernik
// Updated to work with nRF5 SDK in 2020 by Adam Green (https://github.com/adamgreen)

#include "Arduino_ST7789_Fast.h"
#include <limits.h>
#include <nrf_gpio.h>
#include <nrf_delay.h>
#include <nrf_drv_spi.h>

// Fake out AVR PGMREAD functionality for ARM where it can be treated like any other memory region.
#define PROGMEM

// Initialization commands for ST7789.
static const uint8_t PROGMEM initST7789[] = {
    9,                       				// 9 commands in list:
    ST7789_SWRESET,   ST_CMD_DELAY,  		// 1: Software reset, no args, w/delay
      150,                     				// 150 ms delay
    ST7789_SLPOUT ,   ST_CMD_DELAY,  		// 2: Out of sleep mode, no args, w/delay
      255,                    				// 255 = 500 ms delay
    ST7789_COLMOD , 1+ST_CMD_DELAY,  		// 3: Set color mode, 1 arg + delay:
      0x55,                   				// 16-bit color
      10,                     				// 10 ms delay
    ST7789_MADCTL , 1,  					// 4: Memory access ctrl (directions), 1 arg:
      0x00,                   				// Row addr/col addr, bottom to top refresh
    ST7789_INVON ,   ST_CMD_DELAY,  		// 7: Inversion ON
      10,
    ST7789_NORON  ,   ST_CMD_DELAY,  		// 8: Normal display on, no args, w/delay
      10,                     				// 10 ms delay
    ST7789_DISPON ,   ST_CMD_DELAY,  		// 9: Main screen turn on, no args, w/delay
      20 };                  				// 255 = 500 ms delay

// Don't need to do anything for SPI_START/END on nRF51 devices.
#define SPI_START
#define SPI_END

// macros for fast DC and CS state changes
#define DC_DATA     *portSet = dcMask
#define DC_COMMAND  *portClear = dcMask
#define CS_IDLE     *portSet = csMask
#define CS_ACTIVE   *portClear = csMask

// SPI_OBJ will be the SPI peripheral instance to be used by this class.
#define SPI_OBJ ((NRF_SPI_Type*)NRF_DRV_SPI_PERIPHERAL(ST7789_SPI_INSTANCE))

// if CS always connected to the ground then don't do anything for better performance
#ifdef CS_ALWAYS_LOW
#undef  CS_IDLE
#undef  CS_ACTIVE
#define CS_IDLE
#define CS_ACTIVE
#endif

inline void Arduino_ST7789::writeSPI(uint8_t c)
{
    SPI_OBJ->TXD = c;
    // The following NOPs eat up enough CPU time that the byte will have been transmitted before the next call to
    // this function or changing the state of the DC or CS pins occurs.
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
}

// ----------------------------------------------------------
Arduino_ST7789::Arduino_ST7789(uint16_t width, uint16_t height, uint16_t columnOffset, uint16_t rowOffset,
                               uint8_t mosi, uint8_t sck, uint8_t dc, uint8_t rst, uint8_t cs) :
    Adafruit_GFX(width, height)
{
  _colstart = columnOffset;
  _rowstart = rowOffset;
  _width  = width;
  _height = height;
  mosiPin = mosi;
  sckPin = sck;
  csPin = cs;
  dcPin = dc;
  rstPin = rst;
}

// ----------------------------------------------------------
void Arduino_ST7789::init()
{
  commonST7789Init(NULL);

  displayInit(initST7789);
  setRotation(0);
}

// ----------------------------------------------------------
void Arduino_ST7789::writeCmd(uint8_t c)
{
  DC_COMMAND;
  CS_ACTIVE;
  SPI_START;

  writeSPI(c);

  CS_IDLE;
  SPI_END;
}

// ----------------------------------------------------------
void Arduino_ST7789::writeData(uint8_t c)
{
  DC_DATA;
  CS_ACTIVE;
  SPI_START;

  writeSPI(c);

  CS_IDLE;
  SPI_END;
}

// ----------------------------------------------------------
void Arduino_ST7789::displayInit(const uint8_t *addr)
{
  uint8_t  numCommands, numArgs;
  uint16_t ms;
  numCommands = pgm_read_byte(addr++);   // Number of commands to follow
  while(numCommands--) {                 // For each command...
    writeCmd(pgm_read_byte(addr++));     //   Read, issue command
    numArgs  = pgm_read_byte(addr++);    //   Number of args to follow
    ms       = numArgs & ST_CMD_DELAY;   //   If hibit set, delay follows args
    numArgs &= ~ST_CMD_DELAY;            //   Mask out delay bit
    while(numArgs--) writeData(pgm_read_byte(addr++));

    if(ms) {
      ms = pgm_read_byte(addr++); // Read post-command delay time (ms)
      if(ms == 255) ms = 500;     // If 255, delay for 500 ms
      nrf_delay_ms(ms);
    }
  }
}

// ----------------------------------------------------------
// Initialization code common to all ST7789 displays
void Arduino_ST7789::commonST7789Init(const uint8_t *cmdList)
{
  _ystart = _xstart = 0;
  _colstart  = _rowstart = 0; // May be overridden in init func

  nrf_gpio_pin_set(dcPin);
  nrf_gpio_cfg_output(dcPin);

#ifndef CS_ALWAYS_LOW
  nrf_gpio_pin_set(csPin);
  nrf_gpio_cfg_output(csPin);
  csMask = 1 << csPin;
#endif

  portSet = &NRF_GPIO->OUTSET;
  portClear = &NRF_GPIO->OUTCLR;
  dcMask = 1 << dcPin;

  nrf_gpio_pin_clear(sckPin);
  nrf_gpio_cfg(sckPin, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_CONNECT,
               NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE);
  nrf_gpio_pin_clear(mosiPin);
  nrf_gpio_cfg_output(mosiPin);

  SPI_OBJ->FREQUENCY = NRF_SPI_FREQ_8M;
  SPI_OBJ->CONFIG = NRF_SPI_MODE_2 << 1;
  SPI_OBJ->PSELSCK = sckPin;
  SPI_OBJ->PSELMOSI = mosiPin;
  SPI_OBJ->ENABLE = 1;

  CS_ACTIVE;
  if(rstPin != NRF_DRV_SPI_PIN_NOT_USED) {
    nrf_gpio_pin_set(rstPin);
    nrf_gpio_cfg_output(rstPin);
    nrf_delay_ms(50);
    nrf_gpio_pin_clear(rstPin);
    nrf_delay_ms(50);
    nrf_gpio_pin_set(rstPin);
    nrf_delay_ms(50);
  }

  if(cmdList) displayInit(cmdList);
}

// ----------------------------------------------------------
void Arduino_ST7789::setRotation(uint8_t m)
{
  writeCmd(ST7789_MADCTL);
  rotation = m & 3;
  switch (rotation) {
  case 0:
     writeData(ST7789_MADCTL_RGB);
     _xstart = _colstart;
     _ystart = _rowstart;
     break;
   case 1:
     writeData(ST7789_MADCTL_MX | ST7789_MADCTL_MV | ST7789_MADCTL_RGB);
     _ystart = (ST7789_FRAME_BUFFER_WIDTH - _width) - _colstart;
     _xstart = _rowstart;
     break;
   case 2:
     writeData(ST7789_MADCTL_MX | ST7789_MADCTL_MY | ST7789_MADCTL_RGB);
     _xstart = (ST7789_FRAME_BUFFER_WIDTH - _width) - _colstart;
     _ystart = (ST7789_FRAME_BUFFER_HEIGHT - _height) - _rowstart;
     break;
   case 3:
     writeData(ST7789_MADCTL_MY | ST7789_MADCTL_MV | ST7789_MADCTL_RGB);
     _ystart = _colstart;
     _xstart = (ST7789_FRAME_BUFFER_HEIGHT - _height) - _rowstart;
     break;
  }
}

// ----------------------------------------------------------
void Arduino_ST7789::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
  uint16_t xs = x0 + _xstart, xe = x1 + _xstart;
  uint16_t ys = y0 + _ystart, ye = y1 + _ystart;
/*
  writeCmd(ST7789_CASET);
  writeData(xs >> 8); writeData(xs & 0xFF);
  writeData(xe >> 8); writeData(xe & 0xFF);

  writeCmd(ST7789_RASET);
  writeData(ys >> 8); writeData(ys & 0xFF);
  writeData(ye >> 8); writeData(ye & 0xFF);

  writeCmd(ST7789_RAMWR);
*/

  // optimized version
  CS_ACTIVE;
  SPI_START;

  DC_COMMAND; writeSPI(ST7789_CASET);
  DC_DATA;
  writeSPI(xs >> 8); writeSPI(xs & 0xFF);
  writeSPI(xe >> 8); writeSPI(xe & 0xFF);

  DC_COMMAND; writeSPI(ST7789_RASET);
  DC_DATA;
  writeSPI(ys >> 8); writeSPI(ys & 0xFF);
  writeSPI(ye >> 8); writeSPI(ye & 0xFF);

  DC_COMMAND; writeSPI(ST7789_RAMWR);

  CS_IDLE;
  SPI_END;
}

// ----------------------------------------------------------
void Arduino_ST7789::pushColor(uint16_t color)
{
  SPI_START;
  DC_DATA;
  CS_ACTIVE;

  writeSPI(color >> 8); writeSPI(color);

  CS_IDLE;
  SPI_END;
}

// ----------------------------------------------------------
void Arduino_ST7789::drawPixel(int16_t x, int16_t y, uint16_t color)
{
  if(x<0 ||x>=_width || y<0 || y>=_height) return;
  setAddrWindow(x,y,x+1,y+1);

  SPI_START;
  DC_DATA;
  CS_ACTIVE;

  writeSPI(color >> 8); writeSPI(color);

  CS_IDLE;
  SPI_END;
}

// ----------------------------------------------------------
void Arduino_ST7789::drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color)
{
  if(x>=_width || y>=_height || h<=0) return;
  if(y+h-1>=_height) h=_height-y;
  setAddrWindow(x, y, x, y+h-1);

  uint8_t hi = color >> 8, lo = color;

  SPI_START;
  DC_DATA;
  CS_ACTIVE;

  uint8_t num8 = h>>3;
  while(num8--) {
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
  }
  num8 = (uint8_t)h & 7;
  while(num8--) { writeSPI(hi); writeSPI(lo); }

  CS_IDLE;
  SPI_END;
}

// ----------------------------------------------------------
void Arduino_ST7789::drawFastHLine(int16_t x, int16_t y, int16_t w,  uint16_t color)
{
  if(x>=_width || y>=_height || w<=0) return;
  if(x+w-1>=_width)  w=_width-x;
  setAddrWindow(x, y, x+w-1, y);

  uint8_t hi = color >> 8, lo = color;

  SPI_START;
  DC_DATA;
  CS_ACTIVE;

  uint8_t num8 = w>>3;
  while(num8--) {
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
  }
  num8 = (uint8_t)w & 7;
  while(num8--) { writeSPI(hi); writeSPI(lo); }

  CS_IDLE;
  SPI_END;
}

// ----------------------------------------------------------
void Arduino_ST7789::fillScreen(uint16_t color)
{
  fillRect(0, 0,  _width, _height, color);
}

// ----------------------------------------------------------
void Arduino_ST7789::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
  if(x>=_width || y>=_height || w<=0 || h<=0) return;
  if(x+w-1>=_width)  w=_width -x;
  if(y+h-1>=_height) h=_height-y;
  setAddrWindow(x, y, x+w-1, y+h-1);

  uint8_t hi = color >> 8, lo = color;

  SPI_START;
  DC_DATA;
  CS_ACTIVE;

  uint32_t num = (uint32_t)w*h;
  uint16_t num16 = num>>4;
  while(num16--) {
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
    writeSPI(hi); writeSPI(lo);
  }
  uint8_t num8 = num & 0xf;
  while(num8--) { writeSPI(hi); writeSPI(lo); }

  CS_IDLE;
  SPI_END;
}

// ----------------------------------------------------------
// draws image from RAM
void Arduino_ST7789::drawImage(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t *img16)
{
  if(x>=_width || y>=_height || w<=0 || h<=0) return;
  setAddrWindow(x, y, x+w-1, y+h-1);

  SPI_START;
  DC_DATA;
  CS_ACTIVE;

  uint32_t num = (uint32_t)w*h;
  uint16_t num16 = num>>3;
  uint8_t *img = (uint8_t *)img16;
  while(num16--) {
    writeSPI(*(img+1)); writeSPI(*(img+0)); img+=2;
    writeSPI(*(img+1)); writeSPI(*(img+0)); img+=2;
    writeSPI(*(img+1)); writeSPI(*(img+0)); img+=2;
    writeSPI(*(img+1)); writeSPI(*(img+0)); img+=2;
    writeSPI(*(img+1)); writeSPI(*(img+0)); img+=2;
    writeSPI(*(img+1)); writeSPI(*(img+0)); img+=2;
    writeSPI(*(img+1)); writeSPI(*(img+0)); img+=2;
    writeSPI(*(img+1)); writeSPI(*(img+0)); img+=2;
  }
  uint8_t num8 = num & 0x7;
  while(num8--) { writeSPI(*(img+1)); writeSPI(*(img+0)); img+=2; }

  CS_IDLE;
  SPI_END;
}

// ----------------------------------------------------------
// draws image from flash (PROGMEM)
void Arduino_ST7789::drawImageF(int16_t x, int16_t y, int16_t w, int16_t h, const uint16_t *img16)
{
  if(x>=_width || y>=_height || w<=0 || h<=0) return;
  setAddrWindow(x, y, x+w-1, y+h-1);

  SPI_START;
  DC_DATA;
  CS_ACTIVE;

  uint32_t num = (uint32_t)w*h;
  uint16_t num16 = num>>3;
  uint8_t *img = (uint8_t *)img16;
  while(num16--) {
    writeSPI(pgm_read_byte(img+1)); writeSPI(pgm_read_byte(img+0)); img+=2;
    writeSPI(pgm_read_byte(img+1)); writeSPI(pgm_read_byte(img+0)); img+=2;
    writeSPI(pgm_read_byte(img+1)); writeSPI(pgm_read_byte(img+0)); img+=2;
    writeSPI(pgm_read_byte(img+1)); writeSPI(pgm_read_byte(img+0)); img+=2;
    writeSPI(pgm_read_byte(img+1)); writeSPI(pgm_read_byte(img+0)); img+=2;
    writeSPI(pgm_read_byte(img+1)); writeSPI(pgm_read_byte(img+0)); img+=2;
    writeSPI(pgm_read_byte(img+1)); writeSPI(pgm_read_byte(img+0)); img+=2;
    writeSPI(pgm_read_byte(img+1)); writeSPI(pgm_read_byte(img+0)); img+=2;
  }
  uint8_t num8 = num & 0x7;
  while(num8--) { writeSPI(pgm_read_byte(img+1)); writeSPI(pgm_read_byte(img+0)); img+=2; }

  CS_IDLE;
  SPI_END;
}

// ----------------------------------------------------------
// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t Arduino_ST7789::Color565(uint8_t r, uint8_t g, uint8_t b)
{
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

// ----------------------------------------------------------
void Arduino_ST7789::invertDisplay(bool mode)
{
  writeCmd(!mode ? ST7789_INVON : ST7789_INVOFF);  // modes inverted?
}

// ----------------------------------------------------------
void Arduino_ST7789::partialDisplay(bool mode)
{
  writeCmd(mode ? ST7789_PTLON : ST7789_NORON);
}

// ----------------------------------------------------------
void Arduino_ST7789::sleepDisplay(bool mode)
{
  writeCmd(mode ? ST7789_SLPIN : ST7789_SLPOUT);
  nrf_delay_ms(5);
}

// ----------------------------------------------------------
void Arduino_ST7789::enableDisplay(bool mode)
{
  writeCmd(mode ? ST7789_DISPON : ST7789_DISPOFF);
}

// ----------------------------------------------------------
void Arduino_ST7789::idleDisplay(bool mode)
{
  writeCmd(mode ? ST7789_IDMON : ST7789_IDMOFF);
}

// ----------------------------------------------------------
void Arduino_ST7789::resetDisplay()
{
  writeCmd(ST7789_SWRESET);
  nrf_delay_ms(5);
}

// ----------------------------------------------------------
void Arduino_ST7789::setScrollArea(uint16_t tfa, uint16_t bfa)
{
  uint16_t vsa = 320-tfa-bfa; // ST7789 320x240 VRAM
  writeCmd(ST7789_VSCRDEF); // SETSCROLLAREA = 0x33
  writeData(tfa >> 8);
  writeData(tfa);
  writeData(vsa >> 8);
  writeData(vsa);
  writeData(bfa >> 8);
  writeData(bfa);
}

// ----------------------------------------------------------
void Arduino_ST7789::setScroll(uint16_t vsp)
{
  writeCmd(ST7789_VSCRSADD); // VSCRSADD = 0x37
  writeData(vsp >> 8);
  writeData(vsp);
}

// ----------------------------------------------------------
void Arduino_ST7789::setPartArea(uint16_t sr, uint16_t er)
{
  writeCmd(ST7789_PTLAR);  // SETPARTAREA = 0x30
  writeData(sr >> 8);
  writeData(sr);
  writeData(er >> 8);
  writeData(er);
}

// ----------------------------------------------------------
// doesn't work
void Arduino_ST7789::setBrightness(uint8_t br)
{
  //writeCmd(ST7789_WRCACE);
  //writeData(0xb1);  // 80,90,b0, or 00,01,02,03
  //writeCmd(ST7789_WRCABCMB);
  //writeData(120);

  //BCTRL=0x20, dd=0x08, bl=0x04
  int val = 0x04;
  writeCmd(ST7789_WRCTRLD);
  writeData(val);
  writeCmd(ST7789_WRDISBV);
  writeData(br);
}

// ----------------------------------------------------------
// 0 - off
// 1 - idle
// 2 - normal
// 4 - display off
void Arduino_ST7789::powerSave(uint8_t mode)
{
  if(mode==0) {
    writeCmd(ST7789_POWSAVE);
    writeData(0xec|3);
    writeCmd(ST7789_DLPOFFSAVE);
    writeData(0xff);
    return;
  }
  int is = (mode&1) ? 0 : 1;
  int ns = (mode&2) ? 0 : 2;
  writeCmd(ST7789_POWSAVE);
  writeData(0xec|ns|is);
  if(mode&4) {
    writeCmd(ST7789_DLPOFFSAVE);
    writeData(0xfe);
  }
}

// ------------------------------------------------
// Input a value 0 to 511 (85*6) to get a color value.
// The colours are a transition R - Y - G - C - B - M - R.
void Arduino_ST7789::rgbWheel(int idx, uint8_t *_r, uint8_t *_g, uint8_t *_b)
{
  idx &= 0x1ff;
  if(idx < 85) { // R->Y
    *_r = 255; *_g = idx * 3; *_b = 0;
    return;
  } else if(idx < 85*2) { // Y->G
    idx -= 85*1;
    *_r = 255 - idx * 3; *_g = 255; *_b = 0;
    return;
  } else if(idx < 85*3) { // G->C
    idx -= 85*2;
    *_r = 0; *_g = 255; *_b = idx * 3;
    return;
  } else if(idx < 85*4) { // C->B
    idx -= 85*3;
    *_r = 0; *_g = 255 - idx * 3; *_b = 255;
    return;
  } else if(idx < 85*5) { // B->M
    idx -= 85*4;
    *_r = idx * 3; *_g = 0; *_b = 255;
    return;
  } else { // M->R
    idx -= 85*5;
    *_r = 255; *_g = 0; *_b = 255 - idx * 3;
   return;
  }
}

uint16_t Arduino_ST7789::rgbWheel(int idx)
{
  uint8_t r,g,b;
  rgbWheel(idx, &r,&g,&b);
  return RGBto565(r,g,b);
}

// ------------------------------------------------