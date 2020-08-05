// Fast ST7789 IPS 240x240 SPI display library
// (c) 2019 by Pawel A. Hernik
// Updated to work with nRF5 SDK in 2020 by Adam Green (https://github.com/adamgreen)

#ifndef _ST7789_FAST_H_
#define _ST7789_FAST_H_

// ------------------------------
// Set to 1 for LCD boards where CS pin is internally connected to the ground and 0 otherwise.
#define CS_ALWAYS_LOW 1

// Which SPI Master instance (0 or 1) should be used with the screen.
#define ST7789_SPI_INSTANCE 0
// ------------------------------


#include <nrf_drv_spi.h>
#include <Adafruit_GFX.h>
#include "../RREFont/RREFont.h"

// Macro to convert AVR pgm_read_*() calls to simple dereferences on ARM.
#define pgm_read_word(ADDR) (*(uint16_t*)(ADDR))
#define pgm_read_byte(ADDR) (*(uint8_t*)(ADDR))

// The dimensions of the frame buffer on the ST7789. The LCD can be smaller than these dimensions but not larger.
#define ST7789_FRAME_BUFFER_WIDTH  240
#define ST7789_FRAME_BUFFER_HEIGHT 320


#define ST_CMD_DELAY   0x80

#define ST7789_NOP     0x00
#define ST7789_SWRESET 0x01

#define ST7789_SLPIN   0x10  // sleep on
#define ST7789_SLPOUT  0x11  // sleep off
#define ST7789_PTLON   0x12  // partial on
#define ST7789_NORON   0x13  // partial off
#define ST7789_INVOFF  0x20  // invert off
#define ST7789_INVON   0x21  // invert on
#define ST7789_DISPOFF 0x28  // display off
#define ST7789_DISPON  0x29  // display on
#define ST7789_IDMOFF  0x38  // idle off
#define ST7789_IDMON   0x39  // idle on

#define ST7789_CASET   0x2A
#define ST7789_RASET   0x2B
#define ST7789_RAMWR   0x2C
#define ST7789_RAMRD   0x2E

#define ST7789_COLMOD  0x3A
#define ST7789_MADCTL  0x36

#define ST7789_PTLAR    0x30   // partial start/end
#define ST7789_VSCRDEF  0x33   // SETSCROLLAREA
#define ST7789_VSCRSADD 0x37

#define ST7789_WRDISBV  0x51
#define ST7789_WRCTRLD  0x53
#define ST7789_WRCACE   0x55
#define ST7789_WRCABCMB 0x5e

#define ST7789_POWSAVE    0xbc
#define ST7789_DLPOFFSAVE 0xbd

// bits in MADCTL
#define ST7789_MADCTL_MY  0x80
#define ST7789_MADCTL_MX  0x40
#define ST7789_MADCTL_MV  0x20
#define ST7789_MADCTL_ML  0x10
#define ST7789_MADCTL_RGB 0x00


// Color definitions
#define TFT_BLACK 0x0000
#define TFT_WHITE 0xFFFF
#define TFT_RED 0xF800
#define TFT_GREEN 0x07E0
#define TFT_BLUE 0x001F
#define TFT_CYAN 0x07FF
#define TFT_MAGENTA 0xF81F
#define TFT_YELLOW 0xFFE0
#define TFT_ORANGE 0xFC00

#define RGBto565(r,g,b) ((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3))

class Arduino_ST7789 : public Adafruit_GFX, public IFillRect
{
 public:
  Arduino_ST7789(uint16_t width, uint16_t height, uint16_t columnOffset, uint16_t rowOffset,
                 uint8_t MOSI, uint8_t SCK, uint8_t DC,
                 uint8_t RST = NRF_DRV_SPI_PIN_NOT_USED, uint8_t CS = NRF_DRV_SPI_PIN_NOT_USED);

  void init();
  void begin() { init(); }
  void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
  void pushColor(uint16_t color);
  void fillScreen(uint16_t color=TFT_BLACK);
  void clearScreen() { fillScreen(TFT_BLACK); }
  void cls() { fillScreen(TFT_BLACK); }
  void drawPixel(int16_t x, int16_t y, uint16_t color);
  void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
  void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
  void drawImage(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t *img);
  void drawImageF(int16_t x, int16_t y, int16_t w, int16_t h, const uint16_t *img16);
  void drawImageF(int16_t x, int16_t y, const uint16_t *img16) { drawImageF(x,y,pgm_read_word(img16),pgm_read_word(img16+1),img16+3); }
  void setRotation(uint8_t r);
  void invertDisplay(bool mode);
  void partialDisplay(bool mode);
  void sleepDisplay(bool mode);
  void enableDisplay(bool mode);
  void idleDisplay(bool mode);
  void resetDisplay();
  void setScrollArea(uint16_t tfa, uint16_t bfa);
  void setScroll(uint16_t vsp);
  void setPartArea(uint16_t sr, uint16_t er);
  void setBrightness(uint8_t br);
  void powerSave(uint8_t mode);

  uint16_t Color565(uint8_t r, uint8_t g, uint8_t b);
  uint16_t color565(uint8_t r, uint8_t g, uint8_t b) { return Color565(r, g, b); }
  void rgbWheel(int idx, uint8_t *_r, uint8_t *_g, uint8_t *_b);
  uint16_t rgbWheel(int idx);

 protected:
  uint8_t  _colstart, _rowstart, _xstart, _ystart;

  void displayInit(const uint8_t *addr);
  void writeSPI(uint8_t);
  void writeCmd(uint8_t c);
  void writeData(uint8_t d);
  void commonST7789Init(const uint8_t *cmdList);

 private:
  volatile uint32_t*    portSet;
  volatile uint32_t*    portClear;
  uint32_t              csMask;
  uint32_t              dcMask;
  uint8_t               csPin;
  uint8_t               dcPin;
  uint8_t               rstPin;
  uint8_t               mosiPin;
  uint8_t               sckPin;
};

#endif
