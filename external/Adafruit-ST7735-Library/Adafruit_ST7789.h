#ifndef _ADAFRUIT_ST7789H_
#define _ADAFRUIT_ST7789H_

#include "Adafruit_ST77xx.h"

/// Subclass of ST77XX type display for ST7789 TFT Driver
class Adafruit_ST7789 : public Adafruit_ST77xx {
public:
  Adafruit_ST7789(uint16_t width, uint16_t height, nrf_drv_spi_t* pSpi,
                  uint8_t mosiPin, uint8_t sckPin, uint8_t csPin, uint8_t dcPin,
                  uint8_t rstPin = NRF_DRV_SPI_PIN_NOT_USED);

  void setRotation(uint8_t m);
  void init(nrf_drv_spi_frequency_t frequency = NRF_DRV_SPI_FREQ_8M);

private:
  uint16_t windowWidth;
  uint16_t windowHeight;
};

#endif // _ADAFRUIT_ST7789H_
