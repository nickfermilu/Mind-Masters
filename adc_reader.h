#ifndef ADC_READER_H
#define ADC_READER_H

#include <stdint.h>

// Initializes the MCP3008 SPI GPIO pins
void adc_reader_init(void);

// Reads the rectified DC current from the given channel (0-3)
float adc_read_current_ma(uint8_t channel);

#endif // ADC_READER_H
