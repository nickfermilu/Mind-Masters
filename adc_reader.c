/*
 * adc_reader.c - ADC Feedback Monitor
 * 
 * Communicates with the MCP3008 ADC via software bit-bang SPI.
 * Reads the DC-rectified voltage drop across the shunt resistor to calculate
 * current. Utilized by both the PID closed-loop and safety routines.
 */
#include "adc_reader.h"
#include "pico/stdlib.h"
#include <stdio.h>

// Custom SPI Pin Mapping for MCP3008
#define MCP_CLK  18
#define MCP_CS   19  // ON/OFF
#define MCP_MISO 20  // FB_IN (Pico RX, from MCP DOUT)
#define MCP_MOSI 21  // FB_OUT (Pico TX, to MCP DIN)

// Reference voltage for the ADC
#define VREF_VOLTS 3.3f

void adc_reader_init(void) {
    printf("[ADC] Initializing MCP3008 bit-bang SPI on pins 18-21...\n");
    
    gpio_init(MCP_CLK);  gpio_set_dir(MCP_CLK, GPIO_OUT);
    gpio_init(MCP_CS);   gpio_set_dir(MCP_CS, GPIO_OUT); gpio_put(MCP_CS, 1);
    gpio_init(MCP_MOSI); gpio_set_dir(MCP_MOSI, GPIO_OUT);
    gpio_init(MCP_MISO); gpio_set_dir(MCP_MISO, GPIO_IN);
}

// Software SPI (bit-bang) read for MCP3008
static uint16_t mcp3008_read(uint8_t channel) {
    if (channel > 7) return 0;
    
    gpio_put(MCP_CS, 0); // Enable chip
    
    // Command: Start bit (1) + Single/Diff (1 for Single) + 3-bit channel
    uint8_t command = 0x18 | channel; 
    
    // Send 5-bit command
    for(int i = 0; i < 5; i++) {
        gpio_put(MCP_MOSI, (command >> (4 - i)) & 1);
        gpio_put(MCP_CLK, 1); sleep_us(1);
        gpio_put(MCP_CLK, 0); sleep_us(1);
    }
    
    uint16_t result = 0;
    
    // Read 1 null bit + 10 data bits
    for(int i = 0; i < 11; i++) {
        gpio_put(MCP_CLK, 1); sleep_us(1);
        result = (result << 1) | gpio_get(MCP_MISO);
        gpio_put(MCP_CLK, 0); sleep_us(1);
    }
    
    gpio_put(MCP_CS, 1); // Disable chip
    
    return result & 0x3FF; // Mask to 10 bits
}

float adc_read_current_ma(uint8_t channel) {
    uint16_t raw = mcp3008_read(channel);
    
    // Convert 10-bit raw value (0-1023) to Voltage
    float voltage = ((float)raw / 1023.0f) * VREF_VOLTS;
    
    // CALIBRATION: Assuming 1V = 1mA for the rectified signal. 
    // Adjust this multiplier based on your actual resistor/rectifier values!
    float current_ma = voltage * 1.0f; 
    
    return current_ma;
}
