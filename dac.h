// dac.h - DAC8411 16-bit PIO DAC driver
#ifndef DAC_H
#define DAC_H

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// DAC8411 SPECIFICATIONS
// ============================================================================

// Device: DAC8411IDCKR (Texas Instruments)
// Resolution: 16-bit
// Output range: 0V to VREF (VREF = 3.3V)
// Interface: SPI via PIO (24-bit frame, CPOL=1)
// Max SPI clock: 50 MHz (PIO runs at ~12.5 MHz)
// Settling time: 6 µs typical

// ============================================================================
// PIN CONFIGURATION
// ============================================================================

// Two buses, 2 channels each.  Each bus shares SCLK and MOSI;
// channels are selected by individual CS (SYNC) pins.
// CS0 and CS1 on each bus MUST be consecutive GPIOs.

#define DAC_NUM_CHANNELS 4

#define DAC_SPI0_SCK   2    // GP2  - SCLK for DAC 0 & 1
#define DAC_SPI0_MOSI  3    // GP3  - DIN  for DAC 0 & 1
#define DAC_SPI0_CS0   4    // GP4  - SYNC DAC 0
#define DAC_SPI0_CS1   5    // GP5  - SYNC DAC 1

#define DAC_SPI1_SCK   10   // GP10 - SCLK for DAC 2 & 3
#define DAC_SPI1_MOSI  11   // GP11 - DIN  for DAC 2 & 3
#define DAC_SPI1_CS0   12   // GP12 - SYNC DAC 2
#define DAC_SPI1_CS1   13   // GP13 - SYNC DAC 3

// ============================================================================
// PUBLIC API
// ============================================================================

/**
 * @brief Initialize all DAC channels via PIO
 *
 * Loads the PIO SPI program, claims 2 state machines (one per bus),
 * and drives all four DAC outputs to midpoint (1.65 V).
 * Call once at startup.
 */
void dac_init_all(void);

/**
 * @brief Write 16-bit value to a single DAC channel
 *
 * @param channel Channel number (0-3)
 * @param value   16-bit value (0-65535)
 *
 * Output voltage = (value / 65535) * 3.3V
 *
 * Internally pushes both channels on the bus so the PIO
 * program receives the pair it expects.
 */
void dac_write(uint8_t channel, uint16_t value);

/**
 * @brief Write two channels on the same bus in one call
 *
 * @param bus  Bus number (0 = channels 0-1, 1 = channels 2-3)
 * @param val0 Value for the first channel on the bus  (ch0 / ch2)
 * @param val1 Value for the second channel on the bus (ch1 / ch3)
 *
 * Preferred in the waveform ISR — pushes both values back-to-back
 * so the PIO processes them without extra overhead.
 */
void dac_write_bus(uint8_t bus, uint16_t val0, uint16_t val1);

/**
 * @brief Write voltage to DAC
 *
 * @param channel Channel number (0-3)
 * @param voltage Voltage (0.0 - 3.3V)
 */
void dac_write_voltage(uint8_t channel, float voltage);

/**
 * @brief Set DAC to midpoint (1.65V = 0 current output)
 */
void dac_set_midpoint(uint8_t channel);

/**
 * @brief Get last written value
 */
uint16_t dac_get_last_value(uint8_t channel);

/**
 * @brief Check if DAC channel is initialized
 */
bool dac_is_initialized(uint8_t channel);

#endif // DAC_H
