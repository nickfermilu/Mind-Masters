// dac.c — DAC8411 PIO-based SPI driver
//
// Uses two PIO state machines (one per bus) to shift out 24-bit
// SPI frames.  Each SM handles two DAC channels (CS0 / CS1) and
// runs independently, so both buses transfer in parallel.
//
// CPU work per update = 4 × pio_sm_put (< 0.5 µs total).
// PIO work per bus    ≈ 4.5 µs (two 24-bit transfers at ~12.5 MHz).
// Both buses overlap  → all 4 channels done in ~4.5 µs.

#include "dac.h"
#include "dac_spi.pio.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include <stdio.h>

// ============================================================================
// PIO STATE
// ============================================================================

static PIO  dac_pio;                 // PIO block (pio0)
static uint dac_sm[2];               // SM index per bus
static uint dac_pio_offset;          // Program offset in instruction memory

// Shadow registers — last value written per channel
static uint16_t shadow[DAC_NUM_CHANNELS] = {32768, 32768, 32768, 32768};
static bool     ch_initialized[DAC_NUM_CHANNELS];

// ============================================================================
// PUBLIC FUNCTIONS
// ============================================================================

void dac_init_all(void) {
    printf("[DAC] Initializing PIO-based SPI for %d channels...\n",
           DAC_NUM_CHANNELS);

    dac_pio = pio0;

    // Load PIO program (shared by both SMs)
    dac_pio_offset = pio_add_program(dac_pio, &dac_spi_program);

    // Claim two state machines
    dac_sm[0] = pio_claim_unused_sm(dac_pio, true);
    dac_sm[1] = pio_claim_unused_sm(dac_pio, true);

    printf("[DAC] PIO0 SM%d → Bus 0 (GP%d SCK, GP%d DIN, GP%d/GP%d CS)\n",
           dac_sm[0], DAC_SPI0_SCK, DAC_SPI0_MOSI,
           DAC_SPI0_CS0, DAC_SPI0_CS1);
    printf("[DAC] PIO0 SM%d → Bus 1 (GP%d SCK, GP%d DIN, GP%d/GP%d CS)\n",
           dac_sm[1], DAC_SPI1_SCK, DAC_SPI1_MOSI,
           DAC_SPI1_CS0, DAC_SPI1_CS1);

    // Bus 0: channels 0-1
    dac_spi_program_init(dac_pio, dac_sm[0], dac_pio_offset,
                         DAC_SPI0_SCK, DAC_SPI0_MOSI, DAC_SPI0_CS0);

    // Bus 1: channels 2-3
    dac_spi_program_init(dac_pio, dac_sm[1], dac_pio_offset,
                         DAC_SPI1_SCK, DAC_SPI1_MOSI, DAC_SPI1_CS0);

    for (int i = 0; i < DAC_NUM_CHANNELS; i++)
        ch_initialized[i] = true;

    // Drive all outputs to midpoint (1.65 V = zero current)
    dac_write_bus(0, 32768, 32768);
    dac_write_bus(1, 32768, 32768);

    printf("[DAC] PIO SPI ready — 12.5 MHz clock, ~4.5 µs per bus\n");
    printf("[DAC] All outputs set to 1.65 V (midpoint)\n");
}

void dac_write_bus(uint8_t bus, uint16_t val0, uint16_t val1) {
    if (bus > 1) return;

    uint sm = dac_sm[bus];

    // DAC8411 24-bit frame placed at bits 31-8 of 32-bit word:
    //   [PD1=0][PD0=0][D15..D0][X5..X0]
    pio_sm_put_blocking(dac_pio, sm, (uint32_t)val0 << 14);
    pio_sm_put_blocking(dac_pio, sm, (uint32_t)val1 << 14);

    // Update shadow registers
    uint8_t ch0 = bus * 2;
    shadow[ch0]     = val0;
    shadow[ch0 + 1] = val1;
}

void dac_write(uint8_t channel, uint16_t value) {
    if (channel >= DAC_NUM_CHANNELS) return;
    if (!ch_initialized[channel]) return;

    shadow[channel] = value;

    // Must push both channels on the bus (PIO expects a pair)
    uint8_t bus = channel / 2;
    uint8_t ch0 = bus * 2;
    dac_write_bus(bus, shadow[ch0], shadow[ch0 + 1]);
}

void dac_write_voltage(uint8_t channel, float voltage) {
    if (voltage < 0.0f) voltage = 0.0f;
    if (voltage > 3.3f) voltage = 3.3f;
    dac_write(channel, (uint16_t)((voltage / 3.3f) * 65535.0f));
}

void dac_set_midpoint(uint8_t channel) {
    dac_write(channel, 32768);
}

uint16_t dac_get_last_value(uint8_t channel) {
    if (channel >= DAC_NUM_CHANNELS) return 0;
    return shadow[channel];
}

bool dac_is_initialized(uint8_t channel) {
    if (channel >= DAC_NUM_CHANNELS) return false;
    return ch_initialized[channel];
}
