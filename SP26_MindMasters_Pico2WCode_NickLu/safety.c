/*
 * safety.c - Active hardware and software failsafes
 * 
 * Implements hardware watchdog protection, hard-limit overcurrent checks 
 * (10mA absolute threshold), and Exponential Moving Average to prevent DC drift
 * through the tissue. Runs asynchronously from the main wave synthesis.
 */
#include "safety.h"
#include "adc_reader.h"
#include "waveform.h"
#include "hardware/watchdog.h"
#include <stdio.h>
#include <math.h>

// Keep a running exponential moving average for DC offset
static float dc_offset_accum[4] = {0.0f, 0.0f, 0.0f, 0.0f};
// Alpha determines how heavily we low-pass filter the current. 
// A value of 0.05 averages over ~20 samples.
#define DC_ALPHA 0.05f 

void safety_init(void) {
    printf("[SAFETY] Initializing safety monitors...\n");
    // Enable hardware watchdog with 100ms timeout
    watchdog_enable(100, 1);
}

void safety_check(void) {
    // Update hardware watchdog so it doesn't reset the board
    watchdog_update();

    for (int ch = 0; ch < 4; ch++) {
        float current = adc_read_current_ma(ch);
        
        // Calculate DC offset using an Exponential Moving Average (low-pass filter)
        dc_offset_accum[ch] = (DC_ALPHA * current) + ((1.0f - DC_ALPHA) * dc_offset_accum[ch]);
        float dc_offset = fabsf(dc_offset_accum[ch]);

        // 1. Hard Over-Current Cutoff
        if (current > SAFETY_MAX_CURRENT_MA) {
            printf("[SAFETY] CRITICAL: Over-current on ch %d (%.2f mA). Shutting down!\n", ch, current);
            waveform_enable(ch, false);
            waveform_commit();
        }

        // 2. DC Voltage/Offset Check
        if (dc_offset > SAFETY_MAX_DC_OFFSET_MA) {
            printf("[SAFETY] CRITICAL: DC offset too high on ch %d (%.2f mA). Shutting down!\n", ch, dc_offset);
            waveform_enable(ch, false);
            waveform_commit();
        }
    }
}

void safety_on_ble_disconnect(void) {
    printf("[SAFETY] BLE Disconnected! Disabling all output.\n");
    for (int ch = 0; ch < 4; ch++) {
        waveform_enable(ch, false);
    }
    waveform_commit();
}
