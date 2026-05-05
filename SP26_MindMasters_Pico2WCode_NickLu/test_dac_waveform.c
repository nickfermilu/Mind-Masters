// test_dac_waveform.c - Complete test with DAC output
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "dac.h"
#include "waveform.h"

// ============================================================================
// 48 kHz SAMPLE TIMER
// ============================================================================

// This callback is called every 20.8 µs (48 kHz)
bool repeating_timer_callback(struct repeating_timer *t) {
    // Update all channels (generates samples and writes to DACs)
    waveform_update();
    return true;  // Keep repeating
}

// ============================================================================
// MAIN PROGRAM
// ============================================================================

int main() {
    // Initialize stdio (for printf over USB)
    stdio_init_all();
    sleep_ms(2000);  // Wait for USB connection
    
    printf("\n");
    printf("========================================\n");
    printf("  DAC + Waveform Test Program          \n");
    printf("========================================\n\n");
    
    // ========================================
    // STEP 1: Initialize hardware
    // ========================================
    
    printf("Step 1: Initializing DACs...\n");
    dac_init_all();
    printf("\n");
    
    printf("Step 2: Initializing waveform generator...\n");
    waveform_init();
    printf("\n");
    
    // ========================================
    // STEP 2: Configure channels
    // ========================================
    
    printf("Step 3: Configuring channels for temporal interference...\n");
    
    // Channel 0: 2000 Hz
    waveform_set_frequency(0, 2000.0f);
    waveform_set_amplitude(0, 0.5f);  // Start at 50% amplitude
    
    // Channel 1: 2001 Hz (1 Hz beat with channel 0)
    waveform_set_frequency(1, 2001.0f);
    waveform_set_amplitude(1, 0.5f);
    
    // Channel 2: 2000 Hz
    waveform_set_frequency(2, 2000.0f);
    waveform_set_amplitude(2, 0.5f);
    
    // Channel 3: 2001 Hz (1 Hz beat with channel 2)
    waveform_set_frequency(3, 2001.0f);
    waveform_set_amplitude(3, 0.5f);
    
    printf("\n");
    
    // ========================================
    // STEP 3: Start sample timer
    // ========================================
    
    printf("Step 4: Starting 48 kHz sample timer...\n");
    struct repeating_timer timer;
    
    // Period = 1/48000 = 20.833 µs
    // We use negative value for "as soon as possible" scheduling
    add_repeating_timer_us(-21, repeating_timer_callback, NULL, &timer);
    
    printf("Sample timer started!\n");
    printf("\n");
    
    // ========================================
    // STEP 4: Enable channels
    // ========================================
    
    printf("Step 5: Enabling all channels...\n");
    printf("You should now see sine waves on all 4 DAC outputs!\n");
    printf("Channels 0&1 and 2&3 should have 1 Hz beat frequency.\n");
    printf("\n");
    
    waveform_enable(0, true);
    waveform_enable(1, true);
    waveform_enable(2, true);
    waveform_enable(3, true);
    
    printf("========================================\n");
    printf("  System Running                        \n");
    printf("========================================\n\n");
    
    // ========================================
    // MAIN LOOP: Status monitoring
    // ========================================
    
    printf("Monitoring status (Ctrl+C to stop)...\n\n");
    
    uint32_t last_print_time = 0;
    
    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        
        // Print status every 1 second
        if (now - last_print_time >= 1000) {
            printf("Status: ");
            
            for (int ch = 0; ch < 4; ch++) {
                const channel_state_t *state = waveform_get_state(ch);
                uint16_t dac_val = dac_get_last_value(ch);
                
                printf("Ch%d:%.1fHz,%u ", 
                       ch, 
                       state->frequency_hz,
                       dac_val);
            }
            
            printf("\n");
            last_print_time = now;
        }
        
        sleep_ms(100);
    }
    
    return 0;
}
