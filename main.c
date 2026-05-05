/*
 * main.c - Brain Stimulator entry point
 * 
 * Orchestrates PIO SPI for the DAC, ADC bit-bang reads, BLE, safety checks,
 * and PID feedback. Initializes the 48 kHz high-speed interrupt timer.
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/stdio_usb.h"
#include "hardware/timer.h"
#include "dac.h"
#include "waveform.h"
#include "ble.h"
#include "adc_reader.h"
#include "pid.h"
#include "safety.h"

/*  Pin map (Pico 2 W)
 *  ─────────────────────────────────────────────
 *  GP2  – SPI0 SCK   (DAC ch0 / ch1)
 *  GP3  – SPI0 TX
 *  GP4  – SPI0 CS0   (DAC ch0)
 *  GP5  – SPI0 CS1   (DAC ch1)
 *  GP10 – SPI1 SCK   (DAC ch2 / ch3)
 *  GP11 – SPI1 TX
 *  GP12 – SPI1 CS0   (DAC ch2)
 *  GP13 – SPI1 CS1   (DAC ch3)
 *  GP18 – ADC CLK    (MCP3008 SCK)
 *  GP19 – ADC ON/OFF (MCP3008 CS)
 *  GP20 – ADC FB_IN  (MCP3008 DOUT/MISO)
 *  GP21 – ADC FB_OUT (MCP3008 DIN/MOSI)
 *  GP23 – CYW43 (internal)
 *  Available: GP0-1, GP6-9, GP14-17, GP22, GP26-28
 */

bool repeating_timer_callback(struct repeating_timer *t) {
    waveform_update();
    return true;
}

int main() {
    stdio_init_all();

    // Wait for USB serial (up to 10 seconds)
    for (int i = 0; i < 100; i++) {
        if (stdio_usb_connected()) break;
        sleep_ms(100);
    }
    sleep_ms(500);

    printf("\n========================================\n");
    printf("  BrainStimulator                      \n");
    printf("========================================\n\n");

    // Step 1: CYW43
    printf("[BOOT] cyw43_arch_init()...\n");
    if (cyw43_arch_init()) {
        printf("[BOOT] FATAL: CYW43 init failed!\n");
        while(1) sleep_ms(1000);
    }
    printf("[BOOT] CYW43 OK\n");
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    // Step 2: BLE stack + power on
    printf("[BOOT] BLE init...\n");
    ble_init_stack();
    ble_power_on();

    printf("[BOOT] Waiting for BLE...\n");
    uint32_t start = to_ms_since_boot(get_absolute_time());
    while (!ble_is_ready()) {
        async_context_poll(cyw43_arch_async_context());
        async_context_wait_for_work_until(cyw43_arch_async_context(),
            make_timeout_time_ms(10));
        if (to_ms_since_boot(get_absolute_time()) - start > 30000) {
            printf("[BOOT] FATAL: BLE timeout. Halting system for safety.\n");
            while(1) sleep_ms(1000);
        }
    }
    printf("[BOOT] BLE advertising!\n");

    // Step 3: DACs
    printf("[BOOT] DAC init...\n");
    dac_init_all();
    sleep_ms(100);

    // Step 4: Waveform generator
    printf("[BOOT] Waveform init...\n");
    waveform_init();

    // Step 5: External ADC (MCP3008 via bit-banged SPI)
    printf("[BOOT] ADC init...\n");
    adc_reader_init();

    // Step 6: PID controllers
    printf("[BOOT] PID init...\n");
    pid_init();

    // Step 7: Safety monitors
    printf("[BOOT] Safety init...\n");
    safety_init();

    // Configure channel 0 only (single DAC connected)
    waveform_set_frequency(0, 2000.0f);
    waveform_set_amplitude(0, 0.5f);
    waveform_enable(0, false);
    waveform_commit();

    // Start 48 kHz sample timer
    struct repeating_timer timer;
    add_repeating_timer_us(-21, repeating_timer_callback, NULL, &timer);

    printf("========================================\n");
    printf("  System Running!                      \n");
    printf("========================================\n\n");

    // Main event loop – also drives PID at ~20 Hz
    absolute_time_t next_pid = get_absolute_time();
    while (true) {
        async_context_poll(cyw43_arch_async_context());
        async_context_wait_for_work_until(cyw43_arch_async_context(),
            make_timeout_time_ms(10));

        // Run PID update when due
        if (absolute_time_diff_us(next_pid, get_absolute_time()) >= 0) {
            pid_update_all();
            
            // Run safety checks periodically
            safety_check();

            next_pid = delayed_by_ms(next_pid, PID_UPDATE_PERIOD_MS);
        }
    }

    return 0;
}
