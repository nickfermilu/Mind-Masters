// waveform.h - Waveform generation using DDS
#ifndef WAVEFORM_H
#define WAVEFORM_H

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// CONFIGURATION
// ============================================================================

#define NUM_CHANNELS 4          // 4-channel brain stimulator
#define SINE_TABLE_SIZE 256     // Power of 2 for fast indexing
#define SAMPLE_RATE (1000000.0f / 21.0f)  // Hz - matches 21 Âµs timer period (47619 Hz)

// ============================================================================
// CHANNEL STRUCTURE
// ============================================================================

typedef enum {
    WAVE_SHAPE_SINE,
    WAVE_SHAPE_TRIANGLE,
    WAVE_SHAPE_SAWTOOTH
} wave_shape_t;

typedef struct {
    // DDS state (updated every sample at 48 kHz)
    uint32_t phase_accum;       // 32-bit phase accumulator (wraps around)
    uint32_t freq_word;         // Frequency control word
    
    // User-facing parameters
    float frequency_hz;         // Actual frequency in Hz (1.0 - 10000.0)
    float amplitude;            // Amplitude scaling (0.0 - 1.0)
    bool enabled;               // Channel on/off
    wave_shape_t shape;         // Waveform shape
    
    // Statistics (for debugging)
    uint32_t sample_count;      // How many samples generated
} channel_state_t;

// ============================================================================
// PUBLIC API
// ============================================================================

/**
 * @brief Initialize waveform generator
 * 
 * Call this once at startup. Generates sine lookup table and
 * initializes all channels to safe defaults.
 */
void waveform_init(void);

/**
 * @brief Set channel frequency
 * 
 * @param channel Channel number (0-3)
 * @param freq_hz Frequency in Hz (0.5 - 10000.0)
 * 
 * Calculates and updates the frequency control word.
 * For 1 Hz beat: Set ch0=2000.0, ch1=2001.0
 */
void waveform_set_frequency(uint8_t channel, float freq_hz);

/**
 * @brief Set channel amplitude
 * 
 * @param channel Channel number (0-3)
 * @param amplitude Amplitude (0.0 = off, 1.0 = full scale)
 * 
 * Scales the output waveform. Start with 0.1 for testing,
 * increase to 0.5-1.0 for actual use.
 */
void waveform_set_amplitude(uint8_t channel, float amplitude);

/**
 * @brief Set channel waveform shape
 * 
 * @param channel Channel number (0-3)
 * @param shape Waveform shape (SINE, TRIANGLE, SAWTOOTH)
 */
void waveform_set_shape(uint8_t channel, wave_shape_t shape);

/**
 * @brief Enable or disable a channel
 * 
 * @param channel Channel number (0-3)
 * @param enable true = on, false = off
 * 
 * When disabled, output goes to midpoint (1.65V = 0 current)
 */
void waveform_enable(uint8_t channel, bool enable);

/**
 * @brief Get current output value for a channel
 * 
 * @param channel Channel number (0-3)
 * @return uint16_t Current DAC value (0-65535)
 * 
 * Useful for debugging - see what value is being output
 */
uint16_t waveform_get_current_value(uint8_t channel);

/**
 * @brief Update all channels (call at 48 kHz)
 * 
 * This is the "engine" - call from a timer interrupt or
 * repeating timer callback. Generates one sample for each
 * enabled channel.
 * 
 * @note This function should execute quickly (<10 Âµs)
 */
void waveform_update(void);

/**
 * @brief Commit staged parameter changes (double buffer swap)
 *
 * All parameter changes (frequency, amplitude, enable) are written
 * to an inactive staging buffer.  Call this to atomically swap
 * the active buffer so the waveform ISR picks up all changes
 * simultaneously on the next sample.
 *
 * Call after setting one or more parameters.
 * The PID controller calls this automatically after each update cycle.
 */
void waveform_commit(void);

/**
 * @brief Get channel state (for debugging)
 * 
 * @param channel Channel number (0-3)
 * @return const channel_state_t* Pointer to channel state
 */
const channel_state_t* waveform_get_state(uint8_t channel);

#endif // WAVEFORM_H

