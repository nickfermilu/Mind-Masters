/*
 * waveform.c - High-Speed Signal Synthesis
 * 
 * Uses Direct Digital Synthesis (DDS) and lookup tables (sine, triangle, sawtooth)
 * to output audio-rate (48 kHz) stimulus signals via hardware PIO.
 * Double-buffered parameter system ensures artifact-free updates.
 */
#include "waveform.h"
#include "dac.h"
#include "pico/stdlib.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define PI 3.1415926535f
#define TWO_PI (2.0f * PI)

typedef struct {
    uint32_t freq_word;
    float    frequency_hz;
    float    amplitude;
    bool     enabled;
    wave_shape_t shape;
} waveform_params_t;

static waveform_params_t param_bank[2][NUM_CHANNELS];
static volatile uint8_t  active_bank = 0;
#define STAGING_BANK (1 - active_bank)

static uint32_t phase_accum[NUM_CHANNELS];
static uint32_t sample_cnt[NUM_CHANNELS];

static uint16_t sine_table[SINE_TABLE_SIZE];
static uint16_t triangle_table[SINE_TABLE_SIZE];
static uint16_t sawtooth_table[SINE_TABLE_SIZE];

static channel_state_t state_snapshot[NUM_CHANNELS];

static void generate_wave_tables(void) {
    printf("[Waveform] Generating wave tables (%d samples)...\n", SINE_TABLE_SIZE);
    
    for (int i = 0; i < SINE_TABLE_SIZE; i++) {
        // --- SINE ---
        float phase = TWO_PI * (float)i / (float)SINE_TABLE_SIZE;
        float sine_val = sinf(phase);
        sine_table[i] = (uint16_t)(((sine_val + 1.0f) * 0.5f) * 65535.0f);
        
        // --- SAWTOOTH ---
        // Sawtooth goes from 0 at left to full at right
        float saw_val = (float)i / (float)(SINE_TABLE_SIZE - 1);
        sawtooth_table[i] = (uint16_t)(saw_val * 65535.0f);
        
        // --- TRIANGLE ---
        // 0 to SINE_TABLE_SIZE/2 is ramp up, SINE_TABLE_SIZE/2 to end is ramp down
        float tri_val;
        if (i < SINE_TABLE_SIZE / 2) {
            tri_val = (float)i / (float)(SINE_TABLE_SIZE / 2);
        } else {
            tri_val = 1.0f - ((float)(i - SINE_TABLE_SIZE / 2) / (float)(SINE_TABLE_SIZE / 2));
        }
        triangle_table[i] = (uint16_t)(tri_val * 65535.0f);
    }
}

void waveform_init(void) {
    generate_wave_tables();
    
    for (int bank = 0; bank < 2; bank++) {
        for (int i = 0; i < NUM_CHANNELS; i++) {
            param_bank[bank][i].freq_word     = 0;
            param_bank[bank][i].frequency_hz  = 1000.0f;
            param_bank[bank][i].amplitude     = 0.0f;
            param_bank[bank][i].enabled       = false;
            param_bank[bank][i].shape         = WAVE_SHAPE_SINE;
        }
    }
    
    for (int i = 0; i < NUM_CHANNELS; i++) {
        phase_accum[i] = 0;
        sample_cnt[i]  = 0;
    }
    
    active_bank = 0;
    
    for (int i = 0; i < NUM_CHANNELS; i++) waveform_set_frequency(i, 1000.0f);
    waveform_commit();
}

void waveform_set_frequency(uint8_t channel, float freq_hz) {
    if (channel >= NUM_CHANNELS) return;
    if (freq_hz < 0.5f) freq_hz = 0.5f;
    if (freq_hz > 10000.0f) freq_hz = 10000.0f;
    
    double freq_ratio = (double)freq_hz / SAMPLE_RATE;
    uint32_t fw = (uint32_t)(freq_ratio * 4294967296.0);
    
    param_bank[STAGING_BANK][channel].frequency_hz = freq_hz;
    param_bank[STAGING_BANK][channel].freq_word    = fw;
}

void waveform_set_amplitude(uint8_t channel, float amplitude) {
    if (channel >= NUM_CHANNELS) return;
    if (amplitude < 0.0f) amplitude = 0.0f;
    if (amplitude > 1.0f) amplitude = 1.0f;
    param_bank[STAGING_BANK][channel].amplitude = amplitude;
}

void waveform_set_shape(uint8_t channel, wave_shape_t shape) {
    if (channel >= NUM_CHANNELS) return;
    param_bank[STAGING_BANK][channel].shape = shape;
}

void waveform_enable(uint8_t channel, bool enable) {
    if (channel >= NUM_CHANNELS) return;
    param_bank[STAGING_BANK][channel].enabled = enable;
}

void waveform_commit(void) {
    uint8_t new_active = STAGING_BANK;
    for (int i = 0; i < NUM_CHANNELS; i++) {
        param_bank[active_bank][i] = param_bank[new_active][i];
    }
    active_bank = new_active;
}

uint16_t waveform_get_current_value(uint8_t channel) {
    if (channel >= NUM_CHANNELS) return 32768;
    uint8_t bank = active_bank;
    if (!param_bank[bank][channel].enabled) return 32768;
    
    uint8_t table_index = phase_accum[channel] >> 24;
    uint16_t raw_val = 32768;
    
    if (param_bank[bank][channel].shape == WAVE_SHAPE_SINE) {
        raw_val = sine_table[table_index];
    } else if (param_bank[bank][channel].shape == WAVE_SHAPE_TRIANGLE) {
        raw_val = triangle_table[table_index];
    } else if (param_bank[bank][channel].shape == WAVE_SHAPE_SAWTOOTH) {
        raw_val = sawtooth_table[table_index];
    }
    
    float amp = param_bank[bank][channel].amplitude;
    if (amp < 1.0f) {
        int32_t signed_val = (int32_t)raw_val - 32768;
        signed_val = (int32_t)((float)signed_val * amp);
        raw_val = (uint16_t)(signed_val + 32768);
    }
    return raw_val;
}

void waveform_update(void) {
    uint8_t bank = active_bank;
    uint16_t values[NUM_CHANNELS];
    
    for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
        waveform_params_t *p = &param_bank[bank][ch];
        if (!p->enabled) {
            values[ch] = 32768;
            continue;
        }
        
        phase_accum[ch] += p->freq_word;
        uint8_t table_index = phase_accum[ch] >> 24;
        
        uint16_t raw_val = 32768;
        if (p->shape == WAVE_SHAPE_SINE) {
            raw_val = sine_table[table_index];
        } else if (p->shape == WAVE_SHAPE_TRIANGLE) {
            raw_val = triangle_table[table_index];
        } else if (p->shape == WAVE_SHAPE_SAWTOOTH) {
            raw_val = sawtooth_table[table_index];
        }
        
        if (p->amplitude < 1.0f) {
            int32_t signed_val = (int32_t)raw_val - 32768;
            signed_val = (int32_t)((float)signed_val * p->amplitude);
            raw_val = (uint16_t)(signed_val + 32768);
        }
        
        values[ch] = raw_val;
        sample_cnt[ch]++;
    }
    
    dac_write_bus(0, values[0], values[1]);
    dac_write_bus(1, values[2], values[3]);
}

const channel_state_t* waveform_get_state(uint8_t channel) {
    if (channel >= NUM_CHANNELS) return NULL;
    uint8_t bank = active_bank;
    channel_state_t *s = &state_snapshot[channel];
    s->phase_accum  = phase_accum[channel];
    s->freq_word    = param_bank[bank][channel].freq_word;
    s->frequency_hz = param_bank[bank][channel].frequency_hz;
    s->amplitude    = param_bank[bank][channel].amplitude;
    s->enabled      = param_bank[bank][channel].enabled;
    s->shape        = param_bank[bank][channel].shape;
    s->sample_count = sample_cnt[channel];
    return s;
}
