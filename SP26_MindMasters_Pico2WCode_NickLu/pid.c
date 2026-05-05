// pid.c - Per-channel PID current controller implementation
#include "pid.h"
#include "adc_reader.h"
#include "waveform.h"
#include <stdio.h>
#include <math.h>

// ============================================================================
// PRIVATE DATA
// ============================================================================

static pid_channel_t pid_channels[PID_NUM_CHANNELS];

// ============================================================================
// PUBLIC FUNCTIONS
// ============================================================================

void pid_init(void) {
    printf("[PID] Initializing %d PID controllers...\n", PID_NUM_CHANNELS);

    for (int i = 0; i < PID_NUM_CHANNELS; i++) {
        pid_channel_t *p = &pid_channels[i];

        p->kp = PID_DEFAULT_KP;
        p->ki = PID_DEFAULT_KI;
        p->kd = PID_DEFAULT_KD;
        p->target_current_ma = PID_DEFAULT_TARGET_MA;

        p->integral   = 0.0f;
        p->prev_error = 0.0f;
        p->output     = 0.0f;

        p->integral_limit = PID_DEFAULT_INTEGRAL_LIM;
        p->output_min     = PID_DEFAULT_OUTPUT_MIN;
        p->output_max     = PID_DEFAULT_OUTPUT_MAX;
        p->max_delta      = PID_DEFAULT_MAX_DELTA;

        p->last_measured_ma = 0.0f;
        p->last_error       = 0.0f;

        p->enabled = false;   // Disabled by default — enable via BLE

        printf("[PID] Ch%d: Kp=%.2f Ki=%.2f Kd=%.3f target=%.1f mA\n",
               i, (double)p->kp, (double)p->ki, (double)p->kd,
               (double)p->target_current_ma);
    }

    printf("[PID] Update rate: %d Hz (%d ms period)\n",
           1000 / PID_UPDATE_PERIOD_MS, PID_UPDATE_PERIOD_MS);
    printf("[PID] Max delta: %.4f per update (rate-limited for comfort)\n",
           (double)PID_DEFAULT_MAX_DELTA);
    printf("[PID] All channels DISABLED — enable via BLE: PID <ch> ON\n");
}

void pid_update(uint8_t channel, float measured_current_ma) {
    if (channel >= PID_NUM_CHANNELS) return;
    pid_channel_t *p = &pid_channels[channel];

    if (!p->enabled) return;

    p->last_measured_ma = measured_current_ma;

    // ── Error ───────────────────────────────────────────────
    float error = p->target_current_ma - measured_current_ma;
    p->last_error = error;

    // ── Proportional ────────────────────────────────────────
    float p_term = p->kp * error;

    // ── Integral (with anti-windup) ─────────────────────────
    p->integral += p->ki * error;
    if (p->integral >  p->integral_limit) p->integral =  p->integral_limit;
    if (p->integral < -p->integral_limit) p->integral = -p->integral_limit;

    // ── Derivative ──────────────────────────────────────────
    float d_term = p->kd * (error - p->prev_error);
    p->prev_error = error;

    // ── Raw output ──────────────────────────────────────────
    float raw_output = p_term + p->integral + d_term;

    // ── Rate limiting (patient comfort) ─────────────────────
    // Clamp the change per update so the amplitude ramps
    // gently.  At default max_delta=0.02 and 20 Hz, it takes
    // ~2.5 s to go from 0 → full amplitude.
    float delta = raw_output - p->output;
    if (delta >  p->max_delta) delta =  p->max_delta;
    if (delta < -p->max_delta) delta = -p->max_delta;
    float new_output = p->output + delta;

    // ── Output clamping ─────────────────────────────────────
    if (new_output > p->output_max) new_output = p->output_max;
    if (new_output < p->output_min) new_output = p->output_min;

    p->output = new_output;

    // Apply to waveform amplitude (writes to staging buffer)
    waveform_set_amplitude(channel, new_output);
}

void pid_update_all(void) {
    bool any_updated = false;

    for (uint8_t ch = 0; ch < PID_NUM_CHANNELS; ch++) {
        if (!pid_channels[ch].enabled) continue;

        // Read measured current from ADC feedback
        float measured = adc_read_current_ma(ch);

        // Run PID controller
        pid_update(ch, measured);
        any_updated = true;
    }

    // Atomically commit the updated amplitudes to the waveform ISR
    if (any_updated) {
        waveform_commit();
    }
}

// ── Configuration ───────────────────────────────────────────

void pid_set_target_current(uint8_t channel, float target_ma) {
    if (channel >= PID_NUM_CHANNELS) return;
    if (target_ma < 0.0f) target_ma = 0.0f;
    if (target_ma > PID_MAX_TARGET_MA) target_ma = PID_MAX_TARGET_MA;
    pid_channels[channel].target_current_ma = target_ma;
    printf("[PID] Ch%d: Target = %.2f mA\n", channel, (double)target_ma);
}

void pid_set_gains(uint8_t channel, float kp, float ki, float kd) {
    if (channel >= PID_NUM_CHANNELS) return;
    pid_channels[channel].kp = kp;
    pid_channels[channel].ki = ki;
    pid_channels[channel].kd = kd;
    printf("[PID] Ch%d: Kp=%.3f Ki=%.3f Kd=%.4f\n",
           channel, (double)kp, (double)ki, (double)kd);
}

void pid_set_max_delta(uint8_t channel, float max_delta) {
    if (channel >= PID_NUM_CHANNELS) return;
    if (max_delta < 0.001f) max_delta = 0.001f;
    pid_channels[channel].max_delta = max_delta;
    printf("[PID] Ch%d: Max delta = %.4f/update\n", channel, (double)max_delta);
}

void pid_enable(uint8_t channel, bool enable) {
    if (channel >= PID_NUM_CHANNELS) return;

    if (enable && !pid_channels[channel].enabled) {
        // Reset integrator and derivative state on enable
        pid_reset(channel);
    }

    pid_channels[channel].enabled = enable;
    printf("[PID] Ch%d: %s\n", channel, enable ? "ENABLED" : "DISABLED");
}

void pid_reset(uint8_t channel) {
    if (channel >= PID_NUM_CHANNELS) return;
    pid_channels[channel].integral   = 0.0f;
    pid_channels[channel].prev_error = 0.0f;
    // Keep current output to avoid amplitude jumps
    printf("[PID] Ch%d: State reset (integral/derivative cleared)\n", channel);
}

// ── Getters ─────────────────────────────────────────────────

float pid_get_target_current(uint8_t channel) {
    if (channel >= PID_NUM_CHANNELS) return 0.0f;
    return pid_channels[channel].target_current_ma;
}

float pid_get_measured_current(uint8_t channel) {
    if (channel >= PID_NUM_CHANNELS) return 0.0f;
    return pid_channels[channel].last_measured_ma;
}

float pid_get_output(uint8_t channel) {
    if (channel >= PID_NUM_CHANNELS) return 0.0f;
    return pid_channels[channel].output;
}

bool pid_is_enabled(uint8_t channel) {
    if (channel >= PID_NUM_CHANNELS) return false;
    return pid_channels[channel].enabled;
}

const pid_channel_t* pid_get_state(uint8_t channel) {
    if (channel >= PID_NUM_CHANNELS) return NULL;
    return &pid_channels[channel];
}
