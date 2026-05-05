// pid.h - Per-channel PID current controller
#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// PID CONTROLLER FOR CURRENT REGULATION
// ============================================================================
//
// Each stimulation channel has its own independent PID controller.
//
// Signal flow:
//   Target current (mA)  ──► PID ──► Waveform amplitude ──► DAC ──► Amp ──► Brain
//                                                                        │
//                          PID ◄── ADC ◄── Isolation ◄── Shunt ◄─────────┘
//
// The PID adjusts the waveform amplitude to maintain the target current.
// Updates are rate-limited to prevent patient discomfort from rapid changes.
//
// ============================================================================

#define PID_NUM_CHANNELS      4

// Default PID tuning (conservative for patient safety)
#define PID_DEFAULT_KP           0.5f      // Proportional gain
#define PID_DEFAULT_KI           0.1f      // Integral gain
#define PID_DEFAULT_KD           0.01f     // Derivative gain
#define PID_DEFAULT_TARGET_MA    2.0f      // Target current (mA)
#define PID_DEFAULT_OUTPUT_MIN   0.0f      // Minimum amplitude output
#define PID_DEFAULT_OUTPUT_MAX   1.0f      // Maximum amplitude output
#define PID_DEFAULT_INTEGRAL_LIM 0.5f      // Anti-windup integral clamp
#define PID_DEFAULT_MAX_DELTA    0.02f     // Max amplitude change per update

// Update rate — 20 Hz (50 ms).  Slow enough for patient comfort,
// fast enough for stable current regulation.
#define PID_UPDATE_PERIOD_MS     50

// Safety: absolute maximum target current (mA)
#define PID_MAX_TARGET_MA        10.0f

// ============================================================================
// DATA STRUCTURES
// ============================================================================

typedef struct {
    // ── Gains ──
    float kp;
    float ki;
    float kd;

    // ── Setpoint ──
    float target_current_ma;

    // ── Internal state ──
    float integral;           // Accumulated integral term
    float prev_error;         // Previous error (for derivative)
    float output;             // Current output: 0.0–1.0 → waveform amplitude

    // ── Limits ──
    float integral_limit;     // Anti-windup clamp
    float output_min;
    float output_max;
    float max_delta;          // Max output change per update (rate limiter)

    // ── Monitoring ──
    float last_measured_ma;   // Last ADC reading (mA)
    float last_error;         // Last error value

    bool enabled;             // When false, amplitude is manual
} pid_channel_t;

// ============================================================================
// PUBLIC API
// ============================================================================

/** Initialize all PID channels with default (safe) parameters */
void pid_init(void);

/**
 * @brief Run one PID update for a single channel.
 * @param channel  0–3
 * @param measured_current_ma  Current reading from ADC (mA)
 */
void pid_update(uint8_t channel, float measured_current_ma);

/**
 * @brief Run PID update for ALL enabled channels.
 *
 * Reads ADC for each channel, computes PID, updates waveform
 * amplitude, and commits the waveform double buffer.
 *
 * Call this every PID_UPDATE_PERIOD_MS from the main loop.
 */
void pid_update_all(void);

// ── Configuration ──────────────────────────────────────────

/** Set target output current (mA). Clamped to 0–PID_MAX_TARGET_MA. */
void pid_set_target_current(uint8_t channel, float target_ma);

/** Set PID gains (Kp, Ki, Kd) */
void pid_set_gains(uint8_t channel, float kp, float ki, float kd);

/** Set max amplitude change per PID update (comfort rate-limiter) */
void pid_set_max_delta(uint8_t channel, float max_delta);

/** Enable/disable PID for a channel. Resets state on enable. */
void pid_enable(uint8_t channel, bool enable);

/** Reset PID integrator and derivative state */
void pid_reset(uint8_t channel);

// ── Getters ─────────────────────────────────────────────────

float pid_get_target_current(uint8_t channel);
float pid_get_measured_current(uint8_t channel);
float pid_get_output(uint8_t channel);
bool  pid_is_enabled(uint8_t channel);
const pid_channel_t* pid_get_state(uint8_t channel);

#endif // PID_H
