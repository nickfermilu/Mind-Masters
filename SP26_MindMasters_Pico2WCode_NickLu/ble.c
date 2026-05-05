/*
 * ble.c â€” Minimal BLE interface for BrainStimulator
 *
 * Advertises as "BrainStimulator" using Nordic UART Service (NUS).
 * Accepts simple text commands over BLE to control the stimulator:
 *
 *   SHAPE ALL <type>   e.g. "SHAPE ALL SINE" / "TRIANGLE" / "SAWTOOTH"
 *   FREQ ALL <hz>      e.g. "FREQ ALL 1000.0" — set all channels
 *   CURRENT ALL <ma>   e.g. "CURRENT ALL 5.0" — set target current
 *   PID ALL ON         — enable closed-loop current control
 *   STATUS / PIDSTATUS — returns telemetry and loop status
 *
 * Based on Raspberry Pi pico-examples BLE server pattern.
 */

#include "ble.h"
#include "waveform.h"
#include "pid.h"
#include "adc_reader.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/btstack_cyw43.h"
#include "btstack.h"
#include "brain_stimulator.h"   // generated GATT header

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// â”€â”€ Advertising data â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
// Flags + complete local name in primary advertising packet
#define APP_AD_FLAGS 0x06

static uint8_t adv_data[] = {
    // Flags: LE General Discoverable + BR/EDR Not Supported
    0x02, BLUETOOTH_DATA_TYPE_FLAGS, APP_AD_FLAGS,
    // Complete Local Name: "BrainStimulator" (15 chars)
    0x10, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME,
    'B','r','a','i','n','S','t','i','m','u','l','a','t','o','r',
};
static const uint8_t adv_data_len = sizeof(adv_data);

// Scan response: NUS service UUID (sent when scanner asks for more info)
static const uint8_t scan_resp_data[] = {
    // Complete 128-bit service UUID list: Nordic UART Service
    0x11, BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS,
    0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,
    0x93,0xF3,0xA3,0xB5,0x01,0x00,0x40,0x6E,
};

// â”€â”€ State â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
static hci_con_handle_t connection_handle = HCI_CON_HANDLE_INVALID;
static btstack_packet_callback_registration_t hci_event_cb;
static btstack_timer_source_t heartbeat;
static bool ble_ready = false;

// ATT handles from the generated brain_stimulator.h header
#define RX_VALUE_HANDLE ATT_CHARACTERISTIC_6E400002_B5A3_F393_E0A9_E50E24DCCA9E_01_VALUE_HANDLE
#define TX_VALUE_HANDLE ATT_CHARACTERISTIC_6E400003_B5A3_F393_E0A9_E50E24DCCA9E_01_VALUE_HANDLE

// External GATT database (from generated header)
extern uint8_t const profile_data[];

// â”€â”€ Response queue (flow-controlled notifications) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

#define RESP_BUF_SIZE 128
static char    pending_resp[RESP_BUF_SIZE];
static uint16_t pending_resp_len = 0;

static void try_send_pending(void) {
    if (pending_resp_len == 0) return;
    if (connection_handle == HCI_CON_HANDLE_INVALID) {
        printf("[BLE] try_send_pending: no connection, dropping\n");
        pending_resp_len = 0;
        return;
    }
    int result = att_server_notify(connection_handle, TX_VALUE_HANDLE,
                      (const uint8_t *)pending_resp, pending_resp_len);
    printf("[BLE] notify(handle=0x%04x, att=0x%04x, len=%d) => %d\n",
           connection_handle, TX_VALUE_HANDLE, pending_resp_len, result);
    if (result == 0) {
        pending_resp_len = 0;   // sent successfully
    } else {
        // Buffer full â€” request callback when ready
        printf("[BLE] Requesting CAN_SEND_NOW event\n");
        att_server_request_can_send_now_event(connection_handle);
    }
}

static void send_response(const char *msg) {
    if (connection_handle == HCI_CON_HANDLE_INVALID) {
        printf("[BLE] send_response: no connection!\n");
        return;
    }
    uint16_t len = (uint16_t)strlen(msg);
    if (len >= RESP_BUF_SIZE) len = RESP_BUF_SIZE - 1;
    memcpy(pending_resp, msg, len);
    pending_resp_len = len;
    try_send_pending();
}

static void handle_command(const char *raw, uint16_t len) {
    char buf[128];
    uint16_t n = (len < sizeof(buf) - 1) ? len : (uint16_t)(sizeof(buf) - 1);
    memcpy(buf, raw, n);
    buf[n] = '\0';

    // Strip trailing whitespace
    for (int i = (int)n - 1; i >= 0 && (buf[i] == '\r' || buf[i] == '\n' || buf[i] == ' '); i--)
        buf[i] = '\0';

    printf("[BLE] CMD: \"%s\"\n", buf);

    // â”€â”€ STATUS â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    if (strcmp(buf, "STATUS") == 0) {
        char resp[128];
        int pos = 0;
        for (int i = 0; i < 4; i++) {
            const channel_state_t *s = waveform_get_state(i);
            pos += snprintf(resp + pos, sizeof(resp) - pos,
                            "CH%d:%.1fHz A=%.2f ", i,
                            (double)s->frequency_hz, (double)s->amplitude);
        }
        send_response(resp);
        return;
    }

    // â”€â”€ PIDSTATUS â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    if (strcmp(buf, "PIDSTATUS") == 0) {
        char resp[128];
        int pos = 0;
        for (int i = 0; i < 4; i++) {
            const pid_channel_t *p = pid_get_state(i);
            if (!p) continue;
            pos += snprintf(resp + pos, sizeof(resp) - pos,
                            "CH%d:%.1f/%.1fmA %s ",
                            i, (double)p->last_measured_ma,
                            (double)p->target_current_ma,
                            p->enabled ? "ON" : "OFF");
        }
        send_response(resp);
        return;
    }

    // â”€â”€ FREQ ALL <hz> â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    if (strncmp(buf, "FREQ ALL ", 9) == 0) {
        float hz = (float)atof(buf + 9);
        if (hz < 0.5f || hz > 10000.0f) {
            send_response("ERR: 0.5-10000 Hz\n");
            return;
        }
        for (int i = 0; i < 4; i++)
            waveform_set_frequency(i, hz);
        waveform_commit();
        char resp[64];
        snprintf(resp, sizeof(resp), "OK: ALL %.1fHz\n", (double)hz);
        send_response(resp);
        return;
    }

    // â”€â”€ FREQ <ch> <hz> â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    if (strncmp(buf, "FREQ ", 5) == 0) {
        int ch = -1;
        float hz = 0;
        if (sscanf(buf + 5, "%d %f", &ch, &hz) == 2) {
            if (ch < 0 || ch > 3) {
                send_response("ERR: ch 0-3\n");
                return;
            }
            if (hz < 0.5f || hz > 10000.0f) {
                send_response("ERR: 0.5-10000 Hz\n");
                return;
            }
            waveform_set_frequency(ch, hz);
            waveform_commit();
            char resp[64];
            snprintf(resp, sizeof(resp), "OK: CH%d %.1fHz\n", ch, (double)hz);
            send_response(resp);
            return;
        }
    }

    // â”€â”€ CURRENT ALL <mA> â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    if (strncmp(buf, "CURRENT ALL ", 12) == 0) {
        float ma = (float)atof(buf + 12);
        if (ma < 0.0f || ma > 10.0f) {
            send_response("ERR: 0-10 mA\n");
            return;
        }
        for (int i = 0; i < 4; i++)
            pid_set_target_current(i, ma);
        char resp[64];
        snprintf(resp, sizeof(resp), "OK: ALL %.2fmA\n", (double)ma);
        send_response(resp);
        return;
    }

    // â”€â”€ CURRENT <ch> <mA> â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    if (strncmp(buf, "CURRENT ", 8) == 0) {
        int ch = -1;
        float ma = 0;
        if (sscanf(buf + 8, "%d %f", &ch, &ma) == 2) {
            if (ch < 0 || ch > 3) {
                send_response("ERR: ch 0-3\n");
                return;
            }
            if (ma < 0.0f || ma > 10.0f) {
                send_response("ERR: 0-10 mA\n");
                return;
            }
            pid_set_target_current(ch, ma);
            char resp[64];
            snprintf(resp, sizeof(resp), "OK: CH%d %.2fmA\n", ch, (double)ma);
            send_response(resp);
            return;
        }
    }

    // â”€â”€ AMP ALL <val> â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    if (strncmp(buf, "AMP ALL ", 8) == 0) {
        float amp = (float)atof(buf + 8);
        if (amp < 0.0f || amp > 1.0f) {
            send_response("ERR: 0.0-1.0\n");
            return;
        }
        for (int i = 0; i < 4; i++)
            waveform_set_amplitude(i, amp);
        waveform_commit();
        char resp[64];
        snprintf(resp, sizeof(resp), "OK: ALL amp=%.2f\n", (double)amp);
        send_response(resp);
        return;
    }

    // â”€â”€ AMP <ch> <val> â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    if (strncmp(buf, "AMP ", 4) == 0) {
        int ch = -1;
        float amp = 0;
        if (sscanf(buf + 4, "%d %f", &ch, &amp) == 2) {
            if (ch < 0 || ch > 3) {
                send_response("ERR: ch 0-3\n");
                return;
            }
            if (amp < 0.0f || amp > 1.0f) {
                send_response("ERR: 0.0-1.0\n");
                return;
            }
            waveform_set_amplitude(ch, amp);
            waveform_commit();
            char resp[64];
            snprintf(resp, sizeof(resp), "OK: CH%d amp=%.2f\n", ch, (double)amp);
            send_response(resp);
            return;
        }
    }

    // â”€â”€ SHUNT ALL <ohms> â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    if (strncmp(buf, "SHUNT ALL ", 10) == 0) {
        float ohms = (float)atof(buf + 10);
        if (ohms < 0.001f) {
            send_response("ERR: >0 ohms\n");
            return;
        }
        char resp[64];
        snprintf(resp, sizeof(resp), "OK: ALL R=%.1f\n", (double)ohms);
        send_response(resp);
        return;
    }

    // â”€â”€ SHUNT <ch> <ohms> â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    if (strncmp(buf, "SHUNT ", 6) == 0) {
        int ch = -1;
        float ohms = 0;
        if (sscanf(buf + 6, "%d %f", &ch, &ohms) == 2) {
            if (ch < 0 || ch > 3) {
                send_response("ERR: ch 0-3\n");
                return;
            }
            if (ohms < 0.001f) {
                send_response("ERR: >0 ohms\n");
                return;
            }
                        char resp[64];
            snprintf(resp, sizeof(resp), "OK: CH%d R=%.1f\n", ch, (double)ohms);
            send_response(resp);
            return;
        }
    }

    // â”€â”€ GAIN ALL <val> â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    if (strncmp(buf, "GAIN ALL ", 9) == 0) {
        float g = (float)atof(buf + 9);
        if (g < 0.001f) {
            send_response("ERR: >0\n");
            return;
        }
        char resp[64];
        snprintf(resp, sizeof(resp), "OK: ALL G=%.2f\n", (double)g);
        send_response(resp);
        return;
    }

    // â”€â”€ GAIN <ch> <val> â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    if (strncmp(buf, "GAIN ", 5) == 0) {
        int ch = -1;
        float g = 0;
        if (sscanf(buf + 5, "%d %f", &ch, &g) == 2) {
            if (ch < 0 || ch > 3) {
                send_response("ERR: ch 0-3\n");
                return;
            }
            if (g < 0.001f) {
                send_response("ERR: >0\n");
                return;
            }
                        char resp[64];
            snprintf(resp, sizeof(resp), "OK: CH%d G=%.2f\n", ch, (double)g);
            send_response(resp);
            return;
        }
    }

    // â”€â”€ PID ALL ON/OFF â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    if (strcmp(buf, "PID ALL ON") == 0) {
        for (int i = 0; i < 4; i++) pid_enable(i, true);
        send_response("OK: ALL PID ON\n");
        return;
    }
    if (strcmp(buf, "PID ALL OFF") == 0) {
        for (int i = 0; i < 4; i++) pid_enable(i, false);
        send_response("OK: ALL PID OFF\n");
        return;
    }

    // â”€â”€ PID <ch> ON/OFF â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    if (strncmp(buf, "PID ", 4) == 0) {
        int ch = -1;
        char onoff[8] = {0};
        if (sscanf(buf + 4, "%d %7s", &ch, onoff) == 2) {
            if (ch < 0 || ch > 3) {
                send_response("ERR: ch 0-3\n");
                return;
            }
            if (strcmp(onoff, "ON") == 0) {
                pid_enable(ch, true);
                char resp[64];
                snprintf(resp, sizeof(resp), "OK: CH%d PID ON\n", ch);
                send_response(resp);
                return;
            }
            if (strcmp(onoff, "OFF") == 0) {
                pid_enable(ch, false);
                char resp[64];
                snprintf(resp, sizeof(resp), "OK: CH%d PID OFF\n", ch);
                send_response(resp);
                return;
            }
        }
    }

    // â”€â”€ PIDGAINS <ch> <kp> <ki> <kd> â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    if (strncmp(buf, "PIDGAINS ", 9) == 0) {
        int ch = -1;
        float kp = 0, ki = 0, kd = 0;
        if (sscanf(buf + 9, "%d %f %f %f", &ch, &kp, &ki, &kd) == 4) {
            if (ch < 0 || ch > 3) {
                send_response("ERR: ch 0-3\n");
                return;
            }
            pid_set_gains(ch, kp, ki, kd);
            char resp[64];
            snprintf(resp, sizeof(resp), "OK: CH%d PID %.3f/%.3f/%.4f\n",
                     ch, (double)kp, (double)ki, (double)kd);
            send_response(resp);
            return;
        }
    }

    // â”€â”€ MAXDELTA <ch> <val> â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    if (strncmp(buf, "MAXDELTA ", 9) == 0) {
        int ch = -1;
        float md = 0;
        if (sscanf(buf + 9, "%d %f", &ch, &md) == 2) {
            if (ch < 0 || ch > 3) {
                send_response("ERR: ch 0-3\n");
                return;
            }
            pid_set_max_delta(ch, md);
            char resp[64];
            snprintf(resp, sizeof(resp), "OK: CH%d maxD=%.4f\n", ch, (double)md);
            send_response(resp);
            return;
        }
    }

    send_response("ERR: unknown cmd\n");
}

// â”€â”€ ATT callbacks â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

static uint16_t att_read_cb(hci_con_handle_t con, uint16_t att_handle,
                             uint16_t offset, uint8_t *buffer, uint16_t buffer_size) {
    (void)con; (void)att_handle; (void)offset; (void)buffer; (void)buffer_size;
    return 0;
}

static int att_write_cb(hci_con_handle_t con, uint16_t att_handle,
                        uint16_t transaction_mode, uint16_t offset,
                        uint8_t *buffer, uint16_t buffer_size) {
    (void)transaction_mode; (void)offset;
    printf("[BLE] att_write_cb: con=0x%04x att=0x%04x len=%d\n", con, att_handle, buffer_size);
    // Always capture the connection handle from any write
    if (con != HCI_CON_HANDLE_INVALID) {
        connection_handle = con;
    }
    if (att_handle == RX_VALUE_HANDLE) {
        handle_command((const char *)buffer, buffer_size);
    }
    return 0;
}

// â”€â”€ Heartbeat timer (LED blink + status) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

#define HEARTBEAT_PERIOD_MS 1000

static void heartbeat_handler(struct btstack_timer_source *ts) {
    static uint32_t counter = 0;
    counter++;

    // Blink LED every heartbeat
    static bool led_on = false;
    led_on = !led_on;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);

    // Print status every 5 seconds
    if (counter % 5 == 0) {
        printf("\nâ”€â”€â”€ Status â”€â”€â”€ BLE:%s â”€â”€â”€\n",
               ble_ready ? "READY" : "INIT...");
        printf("  CH  FREQ(Hz)   AMP   CURRENT(mA)  TARGET(mA)  PID    OUTPUT\n");
        printf("  â”€â”€  â”€â”€â”€â”€â”€â”€â”€â”€   â”€â”€â”€   â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€  â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€  â”€â”€â”€    â”€â”€â”€â”€â”€â”€\n");
        for (int i = 0; i < 4; i++) {
            const channel_state_t *s = waveform_get_state(i);
            const pid_channel_t *p = pid_get_state(i);
            float measured = p ? (float)p->last_measured_ma : 0.0f;
            float target   = p ? (float)p->target_current_ma : 0.0f;
            float output   = p ? (float)p->output : 0.0f;
            bool  pid_on   = p ? p->enabled : false;
            printf("  %d   %7.1f   %.3f   %7.2f      %7.2f    %s   %.4f\n",
                   i,
                   (double)s->frequency_hz,
                   (double)s->amplitude,
                   (double)measured,
                   (double)target,
                   pid_on ? "ON " : "OFF",
                   (double)output);
        }
    }

    // Restart timer
    btstack_run_loop_set_timer(ts, HEARTBEAT_PERIOD_MS);
    btstack_run_loop_add_timer(ts);
}

// â”€â”€ HCI / ATT packet handler â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

static void packet_handler(uint8_t packet_type, uint16_t channel,
                            uint8_t *packet, uint16_t size) {
    UNUSED(channel);
    UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET) return;

    uint8_t event = hci_event_packet_get_type(packet);

    switch (event) {
        case BTSTACK_EVENT_STATE: {
            uint8_t state = btstack_event_state_get_state(packet);
            printf("[BLE] BTSTACK_EVENT_STATE: %d\n", state);
            if (state != HCI_STATE_WORKING) return;

            bd_addr_t local_addr;
            gap_local_bd_addr(local_addr);
            printf("[BLE] BTstack up and running on %s.\n", bd_addr_to_str(local_addr));

            // Setup advertisements (controller is now ready)
            // Fast advertising: 100ms interval for maximum scan visibility
            uint16_t adv_int_min = 0x00A0;  // 160 * 0.625ms = 100ms
            uint16_t adv_int_max = 0x00A0;  // 160 * 0.625ms = 100ms
            uint8_t adv_type = 0;
            bd_addr_t null_addr;
            memset(null_addr, 0, 6);
            gap_advertisements_set_params(adv_int_min, adv_int_max, adv_type,
                                          0, null_addr, 0x07, 0x00);
            printf("[BLE] adv params set\n");

            assert(adv_data_len <= 31);
            gap_advertisements_set_data(adv_data_len, (uint8_t *)adv_data);
            printf("[BLE] adv data set (%d bytes)\n", adv_data_len);

            gap_scan_response_set_data(sizeof(scan_resp_data), (uint8_t *)scan_resp_data);
            printf("[BLE] scan response set (%d bytes)\n", (int)sizeof(scan_resp_data));

            gap_advertisements_enable(1);
            printf("[BLE] gap_advertisements_enable(1) called\n");

            // Dump raw advertising data for verification
            printf("[BLE] Raw adv data (%d bytes):", adv_data_len);
            for (int i = 0; i < adv_data_len; i++) {
                printf(" %02x", adv_data[i]);
            }
            printf("\n");

            printf("[BLE] Advertising as 'BrainStimulator'\n");
            ble_ready = true;
            break;
        }

        case HCI_EVENT_COMMAND_COMPLETE: {
            uint16_t opcode = hci_event_command_complete_get_command_opcode(packet);
            uint8_t status = packet[5]; // status is first return param
            printf("[BLE] HCI_CMD_COMPLETE: opcode=0x%04x status=0x%02x\n", opcode, status);
            if (status != 0) {
                printf("[BLE] *** COMMAND FAILED! opcode=0x%04x status=0x%02x ***\n", opcode, status);
            }
            break;
        }

        case HCI_EVENT_COMMAND_STATUS:
            printf("[BLE] HCI_EVENT_COMMAND_STATUS: opcode=0x%04x status=0x%02x\n",
                   hci_event_command_status_get_command_opcode(packet),
                   hci_event_command_status_get_status(packet));
            break;

        case HCI_EVENT_LE_META:
            if (hci_event_le_meta_get_subevent_code(packet) ==
                HCI_SUBEVENT_LE_CONNECTION_COMPLETE) {
                connection_handle =
                    hci_subevent_le_connection_complete_get_connection_handle(packet);
                printf("[BLE] Connected (handle 0x%04x)\n", connection_handle);
            }
            break;

        case HCI_EVENT_DISCONNECTION_COMPLETE:
            printf("[BLE] Disconnected\n");
            connection_handle = HCI_CON_HANDLE_INVALID;
            gap_advertisements_enable(1);
            break;

        case ATT_EVENT_CAN_SEND_NOW:
            printf("[BLE] ATT_EVENT_CAN_SEND_NOW\n");
            try_send_pending();
            break;

        default:
            printf("[BLE] event: 0x%02x\n", event);
            break;
    }
}

// â”€â”€ Public API â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

void ble_init_stack(void) {
    printf("[BLE] l2cap_init()...\n");
    l2cap_init();

    printf("[BLE] sm_init()...\n");
    sm_init();

    printf("[BLE] att_server_init()...\n");
    att_server_init(profile_data, att_read_cb, att_write_cb);

    printf("[BLE] ATT handles: RX=0x%04x TX=0x%04x\n", RX_VALUE_HANDLE, TX_VALUE_HANDLE);

    printf("[BLE] Registering HCI event handler...\n");
    hci_event_cb.callback = &packet_handler;
    hci_add_event_handler(&hci_event_cb);

    printf("[BLE] Registering ATT packet handler...\n");
    att_server_register_packet_handler(packet_handler);

    printf("[BLE] Starting heartbeat timer...\n");
    heartbeat.process = &heartbeat_handler;
    btstack_run_loop_set_timer(&heartbeat, HEARTBEAT_PERIOD_MS);
    btstack_run_loop_add_timer(&heartbeat);

    printf("[BLE] Stack init complete.\n");
}

void ble_power_on(void) {
    printf("[BLE] hci_power_control(HCI_POWER_ON)...\n");
    hci_power_control(HCI_POWER_ON);
    printf("[BLE] Power on command sent, waiting for HCI_STATE_WORKING...\n");
}

bool ble_is_ready(void) {
    return ble_ready;
}




