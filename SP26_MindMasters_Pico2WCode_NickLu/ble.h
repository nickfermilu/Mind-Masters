#ifndef BLE_H
#define BLE_H

#include <stdbool.h>

// Initialize BLE stack (L2CAP, SM, ATT, event handlers)
// Call AFTER cyw43_arch_init() has succeeded.
void ble_init_stack(void);

// Power on the HCI controller (starts BT firmware download)
void ble_power_on(void);

// Returns true once BLE is fully initialized and advertising
bool ble_is_ready(void);

#endif // BLE_H
