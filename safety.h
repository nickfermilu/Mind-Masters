#ifndef SAFETY_H
#define SAFETY_H

#include <stdint.h>
#include <stdbool.h>

// Safety limits
#define SAFETY_MAX_CURRENT_MA 10.0f
#define SAFETY_MAX_DC_OFFSET_MA 1.0f

void safety_init(void);
void safety_check(void);
void safety_on_ble_disconnect(void);

#endif // SAFETY_H
