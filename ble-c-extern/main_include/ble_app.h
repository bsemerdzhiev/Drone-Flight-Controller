#ifndef BLE_APP_H
#define BLE_APP_H

#include <stdint.h>

extern void rust_ble_receive(uint8_t *data, uint16_t length);

extern void ble_initial_init();

#endif // BLE_APP_H
