#ifndef BLE_H
#define BLE_H

typedef void (*ble_cmd_cb_t)(const char *cmd);

bool ble_init(ble_cmd_cb_t callback);
void ble_service();

#endif /* BLE_H */
