#include <Arduino.h>
#include <ArduinoBLE.h>
#include <string.h>
#include "ble.h"

#define BLE_BUFFER_SIZE 20

static BLEService customService(
    "00000000-5EC4-4083-81CD-A10B8D5CF6EC");
static BLECharacteristic customCharacteristic(
    "00000001-5EC4-4083-81CD-A10B8D5CF6EC",
    BLERead | BLEWrite | BLENotify, BLE_BUFFER_SIZE, false);

static ble_cmd_cb_t s_callback = NULL;

bool ble_init(ble_cmd_cb_t callback) {
    s_callback = callback;

    if (!BLE.begin()) return false;

    BLE.setLocalName("BLE-DEVICE");
    BLE.setDeviceName("BLE-DEVICE");
    customService.addCharacteristic(customCharacteristic);
    BLE.addService(customService);
    customCharacteristic.writeValue("ready");
    BLE.advertise();
    return true;
}

void ble_service() {
    BLEDevice central = BLE.central();

    if (central && central.connected()) {
        digitalWrite(LED_BUILTIN, HIGH);

        if (customCharacteristic.written()) {
            int len = customCharacteristic.valueLength();
            const unsigned char *data = customCharacteristic.value();

            char buf[BLE_BUFFER_SIZE + 1];
            int  copy = len < BLE_BUFFER_SIZE ? len : BLE_BUFFER_SIZE;
            memcpy(buf, data, copy);
            buf[copy] = '\0';

            if (s_callback) s_callback(buf);
            customCharacteristic.writeValue("ok");
        }
    } else {
        digitalWrite(LED_BUILTIN, LOW);
    }
}
