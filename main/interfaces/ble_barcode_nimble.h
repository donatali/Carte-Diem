#ifndef BLE_BARCODE_NIMBLE_H
#define BLE_BARCODE_NIMBLE_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize BLE barcode service using NimBLE stack
 * 
 * @param device_name Name of the BLE device (will appear in BLE scans)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ble_barcode_init(const char *device_name);

/**
 * @brief Send barcode data over BLE
 * 
 * @param barcode_data Null-terminated barcode string
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if not connected, error code otherwise
 */
esp_err_t ble_barcode_send(const char *barcode_data);

/**
 * @brief Check if a BLE client is connected
 * 
 * @return true if connected, false otherwise
 */
bool ble_barcode_is_connected(void);

/**
 * @brief Deinitialize BLE barcode service
 */
void ble_barcode_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // BLE_BARCODE_NIMBLE_H