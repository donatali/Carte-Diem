#include "ble_barcode_nimble.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include <string.h>

#define TAG "BLE_BARCODE"

// Custom UUIDs (128-bit, in reverse byte order for NimBLE)
// Service UUID: Using a custom service UUID
static const ble_uuid128_t gatt_svr_svc_barcode_uuid = 
    BLE_UUID128_INIT(0x4f, 0xb1, 0x76, 0xb6, 0x4f, 0x79, 0x89, 0xb2,
                     0x89, 0x45, 0x7e, 0x1d, 0x80, 0x6c, 0x6b, 0xe3);

// UPC Characteristic UUID: e36b6c81-1d7e-4589-b289-794fb676b14f
static const ble_uuid128_t gatt_svr_chr_upc_uuid = 
    BLE_UUID128_INIT(0x4f, 0xb1, 0x76, 0xb6, 0x4f, 0x79, 0x89, 0xb2,
                     0x89, 0x45, 0x7e, 0x1d, 0x81, 0x6c, 0x6b, 0xe3);

// Optional RX characteristic (for receiving data from client if needed)
static const ble_uuid128_t gatt_svr_chr_rx_uuid = 
    BLE_UUID128_INIT(0x4f, 0xb1, 0x76, 0xb6, 0x4f, 0x79, 0x89, 0xb2,
                     0x89, 0x45, 0x7e, 0x1d, 0x82, 0x6c, 0x6b, 0xe3);

// BLE state
static bool ble_connected = false;
static uint16_t ble_conn_handle = 0;
static uint16_t upc_char_handle;
static char device_name[32] = "ESP32_Barcode";

// Forward declarations
static int gatt_svr_chr_access_barcode(uint16_t conn_handle, uint16_t attr_handle,
                                       struct ble_gatt_access_ctxt *ctxt, void *arg);
static void ble_advertise(void);

// GATT service definition
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        // Barcode Service
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_barcode_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                // UPC Characteristic (device sends barcode data to client)
                .uuid = &gatt_svr_chr_upc_uuid.u,
                .access_cb = gatt_svr_chr_access_barcode,
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &upc_char_handle,
            },
            {
                // RX Characteristic (optional - device receives data from client)
                .uuid = &gatt_svr_chr_rx_uuid.u,
                .access_cb = gatt_svr_chr_access_barcode,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                0, // No more characteristics in this service
            }
        },
    },
    {
        0, // No more services
    },
};

static int gatt_svr_chr_access_barcode(uint16_t conn_handle, uint16_t attr_handle,
                                       struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            ESP_LOGI(TAG, "GATT read characteristic");
            return 0;

        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            ESP_LOGI(TAG, "GATT write characteristic, len=%d", ctxt->om->om_len);
            // Handle received data if needed
            return 0;

        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_gap_conn_desc desc;
    int rc;

    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            ESP_LOGI(TAG, "Connection %s; status=%d",
                     event->connect.status == 0 ? "established" : "failed",
                     event->connect.status);

            if (event->connect.status == 0) {
                rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
                if (rc == 0) {
                    ESP_LOGI(TAG, "Connected to %02x:%02x:%02x:%02x:%02x:%02x",
                             desc.peer_id_addr.val[5], desc.peer_id_addr.val[4],
                             desc.peer_id_addr.val[3], desc.peer_id_addr.val[2],
                             desc.peer_id_addr.val[1], desc.peer_id_addr.val[0]);
                    ble_connected = true;
                    ble_conn_handle = event->connect.conn_handle;
                }
            } else {
                // Connection failed; resume advertising
                ble_advertise();
            }
            return 0;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "Disconnect; reason=%d", event->disconnect.reason);
            ble_connected = false;
            ble_conn_handle = 0;
            
            // Resume advertising
            ble_advertise();
            return 0;

        case BLE_GAP_EVENT_CONN_UPDATE:
            ESP_LOGI(TAG, "Connection updated; status=%d", event->conn_update.status);
            return 0;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            ESP_LOGI(TAG, "Advertise complete; reason=%d", event->adv_complete.reason);
            ble_advertise();
            return 0;

        case BLE_GAP_EVENT_MTU:
            ESP_LOGI(TAG, "MTU update event; conn_handle=%d cid=%d mtu=%d",
                     event->mtu.conn_handle,
                     event->mtu.channel_id,
                     event->mtu.value);
            return 0;

        case BLE_GAP_EVENT_SUBSCRIBE:
            ESP_LOGI(TAG, "Subscribe event; conn_handle=%d attr_handle=%d "
                          "reason=%d prevn=%d curn=%d previ=%d curi=%d",
                     event->subscribe.conn_handle,
                     event->subscribe.attr_handle,
                     event->subscribe.reason,
                     event->subscribe.prev_notify,
                     event->subscribe.cur_notify,
                     event->subscribe.prev_indicate,
                     event->subscribe.cur_indicate);
            return 0;
    }

    return 0;
}

static void ble_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    // Set the advertisement data
    memset(&fields, 0, sizeof fields);
    
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting advertisement data; rc=%d", rc);
        return;
    }

    // Set scan response data with UUID
    memset(&fields, 0, sizeof fields);
    fields.uuids128 = &gatt_svr_svc_barcode_uuid;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting scan response data; rc=%d", rc);
        // Continue anyway - scan response is optional
    }

    // Begin advertising
    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    
    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                          &adv_params, ble_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error enabling advertisement; rc=%d", rc);
        return;
    }
    
    ESP_LOGI(TAG, "Advertising started");
}

static void ble_on_sync(void)
{
    int rc;
    
    // Make sure we have proper identity address set (public preferred)
    rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error ensuring address; rc=%d", rc);
        return;
    }

    // Print device address
    uint8_t addr_type;
    uint8_t addr[6];
    rc = ble_hs_id_infer_auto(0, &addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error determining address type; rc=%d", rc);
    } else {
        rc = ble_hs_id_copy_addr(addr_type, addr, NULL);
        if (rc == 0) {
            ESP_LOGI(TAG, "Device Address: %02x:%02x:%02x:%02x:%02x:%02x",
                     addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
        }
    }

    // Give the stack a moment to fully initialize
    vTaskDelay(pdMS_TO_TICKS(100));

    // Begin advertising
    ble_advertise();
}

static void ble_on_reset(int reason)
{
    ESP_LOGE(TAG, "Resetting state; reason=%d", reason);
}

static void nimble_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE Host Task Started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t ble_barcode_init(const char *name)
{
    esp_err_t ret;
    
    if (name) {
        strncpy(device_name, name, sizeof(device_name) - 1);
        device_name[sizeof(device_name) - 1] = '\0';
    }
    
    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize NimBLE
    ESP_ERROR_CHECK(nimble_port_init());
    
    // Initialize the NimBLE host configuration
    ble_hs_cfg.sync_cb = ble_on_sync;
    ble_hs_cfg.reset_cb = ble_on_reset;
    ble_hs_cfg.gatts_register_cb = NULL;
    ble_hs_cfg.store_status_cb = NULL;

    // Set device name
    ret = ble_svc_gap_device_name_set(device_name);
    if (ret != 0) {
        ESP_LOGE(TAG, "Error setting device name; rc=%d", ret);
        return ESP_FAIL;
    }

    // Initialize GATT service
    ret = ble_gatts_count_cfg(gatt_svr_svcs);
    if (ret != 0) {
        ESP_LOGE(TAG, "Error counting GATT configuration; rc=%d", ret);
        return ESP_FAIL;
    }

    ret = ble_gatts_add_svcs(gatt_svr_svcs);
    if (ret != 0) {
        ESP_LOGE(TAG, "Error adding GATT services; rc=%d", ret);
        return ESP_FAIL;
    }

    // Initialize GAP and GATT services
    ble_svc_gap_init();
    ble_svc_gatt_init();

    // Start the NimBLE host task
    nimble_port_freertos_init(nimble_host_task);
    
    ESP_LOGI(TAG, "BLE barcode service initialized as '%s'", device_name);
    return ESP_OK;
}

esp_err_t ble_barcode_send(const char *barcode_data)
{
    if (!ble_connected) {
        ESP_LOGW(TAG, "Cannot send barcode: no client connected");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!barcode_data) {
        return ESP_ERR_INVALID_ARG;
    }
    
    size_t len = strlen(barcode_data);
    if (len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Create mbuf for notification
    struct os_mbuf *om;
    om = ble_hs_mbuf_from_flat(barcode_data, len);
    if (!om) {
        ESP_LOGE(TAG, "Failed to allocate mbuf");
        return ESP_ERR_NO_MEM;
    }
    
    // Send notification
    int rc = ble_gattc_notify_custom(ble_conn_handle, upc_char_handle, om);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to send notification; rc=%d", rc);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Sent barcode via BLE: %s", barcode_data);
    return ESP_OK;
}

bool ble_barcode_is_connected(void)
{
    return ble_connected;
}

void ble_barcode_deinit(void)
{
    int rc = nimble_port_stop();
    if (rc != 0) {
        ESP_LOGE(TAG, "Error stopping NimBLE; rc=%d", rc);
    }
    
    nimble_port_deinit();
    ESP_LOGI(TAG, "BLE barcode service deinitialized");
}