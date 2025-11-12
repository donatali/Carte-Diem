#include "imu.h"
#include "math.h"
#include "esp_log.h"

#define TAG "ICM20948"

// Low-pass filter for motion detection
static float accel_magnitude_prev = 0.0f;

static void activity_timer_callback(TimerHandle_t xTimer) {
    ICM20948_t *dev = (ICM20948_t *)pvTimerGetTimerID(xTimer);
    if (dev->idle_callback) dev->idle_callback();
}

// Helper function to read two bytes from I2C
static esp_err_t icm20948_read_bytes(ICM20948_t *device, uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_transmit_receive(device->dev_handle, &reg, 1, data, len, -1);
}

// Helper function to write byte to I2C
static esp_err_t icm20948_write_byte(ICM20948_t *device, uint8_t reg, uint8_t value) {
    uint8_t write_buf[2] = {reg, value};
    return i2c_master_transmit(device->dev_handle, write_buf, sizeof(write_buf), -1);
}

// Read raw 16-bit signed value from register
static int16_t icm20948_read_int16(ICM20948_t *device, uint8_t reg_high) {
    uint8_t data[2];
    icm20948_read_bytes(device, reg_high, data, 2);
    return (int16_t)((data[0] << 8) | data[1]);
}

void icm20948_init(ICM20948_t *device, i2c_master_bus_handle_t bus_handle) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ICM20948_I2C_ADDR,
        .scl_speed_hz = 400000,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &device->dev_handle));
    device->accel.sensitivity = 16384.0f;
    device->gyro.sensitivity  = 131.0f;
    device->mag.sensitivity   = 0.15f;  // µT per LSB per datasheet
    device->status = STANDBY;
    device->direction_deg = 0.0f;
    device->idle_counter_ms = 0;
    device->activity_timer = NULL;

    // Initialize IMU: Set to active mode
    icm20948_write_byte(device, ICM20948_PWR_MGMT_1, 0x01);  // Enable clk source
    vTaskDelay(pdMS_TO_TICKS(100));  // Wait for IMU to wake up
    ESP_LOGI(TAG, "ICM20948 initialized successfully");
}

// -----------------------------------------------------------------------------
// Calculate heading (yaw) in degrees using magnetometer and accelerometer
// -----------------------------------------------------------------------------
float icm20948_compute_heading(ICM20948_t *dev) {
    icm20948_read_mag(dev);
    icm20948_read_accel(dev);

    float pitch = atan2f(-dev->accel.x, sqrtf(dev->accel.y * dev->accel.y + dev->accel.z * dev->accel.z));
    float roll  = atan2f(dev->accel.y, dev->accel.z);

    float xh = dev->mag.x * cosf(pitch) + dev->mag.z * sinf(pitch);
    float yh = dev->mag.x * sinf(roll) * sinf(pitch) + dev->mag.y * cosf(roll) - dev->mag.z * sinf(roll) * cosf(pitch);

    float heading = atan2f(yh, xh) * 180.0f / M_PI;
    if (heading < 0) heading += 360.0f;
    dev->direction_deg = heading;
    return heading;
}

// -----------------------------------------------------------------------------
// Motion detection (detect if active or idle)
// -----------------------------------------------------------------------------
bool icm20948_is_moving(ICM20948_t *dev) {
    icm20948_read_accel(dev);
    float mag = sqrtf(dev->accel.x * dev->accel.x + dev->accel.y * dev->accel.y + dev->accel.z * dev->accel.z);
    float diff = fabsf(mag - accel_magnitude_prev);
    accel_magnitude_prev = mag;
    bool moving = (diff > 0.03f); // ~30 mg threshold

    dev->status = moving ? MOVING : IDLE;
    return moving;
}

// -----------------------------------------------------------------------------
// Start a periodic motion monitor (5 min idle triggers callback)
// -----------------------------------------------------------------------------
void icm20948_start_activity_monitor(ICM20948_t *dev, void (*callback)(void)) {
    dev->idle_callback = callback;
    dev->activity_timer = xTimerCreate("ActivityTimer", pdMS_TO_TICKS(1000), pdTRUE, (void*)dev, activity_timer_callback);
    xTimerStart(dev->activity_timer, 0);
}

// Call this in a FreeRTOS task periodically (e.g., every 1s)
void icm20948_activity_task(ICM20948_t *dev) {
    if (!icm20948_is_moving(dev)) {
        dev->idle_counter_ms += 1000;
        if (dev->idle_counter_ms >= 5 * 60 * 1000) { // 5 minutes
            if (dev->idle_callback) dev->idle_callback();
            dev->idle_counter_ms = 0;
        }
    } else {
        dev->idle_counter_ms = 0;
    }
}

// -----------------------------------------------------------------------------
// Read accelerometer data
// -----------------------------------------------------------------------------
esp_err_t icm20948_read_accel(ICM20948_t *device) {
    int16_t ax_raw = icm20948_read_int16(device, ICM20948_ACCEL_XOUT_H);
    int16_t ay_raw = icm20948_read_int16(device, ICM20948_ACCEL_YOUT_H);
    int16_t az_raw = icm20948_read_int16(device, ICM20948_ACCEL_ZOUT_H);

    device->accel.x = (float)ax_raw / device->accel.sensitivity;
    device->accel.y = (float)ay_raw / device->accel.sensitivity;
    device->accel.z = (float)az_raw / device->accel.sensitivity;

    return ESP_OK;
}

// -----------------------------------------------------------------------------
// Read gyroscope data
// -----------------------------------------------------------------------------
esp_err_t icm20948_read_gyro(ICM20948_t *device) {
    int16_t gx_raw = icm20948_read_int16(device, ICM20948_GYRO_XOUT_H);
    int16_t gy_raw = icm20948_read_int16(device, ICM20948_GYRO_YOUT_H);
    int16_t gz_raw = icm20948_read_int16(device, ICM20948_GYRO_ZOUT_H);

    device->gyro.x = (float)gx_raw / device->gyro.sensitivity;
    device->gyro.y = (float)gy_raw / device->gyro.sensitivity;
    device->gyro.z = (float)gz_raw / device->gyro.sensitivity;

    return ESP_OK;
}

// -----------------------------------------------------------------------------
// Read magnetometer data (via I2C Master of ICM20948)
// Note: Magnetometer is AK8963, accessed through ICM20948's I2C master
// For now, return zero values as magnetometer setup is complex
// -----------------------------------------------------------------------------
esp_err_t icm20948_read_mag(ICM20948_t *device) {
    // Simplified magnetometer read - would require I2C master setup
    // TODO: Implement full AK8963 magnetometer read through ICM20948 I2C master
    device->mag.x = 0.0f;
    device->mag.y = 0.0f;
    device->mag.z = 0.0f;

    return ESP_OK;
}

// -----------------------------------------------------------------------------
// Set gyroscope full-scale range (DPS)
// dps: 250, 500, 1000, or 2000
// -----------------------------------------------------------------------------
esp_err_t icm20948_set_gyroDPS(ICM20948_t *device, uint32_t dps) {
    uint8_t fs_sel = 0;
    switch (dps) {
        case 250:  fs_sel = 0; device->gyro.sensitivity = 131.0f;   break;
        case 500:  fs_sel = 1; device->gyro.sensitivity = 65.5f;    break;
        case 1000: fs_sel = 2; device->gyro.sensitivity = 32.8f;    break;
        case 2000: fs_sel = 3; device->gyro.sensitivity = 16.4f;    break;
        default:   return ESP_FAIL;
    }
    // Write to GYRO_CONFIG_1 register (Bank 2, register 0x01)
    uint8_t reg_bank = 0x20;  // Select bank 2
    icm20948_write_byte(device, ICM20948_REG_BANK_SEL, reg_bank);
    icm20948_write_byte(device, ICM20948_GYRO_CONFIG_1, (fs_sel << 1));
    icm20948_write_byte(device, ICM20948_REG_BANK_SEL, 0x00);  // Back to bank 0
    return ESP_OK;
}

// -----------------------------------------------------------------------------
// Set accelerometer full-scale range (g)
// g: 2, 4, 8, or 16
// -----------------------------------------------------------------------------
esp_err_t icm20948_set_accelG(ICM20948_t *device, uint8_t g) {
    uint8_t fs_sel = 0;
    switch (g) {
        case 2:  fs_sel = 0; device->accel.sensitivity = 16384.0f;  break;
        case 4:  fs_sel = 1; device->accel.sensitivity = 8192.0f;   break;
        case 8:  fs_sel = 2; device->accel.sensitivity = 4096.0f;   break;
        case 16: fs_sel = 3; device->accel.sensitivity = 2048.0f;   break;
        default: return ESP_FAIL;
    }
    // Write to ACCEL_CONFIG register (Bank 2, register 0x14)
    uint8_t reg_bank = 0x20;  // Select bank 2
    icm20948_write_byte(device, ICM20948_REG_BANK_SEL, reg_bank);
    icm20948_write_byte(device, ICM20948_ACCEL_CONFIG, (fs_sel << 1));
    icm20948_write_byte(device, ICM20948_REG_BANK_SEL, 0x00);  // Back to bank 0
    return ESP_OK;
}
