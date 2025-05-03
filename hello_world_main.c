#include <stdio.h>
#include <string.h>
#include <math.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"

#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_err.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"

static const char *TAG = "BMX055_ACC";

// --- BLE UUIDs ---
#define BLE_SERVICE_UUID        0x00FF
#define BLE_CHAR_UUID_STEPS     0xFF01
#define BLE_CHAR_UUID_FALL      0xFF02
#define BLE_CHAR_UUID_PREV_HOUR 0xFF03
#define BLE_CHAR_UUID_CUR_HOUR  0xFF04

// Client Characteristic Configuration descriptor UUID
static esp_bt_uuid_t cccd_uuid = {
    .len         = ESP_UUID_LEN_16,
    .uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG
};

// --- BMX055 over SPI ---
#define BMX055_ACC_CHIP_ID_REG  0x00
#define BMX055_ACC_X_LSB        0x02
#define BMX055_ACC_READ_MASK    0x80

#define PIN_NUM_MOSI 7
#define PIN_NUM_MISO 2
#define PIN_NUM_CLK  6
#define PIN_NUM_CS   21

// --- Detection thresholds ---
#define STEP_THRESHOLD     1.2f
#define STEP_DEBOUNCE_MS   250
#define FREEFALL_THRESHOLD 0.5f
#define IMPACT_THRESHOLD   2.0f
#define FALL_WINDOW_MS     500

// --- UDP Broadcast ---
#define UDP_PORT           3333

// Raw advertisement packet
static uint8_t adv_raw_data[] = {
    0x02, 0x01, 0x06,
    0x03, 0x03, 0xFF, 0x00,
    0x0A, 0x09, 'E','S','P','3','2','_','A','C','C'
};

// BLE advertising parameters
static esp_ble_adv_params_t adv_params = {
    .adv_int_min       = 0x20,
    .adv_int_max       = 0x40,
    .adv_type          = ADV_TYPE_IND,
    .own_addr_type     = BLE_ADDR_TYPE_PUBLIC,
    .channel_map       = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY
};

// --- Globals ---
static uint16_t gatts_if_global     = 0;
static uint16_t conn_id_global      = 0;
static bool     notify_steps_enabled = true;
static bool     notify_fall_enabled  = false;
static uint16_t char_handle_steps, cccd_handle_steps;
static uint16_t char_handle_fall,  cccd_handle_fall;
static uint16_t char_handle_prev_hour, cccd_handle_prev_hour;
static uint16_t char_handle_cur_hour,  cccd_handle_cur_hour;
static bool     notify_prev_hour_enabled = true;
static bool     notify_cur_hour_enabled  = true;
volatile uint32_t steps_prev_hour = 500;
volatile uint32_t steps_cur_hour  = 0;
static TickType_t  hour_start_tick;

// Sensor & state
static spi_device_handle_t spi = NULL;
volatile int16_t  current_x, current_y, current_z;
volatile uint32_t current_steps;
volatile uint8_t  fall_detected;

// ----------------------------------------------------------------------------
// BMX055 SPI Routines
// ----------------------------------------------------------------------------
static esp_err_t bmx055_acc_read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    uint8_t tx[len+1], rx[len+1];
    tx[0] = reg | BMX055_ACC_READ_MASK;
    memset(&tx[1], 0, len);
    spi_transaction_t t = {
        .length    = 8 * (len + 1),
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    esp_err_t ret = spi_device_transmit(spi, &t);
    if (ret == ESP_OK) {
        memcpy(data, &rx[1], len);
    }
    return ret;
}

static esp_err_t bmx055_acc_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (reg & 0x7F), val };
    spi_transaction_t t = {
        .length    = 16,
        .tx_buffer = tx,
    };
    return spi_device_transmit(spi, &t);
}

void bmx055_acc_init(void)
{
    spi_bus_config_t buscfg = {
        .miso_io_num     = PIN_NUM_MISO,
        .mosi_io_num     = PIN_NUM_MOSI,
        .sclk_io_num     = PIN_NUM_CLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 64
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,
        .mode           = 0,
        .spics_io_num   = PIN_NUM_CS,
        .queue_size     = 1
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi));

    uint8_t id;
    ESP_ERROR_CHECK(bmx055_acc_read_reg(BMX055_ACC_CHIP_ID_REG, &id, 1));
    ESP_LOGI(TAG, "BMX055 ACC ID = 0x%02X", id);
    ESP_ERROR_CHECK(bmx055_acc_write_reg(0x0F, 0x0C));  // ±2g
}

void bmx055_acc_read_xyz(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t raw[6];
    if (bmx055_acc_read_reg(BMX055_ACC_X_LSB, raw, 6) == ESP_OK) {
        *x = ((int16_t)(raw[1] << 8 | raw[0])) >> 4;
        *y = ((int16_t)(raw[3] << 8 | raw[2])) >> 4;
        *z = ((int16_t)(raw[5] << 8 | raw[4])) >> 4;
    } else {
        *x = *y = *z = 0;
    }
}

// ----------------------------------------------------------------------------
// Utility: compute accel magnitude
// ----------------------------------------------------------------------------
static inline float accel_magnitude(int16_t x, int16_t y, int16_t z)
{
    float fx = x * 0.001f;
    float fy = y * 0.001f;
    float fz = z * 0.001f;
    return sqrtf(fx*fx + fy*fy + fz*fz);
}

// ----------------------------------------------------------------------------
// Step & Fall Detection
// ----------------------------------------------------------------------------
static TickType_t last_step_tick = 0;
bool step_detected(int16_t x, int16_t y, int16_t z)
{
    float mag = accel_magnitude(x, y, z);
    TickType_t now = xTaskGetTickCount();
    if (mag > STEP_THRESHOLD &&
        now - last_step_tick > pdMS_TO_TICKS(STEP_DEBOUNCE_MS)) {
        last_step_tick = now;
        return true;
    }
    return false;
}

static bool in_freefall = false;
static TickType_t freefall_start = 0;
bool fall_condition(int16_t x, int16_t y, int16_t z)
{
    float mag = accel_magnitude(x, y, z);
    TickType_t now = xTaskGetTickCount();
    if (!in_freefall && mag < FREEFALL_THRESHOLD) {
        in_freefall = true;
        freefall_start = now;
        return false;
    }
    if (in_freefall) {
        if (mag > IMPACT_THRESHOLD &&
            now - freefall_start < pdMS_TO_TICKS(FALL_WINDOW_MS)) {
            in_freefall = false;
            return true;
        }
        if (now - freefall_start >= pdMS_TO_TICKS(FALL_WINDOW_MS)) {
            in_freefall = false;
        }
    }
    return false;
}

// ----------------------------------------------------------------------------
// Wi-Fi SoftAP + UDP Broadcast
// ----------------------------------------------------------------------------
void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t apcfg = {
        .ap = {
            .ssid = "ESP32_AP",
            .channel = 1,
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .password = "esp32pass"
        }
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &apcfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "SoftAP started: SSID=%s", apcfg.ap.ssid);
}

void udp_broadcast_task(void *pv)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    int br = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &br, sizeof(br));

    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(UDP_PORT),
        .sin_addr.s_addr = inet_addr("255.255.255.255")
    };
    char msg[64];

    for (;;) {
        snprintf(msg, sizeof(msg), "X:%d Y:%d Z:%d", current_x, current_y, current_z);
        sendto(sock, msg, strlen(msg), 0, (struct sockaddr*)&addr, sizeof(addr));
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ----------------------------------------------------------------------------
// BLE GAP callback
// ----------------------------------------------------------------------------
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    if (event == ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT) {
        ESP_LOGI(TAG, "Starting advertising");
        esp_ble_gap_start_advertising(&adv_params);
    }
}

// ----------------------------------------------------------------------------
// BLE GATTS callback
// ----------------------------------------------------------------------------
static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
        esp_ble_gap_set_device_name("ESP32_ACC");
        esp_ble_gap_config_adv_data_raw(adv_raw_data, sizeof(adv_raw_data));
        esp_ble_gatts_create_service(gatts_if, &(esp_gatt_srvc_id_t){
            .is_primary = true,
            .id.inst_id = 0,
            .id.uuid    = { .len = ESP_UUID_LEN_16,
                            .uuid.uuid16 = BLE_SERVICE_UUID }
        },20);
        break;

    case ESP_GATTS_CREATE_EVT: {
        uint16_t svc = param->create.service_handle;
        esp_ble_gatts_start_service(svc);

        // Steps characteristic
        esp_bt_uuid_t uuid = {
            .len = ESP_UUID_LEN_16,
            .uuid.uuid16 = BLE_CHAR_UUID_STEPS
        };
        esp_ble_gatts_add_char(svc, &uuid,
            ESP_GATT_PERM_READ,
            ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
            NULL, NULL);
        esp_ble_gatts_add_char_descr(svc, &cccd_uuid,
            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            NULL, NULL);
        // Presentation Format descriptor for Steps
        esp_bt_uuid_t fmt_uuid = {
            .len = ESP_UUID_LEN_16,
            .uuid.uuid16 = ESP_GATT_UUID_CHAR_PRESENT_FORMAT
        };
        static const uint8_t fmt_value[7] = {
            0x08, 0x00, 0x00,0x27, 0x01, 0x00,0x00
        };
        esp_attr_value_t fmt_attr = {
            .attr_max_len = sizeof(fmt_value),
            .attr_len     = sizeof(fmt_value),
            .attr_value   = (uint8_t*)fmt_value
        };
        esp_ble_gatts_add_char_descr(svc, &fmt_uuid,
            ESP_GATT_PERM_READ, &fmt_attr, NULL);
        // User Description for Steps
        esp_bt_uuid_t user_desc_uuid = {
            .len = ESP_UUID_LEN_16,
            .uuid.uuid16 = ESP_GATT_UUID_CHAR_DESCRIPTION
        };
        static const uint8_t steps_user_desc[] = "Steps";
        esp_attr_value_t steps_desc_val = {
            .attr_max_len = sizeof(steps_user_desc)-1,
            .attr_len     = sizeof(steps_user_desc)-1,
            .attr_value   = (uint8_t*)steps_user_desc
        };
        esp_ble_gatts_add_char_descr(svc, &user_desc_uuid,
            ESP_GATT_PERM_READ, &steps_desc_val, NULL);

        // Fall characteristic
        uuid.uuid.uuid16 = BLE_CHAR_UUID_FALL;
        esp_ble_gatts_add_char(svc, &uuid,
            ESP_GATT_PERM_READ,
            ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
            NULL, NULL);
        esp_ble_gatts_add_char_descr(svc, &cccd_uuid,
            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            NULL, NULL);
        // User Description for Fall
        static const uint8_t fall_user_desc[] = "Fall";
        esp_attr_value_t fall_desc_val = {
            .attr_max_len = sizeof(fall_user_desc)-1,
            .attr_len     = sizeof(fall_user_desc)-1,
            .attr_value   = (uint8_t*)fall_user_desc
        };
        esp_ble_gatts_add_char_descr(svc, &user_desc_uuid,
            ESP_GATT_PERM_READ, &fall_desc_val, NULL);

        // Previous-hour steps characteristic
        uuid.uuid.uuid16 = BLE_CHAR_UUID_PREV_HOUR;
        esp_ble_gatts_add_char(svc, &uuid,
            ESP_GATT_PERM_READ,
            ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
            NULL, NULL);
        esp_ble_gatts_add_char_descr(svc, &cccd_uuid,
            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            NULL, NULL);
        static const uint8_t prev_desc[] = "PrevHourSteps";
        esp_attr_value_t prev_desc_val = {
            .attr_max_len = sizeof(prev_desc)-1,
            .attr_len     = sizeof(prev_desc)-1,
            .attr_value   = (uint8_t*)prev_desc
        };
        esp_ble_gatts_add_char_descr(svc, &user_desc_uuid,
            ESP_GATT_PERM_READ, &prev_desc_val, NULL);

        // Current-hour steps characteristic
        uuid.uuid.uuid16 = BLE_CHAR_UUID_CUR_HOUR;
        esp_ble_gatts_add_char(svc, &uuid,
            ESP_GATT_PERM_READ,
            ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
            NULL, NULL);
        esp_ble_gatts_add_char_descr(svc, &cccd_uuid,
            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            NULL, NULL);
        static const uint8_t cur_desc[] = "CurHourSteps";
        esp_attr_value_t cur_desc_val = {
            .attr_max_len = sizeof(cur_desc)-1,
            .attr_len     = sizeof(cur_desc)-1,
            .attr_value   = (uint8_t*)cur_desc
        };
        esp_ble_gatts_add_char_descr(svc, &user_desc_uuid,
            ESP_GATT_PERM_READ, &cur_desc_val, NULL);

        break;
    }

    case ESP_GATTS_ADD_CHAR_EVT: {
        uint16_t u = param->add_char.char_uuid.uuid.uuid16;
        if (u == BLE_CHAR_UUID_STEPS) {
            char_handle_steps = param->add_char.attr_handle;
        } else if (u == BLE_CHAR_UUID_FALL) {
            char_handle_fall = param->add_char.attr_handle;
        } else if (u == BLE_CHAR_UUID_PREV_HOUR) {
            char_handle_prev_hour = param->add_char.attr_handle;
        } else if (u == BLE_CHAR_UUID_CUR_HOUR) {
            char_handle_cur_hour = param->add_char.attr_handle;
        }
        break;
    }

    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        if (param->add_char_descr.descr_uuid.uuid.uuid16 ==
            ESP_GATT_UUID_CHAR_CLIENT_CONFIG) {
            if (cccd_handle_steps == 0) {
                cccd_handle_steps = param->add_char_descr.attr_handle;
            } else if (cccd_handle_fall == 0) {
                cccd_handle_fall = param->add_char_descr.attr_handle;
            } else if (cccd_handle_prev_hour == 0) {
                cccd_handle_prev_hour = param->add_char_descr.attr_handle;
            } else {
                cccd_handle_cur_hour = param->add_char_descr.attr_handle;
            }
        }
        break;

    case ESP_GATTS_READ_EVT: {
        esp_gatt_rsp_t rsp = {0};
        rsp.attr_value.handle = param->read.handle;
        if (param->read.handle == char_handle_steps) {
            uint32_t v = current_steps;
            memcpy(rsp.attr_value.value, &v, 4);
            rsp.attr_value.len = 4;
        } else if (param->read.handle == char_handle_prev_hour) {
            uint32_t v = steps_prev_hour;
            memcpy(rsp.attr_value.value, &v, 4);
            rsp.attr_value.len = 4;
        } else if (param->read.handle == char_handle_cur_hour) {
            uint32_t v = steps_cur_hour;
            memcpy(rsp.attr_value.value, &v, 4);
            rsp.attr_value.len = 4;
        } else if (param->read.handle == char_handle_fall) {
            rsp.attr_value.value[0] = fall_detected;
            rsp.attr_value.len = 1;
        }
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id,
                                    param->read.trans_id, ESP_GATT_OK, &rsp);
        break;
    }

    case ESP_GATTS_WRITE_EVT: {
        uint16_t h   = param->write.handle;
        uint16_t val = param->write.value[1] << 8 |
                       param->write.value[0];
        if (h == cccd_handle_steps) {
            notify_steps_enabled = (val & 0x0003) != 0;
            ESP_LOGI(TAG, "Steps notifications %s",
                     notify_steps_enabled ? "ENABLED" : "DISABLED");
        } else if (h == cccd_handle_fall) {
            notify_fall_enabled = (val & 0x0003) != 0;
            ESP_LOGI(TAG, "Fall notifications %s",
                     notify_fall_enabled ? "ENABLED" : "DISABLED");
        } else if (h == cccd_handle_prev_hour) {
            //notify_prev_hour_enabled = (val & 0x0003) != 0;
            ESP_LOGI(TAG, "Prev-hour notifications %s",
                     notify_prev_hour_enabled ? "ENABLED" : "DISABLED");
        } else if (h == cccd_handle_cur_hour) {
            //notify_cur_hour_enabled = (val & 0x0003) != 0;
            ESP_LOGI(TAG, "Cur-hour notifications %s",
                     notify_cur_hour_enabled ? "ENABLED" : "DISABLED");
        }
        if (param->write.need_rsp) {
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                        param->write.trans_id,
                                        ESP_GATT_OK, NULL);
        }
        break;
    }

    case ESP_GATTS_CONNECT_EVT:
        gatts_if_global = gatts_if;
        conn_id_global = param->connect.conn_id;
        ESP_LOGI(TAG, "Client connected, conn_id=%d", conn_id_global);
        notify_steps_enabled      = true;
        notify_fall_enabled       = false;
        notify_prev_hour_enabled  = true;
        notify_cur_hour_enabled   = true;
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        conn_id_global = 0;
        notify_steps_enabled      = false;
        notify_fall_enabled       = false;
        notify_prev_hour_enabled  = false;
        notify_cur_hour_enabled   = false;
        esp_ble_gap_start_advertising(&adv_params);
        ESP_LOGI(TAG, "Client disconnected");
        break;

    default:
        break;
    }
}

// BLE Initialization
void ble_init(void)
{
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(0));
}

// Main Application
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    // Wi-Fi + UDP
    wifi_init_softap();
    xTaskCreate(udp_broadcast_task, "udp_bcast", 4096, NULL, 5, NULL);

    // Sensor init
    bmx055_acc_init();

    // BLE init + advertising
    ble_init();
    hour_start_tick = xTaskGetTickCount();

    for (;;) {
        int16_t x, y, z;
        bmx055_acc_read_xyz(&x, &y, &z);
        current_x = x; current_y = y; current_z = z;

        if (step_detected(x, y, z))  current_steps++;
        if (fall_condition(x, y, z)) fall_detected = 1;
        current_steps++;

        // Hourly rollover
        if (xTaskGetTickCount() - hour_start_tick >= pdMS_TO_TICKS(3600000)) {
            steps_prev_hour = steps_cur_hour;
            steps_cur_hour  = 0;
            hour_start_tick += pdMS_TO_TICKS(3600000);
        }
        steps_cur_hour++;
        
            if (notify_steps_enabled) {
                esp_ble_gatts_send_indicate(
                    gatts_if_global, conn_id_global,
                    char_handle_steps, sizeof(current_steps),
                    (uint8_t*)&current_steps, false);
            }
            if (notify_fall_enabled) {
                uint8_t fv = fall_detected;
                esp_ble_gatts_send_indicate(
                    gatts_if_global, conn_id_global,
                    char_handle_fall, sizeof(fv),
                    &fv, false);
                fall_detected = 0;
            }
            if (notify_prev_hour_enabled) {
                esp_ble_gatts_send_indicate(
                    gatts_if_global, conn_id_global,
                    char_handle_prev_hour, sizeof(steps_prev_hour),
                    (uint8_t*)&steps_prev_hour, false);
            }
            if (notify_cur_hour_enabled) {
                esp_ble_gatts_send_indicate(
                    gatts_if_global, conn_id_global,
                    char_handle_cur_hour, sizeof(steps_cur_hour),
                    (uint8_t*)&steps_cur_hour, false);
            }
        

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}