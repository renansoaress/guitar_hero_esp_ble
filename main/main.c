/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_hidd_prf_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "driver/gpio.h"
#include "hid_dev.h"

#define VERSION "1.0.0.0"

#define BTN_GREEN_GPIO_NUM 13
#define BTN_RED_GPIO_NUM 12
#define BTN_YELLOW_GPIO_NUM 14
#define BTN_BLUE_GPIO_NUM 27
#define BTN_ORANGE_GPIO_NUM 26
#define BTN_UP_GPIO_NUM 25
#define BTN_DOWN_GPIO_NUM 33
#define BTN_START_GPIO_NUM 32
#define BTN_SELECT_GPIO_NUM 4
#define BTN_STRUM_GPIO_NUM 5
#define BTN_LED_GPIO_NUM 15 // OUTPUT

#define GPIO_INPUT_PIN_SEL (                                      \
    (1ULL << BTN_GREEN_GPIO_NUM) | (1ULL << BTN_RED_GPIO_NUM) |   \
    (1ULL << BTN_YELLOW_GPIO_NUM) | (1ULL << BTN_BLUE_GPIO_NUM) | \
    (1ULL << BTN_ORANGE_GPIO_NUM) | (1ULL << BTN_UP_GPIO_NUM) |   \
    (1ULL << BTN_DOWN_GPIO_NUM) | (1ULL << BTN_START_GPIO_NUM) |  \
    (1ULL << BTN_SELECT_GPIO_NUM))
#define GPIO_OUTPUT_PIN_SEL ((1ULL << BTN_LED_GPIO_NUM))

#define HID_TAG "HID"

static uint16_t hid_conn_id = 0;
static bool sec_conn = false;
#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);

#define HIDD_DEVICE_NAME "GUITAR KEYBOARD BLE"
static uint8_t hidd_service_uuid128[] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    // first uuid, 16bit, [12],[13] is the value
    0xfb,
    0x34,
    0x9b,
    0x5f,
    0x80,
    0x00,
    0x00,
    0x80,
    0x00,
    0x10,
    0x00,
    0x00,
    0x12,
    0x18,
    0x00,
    0x00,
};

static esp_ble_adv_data_t hidd_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,                        // slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010,                        // slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = ESP_BLE_APPEARANCE_HID_KEYBOARD, // HID Generic,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(hidd_service_uuid128),
    .p_service_uuid = hidd_service_uuid128,
    .flag = 0x6,
};

static esp_ble_adv_params_t hidd_adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x30,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch (event)
    {
    case ESP_HIDD_EVENT_REG_FINISH:
    {
        if (param->init_finish.state == ESP_HIDD_INIT_OK)
        {
            // esp_bd_addr_t rand_addr = {0x04,0x11,0x11,0x11,0x11,0x05};
            esp_ble_gap_set_device_name(HIDD_DEVICE_NAME);
            esp_ble_gap_config_adv_data(&hidd_adv_data);
        }
        break;
    }
    case ESP_BAT_EVENT_REG:
    {
        break;
    }
    case ESP_HIDD_EVENT_DEINIT_FINISH:
        break;
    case ESP_HIDD_EVENT_BLE_CONNECT:
    {
        ESP_LOGI(HID_TAG, "ESP_HIDD_EVENT_BLE_CONNECT");
        hid_conn_id = param->connect.conn_id;
        break;
    }
    case ESP_HIDD_EVENT_BLE_DISCONNECT:
    {
        sec_conn = false;
        ESP_LOGI(HID_TAG, "ESP_HIDD_EVENT_BLE_DISCONNECT");
        esp_ble_gap_start_advertising(&hidd_adv_params);
        break;
    }
    case ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT:
    {
        ESP_LOGI(HID_TAG, "%s, ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT", __func__);
        ESP_LOG_BUFFER_HEX(HID_TAG, param->vendor_write.data, param->vendor_write.length);
        break;
    }
    case ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT:
    {
        ESP_LOGI(HID_TAG, "ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT");
        ESP_LOG_BUFFER_HEX(HID_TAG, param->led_write.data, param->led_write.length);
        break;
    }
    default:
        break;
    }
    return;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&hidd_adv_params);
        break;
    case ESP_GAP_BLE_SEC_REQ_EVT:
        for (int i = 0; i < ESP_BD_ADDR_LEN; i++)
        {
            ESP_LOGD(HID_TAG, "%x:", param->ble_security.ble_req.bd_addr[i]);
        }
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        sec_conn = true;
        esp_bd_addr_t bd_addr;
        memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(HID_TAG, "remote BD_ADDR: %08x%04x",
                 (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
                 (bd_addr[4] << 8) + bd_addr[5]);
        ESP_LOGI(HID_TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
        ESP_LOGI(HID_TAG, "pair status = %s", param->ble_security.auth_cmpl.success ? "success" : "fail");
        if (!param->ble_security.auth_cmpl.success)
        {
            ESP_LOGE(HID_TAG, "fail reason = 0x%x", param->ble_security.auth_cmpl.fail_reason);
        }
        break;
    default:
        break;
    }
}

void led_task(void *pvParameters)
{
    while (true)
    {
        if (sec_conn)
        {
            gpio_set_level(BTN_LED_GPIO_NUM, 1);
        }
        else
        {
            gpio_set_level(BTN_LED_GPIO_NUM, 1);
            vTaskDelay(200 / portTICK_PERIOD_MS);
            gpio_set_level(BTN_LED_GPIO_NUM, 0);
        }
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

void hid_task(void *pvParameters)
{
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    while (1)
    {
        // vTaskDelay(2000 / portTICK_PERIOD_MS);
        if (sec_conn)
        {

            // ESP_LOGI(HID_TAG, "Send test");

            uint8_t btn_green = gpio_get_level(BTN_GREEN_GPIO_NUM);
            uint8_t btn_red = gpio_get_level(BTN_RED_GPIO_NUM);
            uint8_t btn_yellow = gpio_get_level(BTN_YELLOW_GPIO_NUM);
            uint8_t btn_blue = gpio_get_level(BTN_BLUE_GPIO_NUM);
            uint8_t btn_orange = gpio_get_level(BTN_ORANGE_GPIO_NUM);
            uint8_t btn_up = gpio_get_level(BTN_UP_GPIO_NUM);
            uint8_t btn_down = gpio_get_level(BTN_DOWN_GPIO_NUM);
            uint8_t btn_start = gpio_get_level(BTN_START_GPIO_NUM);
            uint8_t btn_select = gpio_get_level(BTN_SELECT_GPIO_NUM);
            uint8_t btn_strum = gpio_get_level(BTN_STRUM_GPIO_NUM);

            uint8_t keys_press[8] = {0};
            uint8_t keys_count_press = 0;
            if (!btn_green)
            {
                keys_press[keys_count_press] = HID_KEY_A;
                keys_count_press++;
            }

            if (!btn_red)
            {
                keys_press[keys_count_press] = HID_KEY_S;
                keys_count_press++;
            }

            if (!btn_yellow)
            {
                keys_press[keys_count_press] = HID_KEY_J;
                keys_count_press++;
            }

            if (!btn_blue)
            {
                keys_press[keys_count_press] = HID_KEY_K;
                keys_count_press++;
            }

            if (!btn_orange)
            {
                keys_press[keys_count_press] = HID_KEY_L;
                keys_count_press++;
            }

            if (!btn_up)
            {
                keys_press[keys_count_press] = HID_KEY_UP_ARROW;
                keys_count_press++;
            }
            else if (!btn_down)
            {
                keys_press[keys_count_press] = HID_KEY_DOWN_ARROW;
                keys_count_press++;
            }

            if (!btn_start)
            {
                keys_press[keys_count_press] = HID_KEY_ENTER;
                keys_count_press++;
            }
            else if (!btn_select)
            {
                keys_press[keys_count_press] = HID_KEY_LEFT_ARROW;
                keys_count_press++;
            }

            if (!btn_strum)
            {
                keys_press[keys_count_press] = HID_KEY_SPACEBAR;
                keys_count_press++;
            }

            esp_hidd_send_keyboard_value(hid_conn_id, 0, &keys_press, keys_count_press);

            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        else
        {
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
}

void app_main(void)
{
    printf("VERSION: (%s)\n", VERSION);

    esp_err_t ret;
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(HID_TAG, "%s initialize controller failed", __func__);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(HID_TAG, "%s enable controller failed", __func__);
        return;
    }

    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg);
    if (ret)
    {
        ESP_LOGE(HID_TAG, "%s init bluedroid failed", __func__);
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(HID_TAG, "%s init bluedroid failed", __func__);
        return;
    }

    if ((ret = esp_hidd_profile_init()) != ESP_OK)
    {
        ESP_LOGE(HID_TAG, "%s init bluedroid failed", __func__);
    }

    esp_ble_gap_register_callback(gap_event_handler);
    esp_hidd_register_callbacks(hidd_event_callback);

    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;
    uint8_t key_size = 16;
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    gpio_config_t btns;
    btns.intr_type = GPIO_INTR_DISABLE;
    btns.mode = GPIO_MODE_INPUT;
    btns.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    btns.pull_down_en = GPIO_PULLDOWN_DISABLE;
    btns.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&btns);

    gpio_config_t led_config;
    led_config.intr_type = GPIO_INTR_DISABLE;
    led_config.mode = GPIO_MODE_OUTPUT;
    led_config.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    led_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    led_config.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&led_config);

    gpio_set_level(BTN_LED_GPIO_NUM, 1);

    xTaskCreatePinnedToCore(&led_task, "led_task", 2048 * 2, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(&hid_task, "hid_task", 2048 * 2, NULL, 6, NULL, 0);
}
