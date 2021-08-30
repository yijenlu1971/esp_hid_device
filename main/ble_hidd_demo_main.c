/* This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this software is 
   distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR  
   CONDITIONS OF ANY KIND, either express or implied.
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

#include "esp_adc_cal.h"
#include "driver/gpio.h"
#include "driver/adc.h"

/**
 * Brief:
 * This example Implemented BLE HID device profile related functions, in which the HID device
 * has 4 Reports (1 is mouse, 2 is keyboard and LED, 3 is Consumer Devices, 4 is Vendor devices).
 * Users can choose different reports according to their own application scenarios.
 * BLE HID profile inheritance and USB HID class.
 */

/** 
 * Note:
 * 1. Win10 does not support vendor report , So SUPPORT_REPORT_VENDOR is always set to FALSE, it defines in hidd_le_prf_int.h
 * 2. Update connection parameters are not allowed during iPhone HID encryption, slave turns 
 * off the ability to automatically update connection parameters during encryption.
 * 3. After our HID device is connected, the iPhones write 1 to the Report Characteristic Configuration Descriptor, 
 * even if the HID encryption is not completed. This should actually be written 1 after the HID encryption is completed.
 * we modify the permissions of the Report Characteristic Configuration Descriptor to `ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE_ENCRYPTED`.
 * if you got `GATT_INSUF_ENCRYPTION` error, please ignore.
 */

#define HID_DEMO_TAG "HID_DEMO"

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling
#define BLINK_GPIO      2           // CONFIG_BLINK_GPIO

static esp_adc_cal_characteristics_t *adc_chars;
int axis_mouse[3];

static uint16_t hid_conn_id = 0;
static bool sec_conn = false;
static bool send_volum_up = false;
#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);

#define HIDD_DEVICE_NAME            "HID"
static uint8_t hidd_service_uuid128[] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
};

static esp_ble_adv_data_t hidd_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x03c0,       //HID Generic,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(hidd_service_uuid128),
    .p_service_uuid = hidd_service_uuid128,
    .flag = 0x6,
};

static esp_ble_adv_params_t hidd_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x30,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};


static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch(event) {
        case ESP_HIDD_EVENT_REG_FINISH: {
            if (param->init_finish.state == ESP_HIDD_INIT_OK) {
                //esp_bd_addr_t rand_addr = {0x04,0x11,0x11,0x11,0x11,0x05};
                esp_ble_gap_set_device_name(HIDD_DEVICE_NAME);
                esp_ble_gap_config_adv_data(&hidd_adv_data);
                
            }
            break;
        }
        case ESP_BAT_EVENT_REG: {
            break;
        }
        case ESP_HIDD_EVENT_DEINIT_FINISH:
	     break;
		case ESP_HIDD_EVENT_BLE_CONNECT: {
            ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_BLE_CONNECT conn_id=%d", param->connect.conn_id);
            hid_conn_id = param->connect.conn_id;
            break;
        }
        case ESP_HIDD_EVENT_BLE_DISCONNECT: {
            sec_conn = false;
            ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_BLE_DISCONNECT");
            esp_ble_gap_start_advertising(&hidd_adv_params);
            break;
        }
        case ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT: {
            ESP_LOGI(HID_DEMO_TAG, "%s, ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT", __func__);
            ESP_LOG_BUFFER_HEX(HID_DEMO_TAG, param->vendor_write.data, param->vendor_write.length);
        }    
        default:
            break;
    }
    return;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&hidd_adv_params);
        break;
     case ESP_GAP_BLE_SEC_REQ_EVT:
        for(int i = 0; i < ESP_BD_ADDR_LEN; i++) {
             ESP_LOGD(HID_DEMO_TAG, "%x:",param->ble_security.ble_req.bd_addr[i]);
        }
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
	 break;
     case ESP_GAP_BLE_AUTH_CMPL_EVT:
        sec_conn = true;
        esp_bd_addr_t bd_addr;
        memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(HID_DEMO_TAG, "remote BD_ADDR: %08x%04x",\
                (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
                (bd_addr[4] << 8) + bd_addr[5]);
        ESP_LOGI(HID_DEMO_TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
        ESP_LOGI(HID_DEMO_TAG, "pair status = %s",param->ble_security.auth_cmpl.success ? "success" : "fail");
        if(!param->ble_security.auth_cmpl.success) {
            ESP_LOGE(HID_DEMO_TAG, "fail reason = 0x%x",param->ble_security.auth_cmpl.fail_reason);
        }
        break;
    default:
        break;
    }
}

void hid_demo_task(void *pvParameters)
{
    int8_t  val_x = 0, val_y = 0;

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    while(1) {
//        vTaskDelay(2000 / portTICK_PERIOD_MS);
        if (sec_conn) {

            if(axis_mouse[1] >= 127) axis_mouse[1] = 127;
            if(axis_mouse[2] >= 127) axis_mouse[2] = 127;
            if(axis_mouse[1] <= -127) axis_mouse[1] = -127;
            if(axis_mouse[2] <= -127) axis_mouse[1] = -127;

            if( axis_mouse[1] < 4 && axis_mouse[1] > -4 ) axis_mouse[1] = 0;
            if( axis_mouse[2] < 4 && axis_mouse[2] > -4 ) axis_mouse[2] = 0;

            if( (0 != axis_mouse[1]) || (0 != axis_mouse[2]) )
            {
                val_x = axis_mouse[1];
                val_y = axis_mouse[2];
                esp_hidd_send_mouse_value(hid_conn_id, 0, val_x, val_y);
            }

            vTaskDelay(100 / portTICK_PERIOD_MS);
/*            ESP_LOGI(HID_DEMO_TAG, "[%d] Send the volume", xPortGetCoreID());
            send_volum_up = true;
            //uint8_t key_vaule = {HID_KEY_A};
            //esp_hidd_send_keyboard_value(hid_conn_id, 0, &key_vaule, 1);
            //esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_UP, true);
            //esp_hidd_send_mouse_value(hid_conn_id, 0, val_x, val_y);

            vTaskDelay(3000 / portTICK_PERIOD_MS);

            if (send_volum_up) {
                send_volum_up = false;
                
//                ESP_LOGI(HID_DEMO_TAG, "[%d] Send mouse %d,%d", xPortGetCoreID(), val_x, val_y);
                esp_hidd_send_mouse_value(hid_conn_id, 0, axis_mouse[1], axis_mouse[2]);

                //esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_UP, false);
                //esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_DOWN, true);
                //vTaskDelay(3000 / portTICK_PERIOD_MS);
                //esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_DOWN, false);
            }*/
        }
    }
}

void adc_val_task(void *pvParameters)
{
#define CAL_LOOP    15

    uint16_t    delay, bOn = 0x01;
    int     i, n, k;
    uint32_t adc_reading[3], init_axis[3] = {0};
	int axis_Val[3] = {0}, prev_Val[3] = {0}; 
    char    axis[3] = {'X', 'Y', 'Z'};
    adc_channel_t channel[3] = {ADC_CHANNEL_5, ADC_CHANNEL_4, ADC_CHANNEL_7};

    // calibration initial axis
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_LOGI(HID_DEMO_TAG, "Calibrating axis ...... \n");
    for(k = 0; k < CAL_LOOP; k++)
    {
        for(n = 0; n < 3; n++)
        {
            adc_reading[n] = 0;
            //Multisampling
            for (i = 0; i < NO_OF_SAMPLES; i++) adc_reading[n] += adc1_get_raw(channel[n]);

            adc_reading[n] /= NO_OF_SAMPLES;
            //Convert adc_reading to voltage in mV
            uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading[n], adc_chars);
            init_axis[n] += voltage;
        }

        gpio_set_level(BLINK_GPIO, bOn);
        bOn = !bOn;
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    for(n = 0; n < 3; n++)
    {
        init_axis[n] /= CAL_LOOP;
        init_axis[n] >>= 3;
    }
    ESP_LOGI(HID_DEMO_TAG, "End of calibration (%d,%d,%d) \n", init_axis[0], init_axis[1], init_axis[2]);

    while(1) {
        for(n = 0; n < 3; n++)
        {
            adc_reading[n] = 0;
            //Multisampling
            for (i = 0; i < NO_OF_SAMPLES; i++) adc_reading[n] += adc1_get_raw(channel[n]);

            adc_reading[n] /= NO_OF_SAMPLES;
            //Convert adc_reading to voltage in mV
            axis_Val[n] = esp_adc_cal_raw_to_voltage(adc_reading[n], adc_chars);
            axis_Val[n] >>= 3;
            axis_mouse[n] = axis_Val[n] - init_axis[n];
            
            if( prev_Val[n] != axis_mouse[n] )
            {
                prev_Val[n] = axis_mouse[n];
//                ESP_LOGI(HID_DEMO_TAG, "%c: %d\n", axis[n], axis_mouse[n]);
            }
        }
        
//        ESP_LOGI(HID_DEMO_TAG, "[%d] blink=%d", xPortGetCoreID(), bOn);
        gpio_set_level(BLINK_GPIO, bOn);
        bOn = !bOn;

        delay = (sec_conn) ? 200 : 1000;
        vTaskDelay(delay / portTICK_PERIOD_MS);
    }
}

static void check_efuse(void)
{
#if CONFIG_IDF_TARGET_ESP32
    //Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        ESP_LOGI(HID_DEMO_TAG, "eFuse Two Point: Supported\n");
    } else {
        ESP_LOGI(HID_DEMO_TAG, "eFuse Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        ESP_LOGI(HID_DEMO_TAG, "eFuse Vref: Supported\n");
    } else {
        ESP_LOGI(HID_DEMO_TAG, "eFuse Vref: NOT supported\n");
    }
#elif CONFIG_IDF_TARGET_ESP32S2
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        ESP_LOGI(HID_DEMO_TAG, "eFuse Two Point: Supported\n");
    } else {
        ESP_LOGI(HID_DEMO_TAG, "Cannot retrieve eFuse Two Point calibration values. Default calibration values will be used.\n");
    }
#else
#error "This example is configured for ESP32/ESP32S2."
#endif
}

void app_main(void)
{
    esp_err_t ret;

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_CHANNEL_5, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC_CHANNEL_4, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC_CHANNEL_7, ADC_ATTEN_DB_11);

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, 
                    ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s initialize controller failed\n", __func__);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s enable controller failed\n", __func__);
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed\n", __func__);
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed\n", __func__);
        return;
    }

    if((ret = esp_hidd_profile_init()) != ESP_OK) {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed\n", __func__);
    }

    ///register the callback function to the gap module
    esp_ble_gap_register_callback(gap_event_handler);
    esp_hidd_register_callbacks(hidd_event_callback);

    /* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;     //bonding with peer device after authentication
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;           //set the IO capability to No output No input
    uint8_t key_size = 16;      //the key size should be 7~16 bytes
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    /* If your BLE device act as a Slave, the init_key means you hope which types of key of the master should distribute to you,
    and the response key means which key you can distribute to the Master;
    If your BLE device act as a master, the response key means you hope which types of key of the slave should distribute to you, 
    and the init key means which key you can distribute to the slave. */
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    gpio_pad_select_gpio(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

//    xTaskCreate(&hid_demo_task, "hid_task", 2048, NULL, 5, NULL);
    xTaskCreatePinnedToCore(&hid_demo_task, "hid_task", 2048, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(&adc_val_task, "rout_task", 2048, NULL, 5, NULL, 1);
}

