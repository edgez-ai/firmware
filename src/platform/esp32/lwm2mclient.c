/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/portable.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <esp_timer.h>
#include <string.h>
#include <strings.h>
#include <sys/param.h>
#include "esp_wifi.h"
#include "esp_sleep.h" // Required for esp_sleep_enable_ext0_wakeup
#include "driver/temp_sensor.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "lwm2mclient.h"
#include "pb.h"
#include "pb_decode.h"
//#define LWM2M_SERVER_URI "coaps://192.168.10.148:5685"
#define WITH_TINYDTLS 1
#define OBJ_ARRAY_SIZE 4
static const char *TAG = "client";
static uint8_t rx_buffer[2048];
static RTC_DATA_ATTR struct timeval sleep_enter_time;

RTC_DATA_ATTR char rtc_lwm2m_server_uri[128] = {0};
RTC_DATA_ATTR char rtc_lwm2m_identity[64] = {0};
RTC_DATA_ATTR char rtc_lwm2m_psk[17] = {0};
RTC_DATA_ATTR client_data_t client_data = {0};
float tsens_out;
char serialNumber[64] = {0};

RTC_FAST_ATTR uint8_t proto_buffer[8000]; // Buffer for lwm2m proto model

Lwm2mModel proto_model = Lwm2mModel_init_zero;

esp_err_t flash_readSerialFromFactory(char *serial_out) {
  const esp_partition_t *serial_part = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "serialnumber");
    char serial_buf[256] = {0};
    if (serial_part) {
        esp_err_t err = esp_partition_read(serial_part, 0, serial_buf, sizeof(serial_buf) - 1);
        if (err == ESP_OK) {
            // Trim trailing non-printable characters
            int len = sizeof(serial_buf) - 1;
            while (len > 0 && (serial_buf[len-1] < 32 || serial_buf[len-1] > 126)) {
                serial_buf[len-1] = '\0';
                len--;
            }
            ESP_LOGI(TAG, "serialnumber: %s", serial_buf);
        } else {
            ESP_LOGW(TAG, "Failed to read serialnumber partition: %s", esp_err_to_name(err));
            return ESP_ERR_NOT_FOUND;
        }
        // Ensure null-terminated and copy to output
        serial_buf[sizeof(serial_buf) - 1] = '\0';
        size_t actual_len = strnlen(serial_buf, sizeof(serial_buf));
        memcpy(serial_out, serial_buf, actual_len + 1); // include null terminator
        return ESP_OK;
    }else {
        ESP_LOGE(TAG, "Serial partition not found");
        return ESP_ERR_NOT_FOUND;
    }


}

// Helper to save security info to RTC
void save_security_info_to_rtc(const char *uri, const char *identity, size_t identity_len, const char *psk, size_t psk_len) {
    if (uri) {
        strncpy(rtc_lwm2m_server_uri, uri, sizeof(rtc_lwm2m_server_uri) - 1);
        rtc_lwm2m_server_uri[sizeof(rtc_lwm2m_server_uri) - 1] = '\0';
    }
    if (identity) {
        memcpy(rtc_lwm2m_identity, identity, MIN(identity_len, sizeof(rtc_lwm2m_identity)));
    }
    if (psk) {
        memcpy(rtc_lwm2m_psk, psk, MIN(psk_len, sizeof(rtc_lwm2m_psk)));
    }
}

const char *localPort = "56830";


/********************* Security Obj Helpers **********************/
char *security_get_uri2(lwm2m_context_t *lwm2mH, lwm2m_object_t *obj, int instanceId, char *uriBuffer,
                       size_t bufferSize) {
    int size = 1;
    lwm2m_data_t * dataP = lwm2m_data_new(size);
    dataP->id = 0; // security server uri

    obj->readFunc(lwm2mH, instanceId, &size, &dataP, obj);
    if (dataP != NULL &&
            dataP->type == LWM2M_TYPE_STRING &&
            dataP->value.asBuffer.length > 0)
    {
        if (bufferSize > dataP->value.asBuffer.length){
            memset(uriBuffer,0,dataP->value.asBuffer.length+1);
            strncpy(uriBuffer,(const char *)dataP->value.asBuffer.buffer,dataP->value.asBuffer.length);
            lwm2m_data_free(size, dataP);
            return uriBuffer;
        }
    }
    lwm2m_data_free(size, dataP);
    return NULL;
}

char *security_get_public_id2(lwm2m_context_t *lwm2mH, lwm2m_object_t *obj, int instanceId, size_t *length) {
    int size = 1;
    lwm2m_data_t * dataP = lwm2m_data_new(size);
    dataP->id = 3; // public key or id

    obj->readFunc(lwm2mH, instanceId, &size, &dataP, obj);
    if (dataP != NULL &&
        dataP->type == LWM2M_TYPE_OPAQUE)
    {
        char * buff;

        buff = (char*)lwm2m_malloc(dataP->value.asBuffer.length);
        if (buff != 0)
        {
            memcpy(buff, dataP->value.asBuffer.buffer, dataP->value.asBuffer.length);
            *length = dataP->value.asBuffer.length;
        }
        lwm2m_data_free(size, dataP);

        return buff;
    } else {
        return NULL;
    }
}

char *security_get_secret_key2(lwm2m_context_t *lwm2mH, lwm2m_object_t *obj, int instanceId, size_t *length) {
    int size = 1;
    lwm2m_data_t * dataP = lwm2m_data_new(size);
    dataP->id = 5; // secret key

    obj->readFunc(lwm2mH, instanceId, &size, &dataP, obj);
    if (dataP != NULL &&
        dataP->type == LWM2M_TYPE_OPAQUE)
    {
        char * buff;

        buff = (char*)lwm2m_malloc(dataP->value.asBuffer.length);
        if (buff != 0)
        {
            memcpy(buff, dataP->value.asBuffer.buffer, dataP->value.asBuffer.length);
            *length = dataP->value.asBuffer.length;
        }
        lwm2m_data_free(size, dataP);

        return buff;
    } else {
        return NULL;
    }
}

static void client_task(void *pvParameters)
{
    char serial[256] = {0};
    flash_readSerialFromFactory(serial);
    // Split serial string into segments: serialNumber:PinCode:PSK:Server
    char pinCode[32] = {0};
    char psk_key[64] = {0};
    char server[128] = {0};
    char *token;
    char *rest = serial;
    // serialNumber
    token = strtok_r(rest, ":", &rest);
    if (token) strncpy(serialNumber, token, sizeof(serialNumber) - 1);
    // pinCode
    token = strtok_r(NULL, ":", &rest);
    if (token) strncpy(pinCode, token, sizeof(pinCode) - 1);
    // psk
    token = strtok_r(NULL, ":", &rest);
    if (token) strncpy(psk_key, token, sizeof(psk_key) - 1);
    // server
    token = strtok_r(NULL, ":", &rest);
    if (token) strncpy(server, token, sizeof(server) - 1);
    ESP_LOGI(TAG, "serialNumber: %s, pinCode: %s, psk: %s, server: %s", serialNumber, pinCode, psk_key, server);
    char LWM2M_SERVER_URI[160] = {0};
    char resolved_ip[64] = {0};
    // Check if server is already an IP address
    struct in_addr addr;
    if (inet_aton(server, &addr) != 0) {
        // server is already an IP address
        strncpy(resolved_ip, server, sizeof(resolved_ip) - 1);
    } else {
        // Try to resolve hostname
        struct addrinfo hints = {0};
        struct addrinfo *res = NULL;
        hints.ai_family = AF_INET;
        int err = getaddrinfo(server, NULL, &hints, &res);
        if (err == 0 && res != NULL) {
            struct sockaddr_in *ipv4 = (struct sockaddr_in *)res->ai_addr;
            strncpy(resolved_ip, inet_ntoa(ipv4->sin_addr), sizeof(resolved_ip) - 1);
            freeaddrinfo(res);
        } else {
            ESP_LOGE(TAG, "Failed to resolve server hostname: %s", server);
            strncpy(resolved_ip, server, sizeof(resolved_ip) - 1); // fallback to original
        }
    }
    snprintf(LWM2M_SERVER_URI, sizeof(LWM2M_SERVER_URI), "coaps://%s:5685", resolved_ip);
    ESP_LOGI(TAG, "Resolved server hostname: %s %s", resolved_ip, LWM2M_SERVER_URI);
    lwm2m_object_t *objArray[OBJ_ARRAY_SIZE] = {0};
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    struct timeval now;
    gettimeofday(&now, NULL);
    int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;

    switch (esp_sleep_get_wakeup_cause()) {
        case ESP_SLEEP_WAKEUP_TIMER: {
            printf("Wake up from timer. Time spent in deep sleep: %dms\n", sleep_time_ms);
            break;
        }


        case ESP_SLEEP_WAKEUP_EXT1: {
            uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
            if (wakeup_pin_mask != 0) {
                int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
                printf("Wake up from GPIO %d\n", pin);
            } else {
                printf("Wake up from GPIO\n");
            }
            break;
        }

        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:
            printf("Not a deep sleep reset\n");
    }

    if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
        //dtls_set_log_level(DTLS_LOG_DEBUG);
        ESP_LOGI(TAG, "Woke from timer, restoring LwM2M config from RTC memory");
        ESP_LOGI(TAG, "RTC LwM2M Server URI: %s", rtc_lwm2m_server_uri);
        ESP_LOGI(TAG, "RTC LwM2M Identity: %s", rtc_lwm2m_identity);
        ESP_LOGI(TAG, "RTC LwM2M PSK Key: %s", rtc_lwm2m_psk);
        objArray[0] = get_security_object(1, rtc_lwm2m_server_uri, rtc_lwm2m_identity, rtc_lwm2m_psk, strlen(rtc_lwm2m_psk), false);
    } else {
        ESP_LOGI(TAG, "Power cycle or other wakeup, performing bootstrap");
        char psk_identity[96] = {0};
        snprintf(psk_identity, sizeof(psk_identity), "%s%s", serialNumber, pinCode);
        //const char *psk_key = "samplepsk1234";
        objArray[0] = get_security_object(1, LWM2M_SERVER_URI, psk_identity, psk_key, strlen(psk_key), true);
    }

    //objArray[0] = get_security_object(1, lwm2m_server_uri, psk_identity, psk_key, strlen(psk_key), is_bootstrap);
    objArray[1] = get_server_object(1, "U", 300, false);
    objArray[2] = get_object_device();
    objArray[3] = get_test_object();

    //memset(&client_data, 0, sizeof(client_data_t));
    client_data.sock = create_socket(localPort, client_data.addressFamily);
    if (client_data.sock < 0) {
        fprintf(stderr, "Failed to open socket: %d %s\r\n", errno, strerror(errno));
        return;
    }
    // Set socket to non-blocking mode
    int flags = lwip_fcntl(client_data.sock, F_GETFL, 0);
    lwip_fcntl(client_data.sock, F_SETFL, flags | O_NONBLOCK);
    client_data.securityObjP = objArray[0];
    lwm2m_context_t *client_handle = lwm2m_init(&client_data);
    if (!client_handle) {
        ESP_LOGE(TAG, "Failed to initialize LwM2M client");
        vTaskDelete(NULL);
    }
    client_data.lwm2mH = client_handle;
    int res = lwm2m_configure(client_handle, serialNumber, NULL, NULL, OBJ_ARRAY_SIZE, objArray);
    if (res != 0) {
        ESP_LOGE(TAG, "lwm2m_configure failed");
        vTaskDelete(NULL);
    }
    if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
       // client_handle->state = STATE_READY;
        ESP_LOGI(TAG, "Restored LwM2M client state to STATE_READY");
    }
    int inactivity_counter = 0;
    const int inactivity_limit = 40; // 40 seconds
    while (1) {
        time_t tv = lwm2m_gettime();
        lwm2m_step(client_handle, &tv);

        struct sockaddr_storage source_addr;
        socklen_t socklen = sizeof(source_addr);
        int len = recvfrom(client_data.sock, rx_buffer, sizeof(rx_buffer), 0, (struct sockaddr *)&source_addr, &socklen);

        if (len > 0) {
            ESP_LOGI(TAG, "Received %d bytes", len);
            connection_handle_packet(client_data.connList, rx_buffer, len);
            inactivity_counter = 0; // reset inactivity counter
        } else {
            if (client_handle->state == STATE_READY) {
                inactivity_counter++;
                test_data_t *device_data = (test_data_t *)objArray[4]->userData;
                if (device_data->test_integer != (int)tsens_out) {
                    ESP_LOGI(TAG, "Temperature changed from %d, updating resource to %.2f", device_data->test_integer, tsens_out);
                    device_data->test_integer = (int)tsens_out;
                    lwm2m_uri_t uri;
                    uri.objectId = 3442; // Device object
                    uri.instanceId = 0; // Instance 0
                    uri.resourceId = 120; // Test integer resource
                    lwm2m_resource_value_changed(client_handle, &uri);
                }

            }
        }

        if (inactivity_counter >= inactivity_limit) {
            if ( client_handle->state == STATE_READY ) {
                ESP_LOGI(TAG, "No message for 5 seconds and LwM2M STATE_READY, shutting off WiFi and going to deep sleep");
                esp_wifi_stop();

                // print security object info being saved to RTC memory
                ESP_LOGI(TAG, "Saving LwM2M config to RTC memory");
                lwm2m_object_t * securityObj = client_data.securityObjP;
                // copy struct to RTC memory
               // memcpy(&securityObjP, securityObj, sizeof(lwm2m_object_t));
               // memcpy(&serverObject, client_data.serverObject, sizeof(lwm2m_object_t));
               // memcpy(&connList, client_data.connList, sizeof(dtls_connection_t));
                if (securityObj != NULL) {
                   // print security object info
                    char uri_buf[128] = {0};

                    char *uri = security_get_uri2(client_handle, securityObj, 1, uri_buf, sizeof(uri_buf));
                    size_t identity_len = 32;
                    char *identity = security_get_public_id2(client_handle, securityObj, 1, &identity_len);
                    size_t psk_len = 16;
                    char *psk = security_get_secret_key2(client_handle, securityObj, 1, &psk_len);

                    if (uri != NULL) {
                            ESP_LOGI(TAG, "Security Object URI: %s", uri);
                    }
                    if (identity != NULL) {
                            ESP_LOGI(TAG, "Security Object Identity: %s", identity);
                    }

                    save_security_info_to_rtc(uri, identity,identity_len, psk, psk_len);
                }

                const int wakeup_time_sec = 20;
                printf("Enabling timer wakeup, %ds\n", wakeup_time_sec);
                ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000));



                printf("Entering deep sleep\n");

                // get deep sleep enter time
                gettimeofday(&sleep_enter_time, NULL);

                // enter deep sleep
                esp_deep_sleep_start();

                inactivity_counter = 0; // reset after waking up
            } else {
                ESP_LOGI(TAG, "No message for 5 seconds but LwM2M not ready, skipping sleep");
            }
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}



#define FUNC_SEND_WAIT_SEM(func, sem) do {\
        esp_err_t __err_rc = (func);\
        if (__err_rc != ESP_OK) { \
            ESP_LOGE(LOG_TAG, "%s, message send fail, error = %d", __func__, __err_rc); \
        } \
        xSemaphoreTake(sem, portMAX_DELAY); \
} while(0);

#define LOG_TAG "PERIODIC_SYNC"
#define EXT_SCAN_DURATION     0
#define EXT_SCAN_PERIOD       0

static char remote_device_name[ESP_BLE_ADV_NAME_LEN_MAX] = "ESP_EXTENDED_ADV";
static SemaphoreHandle_t test_sem = NULL;

static esp_ble_ext_scan_params_t ext_scan_params = {
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_duplicate = BLE_SCAN_DUPLICATE_ENABLE,
    .cfg_mask = ESP_BLE_GAP_EXT_SCAN_CFG_UNCODE_MASK | ESP_BLE_GAP_EXT_SCAN_CFG_CODE_MASK,
    .uncoded_cfg = {BLE_SCAN_TYPE_ACTIVE, 40, 40},
    .coded_cfg = {BLE_SCAN_TYPE_ACTIVE, 40, 40},
};

static esp_ble_gap_periodic_adv_sync_params_t periodic_adv_sync_params = {
    .filter_policy = 0,
    .sid = 0,
    .addr_type = BLE_ADDR_TYPE_RANDOM,
    .skip = 10,
    .sync_timeout = 1000,
};

bool periodic_sync = false;


static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_SET_EXT_SCAN_PARAMS_COMPLETE_EVT:
        xSemaphoreGive(test_sem);
        ESP_LOGI(LOG_TAG, "Extended scanning params set, status %d", param->set_ext_scan_params.status);
        break;
    case ESP_GAP_BLE_EXT_SCAN_START_COMPLETE_EVT:
        xSemaphoreGive(test_sem);
        ESP_LOGI(LOG_TAG, "Extended scanning start, status %d", param->ext_scan_start.status);
        break;
    case ESP_GAP_BLE_EXT_SCAN_STOP_COMPLETE_EVT:
        xSemaphoreGive(test_sem);
        ESP_LOGI(LOG_TAG, "Extended scanning stop, status %d", param->period_adv_stop.status);
        break;
    case ESP_GAP_BLE_PERIODIC_ADV_CREATE_SYNC_COMPLETE_EVT:
        ESP_LOGI(LOG_TAG, "Periodic advertising create sync, status %d", param->period_adv_create_sync.status);
        break;
    case ESP_GAP_BLE_PERIODIC_ADV_SYNC_CANCEL_COMPLETE_EVT:
        ESP_LOGI(LOG_TAG, "Periodic advertising sync cancel, status %d", param->period_adv_sync_cancel.status);
        break;
    case ESP_GAP_BLE_PERIODIC_ADV_SYNC_TERMINATE_COMPLETE_EVT:
        ESP_LOGI(LOG_TAG, "Periodic advertising sync terminate, status %d", param->period_adv_sync_term.status);
        break;
    case ESP_GAP_BLE_PERIODIC_ADV_SYNC_LOST_EVT:
        ESP_LOGI(LOG_TAG, "Periodic advertising sync lost, sync handle %d", param->periodic_adv_sync_lost.sync_handle);
        break;
    case ESP_GAP_BLE_PERIODIC_ADV_SYNC_ESTAB_EVT:
        ESP_LOGI(LOG_TAG, "Periodic advertising sync establish, status %d", param->periodic_adv_sync_estab.status);
        ESP_LOGI(LOG_TAG, "address "ESP_BD_ADDR_STR"", ESP_BD_ADDR_HEX(param->periodic_adv_sync_estab.adv_addr));
        ESP_LOGI(LOG_TAG, "sync handle %d sid %d perioic adv interval %d adv phy %d", param->periodic_adv_sync_estab.sync_handle,
                                                                                      param->periodic_adv_sync_estab.sid,
                                                                                      param->periodic_adv_sync_estab.period_adv_interval,
                                                                                      param->periodic_adv_sync_estab.adv_phy);
        break;
    case ESP_GAP_BLE_EXT_ADV_REPORT_EVT: {
        uint8_t *adv_name = NULL;
        uint8_t adv_name_len = 0;
	    adv_name = esp_ble_resolve_adv_data_by_type(param->ext_adv_report.params.adv_data,
                                            param->ext_adv_report.params.adv_data_len,
                                            ESP_BLE_AD_TYPE_NAME_CMPL,
                                            &adv_name_len);
	    if ((adv_name != NULL) && (memcmp(adv_name, remote_device_name, adv_name_len) == 0) && !periodic_sync) {
            // Note: If there are multiple devices with the same device name, the device may sync to an unintended one.
            // It is recommended to change the default device name to ensure it is unique.
            periodic_sync = true;
	        char adv_temp_name[30] = {'0'};
	        memcpy(adv_temp_name, adv_name, adv_name_len);
	        ESP_LOGI(LOG_TAG, "Create sync with the peer device %s", adv_temp_name);
            periodic_adv_sync_params.sid = param->ext_adv_report.params.sid;
	        periodic_adv_sync_params.addr_type = param->ext_adv_report.params.addr_type;
	        memcpy(periodic_adv_sync_params.addr, param->ext_adv_report.params.addr, sizeof(esp_bd_addr_t));
            esp_ble_gap_periodic_adv_create_sync(&periodic_adv_sync_params);
	    }
    }
        break;
    case ESP_GAP_BLE_PERIODIC_ADV_REPORT_EVT:
        ESP_LOGI(LOG_TAG, "Periodic adv report, sync handle %d, data status %d, data len %d, rssi %d", param->period_adv_report.params.sync_handle,
                                                                                                    param->period_adv_report.params.data_status,
                                                                                                    param->period_adv_report.params.data_length,
                                                                                                    param->period_adv_report.params.rssi);
        // print periodic adv data
        ESP_LOG_BUFFER_HEXDUMP(LOG_TAG, param->period_adv_report.params.data, param->period_adv_report.params.data_length, ESP_LOG_INFO);
        break;

    default:
        break;
    }
}

void app_main33(void)
{
        // Default config
    temp_sensor_config_t temp_sensor = TSENS_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(temp_sensor_set_config(temp_sensor));
    ESP_ERROR_CHECK(temp_sensor_start());

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());
    ESP_ERROR_CHECK(temp_sensor_read_celsius(&tsens_out));
    ESP_LOGI(TAG, "Temperature: %.2f °C", tsens_out);

    // if proto buffer is not empty, parse it
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER && proto_buffer[0] != 0) {
        pb_istream_t stream = pb_istream_from_buffer(proto_buffer, sizeof(proto_buffer));
        if (!pb_decode(&stream, Lwm2mModel_fields, &proto_model)) {
            ESP_LOGE(TAG, "Failed to parse LwM2M proto model: %s", PB_GET_ERROR(&stream));
        } else {
            ESP_LOGI(TAG, "Parsed LwM2M proto model successfully");
        }
    } else {
        ESP_LOGI(TAG, "No LwM2M proto model in buffer, starting fresh");
    }

    xTaskCreate(client_task, "client_lwm2m", 8192, NULL, 5, NULL);

    // handle device advertisement and report
    //xTaskCreate(device_report_task, "device_advertisement", 8192, NULL, 5, NULL);
    
        // handle device advertisement and report
        ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_err_t ret;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(LOG_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(LOG_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(LOG_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(LOG_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(LOG_TAG, "gap register error, error code = %x", ret);
        return;
    }

    vTaskDelay(200 / portTICK_PERIOD_MS);

    test_sem = xSemaphoreCreateBinary();

    FUNC_SEND_WAIT_SEM(esp_ble_gap_set_ext_scan_params(&ext_scan_params), test_sem);
    FUNC_SEND_WAIT_SEM(esp_ble_gap_start_ext_scan(EXT_SCAN_DURATION, EXT_SCAN_PERIOD), test_sem);


}
