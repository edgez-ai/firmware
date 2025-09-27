#include "PowerFSM.h"
#include "PowerMon.h"
#include "configuration.h"
#include "esp_task_wdt.h"
#include "main.h"

#if !defined(CONFIG_IDF_TARGET_ESP32S2) && !MESHTASTIC_EXCLUDE_BLUETOOTH
#include "BleOta.h"
#include "nimble/NimbleBluetooth.h"
#endif

#include <WiFiOTA.h>

#if HAS_WIFI
#include "mesh/wifi/WiFiAPClient.h"
#endif

#include "esp_mac.h"
#include "meshUtils.h"
#include "sleep.h"
#include "soc/rtc.h"
#include "target_specific.h"
#include <Preferences.h>
#include <driver/rtc_io.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <algorithm>

// LwM2M client includes
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "examples/client/lwm2mclient.h"
#include "examples/shared/dtlsconnection.h"
#include "examples/shared/tinydtls/dtls_debug.h"
#include "driver/temp_sensor.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_wifi.h"

// Forward declarations
extern "C" {
    char *security_get_uri(lwm2m_context_t *lwm2mH, lwm2m_object_t *obj, int instanceId, char *uriBuffer, size_t bufferSize);
    char *security_get_public_id(lwm2m_context_t *lwm2mH, lwm2m_object_t *obj, int instanceId, size_t *length);
    char *security_get_secret_key(lwm2m_context_t *lwm2mH, lwm2m_object_t *obj, int instanceId, size_t *length);
}

// LwM2M client constants and global variables
#define OBJ_ARRAY_SIZE 4
static const char *LWM2M_TAG = "lwm2m_client";
static uint8_t rx_buffer[2048];
static RTC_DATA_ATTR struct timeval sleep_enter_time;

RTC_DATA_ATTR char rtc_lwm2m_server_uri[128] = {0};
RTC_DATA_ATTR char rtc_lwm2m_identity[64] = {0};
RTC_DATA_ATTR char rtc_lwm2m_psk[17] = {0};
RTC_DATA_ATTR client_data_t client_data = {0};
float tsens_out;
char serialNumber[64] = {0};

RTC_FAST_ATTR uint8_t proto_buffer[8000]; // Buffer for lwm2m proto model
const char *localPort = "56830";

#if !defined(CONFIG_IDF_TARGET_ESP32S2) && !MESHTASTIC_EXCLUDE_BLUETOOTH
void setBluetoothEnable(bool enable)
{
#ifdef USE_WS5500
    if ((config.bluetooth.enabled == true) && (config.network.wifi_enabled == false))
#elif HAS_WIFI
    if (!isWifiAvailable() && config.bluetooth.enabled == true)
#else
    if (config.bluetooth.enabled == true)
#endif
    {
        if (!nimbleBluetooth) {
            nimbleBluetooth = new NimbleBluetooth();
        }
        if (enable && !nimbleBluetooth->isActive()) {
            powerMon->setState(meshtastic_PowerMon_State_BT_On);
            nimbleBluetooth->setup();
        }
        // For ESP32, no way to recover from bluetooth shutdown without reboot
        // BLE advertising automatically stops when MCU enters light-sleep(?)
        // For deep-sleep, shutdown hardware with nimbleBluetooth->deinit(). Requires reboot to reverse
    }
}
#else
void setBluetoothEnable(bool enable) {}
void updateBatteryLevel(uint8_t level) {}
#endif

// LwM2M helper functions
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
            ESP_LOGI(LWM2M_TAG, "serialnumber: %s", serial_buf);
        } else {
            ESP_LOGW(LWM2M_TAG, "Failed to read serialnumber partition: %s", esp_err_to_name(err));
            return ESP_ERR_NOT_FOUND;
        }
        // Ensure null-terminated and copy to output
        serial_buf[sizeof(serial_buf) - 1] = '\0';
        size_t actual_len = strnlen(serial_buf, sizeof(serial_buf));
        memcpy(serial_out, serial_buf, actual_len + 1); // include null terminator
        return ESP_OK;
    } else {
        ESP_LOGE(LWM2M_TAG, "Serial partition not found");
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
        memcpy(rtc_lwm2m_identity, identity, (identity_len < sizeof(rtc_lwm2m_identity)) ? identity_len : sizeof(rtc_lwm2m_identity));
    }
    if (psk) {
        memcpy(rtc_lwm2m_psk, psk, (psk_len < sizeof(rtc_lwm2m_psk)) ? psk_len : sizeof(rtc_lwm2m_psk));
    }
}

void getMacAddr(uint8_t *dmac)
{
#if defined(CONFIG_IDF_TARGET_ESP32C6) && defined(CONFIG_SOC_IEEE802154_SUPPORTED)
    auto res = esp_base_mac_addr_get(dmac);
    assert(res == ESP_OK);
#else
    auto res = esp_efuse_mac_get_default(dmac);
    assert(res == ESP_OK);
#endif
}

#if HAS_32768HZ
#define CALIBRATE_ONE(cali_clk) calibrate_one(cali_clk, #cali_clk)

static uint32_t calibrate_one(rtc_cal_sel_t cal_clk, const char *name)
{
    const uint32_t cal_count = 1000;
    // const float factor = (1 << 19) * 1000.0f; unused var?
    uint32_t cali_val;
    for (int i = 0; i < 5; ++i) {
        cali_val = rtc_clk_cal(cal_clk, cal_count);
    }
    return cali_val;
}

void enableSlowCLK()
{
    rtc_clk_32k_enable(true);

    CALIBRATE_ONE(RTC_CAL_RTC_MUX);
    uint32_t cal_32k = CALIBRATE_ONE(RTC_CAL_32K_XTAL);

    if (cal_32k == 0) {
        LOG_DEBUG("32k XTAL OSC has not started up");
    } else {
        rtc_clk_slow_freq_set(RTC_SLOW_FREQ_32K_XTAL);
        LOG_DEBUG("Switch RTC Source to 32.768kHz succeeded, using 32k XTAL");
        CALIBRATE_ONE(RTC_CAL_RTC_MUX);
        CALIBRATE_ONE(RTC_CAL_32K_XTAL);
    }
    CALIBRATE_ONE(RTC_CAL_RTC_MUX);
    CALIBRATE_ONE(RTC_CAL_32K_XTAL);
    if (rtc_clk_slow_freq_get() != RTC_SLOW_FREQ_32K_XTAL) {
        LOG_WARN("Failed to switch 32K XTAL RTC source to 32.768kHz !!! ");
        return;
    }
}
#endif

extern "C" {
char *security_get_uri2(lwm2m_context_t *lwm2mH, lwm2m_object_t *obj, int instanceId, char *uriBuffer, size_t bufferSize) {
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
} // extern "C"


void esp32Setup()
{
    /* We explicitly don't want to do call randomSeed,
    // as that triggers the esp32 core to use a less secure pseudorandom function.
    uint32_t seed = esp_random();
    LOG_DEBUG("Set random seed %u", seed);
    randomSeed(seed);
    */

#ifdef ADC_V
    pinMode(ADC_V, INPUT);
#endif

    LOG_DEBUG("Total heap: %d", ESP.getHeapSize());
    LOG_DEBUG("Free heap: %d", ESP.getFreeHeap());
    LOG_DEBUG("Total PSRAM: %d", ESP.getPsramSize());
    LOG_DEBUG("Free PSRAM: %d", ESP.getFreePsram());

    nvs_stats_t nvs_stats;
    auto res = nvs_get_stats(NULL, &nvs_stats);
    assert(res == ESP_OK);
    LOG_DEBUG("NVS: UsedEntries %d, FreeEntries %d, AllEntries %d, NameSpaces %d", nvs_stats.used_entries, nvs_stats.free_entries,
              nvs_stats.total_entries, nvs_stats.namespace_count);

    LOG_DEBUG("Setup Preferences in Flash Storage");

    // Create object to store our persistent data
    Preferences preferences;
    preferences.begin("meshtastic", false);

    uint32_t rebootCounter = preferences.getUInt("rebootCounter", 0);
    rebootCounter++;
    preferences.putUInt("rebootCounter", rebootCounter);
    // store firmware version and hwrevision for access from OTA firmware
    String fwrev = preferences.getString("firmwareVersion", "");
    if (fwrev.compareTo(optstr(APP_VERSION)) != 0)
        preferences.putString("firmwareVersion", optstr(APP_VERSION));
    uint8_t hwven = preferences.getUInt("hwVendor", 0);
    if (hwven != HW_VENDOR)
        preferences.putUInt("hwVendor", HW_VENDOR);
    preferences.end();
    LOG_DEBUG("Number of Device Reboots: %d", rebootCounter);
#if !MESHTASTIC_EXCLUDE_BLUETOOTH
    String BLEOTA = BleOta::getOtaAppVersion();
    if (BLEOTA.isEmpty()) {
        LOG_INFO("No BLE OTA firmware available");
    } else {
        LOG_INFO("BLE OTA firmware version %s", BLEOTA.c_str());
    }
#endif
#if !MESHTASTIC_EXCLUDE_WIFI
    String version = WiFiOTA::getVersion();
    if (version.isEmpty()) {
        LOG_INFO("No WiFi OTA firmware available");
    } else {
        LOG_INFO("WiFi OTA firmware version %s", version.c_str());
    }
    WiFiOTA::initialize();
#endif

    // enableModemSleep();

// Since we are turning on watchdogs rather late in the release schedule, we really don't want to catch any
// false positives.  The wait-to-sleep timeout for shutting down radios is 30 secs, so pick 45 for now.
// #define APP_WATCHDOG_SECS 45
#define APP_WATCHDOG_SECS 90

#ifdef CONFIG_IDF_TARGET_ESP32C6
    esp_task_wdt_config_t *wdt_config = (esp_task_wdt_config_t *)malloc(sizeof(esp_task_wdt_config_t));
    wdt_config->timeout_ms = APP_WATCHDOG_SECS * 1000;
    wdt_config->trigger_panic = true;
    res = esp_task_wdt_init(wdt_config);
    assert(res == ESP_OK);
#else
    res = esp_task_wdt_init(APP_WATCHDOG_SECS, true);
    assert(res == ESP_OK);
#endif
    res = esp_task_wdt_add(NULL);
    assert(res == ESP_OK);

#if HAS_32768HZ
    enableSlowCLK();
#endif

    // Initialize LwM2M client components
    //LOG_DEBUG("Initializing LwM2M client components");
    
    // Start LwM2M client (will be implemented in a separate task)
    //startLwM2MClient();
}

/// loop code specific to ESP32 targets
void esp32Loop()
{
    esp_task_wdt_reset(); // service our app level watchdog

    // for debug printing
    // radio.radioIf.canSleep();
}

void cpuDeepSleep(uint32_t msecToWake)
{
    /*
    Some ESP32 IOs have internal pullups or pulldowns, which are enabled by default.
    If an external circuit drives this pin in deep sleep mode, current consumption may
    increase due to current flowing through these pullups and pulldowns.

    To isolate a pin, preventing extra current draw, call rtc_gpio_isolate() function.
    For example, on ESP32-WROVER module, GPIO12 is pulled up externally.
    GPIO12 also has an internal pulldown in the ESP32 chip. This means that in deep sleep,
    some current will flow through these external and internal resistors, increasing deep
    sleep current above the minimal possible value.

    Note: we don't isolate pins that are used for the LORA, LED, i2c, or ST7735 Display for the Chatter2, spi or the wake
    button(s), maybe we should not include any other GPIOs...
    */
#if SOC_RTCIO_HOLD_SUPPORTED
    static const uint8_t rtcGpios[] = {
#ifndef HELTEC_VISION_MASTER_E213
        // For this variant, >20mA leaks through the display if pin 2 held
        // Todo: check if it's safe to remove this pin for all variants
        2,
#endif
#ifndef USE_JTAG
        13,
#endif
        34, 35, 37};

    for (int i = 0; i < sizeof(rtcGpios); i++)
        rtc_gpio_isolate((gpio_num_t)rtcGpios[i]);
#endif

        // FIXME, disable internal rtc pullups/pulldowns on the non isolated pins. for inputs that we aren't using
        // to detect wake and in normal operation the external part drives them hard.
#ifdef BUTTON_PIN
        // Only GPIOs which are have RTC functionality can be used in this bit map: 0,2,4,12-15,25-27,32-39.
#if SOC_RTCIO_HOLD_SUPPORTED && SOC_PM_SUPPORT_EXT_WAKEUP
    uint64_t gpioMask = (1ULL << (config.device.button_gpio ? config.device.button_gpio : BUTTON_PIN));
#endif

#ifdef BUTTON_NEED_PULLUP
    gpio_pullup_en((gpio_num_t)BUTTON_PIN);
#endif

    // Not needed because both of the current boards have external pullups
    // FIXME change polarity in hw so we can wake on ANY_HIGH instead - that would allow us to use all three buttons (instead
    // of just the first) gpio_pullup_en((gpio_num_t)BUTTON_PIN);

#ifdef ESP32S3_WAKE_TYPE
    esp_sleep_enable_ext1_wakeup(gpioMask, ESP32S3_WAKE_TYPE);
#else
#if SOC_PM_SUPPORT_EXT_WAKEUP
#ifdef CONFIG_IDF_TARGET_ESP32
    // ESP_EXT1_WAKEUP_ALL_LOW has been deprecated since esp-idf v5.4 for any other target.
    esp_sleep_enable_ext1_wakeup(gpioMask, ESP_EXT1_WAKEUP_ALL_LOW);
#else
    esp_sleep_enable_ext1_wakeup(gpioMask, ESP_EXT1_WAKEUP_ANY_LOW);
#endif
#endif

#endif // #end ESP32S3_WAKE_TYPE
#endif

    // We want RTC peripherals to stay on
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

    esp_sleep_enable_timer_wakeup(msecToWake * 1000ULL); // call expects usecs
    esp_deep_sleep_start();                              // TBD mA sleep current (battery)
}

// LwM2M client implementation
extern "C" {
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
        
        ESP_LOGI(LWM2M_TAG, "serialNumber: %s, pinCode: %s, psk: %s, server: %s", serialNumber, pinCode, psk_key, server);
        
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
                ESP_LOGE(LWM2M_TAG, "Failed to resolve server hostname: %s", server);
                strncpy(resolved_ip, server, sizeof(resolved_ip) - 1); // fallback to original
            }
        }
        snprintf(LWM2M_SERVER_URI, sizeof(LWM2M_SERVER_URI), "coaps://%s:5685", resolved_ip);
        ESP_LOGI(LWM2M_TAG, "Resolved server hostname: %s %s", resolved_ip, LWM2M_SERVER_URI);
        
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
            ESP_LOGI(LWM2M_TAG, "Woke from timer, restoring LwM2M config from RTC memory");
            ESP_LOGI(LWM2M_TAG, "RTC LwM2M Server URI: %s", rtc_lwm2m_server_uri);
            ESP_LOGI(LWM2M_TAG, "RTC LwM2M Identity: %s", rtc_lwm2m_identity);
            ESP_LOGI(LWM2M_TAG, "RTC LwM2M PSK Key: %s", rtc_lwm2m_psk);
            objArray[0] = get_security_object(1, rtc_lwm2m_server_uri, rtc_lwm2m_identity, rtc_lwm2m_psk, strlen(rtc_lwm2m_psk), false);
        } else {
            ESP_LOGI(LWM2M_TAG, "Power cycle or other wakeup, performing bootstrap");
            char psk_identity[96] = {0};
            snprintf(psk_identity, sizeof(psk_identity), "%s%s", serialNumber, pinCode);
            objArray[0] = get_security_object(1, LWM2M_SERVER_URI, psk_identity, psk_key, strlen(psk_key), true);
        }

        objArray[1] = get_server_object(1, "U", 300, false);
        objArray[2] = get_object_device();
        objArray[3] = get_test_object();

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
            ESP_LOGE(LWM2M_TAG, "Failed to initialize LwM2M client");
            vTaskDelete(NULL);
        }
        
        client_data.lwm2mH = client_handle;
        int res = lwm2m_configure(client_handle, serialNumber, NULL, NULL, OBJ_ARRAY_SIZE, objArray);
        if (res != 0) {
            ESP_LOGE(LWM2M_TAG, "lwm2m_configure failed");
            vTaskDelete(NULL);
        }
        
        if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
            ESP_LOGI(LWM2M_TAG, "Restored LwM2M client state to STATE_READY");
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
                ESP_LOGI(LWM2M_TAG, "Received %d bytes", len);
                connection_handle_packet(client_data.connList, rx_buffer, len);
                inactivity_counter = 0; // reset inactivity counter
            } else {
                if (client_handle->state == STATE_READY) {
                    inactivity_counter++;
                    test_data_t *device_data = (test_data_t *)objArray[3]->userData;
                    if (device_data->test_integer != (int)tsens_out) {
                        ESP_LOGI(LWM2M_TAG, "Temperature changed from %d, updating resource to %.2f", device_data->test_integer, tsens_out);
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
                if (client_handle->state == STATE_READY) {
                    ESP_LOGI(LWM2M_TAG, "No message for 40 seconds and LwM2M STATE_READY, shutting off WiFi and going to deep sleep");
                    esp_wifi_stop();

                    // Save LwM2M config to RTC memory
                    ESP_LOGI(LWM2M_TAG, "Saving LwM2M config to RTC memory");
                    lwm2m_object_t * securityObj = client_data.securityObjP;
                    if (securityObj != NULL) {
                        char uri_buf[128] = {0};
                        char *uri = security_get_uri2(client_handle, securityObj, 1, uri_buf, sizeof(uri_buf));
                        size_t identity_len = 32;
                        char *identity = security_get_public_id2(client_handle, securityObj, 1, &identity_len);
                        size_t psk_len = 16;
                        char *psk = security_get_secret_key2(client_handle, securityObj, 1, &psk_len);

                        if (uri != NULL) {
                            ESP_LOGI(LWM2M_TAG, "Security Object URI: %s", uri);
                        }
                        if (identity != NULL) {
                            ESP_LOGI(LWM2M_TAG, "Security Object Identity: %s", identity);
                        }

                        save_security_info_to_rtc(uri, identity, identity_len, psk, psk_len);
                    }

                    const int wakeup_time_sec = 20;
                    printf("Enabling timer wakeup, %ds\n", wakeup_time_sec);
                    //ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000));

                    //printf("Entering deep sleep\n");
                    //gettimeofday(&sleep_enter_time, NULL);
                    //esp_deep_sleep_start();

                    inactivity_counter = 0; // reset after waking up
                } else {
                    ESP_LOGI(LWM2M_TAG, "No message for 40 seconds but LwM2M not ready, skipping sleep");
                }
            }

            vTaskDelay(10 / portTICK_PERIOD_MS);
        }

        vTaskDelete(NULL);
    }
}

void startLwM2MClient() {
    // Initialize temperature sensor
    temp_sensor_config_t temp_sensor = TSENS_CONFIG_DEFAULT();
    auto temp_res = temp_sensor_set_config(temp_sensor);
    if (temp_res == ESP_OK) {
        temp_res = temp_sensor_start();
        if (temp_res == ESP_OK) {
            temp_res = temp_sensor_read_celsius(&tsens_out);
            if (temp_res == ESP_OK) {
                LOG_DEBUG("Temperature sensor initialized: %.2f °C", tsens_out);
            }
        }
    }
    
    // Initialize networking components
    auto net_res = esp_netif_init();
    if (net_res != ESP_OK) {
        LOG_ERROR("Failed to initialize netif: %s", esp_err_to_name(net_res));
    }
    
    auto event_res = esp_event_loop_create_default();
    if (event_res != ESP_OK && event_res != ESP_ERR_INVALID_STATE) {
        LOG_ERROR("Failed to create event loop: %s", esp_err_to_name(event_res));
    }
    
    // Set DTLS debug level
    //dtls_set_log_level(DTLS_LOG_DEBUG);
    
    // Check for proto buffer data from previous sleep
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER && proto_buffer[0] != 0) {
        LOG_DEBUG("LwM2M proto model found in buffer from previous sleep");
    } else {
        LOG_DEBUG("No LwM2M proto model in buffer, starting fresh");
    }
    
    // Create LwM2M client task
    xTaskCreate(client_task, "lwm2m_client", 8192, NULL, 5, NULL);
}
