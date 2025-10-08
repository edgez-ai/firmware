#include "PowerFSM.h"
#include "PowerMon.h"
#include "configuration.h"
#include "esp_task_wdt.h"
#include "main.h"

#if !defined(CONFIG_IDF_TARGET_ESP32S2) && !MESHTASTIC_EXCLUDE_BLUETOOTH
#include "BleOta.h"
#ifdef USE_BLUEDROID_BLE
#include "bluedroid/BluedroidBluetooth.h"
#else
#include "nimble/NimbleBluetooth.h"
#endif
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
#ifdef USE_PERIODIC_ADV_SYNC_DEMO
#include <esp_bt.h>
#include <esp_gap_ble_api.h>
#include <esp_bt_main.h>
#include <esp_gatt_common_api.h>
#include <freertos/semphr.h>
#ifndef ESP_BLE_ADV_NAME_LEN_MAX
#define ESP_BLE_ADV_NAME_LEN_MAX 32
#endif
// Forward declarations for demo helpers used later in this file
static void periodicAdvSyncDemoInit();
#endif
#include <Preferences.h>
#include <driver/rtc_io.h>
#include <nvs.h>
#include <nvs_flash.h>
extern void loadSerialNumber();
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
#ifdef USE_BLUEDROID_BLE
        if (!bluedroidBluetooth) bluedroidBluetooth = new BluedroidBluetooth();
        if (enable && !bluedroidBluetooth->isActive()) {
            powerMon->setState(meshtastic_PowerMon_State_BT_On);
            bluedroidBluetooth->setup();
        }
#else
        if (!nimbleBluetooth) nimbleBluetooth = new NimbleBluetooth();
        if (enable && !nimbleBluetooth->isActive()) {
            powerMon->setState(meshtastic_PowerMon_State_BT_On);
            nimbleBluetooth->setup();
        }
#endif
    }
}
#else
void setBluetoothEnable(bool enable) {}
void updateBatteryLevel(uint8_t level) {}
#endif

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
    loadSerialNumber();
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

#ifdef USE_PERIODIC_ADV_SYNC_DEMO
    periodicAdvSyncDemoInit();
#endif

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
}

/// loop code specific to ESP32 targets
void esp32Loop()
{
    esp_task_wdt_reset(); // service our app level watchdog

    // for debug printing
    // radio.radioIf.canSleep();
}

#ifdef USE_PERIODIC_ADV_SYNC_DEMO
// ================= Periodic Advertising Sync Demo Integration =================

// Lightweight integration of original periodic_sync_demo.c
// Only compiled when USE_PERIODIC_ADV_SYNC_DEMO is defined.

static const char *PAS_TAG = "PERIODIC_SYNC";
static SemaphoreHandle_t pas_sem = nullptr;
static bool pas_periodic_sync = false;
static char pas_remote_name[ESP_BLE_ADV_NAME_LEN_MAX] = "ESP_EXTENDED_ADV";

// Extended scan params (both uncoded & coded, active scan)
static esp_ble_ext_scan_params_t pas_ext_scan_params = {
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_duplicate = BLE_SCAN_DUPLICATE_ENABLE,
    .cfg_mask = ESP_BLE_GAP_EXT_SCAN_CFG_UNCODE_MASK | ESP_BLE_GAP_EXT_SCAN_CFG_CODE_MASK,
    .uncoded_cfg = {BLE_SCAN_TYPE_ACTIVE, 160, 80},
    .coded_cfg   = {BLE_SCAN_TYPE_ACTIVE, 160, 80},
};

static esp_ble_gap_periodic_adv_sync_params_t pas_periodic_params = {
    .filter_policy = 0,
    .sid = 0,
    .addr_type = BLE_ADDR_TYPE_RANDOM,
    .skip = 0,
    .sync_timeout = 800, // 8 s (10 ms units)
};

static void pas_validate_config(uint16_t interval_1_25ms)
{
    if (!interval_1_25ms) return;
    uint32_t ms = interval_1_25ms * 125 / 100; // integer ~ms
    uint32_t effective = ms * (pas_periodic_params.skip + 1);
    uint32_t timeout_ms = pas_periodic_params.sync_timeout * 10;
    if (effective >= timeout_ms) {
        ESP_LOGW(PAS_TAG, "Periodic sync risk: effective=%u ms >= timeout=%u ms", effective, timeout_ms);
    } else {
        ESP_LOGI(PAS_TAG, "Periodic sync ok: interval=%u ms effective=%u ms timeout=%u ms", ms, effective, timeout_ms);
    }
}

static void pas_gap_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_SET_EXT_SCAN_PARAMS_COMPLETE_EVT:
        ESP_LOGI(PAS_TAG, "Set ext scan params status=%d", param->set_ext_scan_params.status);
        if (pas_sem) xSemaphoreGive(pas_sem);
        break;
    case ESP_GAP_BLE_EXT_SCAN_START_COMPLETE_EVT:
        ESP_LOGI(PAS_TAG, "Ext scan start status=%d", param->ext_scan_start.status);
        if (pas_sem) xSemaphoreGive(pas_sem);
        break;
    case ESP_GAP_BLE_PERIODIC_ADV_SYNC_LOST_EVT:
        ESP_LOGW(PAS_TAG, "Periodic adv sync lost handle=%d", param->periodic_adv_sync_lost.sync_handle);
        pas_periodic_sync = false;
        break;
    case ESP_GAP_BLE_PERIODIC_ADV_SYNC_ESTAB_EVT:
        ESP_LOGI(PAS_TAG, "Sync estab status=%d handle=%d sid=%d interval=%.2f ms phy=%d",
                 param->periodic_adv_sync_estab.status,
                 param->periodic_adv_sync_estab.sync_handle,
                 param->periodic_adv_sync_estab.sid,
                 param->periodic_adv_sync_estab.period_adv_interval * 1.25f,
                 param->periodic_adv_sync_estab.adv_phy);
        pas_validate_config(param->periodic_adv_sync_estab.period_adv_interval);
        break;
    case ESP_GAP_BLE_EXT_ADV_REPORT_EVT: {
        if (pas_periodic_sync) break;
        uint8_t name_len = 0;
        uint8_t *adv_name = esp_ble_resolve_adv_data(param->ext_adv_report.params.adv_data,
                                                     ESP_BLE_AD_TYPE_NAME_CMPL,
                                                     &name_len);
        if (adv_name && (memcmp(adv_name, pas_remote_name, name_len) == 0)) {
            pas_periodic_sync = true;
            pas_periodic_params.sid = param->ext_adv_report.params.sid;
            pas_periodic_params.addr_type = (esp_ble_addr_type_t)param->ext_adv_report.params.addr_type;
            memcpy(pas_periodic_params.addr, param->ext_adv_report.params.addr, sizeof(esp_bd_addr_t));
            esp_err_t r = esp_ble_gap_periodic_adv_create_sync(&pas_periodic_params);
            if (r != ESP_OK) {
                ESP_LOGE(PAS_TAG, "Create sync failed %s", esp_err_to_name(r));
                pas_periodic_sync = false;
            } else {
                ESP_LOGI(PAS_TAG, "Creating periodic sync with %.*s", name_len, (char*)adv_name);
            }
        }
    } break;
    case ESP_GAP_BLE_PERIODIC_ADV_REPORT_EVT:
        ESP_LOGD(PAS_TAG, "Periodic adv report len=%d rssi=%d", param->period_adv_report.params.data_length, param->period_adv_report.params.rssi);
        break;
    default:
        break;
    }
}

static void periodicAdvSyncDemoInit()
{
    // Release classic BT memory to save RAM
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (!pas_sem) pas_sem = xSemaphoreCreateBinary();
    esp_err_t ret;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg); if (ret) { ESP_LOGE(PAS_TAG, "controller init failed %s", esp_err_to_name(ret)); return; }
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE); if (ret) { ESP_LOGE(PAS_TAG, "controller enable failed %s", esp_err_to_name(ret)); return; }
    ret = esp_bluedroid_init(); if (ret) { ESP_LOGE(PAS_TAG, "bluedroid init failed %s", esp_err_to_name(ret)); return; }
    ret = esp_bluedroid_enable(); if (ret) { ESP_LOGE(PAS_TAG, "bluedroid enable failed %s", esp_err_to_name(ret)); return; }
    ret = esp_ble_gap_register_callback(pas_gap_handler); if (ret) { ESP_LOGE(PAS_TAG, "gap cb reg failed %s", esp_err_to_name(ret)); return; }

    // Start extended scanning
    esp_ble_gap_set_ext_scan_params(&pas_ext_scan_params);
    esp_ble_gap_start_ext_scan(0, 0); // continuous
    ESP_LOGI(PAS_TAG, "Extended scan started (demo mode)");
}

#endif // USE_PERIODIC_ADV_SYNC_DEMO

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
