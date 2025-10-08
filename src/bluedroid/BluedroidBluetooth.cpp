// BluedroidBluetooth.cpp - ESP-IDF (Bluedroid host) based replacement for NimBLEBluetooth
#include "configuration.h"
#if !MESHTASTIC_EXCLUDE_BLUETOOTH && defined(ARCH_ESP32) && defined(USE_BLUEDROID_BLE)

#include "BluedroidBluetooth.h"
#include "BluetoothCommon.h"

#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_gap_ble_api.h>
#include <esp_gatt_common_api.h>

#include <cstring>

// Reuse logging macros already existing

BluedroidBluetooth *bluedroidBluetooth = nullptr;
BluedroidBluetooth *BluedroidBluetooth::instance = nullptr;
bool BluedroidBluetooth::passkeyShowing = false;
uint32_t BluedroidBluetooth::currentPasskey = 0;

int BluedroidBluetooth::BluetoothPhoneAPIImpl::runOnce()
{
    return INT32_MAX; // dummy - do nothing
}

void BluedroidBluetooth::BluetoothPhoneAPIImpl::onNowHasData(uint32_t fromRadioNum)
{
    // dummy - do nothing
}

bool BluedroidBluetooth::initController()
{
    LOG_INFO("BT controller init for periodic sync demo");
    
    // Release classic BT memory to save RAM
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    
    // Use minimal BT controller config for scanning only
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    
    // Reduce stack size to save memory - we only need scanning
    bt_cfg.controller_task_stack_size = 3072;  // Reduced from 4096
    bt_cfg.controller_task_prio = 100;
    bt_cfg.ble_max_act = 3;  // Reduced from default
    bt_cfg.normal_adv_size = 20;    // Reduced
    bt_cfg.mesh_adv_size = 0;       // Not needed
    
    esp_err_t ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        LOG_ERROR("BT controller init failed %s", esp_err_to_name(ret));
        return false;
    }
    
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        LOG_ERROR("BT controller enable failed %s", esp_err_to_name(ret));
        return false;
    }
    
    ret = esp_bluedroid_init();
    if (ret) {
        LOG_ERROR("Bluedroid init failed %s", esp_err_to_name(ret));
        return false;
    }
    
    ret = esp_bluedroid_enable();
    if (ret) {
        LOG_ERROR("Bluedroid enable failed %s", esp_err_to_name(ret));
        return false;
    }
    
    LOG_INFO("BT Initialization Complete for scanning");
    
    esp_ble_gap_register_callback(gapEventHandler);
    return true;
}

void BluedroidBluetooth::initSecurity()
{
    // dummy - do nothing
}

bool BluedroidBluetooth::isActive() { return servicesCreated; }
bool BluedroidBluetooth::isConnected() { return false; } // always false - no connections in demo

int BluedroidBluetooth::getRssi() { return 0; } // always 0 - no connections

void BluedroidBluetooth::createServices()
{
    // dummy - do nothing (no GATT services for periodic sync demo)
}

void BluedroidBluetooth::startAdvertising()
{
    // dummy - do nothing (not advertising, only scanning)
}

void BluedroidBluetooth::notifyFromNum(uint32_t fromNum)
{
    // dummy - do nothing
}

void BluedroidBluetooth::sendLog(const uint8_t *logMessage, size_t length)
{
    // dummy - do nothing
}

void BluedroidBluetooth::updateBattery(uint8_t level)
{
    // dummy - do nothing
}

void BluedroidBluetooth::showPasskey(uint32_t passkey)
{
    // dummy - do nothing
}

void BluedroidBluetooth::clearBonds()
{
    // dummy - do nothing
}

void BluedroidBluetooth::shutdown()
{
    // dummy - do nothing
}

void BluedroidBluetooth::deinit()
{
    if (isDeInited) return;
    shutdown();
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    isDeInited = true;
}

void BluedroidBluetooth::setup()
{
    if (servicesCreated) return;
    instance = this;
    if (!initController()) {
        LOG_ERROR("BT controller initialization failed, aborting setup");
        return;
    }
    servicesCreated = true;
#ifdef USE_PERIODIC_ADV_SYNC_DEMO
    enablePeriodicAdvSyncDemo();
#endif
}

// GAP callback static
void BluedroidBluetooth::gapEventHandler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    auto *self = BluedroidBluetooth::instance;
    if (!self) return;
    switch (event) {
    case ESP_GAP_BLE_SET_EXT_SCAN_PARAMS_COMPLETE_EVT:
        break;
    case ESP_GAP_BLE_EXT_SCAN_START_COMPLETE_EVT:
        LOG_INFO("Extended scan start status=%d", param->ext_scan_start.status);
        break;
    case ESP_GAP_BLE_EXT_ADV_REPORT_EVT: {
#ifdef USE_PERIODIC_ADV_SYNC_DEMO
        if (!self->pasPeriodicSync) {
            uint8_t name_len = 0;
            uint8_t *adv_name = esp_ble_resolve_adv_data(param->ext_adv_report.params.adv_data, ESP_BLE_AD_TYPE_NAME_CMPL, &name_len);
            if (adv_name && (memcmp(adv_name, self->pasRemoteName, name_len) == 0)) {
                self->pasPeriodicSync = true;
                self->pasPeriodicParams.sid = param->ext_adv_report.params.sid;
                self->pasPeriodicParams.addr_type = (esp_ble_addr_type_t)param->ext_adv_report.params.addr_type;
                memcpy(self->pasPeriodicParams.addr, param->ext_adv_report.params.addr, sizeof(esp_bd_addr_t));
                esp_err_t r = esp_ble_gap_periodic_adv_create_sync(&self->pasPeriodicParams);
                if (r != ESP_OK) {
                    LOG_ERROR("Create sync failed %s", esp_err_to_name(r));
                    self->pasPeriodicSync = false;
                } else {
                    LOG_INFO("Creating periodic sync with %.*s", name_len, (char*)adv_name);
                }
            }
        }
#endif
    } break;
    case ESP_GAP_BLE_PERIODIC_ADV_SYNC_ESTAB_EVT: {
#ifdef USE_PERIODIC_ADV_SYNC_DEMO
        LOG_INFO("Sync estab status=%d handle=%d sid=%d interval=%.2f ms phy=%d", param->periodic_adv_sync_estab.status,
                 param->periodic_adv_sync_estab.sync_handle, param->periodic_adv_sync_estab.sid,
                 param->periodic_adv_sync_estab.period_adv_interval * 1.25f, param->periodic_adv_sync_estab.adv_phy);
        self->pasValidateConfig(param->periodic_adv_sync_estab.period_adv_interval);
#endif
    } break;
    case ESP_GAP_BLE_PERIODIC_ADV_SYNC_LOST_EVT: {
#ifdef USE_PERIODIC_ADV_SYNC_DEMO
        LOG_WARN("Periodic adv sync lost handle=%d", param->periodic_adv_sync_lost.sync_handle);
        self->pasPeriodicSync = false;
#endif
    } break;
    case ESP_GAP_BLE_PERIODIC_ADV_REPORT_EVT: {
#ifdef USE_PERIODIC_ADV_SYNC_DEMO
        LOG_DEBUG("Periodic adv report len=%d rssi=%d", param->period_adv_report.params.data_length, param->period_adv_report.params.rssi);
#endif
    } break;
    default:
        break;
    }
}

// GATTS callback static - dummy, not used
void BluedroidBluetooth::gattsEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    // dummy - do nothing (no GATT server for periodic sync demo)
}


#ifdef USE_PERIODIC_ADV_SYNC_DEMO
void BluedroidBluetooth::enablePeriodicAdvSyncDemo()
{
    // Start extended scanning for periodic adv sync demo
    esp_ble_gap_set_ext_scan_params(&pasExtScanParams);
    esp_ble_gap_start_ext_scan(0, 0);
    LOG_INFO("Extended scan started (demo mode)");
}

void BluedroidBluetooth::pasValidateConfig(uint16_t interval_1_25ms)
{
    if (!interval_1_25ms) return;
    uint32_t ms = interval_1_25ms * 125 / 100;
    uint32_t effective = ms * (pasPeriodicParams.skip + 1);
    uint32_t timeout_ms = pasPeriodicParams.sync_timeout * 10;
    if (effective >= timeout_ms) {
        LOG_WARN("Periodic sync risk: effective=%u ms >= timeout=%u ms", effective, timeout_ms);
    } else {
        LOG_INFO("Periodic sync ok: interval=%u ms effective=%u ms timeout=%u ms", ms, effective, timeout_ms);
    }
}
#endif

void BluedroidBluetooth::updateStatusPairing(uint32_t passkey)
{
    // dummy - do nothing
}
void BluedroidBluetooth::updateStatusConnected()
{
    // dummy - do nothing
}
void BluedroidBluetooth::updateStatusDisconnected()
{
    // dummy - do nothing
}

// Global free function used elsewhere
void updateBatteryLevel(uint8_t level)
{
    // dummy - do nothing
}

#endif // conditions
