// BluedroidBluetooth.cpp - ESP-IDF (Bluedroid host) based replacement for NimBLEBluetooth
#include "configuration.h"
#if !MESHTASTIC_EXCLUDE_BLUETOOTH && defined(ARCH_ESP32) && defined(USE_BLUEDROID_BLE)

#include "BluedroidBluetooth.h"
#include "BluetoothCommon.h"
#include "mesh/mesh-pb-constants.h"
#include "BluetoothStatus.h"
#include "mesh/PhoneAPI.h"
#include "sleep.h"
#include "PowerFSM.h"
#include "main.h"

#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_gap_ble_api.h>
#include <esp_gatts_api.h>
#include <esp_gatt_common_api.h>
#include <nvs_flash.h>

#include <cstring>

// Reuse logging macros already existing

BluedroidBluetooth *bluedroidBluetooth = nullptr;
BluedroidBluetooth *BluedroidBluetooth::instance = nullptr;
bool BluedroidBluetooth::passkeyShowing = false;
uint32_t BluedroidBluetooth::currentPasskey = 0;

// Index helpers for adding characteristics dynamically
static const char *TAG = "BLUEDROID_BT";

// Utility to convert a string UUID to esp_bt_uuid_t (128-bit only used here)
static esp_bt_uuid_t make128Uuid(const char *uuidStr)
{
    esp_bt_uuid_t uuid = {};
    uuid.len = ESP_UUID_LEN_128;
    // uuidStr format 8-4-4-4-12
    // Parse into bytes (big endian groups). We'll parse hex pairs ignoring dashes.
    uint8_t bytes[16];
    int bi = 0;
    for (const char *p = uuidStr; *p && bi < 16;) {
        if (*p == '-') { ++p; continue; }
        char h[3] = {p[0], p[1], 0};
        bytes[bi++] = (uint8_t)strtol(h, nullptr, 16);
        p += 2;
    }
    // ESP stores in little-endian? For esp_ble_gatts_add_char we can just place raw 128
    memcpy(uuid.uuid.uuid128, bytes, 16);
    return uuid;
}

int BluedroidBluetooth::BluetoothPhoneAPIImpl::runOnce()
{
    std::lock_guard<std::mutex> g(mtx);
    if (queued > 0) {
        for (uint8_t i = 0; i < queued; ++i) {
            handleToRadio(queue[i].data(), queue[i].size());
        }
        queued = 0;
    }
    if (!hasChecked && phoneWants) {
        numBytes = getFromRadio(fromRadioBytes);
        hasChecked = true;
    }
    return INT32_MAX; // sleep until externally poked
}

void BluedroidBluetooth::BluetoothPhoneAPIImpl::onNowHasData(uint32_t fromRadioNum)
{
    PhoneAPI::onNowHasData(fromRadioNum);
    if (BluedroidBluetooth::instance) {
        BluedroidBluetooth::instance->notifyFromNum(fromRadioNum);
    }
}

void BluedroidBluetooth::initController()
{
    // Release classic BT memory to save RAM
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        LOG_ERROR("BT controller init failed %s", esp_err_to_name(ret));
        return;
    }
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        LOG_ERROR("BT controller enable failed %s", esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_init();
    if (ret) {
        LOG_ERROR("Bluedroid init failed %s", esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        LOG_ERROR("Bluedroid enable failed %s", esp_err_to_name(ret));
        return;
    }
    esp_ble_gap_register_callback(gapEventHandler);
    esp_ble_gatts_register_callback(gattsEventHandler);
    esp_ble_gatts_app_register(0x55); // arbitrary app id
}

void BluedroidBluetooth::initSecurity()
{
    if (config.bluetooth.mode == meshtastic_Config_BluetoothConfig_PairingMode_NO_PIN) return;
    uint8_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(auth_req));
    uint8_t iocap = ESP_IO_CAP_OUT;
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(iocap));
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(init_key));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(rsp_key));
}

bool BluedroidBluetooth::isActive() { return servicesCreated; }
bool BluedroidBluetooth::isConnected() { return connected.load(); }

int BluedroidBluetooth::getRssi() { return lastRssi; }

void BluedroidBluetooth::createServices()
{
    phoneAPI = new BluetoothPhoneAPIImpl();

    // Create Mesh Service
    esp_gatt_srvc_id_t service_id = {};
    service_id.id.inst_id = 0;
    auto meshUuid = make128Uuid(MESH_SERVICE_UUID);
    service_id.id.uuid = meshUuid; // assign full uuid struct
    service_id.is_primary = true;
    esp_ble_gatts_create_service(gattsIf, &service_id, 20); // attr count estimate
    // Battery service created in CREATE_EVT after mesh service
}

void BluedroidBluetooth::startAdvertising()
{
    static esp_ble_adv_params_t adv_params = {
        .adv_int_min = 0x0200, // ~320ms
        .adv_int_max = 0x0200,
        .adv_type = ADV_TYPE_IND,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .channel_map = ADV_CHNL_ALL,
        .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    };

    // Build advertising data
    esp_ble_adv_data_t adv_data = {};
    adv_data.set_scan_rsp = false;
    adv_data.include_name = true;
    adv_data.include_txpower = true;
    adv_data.min_interval = 0x00A0; // for iOS fast Adv interval in units of 1.25ms
    adv_data.max_interval = 0x00F0;
    adv_data.flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT);
    // Service UUID list
    static uint8_t service_uuid128[16];
    auto u = make128Uuid(MESH_SERVICE_UUID);
    memcpy(service_uuid128, u.uuid.uuid128, 16);
    adv_data.service_uuid_len = 16;
    adv_data.p_service_uuid = service_uuid128;

    esp_ble_gap_config_adv_data(&adv_data);
    advertising = true; // Will actually start in GAP config complete event
}

void BluedroidBluetooth::notifyFromNum(uint32_t fromNum)
{
    if (!connected || !charHandles[(size_t)MeshCharId::FromNum]) return;
    uint8_t v[4];
    put_le32(v, fromNum);
    esp_ble_gatts_set_attr_value(charHandles[(size_t)MeshCharId::FromNum], sizeof(v), v);
    if (notifyFromNumEnabled) {
        esp_ble_gatts_send_indicate(gattsIf, connId, charHandles[(size_t)MeshCharId::FromNum], sizeof(v), v, false);
    }
}

void BluedroidBluetooth::sendLog(const uint8_t *logMessage, size_t length)
{
    if (!connected || !notifyLogEnabled || length > 512) return;
    esp_ble_gatts_set_attr_value(charHandles[(size_t)MeshCharId::LogRadio], length, (uint8_t *)logMessage);
    esp_ble_gatts_send_indicate(gattsIf, connId, charHandles[(size_t)MeshCharId::LogRadio], length, (uint8_t *)logMessage, false);
}

void BluedroidBluetooth::updateBattery(uint8_t level)
{
    if (!connected || !charHandles[(size_t)MeshCharId::BatteryLevel]) return;
    esp_ble_gatts_set_attr_value(charHandles[(size_t)MeshCharId::BatteryLevel], 1, &level);
    if (notifyBatteryEnabled)
        esp_ble_gatts_send_indicate(gattsIf, connId, charHandles[(size_t)MeshCharId::BatteryLevel], 1, &level, false);
}

void BluedroidBluetooth::showPasskey(uint32_t passkey)
{
    // Mirror NimBLE UX (simplified: just log)
    LOG_INFO("*** Enter passkey %06u on peer ***", passkey);
    passkeyShowing = true;
    updateStatusPairing(passkey);
#if HAS_SCREEN
    if (screen) {
        screen->startAlert([passkey](OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y) -> void {
            char btPIN[16] = "000000";
            snprintf(btPIN, sizeof(btPIN), "%06u", passkey);
            int x_offset = display->width() / 2;
            int y_offset = display->height() <= 80 ? 0 : 12;
            display->setTextAlignment(TEXT_ALIGN_CENTER);
            display->setFont(FONT_MEDIUM);
            display->drawString(x_offset + x, y_offset + y, "Bluetooth");
            display->setFont(FONT_SMALL);
            y_offset = display->height() == 64 ? y_offset + FONT_HEIGHT_MEDIUM - 4 : y_offset + FONT_HEIGHT_MEDIUM + 5;
            display->drawString(x_offset + x, y_offset + y, "Enter this code");
            display->setFont(FONT_LARGE);
            char pin[8];
            snprintf(pin, sizeof(pin), "%.3s %.3s", btPIN, btPIN + 3);
            y_offset = display->height() == 64 ? y_offset + FONT_HEIGHT_SMALL - 5 : y_offset + FONT_HEIGHT_SMALL + 5;
            display->drawString(x_offset + x, y_offset + y, pin);
        });
    }
#endif
}

void BluedroidBluetooth::clearBonds()
{
    int dev_num = esp_ble_get_bond_device_num();
    if (dev_num) {
        esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
        if (esp_ble_get_bond_device_list(&dev_num, dev_list) == ESP_OK) {
            for (int i = 0; i < dev_num; ++i) {
                esp_ble_remove_bond_device(dev_list[i].bd_addr);
            }
        }
        free(dev_list);
    }
}

void BluedroidBluetooth::shutdown()
{
    if (advertising) {
        esp_ble_gap_stop_advertising();
        advertising = false;
    }
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
    initController();
    initSecurity();
    // Request larger MTU for log characteristic
    esp_ble_gatt_set_local_mtu(517);
    // Create periodic RSSI timer (lazy start on connect)
    const esp_timer_create_args_t targs = {
        .callback = [](void *arg){
            auto *self = static_cast<BluedroidBluetooth*>(arg);
            if (self->connected) esp_ble_gap_read_rssi(self->peerBda);
        },
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "ble_rssi"};
    esp_timer_create(&targs, &rssiTimer);
    servicesCreated = true; // Mark early, actual handles assigned later
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
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(nullptr); // use last params
        break;
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
    case ESP_GAP_BLE_SEC_REQ_EVT:
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_PASSKEY_REQ_EVT: {
        // Generate or use fixed passkey
        uint32_t passkey = config.bluetooth.fixed_pin;
        if (config.bluetooth.mode == meshtastic_Config_BluetoothConfig_PairingMode_RANDOM_PIN) {
            passkey = random(100000, 999999);
        }
        currentPasskey = passkey;
        self->showPasskey(passkey);
        esp_ble_passkey_reply(param->ble_security.ble_req.bd_addr, true, passkey);
    } break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT: {
        if (param->ble_security.auth_cmpl.success) {
            self->updateStatusConnected();
        } else {
            self->updateStatusDisconnected();
        }
        if (passkeyShowing) {
            passkeyShowing = false;
            if (screen) screen->endAlert();
        }
    } break;
    case ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT: {
        if (param->read_rssi_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            self->lastRssi = param->read_rssi_cmpl.rssi;
        }
    } break;
    default:
        break;
    }
}

// GATTS callback static
void BluedroidBluetooth::gattsEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    auto *self = BluedroidBluetooth::instance;
    if (!self) return;
    if (event == ESP_GATTS_REG_EVT) {
        self->gattsIf = gatts_if;
        esp_ble_gap_set_device_name(getDeviceName());
        self->createServices();
        return;
    }
    switch (event) {
    case ESP_GATTS_CREATE_EVT: {
        if (param->create.status == ESP_GATT_OK) {
            if (self->meshServiceHandle == 0) {
                self->meshServiceHandle = param->create.service_handle;
                esp_ble_gatts_start_service(self->meshServiceHandle);
                // Add characteristics now
                esp_bt_uuid_t cUuid;
                esp_gatt_char_prop_t propWrite = (esp_gatt_char_prop_t)(ESP_GATT_CHAR_PROP_BIT_WRITE);
                esp_gatt_char_prop_t propRead = (esp_gatt_char_prop_t)(ESP_GATT_CHAR_PROP_BIT_READ);
                esp_gatt_char_prop_t propNotifyRead = (esp_gatt_char_prop_t)(ESP_GATT_CHAR_PROP_BIT_NOTIFY | ESP_GATT_CHAR_PROP_BIT_READ);
                // ToRadio
                cUuid = make128Uuid(TORADIO_UUID);
                esp_ble_gatts_add_char(self->meshServiceHandle, &cUuid, ESP_GATT_PERM_WRITE,
                                       propWrite, nullptr, nullptr);
                // FromRadio
                cUuid = make128Uuid(FROMRADIO_UUID);
                esp_ble_gatts_add_char(self->meshServiceHandle, &cUuid, ESP_GATT_PERM_READ,
                                       propRead, nullptr, nullptr);
                // FromNum
                cUuid = make128Uuid(FROMNUM_UUID);
                esp_ble_gatts_add_char(self->meshServiceHandle, &cUuid, ESP_GATT_PERM_READ,
                                       propNotifyRead, nullptr, nullptr);
                // LogRadio
                cUuid = make128Uuid(LOGRADIO_UUID);
                esp_ble_gatts_add_char(self->meshServiceHandle, &cUuid, ESP_GATT_PERM_READ,
                                       propNotifyRead, nullptr, nullptr);
                // Battery service separate
                esp_gatt_srvc_id_t batt_id = {};
                batt_id.is_primary = true;
                batt_id.id.inst_id = 0;
                batt_id.id.uuid.len = ESP_UUID_LEN_16;
                batt_id.id.uuid.uuid.uuid16 = 0x180F;
                esp_ble_gatts_create_service(gatts_if, &batt_id, 5);
            }
            else if (self->batteryServiceHandle == 0) {
                self->batteryServiceHandle = param->create.service_handle;
                esp_ble_gatts_start_service(self->batteryServiceHandle);
                esp_bt_uuid_t battCharUuid = {.len = ESP_UUID_LEN_16, .uuid = {.uuid16 = 0x2A19}};
                esp_gatt_char_prop_t props = (esp_gatt_char_prop_t)(ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY);
                esp_ble_gatts_add_char(self->batteryServiceHandle, &battCharUuid, ESP_GATT_PERM_READ, props, nullptr, nullptr);
            }
        }
    } break;
    case ESP_GATTS_ADD_CHAR_EVT: {
        // Assign next free slot
        if (param->add_char.status == ESP_GATT_OK) {
            // Simple linear fill based on first four mesh chars, then battery char.
            for (size_t i = 0; i < (size_t)MeshCharId::COUNT; ++i) {
                if (self->charHandles[i] == 0) { self->charHandles[i] = param->add_char.attr_handle; break; }
            }
            // Immediately add CCCD descriptor for notify types
            esp_bt_uuid_t cccdUuid = {.len = ESP_UUID_LEN_16, .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG}};
            if (param->add_char.char_uuid.len == ESP_UUID_LEN_16 && param->add_char.char_uuid.uuid.uuid16 == 0x2A19) {
                self->batteryLevelCccdHandle = 0; // will be set in ADD_CHAR_DESCR_EVT
                esp_ble_gatts_add_char_descr(self->batteryServiceHandle, &cccdUuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, nullptr, nullptr);
            } else if (param->add_char.char_uuid.len == ESP_UUID_LEN_128) {
                // All 128-bit notifies: FromNum, LogRadio
                esp_ble_gatts_add_char_descr(self->meshServiceHandle, &cccdUuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, nullptr, nullptr);
            }
        }
    } break;
    case ESP_GATTS_ADD_CHAR_DESCR_EVT: {
        if (param->add_char_descr.status == ESP_GATT_OK) {
            // Heuristically assign to first descriptor needing a handle
            if (self->fromNumCccdHandle == 0 && self->charHandles[(size_t)MeshCharId::FromNum]) self->fromNumCccdHandle = param->add_char_descr.attr_handle;
            else if (self->logRadioCccdHandle == 0 && self->charHandles[(size_t)MeshCharId::LogRadio]) self->logRadioCccdHandle = param->add_char_descr.attr_handle;
            else if (self->batteryLevelCccdHandle == 0 && self->charHandles[(size_t)MeshCharId::BatteryLevel]) self->batteryLevelCccdHandle = param->add_char_descr.attr_handle;
        }
    } break;
    case ESP_GATTS_START_EVT: {
        if (self->meshServiceHandle && self->batteryServiceHandle == 0 && param->start.status == ESP_GATT_OK) {
            // Battery service started? If not, we might be starting mesh.
        }
    } break;
    case ESP_GATTS_CONNECT_EVT: {
        self->connected = true;
        self->connId = param->connect.conn_id;
        memcpy(self->peerBda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        self->updateStatusConnected();
        esp_ble_gap_read_rssi(param->connect.remote_bda); // trigger RSSI read
        if (self->rssiTimer) esp_timer_start_periodic(self->rssiTimer, 5000000ULL); // 5s
    } break;
    case ESP_GATTS_DISCONNECT_EVT: {
        self->connected = false;
        self->notifyFromNumEnabled = false;
        self->notifyLogEnabled = false;
        self->notifyBatteryEnabled = false;
        self->updateStatusDisconnected();
        if (self->rssiTimer) esp_timer_stop(self->rssiTimer);
        esp_ble_gap_start_advertising(nullptr);
    } break;
    case ESP_GATTS_WRITE_EVT: {
        if (!param->write.is_prep) {
            uint16_t handle = param->write.handle;
            // Match ToRadio
            if (handle == self->charHandles[(size_t)MeshCharId::ToRadio]) {
                const uint8_t *val = param->write.value;
                size_t len = param->write.len;
                if (len < sizeof(self->lastToRadio) && (len != self->lastToRadioLen || memcmp(val, self->lastToRadio, len) != 0)) {
                    memcpy(self->lastToRadio, val, len);
                    self->lastToRadioLen = len;
                    if (self->phoneAPI) {
                        std::lock_guard<std::mutex> g(self->phoneAPI->mtx);
                        if (self->phoneAPI->queued < 3) {
                            self->phoneAPI->queue.emplace_back(val, val + len);
                            self->phoneAPI->queued++;
                            self->phoneAPI->setIntervalFromNow(0);
                        }
                    }
                }
            }
            else if (handle == self->fromNumCccdHandle) {
                if (param->write.len >= 2) self->notifyFromNumEnabled = (param->write.value[0] & 0x01) != 0;
            }
            else if (handle == self->logRadioCccdHandle) {
                if (param->write.len >= 2) self->notifyLogEnabled = (param->write.value[0] & 0x01) != 0;
            }
            else if (handle == self->batteryLevelCccdHandle) {
                if (param->write.len >= 2) self->notifyBatteryEnabled = (param->write.value[0] & 0x01) != 0;
            }
        }
    } break;
    case ESP_GATTS_READ_EVT: {
        uint16_t handle = param->read.handle;
        if (handle == self->charHandles[(size_t)MeshCharId::FromRadio]) {
            if (self->phoneAPI) {
                self->phoneAPI->phoneWants = true;
                int tries = 0;
                while (!self->phoneAPI->hasChecked && tries < 25) {
                    self->phoneAPI->setIntervalFromNow(0);
                    delay(10);
                    tries++;
                }
                esp_gatt_rsp_t rsp = {};
                rsp.attr_value.handle = handle;
                rsp.attr_value.len = self->phoneAPI->numBytes;
                if (rsp.attr_value.len > 0 && rsp.attr_value.len < ESP_GATT_MAX_ATTR_LEN) {
                    memcpy(rsp.attr_value.value, self->phoneAPI->fromRadioBytes, rsp.attr_value.len);
                }
                esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
                self->phoneAPI->numBytes = 0;
                self->phoneAPI->hasChecked = false;
                self->phoneAPI->phoneWants = false;
            }
        }
    } break;
    default:
        break;
    }
}

void BluedroidBluetooth::startPeriodicSyncScan()
{
    // Placeholder: could migrate periodic sync code here (currently remains in main when demo flag enabled)
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
    if (!bluetoothStatus) return;
    meshtastic::BluetoothStatus newStatus(std::to_string(passkey));
    bluetoothStatus->updateStatus(&newStatus);
}
void BluedroidBluetooth::updateStatusConnected()
{
    if (!bluetoothStatus) return;
    meshtastic::BluetoothStatus newStatus(meshtastic::BluetoothStatus::ConnectionState::CONNECTED);
    bluetoothStatus->updateStatus(&newStatus);
}
void BluedroidBluetooth::updateStatusDisconnected()
{
    if (!bluetoothStatus) return;
    meshtastic::BluetoothStatus newStatus(meshtastic::BluetoothStatus::ConnectionState::DISCONNECTED);
    bluetoothStatus->updateStatus(&newStatus);
}

// Global free function used elsewhere
void updateBatteryLevel(uint8_t level)
{
    if (bluedroidBluetooth) bluedroidBluetooth->updateBattery(level);
}

#endif // conditions
