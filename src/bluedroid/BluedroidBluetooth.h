// BluedroidBluetooth.h - ESP-IDF (Bluedroid host) based replacement for NimbleBluetooth
#pragma once

#include "configuration.h"
#if !MESHTASTIC_EXCLUDE_BLUETOOTH && defined(ARCH_ESP32) && defined(USE_BLUEDROID_BLE)

#include <esp_bt.h>
#include <esp_gap_ble_api.h>
#include <esp_gatts_api.h>
#include <esp_bt_main.h>
#include <esp_gatt_common_api.h>

#include "BluetoothCommon.h"
#include "mesh/PhoneAPI.h"

// Forward declarations for global helpers already existing in codebase (definitions come via main.cpp and other units)
extern const char *getDeviceName();

// Map characteristic roles to indices for handle lookup - kept for compatibility
enum class MeshCharId : uint8_t {
    ToRadio = 0,
    FromRadio,
    FromNum,
    LogRadio,
    BatteryLevel,
    COUNT
};

class BluedroidBluetooth : public BluetoothApi
{
  public:
    BluedroidBluetooth() = default;
    void setup() override;          // Initialize controller for scanning only
    void shutdown() override;       // Dummy - do nothing
    void deinit();                  // Full deinit (requires reboot to restore)
    void clearBonds() override;     // Dummy - do nothing
    bool isConnected() override;    // Always false
    bool isActive();                // Has setup run
    int getRssi() override;         // Always 0
    void sendLog(const uint8_t *logMessage, size_t length); // Dummy
    void notifyFromNum(uint32_t fromNum); // Dummy
    void updateBattery(uint8_t level); // Dummy

    void enablePeriodicAdvSyncDemo();

  private:
    // Internal helpers
    bool initController();
    void initSecurity(); // Dummy
    void createServices(); // Dummy
    void startAdvertising(); // Dummy
    static void gapEventHandler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
    static void gattsEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
    static BluedroidBluetooth *instance; // singleton style pointer used from static callbacks

    // Minimal state
    bool servicesCreated = false;
    bool isDeInited = false;

    // PhoneAPI bridge - kept for compatibility but not used
    class BluetoothPhoneAPIImpl : public PhoneAPI, public concurrency::OSThread {
      public:
        BluetoothPhoneAPIImpl() : concurrency::OSThread("BluedroidBluetooth") { }
      protected:
        int32_t runOnce() override;
        void onNowHasData(uint32_t fromRadioNum) override;
      public:
        bool checkIsConnected() override { return false; }
    };

    BluetoothPhoneAPIImpl *phoneAPI = nullptr;

    // Security / pairing state - dummy
    static bool passkeyShowing;
    static uint32_t currentPasskey;
    void showPasskey(uint32_t passkey);
    void updateStatusPairing(uint32_t passkey);
    void updateStatusConnected();
    void updateStatusDisconnected();

#ifdef USE_PERIODIC_ADV_SYNC_DEMO
  // Periodic advertising sync demo state
  bool pasPeriodicSync = false;
  char pasRemoteName[32] = "ESP_EXTENDED_ADV";
  esp_ble_ext_scan_params_t pasExtScanParams = {
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_duplicate = BLE_SCAN_DUPLICATE_ENABLE,
    .cfg_mask = ESP_BLE_GAP_EXT_SCAN_CFG_UNCODE_MASK | ESP_BLE_GAP_EXT_SCAN_CFG_CODE_MASK,
    .uncoded_cfg = {BLE_SCAN_TYPE_ACTIVE, 160, 80},
    .coded_cfg   = {BLE_SCAN_TYPE_ACTIVE, 160, 80},
  };
  esp_ble_gap_periodic_adv_sync_params_t pasPeriodicParams = {
    .filter_policy = 0,
    .sid = 0,
    .addr_type = BLE_ADDR_TYPE_RANDOM,
    .skip = 0,
    .sync_timeout = 800,
  };
  void pasValidateConfig(uint16_t interval_1_25ms);
#endif
};

// Global pointer for other translation units to update battery/log etc.
extern BluedroidBluetooth *bluedroidBluetooth;

#endif // conditions
