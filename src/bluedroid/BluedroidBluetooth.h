// BluedroidBluetooth.h - ESP-IDF (Bluedroid host) based replacement for NimbleBluetooth
#pragma once

#include "configuration.h"
#if !MESHTASTIC_EXCLUDE_BLUETOOTH && defined(ARCH_ESP32) && defined(USE_BLUEDROID_BLE)

#include <esp_bt.h>
#include <esp_gap_ble_api.h>
#include <esp_gatts_api.h>
#include <esp_bt_main.h>
#include <esp_gatt_common_api.h>
#include <esp_timer.h>
#include <atomic>
#include <mutex>
#include <vector>

#include "BluetoothCommon.h"
#include "mesh/PhoneAPI.h"
#include "PowerFSM.h"

// Forward declarations for global helpers already existing in codebase (definitions come via main.cpp and other units)
extern const char *getDeviceName();
extern void put_le32(uint8_t *dst, uint32_t v);

// Map characteristic roles to indices for handle lookup
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
    void setup() override;          // Initialize controller + GATT server + advertising
    void shutdown() override;       // Stop advertising (light shutdown)
    void deinit();                  // Full deinit (requires reboot to restore)
    void clearBonds() override;     // Delete all bonded devices
    bool isConnected() override;    // Any central connected?
    bool isActive();                // Has setup run
  int getRssi() override;         // Last cached RSSI (actively requested)
    void sendLog(const uint8_t *logMessage, size_t length);
    void notifyFromNum(uint32_t fromNum);
    void updateBattery(uint8_t level);

  void enablePeriodicAdvSyncDemo();

  private:
    // Internal helpers
    bool initController();
    void initSecurity();
    void createServices();
    void startAdvertising();
    static void gapEventHandler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
    static void gattsEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
    static BluedroidBluetooth *instance; // singleton style pointer used from static callbacks

    // State
    esp_gatt_if_t gattsIf = ESP_GATT_IF_NONE;
    bool servicesCreated = false;
    bool advertising = false;
    std::atomic<bool> connected{false};
    uint16_t connId = ESP_GATT_IF_NONE;
    esp_bd_addr_t peerBda{};
  bool notifyFromNumEnabled = false;
  bool notifyLogEnabled = false;
  bool notifyBatteryEnabled = false;
  int lastRssi = 0;
  esp_timer_handle_t rssiTimer = nullptr;
    bool isDeInited = false;

    // Characteristic handles (value handles) after creation
    uint16_t meshServiceHandle = 0;
    uint16_t batteryServiceHandle = 0;
    uint16_t charHandles[(size_t)MeshCharId::COUNT] = {0};
    uint16_t fromNumCccdHandle = 0;
    uint16_t logRadioCccdHandle = 0;
    uint16_t batteryLevelCccdHandle = 0;

    // Buffer for last ToRadio to suppress duplicates
    uint8_t lastToRadio[512] = {0};
    size_t lastToRadioLen = 0;

    // PhoneAPI bridge (mirrors NimBLE implementation pattern)
    class BluetoothPhoneAPIImpl : public PhoneAPI, public concurrency::OSThread {
      public:
        BluetoothPhoneAPIImpl() : concurrency::OSThread("BluedroidBluetooth") { queue.reserve(3); }
        std::vector<std::vector<uint8_t>> queue; // small ring buffer like usage
        std::mutex mtx;
        uint8_t queued = 0;
        bool hasFromRadio = false;
        uint8_t fromRadioBytes[meshtastic_FromRadio_size] = {0};
        size_t numBytes = 0;
        bool hasChecked = false;
        bool phoneWants = false;

      protected:
        int32_t runOnce() override;
        void onNowHasData(uint32_t fromRadioNum) override;
      public:
        bool checkIsConnected() override { return BluedroidBluetooth::instance && BluedroidBluetooth::instance->isConnected(); }
    };

    BluetoothPhoneAPIImpl *phoneAPI = nullptr;

    // Security / pairing state
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
