#include "BluetoothCommon.h"
#include "configuration.h"

// NRF52 wants these constants as byte arrays
// Generated here https://yupana-engineering.com/online-uuid-to-c-array-converter - but in REVERSE BYTE ORDER
const uint8_t MESH_SERVICE_UUID_16[16u] = {0xfd, 0xea, 0x73, 0xe2, 0xca, 0x5d, 0xa8, 0x9f,
                                           0x1f, 0x46, 0xa8, 0x15, 0x18, 0xb2, 0xa1, 0x6b};
const uint8_t TORADIO_UUID_16[16u] = {0xe7, 0x01, 0x44, 0x12, 0x66, 0x78, 0xdd, 0xa1,
                                      0xad, 0x4d, 0x9e, 0x12, 0xd2, 0x76, 0x5c, 0xf7};
const uint8_t FROMRADIO_UUID_16[16u] = {0x02, 0x00, 0x12, 0xac, 0x42, 0x02, 0x78, 0xb8,
                                        0xed, 0x11, 0x93, 0x49, 0x9e, 0xe6, 0x55, 0x2c};
const uint8_t FROMNUM_UUID_16[16u] = {0x53, 0x44, 0xe3, 0x47, 0x75, 0xaa, 0x70, 0xa6,
                                      0x66, 0x4f, 0x00, 0xa8, 0x8c, 0xa1, 0x9d, 0xed};
const uint8_t LEGACY_LOGRADIO_UUID_16[16u] = {0xe2, 0xf2, 0x1e, 0xbe, 0xc5, 0x15, 0xcf, 0xaa,
                                              0x6b, 0x43, 0xfa, 0x78, 0x38, 0xd2, 0x6f, 0x6c};
const uint8_t LOGRADIO_UUID_16[16u] = {0x47, 0x95, 0xDF, 0x8C, 0xDE, 0xE9, 0x44, 0x99,
                                       0x23, 0x44, 0xE6, 0x06, 0x49, 0x6E, 0x3D, 0x5A};

// Generic sensor advertisement packing (shared for all platforms).
static uint8_t lastAdvPayload[24];
static uint8_t lastAdvLen = 0;
static uint32_t lastAdvMs = 0;
static const uint32_t SENSOR_ADV_MIN_INTERVAL_MS = 2000; // Throttle updates

int packAndAdvertiseSensorData(const SensorAdvData &d, bool force)
{
#if MESHTASTIC_EXCLUDE_BLUETOOTH
    return 0;
#else
    uint8_t payload[24];
    uint8_t *p = payload;
    /* Layout (little endian):
     * Byte0: version (lower 5 bits) + flags (upper 3 bits user extraFlags bits 5-7)
     *        version = 0x01 currently
     * Byte1: flags2: bit0 temp valid, bit1 humidity valid, bit2 pressure valid, bit3 battery valid, bits4-7 reserved
     * Byte2-3: temperature * 100 (int16)
     * Byte4: humidity (uint8 0-100)
     * Byte5-6: pressure hPa * 10 (uint16)
     * Byte7: battery percent (0-100)
     * (Future expansion could append more fields; current fixed length <= 8 bytes if fewer fields present)
     */
    uint8_t version = 0x01 & 0x1F;
    uint8_t head = version | (d.extraFlags & 0xE0);
    *p++ = head;
    uint8_t flags2 = 0;
    if (d.hasTemperature) flags2 |= 0x01;
    if (d.hasHumidity) flags2 |= 0x02;
    if (d.hasPressure) flags2 |= 0x04;
    if (d.hasBattery) flags2 |= 0x08;
    *p++ = flags2;
    if (d.hasTemperature) {
        int16_t t100 = (int16_t)(d.temperatureC * 100.0f);
        memcpy(p, &t100, sizeof(t100));
        p += sizeof(t100);
    }
    if (d.hasHumidity) {
        *p++ = (uint8_t)d.humidityPercent;
    }
    if (d.hasPressure) {
        uint16_t p10 = (uint16_t)(d.pressureHpa * 10.0f);
        memcpy(p, &p10, sizeof(p10));
        p += sizeof(p10);
    }
    if (d.hasBattery) {
        *p++ = d.batteryPercent;
    }
    uint8_t total = (uint8_t)(p - payload);
    uint32_t now = millis();
    if (!force) {
        if (now - lastAdvMs < SENSOR_ADV_MIN_INTERVAL_MS) {
            if (total == lastAdvLen && memcmp(payload, lastAdvPayload, total) == 0) {
                return 0; // unchanged within interval
            }
        }
    }
    memcpy(lastAdvPayload, payload, total);
    lastAdvLen = total;
    lastAdvMs = now;
    platformUpdateSensorAdv(payload, total);
    return total;
#endif
}