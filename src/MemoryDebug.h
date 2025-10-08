// MemoryDebug.h - Memory diagnostic utilities for ESP32
#pragma once

#ifdef ARCH_ESP32
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <esp_log.h>
#include <Arduino.h>

class SerialConsole;
extern SerialConsole *console;

constexpr const char MEM_TAG[] = "MemDbg";

#define LOG_INFO_FALLBACK(fmt, ...)                                                                                           \
    do {                                                                                                                       \
        if (console) {                                                                                                         \
            LOG_INFO(fmt, ##__VA_ARGS__);                                                                                      \
        } else {                                                                                                               \
            ESP_LOGI(MEM_TAG, fmt, ##__VA_ARGS__);                                                                             \
        }                                                                                                                      \
    } while (0)

#define LOG_WARN_FALLBACK(fmt, ...)                                                                                           \
    do {                                                                                                                       \
        if (console) {                                                                                                         \
            LOG_WARN(fmt, ##__VA_ARGS__);                                                                                      \
        } else {                                                                                                               \
            ESP_LOGW(MEM_TAG, fmt, ##__VA_ARGS__);                                                                             \
        }                                                                                                                      \
    } while (0)

// Print detailed memory information
inline void printMemoryInfo(const char *label = nullptr) {
    if (label) {
        LOG_INFO_FALLBACK("=== Memory Info: %s ===", label);
    } else {
        LOG_INFO_FALLBACK("=== Memory Info ===");
    }
    
    LOG_INFO_FALLBACK("Total heap: %u bytes", ESP.getHeapSize());
    LOG_INFO_FALLBACK("Free heap: %u bytes", esp_get_free_heap_size());
    LOG_INFO_FALLBACK("Min free heap ever: %u bytes", esp_get_minimum_free_heap_size());
    LOG_INFO_FALLBACK("Largest free block: %u bytes", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    
    size_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    size_t total_internal = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
    LOG_INFO_FALLBACK("Internal RAM: %u / %u bytes free (%.1f%% used)", 
        free_internal, total_internal, 
        100.0 * (total_internal - free_internal) / total_internal);
    
#ifdef BOARD_HAS_PSRAM
    size_t free_spiram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    size_t total_spiram = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    if (total_spiram > 0) {
        LOG_INFO_FALLBACK("SPIRAM: %u / %u bytes free (%.1f%% used)", 
              free_spiram, total_spiram,
              100.0 * (total_spiram - free_spiram) / total_spiram);
    } else {
        LOG_INFO_FALLBACK("SPIRAM: Not available");
    }
#endif

    size_t free_dma = heap_caps_get_free_size(MALLOC_CAP_DMA);
    LOG_INFO_FALLBACK("DMA capable: %u bytes free", free_dma);
    
    LOG_INFO_FALLBACK("Heap fragmentation: %u%%", 
          100 - (100 * heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) / esp_get_free_heap_size()));
}

// Print compact one-line memory summary
inline void printMemorySummary(const char *label = nullptr) {
    if (label) {
      LOG_INFO_FALLBACK("[MEM] %s: Free=%u Min=%u Largest=%u", 
              label,
              esp_get_free_heap_size(),
              esp_get_minimum_free_heap_size(),
              heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    } else {
      LOG_INFO_FALLBACK("[MEM] Free=%u Min=%u Largest=%u", 
              esp_get_free_heap_size(),
              esp_get_minimum_free_heap_size(),
              heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    }
}

// Check if there's enough memory for a given allocation
inline bool checkMemoryAvailable(size_t required_bytes, const char *purpose = nullptr) {
    size_t largest_block = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    bool available = largest_block >= required_bytes;
    
    if (!available) {
        if (purpose) {
        LOG_WARN_FALLBACK("Insufficient memory for %s: need %u bytes, largest block=%u", 
                    purpose, required_bytes, largest_block);
        } else {
        LOG_WARN_FALLBACK("Insufficient memory: need %u bytes, largest block=%u", 
                    required_bytes, largest_block);
        }
    }
    
    return available;
}

#else
// Stubs for non-ESP32 platforms
inline void printMemoryInfo(const char *label = nullptr) {}
inline void printMemorySummary(const char *label = nullptr) {}
inline bool checkMemoryAvailable(size_t required_bytes, const char *purpose = nullptr) { return true; }
#endif
