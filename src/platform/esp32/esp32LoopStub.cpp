// Stub / lightweight main loop implementation for ESP32 platform.
// Adds periodic feeding of the Task Watchdog for the task that was
// registered in esp32Setup(). If/when the real esp32Loop is restored,
// merge this logic or ensure that the main task calls esp_task_wdt_reset()
// at an interval < APP_WATCHDOG_SECS.

#include "main.h"
#include "esp_task_wdt.h"
#include <esp_timer.h>

void esp32Loop() {
	// Feed at ~1 Hz; inexpensive and keeps plenty of margin vs 90 s timeout.
	static uint64_t lastFeedUs = 0;
	uint64_t nowUs = esp_timer_get_time();
	if (nowUs - lastFeedUs >= 1000000ULL) { // 1 second
		esp_task_wdt_reset();
		lastFeedUs = nowUs;
	}
	// Place other periodic application housekeeping here as needed.
}
