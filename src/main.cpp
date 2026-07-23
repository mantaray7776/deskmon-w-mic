#include <Arduino.h>
#include "Config.h"
#include "NavState.h"

// ─────────────────────────────────────────────────────────────────────────────
//  DeskMon — ESP32-S3 desk environment monitor
//  main.cpp — hardware init, mutex creation, task spawning
//
//  Task map:
//    Core 1 │ SensorTask  (100 Hz) — IMU + compass + AHRS
//    Core 1 │ DisplayTask  (30 Hz) — TFT sprite render + OLED HUD
//    Core 0 │ WebTask  (event)     — Wi-Fi, WebSocket, OTA
//    Core 0 │ DistractTask (10 Hz) — distraction score EMA
//    Core 0 │ SessionTask   (1 Hz) — pomodoro timer, NVS persistence
// ─────────────────────────────────────────────────────────────────────────────

// Forward declarations (defined in each task .cpp)
extern bool sensor_init();
extern bool display_init();
extern void SensorTask(void*);
extern void DisplayTask(void*);
extern void WebTask(void*);
extern void DistractTask(void*);
extern void SessionTask(void*);
extern bool mic_init();
extern void MicTask(void*);


// ── Optional DNP task forward declarations ───────────────────────────────────
#if FEATURE_BATT
static void BattTask(void* pv) {
    // ADC reads VBAT through 100k/100k divider on PIN_BATT_ADC
    // ESP32-S3 ADC is 12-bit, Vref ≈ 3.3 V
    const float ADC_SCALE = 3.3f / 4095.0f;
    while (true) {
        float raw = analogRead(PIN_BATT_ADC) * ADC_SCALE * BATT_DIVIDER_RATIO;
        float pct = (raw - BATT_EMPTY_V) / (BATT_FULL_V - BATT_EMPTY_V) * 100.0f;
        pct = fminf(fmaxf(pct, 0), 100);
        bool charging = (digitalRead(PIN_BOOT) == HIGH); // VBUS sense placeholder
        WITH_STATE([&]{
            g_state.power.batt_voltage = raw;
            g_state.power.batt_percent = (uint8_t)pct;
            g_state.power.charging     = charging;
        });
        vTaskDelay(pdMS_TO_TICKS(5000)); // every 5 s
    }
}
#endif

#if FEATURE_ENCODER
#include <ESP32Encoder.h>
static ESP32Encoder s_enc;
static void EncoderTask(void* pv) {
    // Encoder pulses map to session duration adjustment (±1 min per click)
    int64_t prev = s_enc.getCount();
    while (true) {
        int64_t curr = s_enc.getCount();
        int32_t delta = (int32_t)(curr - prev);
        if (delta != 0) {
            prev = curr;
            WITH_STATE([&]{
                int32_t dur = (int32_t)g_state.session.session_duration + delta * 60;
                if (dur < 60)    dur = 60;    // minimum 1 min
                if (dur > 90*60) dur = 90*60; // maximum 90 min
                g_state.session.session_duration = (uint32_t)dur;
            });
        }
        // Push button = start / stop session
        if (digitalRead(PIN_ENC_SW) == LOW) {
            WITH_STATE([&]{
                g_state.session.active = !g_state.session.active;
                if (g_state.session.active) g_state.session.elapsed_sec = 0;
            });
            vTaskDelay(pdMS_TO_TICKS(500)); // debounce
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
#endif

// ── Startup self-test ────────────────────────────────────────────────────────
static void run_selftest() {
    log_i("=== DeskMon self-test ===");
    log_i("Chip: %s rev%d",
          ESP.getChipModel(), ESP.getChipRevision());
    log_i("Flash: %u MB  PSRAM: %u MB",
          ESP.getFlashChipSize() / (1024*1024),
          ESP.getPsramSize()     / (1024*1024));
    log_i("Free heap: %u bytes", esp_get_free_heap_size());
    log_i("CPU freq: %u MHz", getCpuFrequencyMhz());
    log_i("Features: SD=%d LORA=%d ENC=%d BATT=%d BMP=%d",
          FEATURE_SD, FEATURE_LORA, FEATURE_ENCODER,
          FEATURE_BATT, FEATURE_BMP);
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("=== SETUP START ===");

    run_selftest();

    g_state_mutex = xSemaphoreCreateMutex();
    configASSERT(g_state_mutex);

    g_state.session.session_duration = SESSION_DEFAULT_SEC;
    g_state.session.active = false;

    Serial.println("=== SENSOR INIT ===");
    if (!sensor_init()) {
        Serial.println("FATAL: sensor init failed");
        while (true) delay(1000);
    }

    Serial.println("=== DISPLAY INIT ===");
    if (!display_init()) {
        Serial.println("Display init partial");
    }

    Serial.println("=== MIC INIT ===");
    if (!mic_init()) {
        Serial.println("Mic init failed");
    }

    Serial.println("=== SPAWNING TASKS ===");
    xTaskCreatePinnedToCore(SensorTask,  "sensor",   STACK_SENSOR,   nullptr, PRI_SENSOR,   nullptr, CORE_SENSOR);
    xTaskCreatePinnedToCore(DisplayTask, "display",  STACK_DISPLAY,  nullptr, PRI_DISPLAY,  nullptr, CORE_DISPLAY);
    xTaskCreatePinnedToCore(WebTask,     "web",      STACK_WEB,      nullptr, PRI_WEB,      nullptr, CORE_WEB);
    xTaskCreatePinnedToCore(DistractTask,"distract", STACK_DISTRACT, nullptr, PRI_DISTRACT, nullptr, CORE_DISTRACT);
    xTaskCreatePinnedToCore(SessionTask, "session",  STACK_SESSION,  nullptr, PRI_SESSION,  nullptr, CORE_SESSION);
    xTaskCreatePinnedToCore(MicTask,     "mic",      6144,           nullptr, 2,            nullptr, CORE_DISTRACT);

    Serial.println("=== DONE ===");
}

// loop() is unused — all work is in FreeRTOS tasks
// Give the idle task something clean to do
void loop() {
    vTaskDelay(pdMS_TO_TICKS(10000));

}
