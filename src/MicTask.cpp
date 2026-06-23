// ─────────────────────────────────────────────────────────────────────────────
//  MicTask.cpp  —  INMP441 MEMS microphone via I2S
//  Wiring: VDD→3.3V  GND→GND  SCK→GPIO38  WS→GPIO39  SD→GPIO40  L/R→GND
// ─────────────────────────────────────────────────────────────────────────────
#include <Arduino.h>
#include <driver/i2s.h>
#include <cmath>
#include "Config.h"
#include "NavState.h"

static const i2s_port_t MIC_I2S_PORT = I2S_NUM_1;

bool mic_init() {
    i2s_config_t cfg = {
        .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate          = MIC_SAMPLE_RATE,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count        = 4,
        .dma_buf_len          = MIC_BUF_LEN,
        .use_apll             = false,
        .tx_desc_auto_clear   = false,
        .fixed_mclk           = 0,
    };
    esp_err_t err = i2s_driver_install(MIC_I2S_PORT, &cfg, 0, NULL);
    if (err != ESP_OK) { log_e("MicTask: i2s_driver_install failed (%d)", err); return false; }

    i2s_pin_config_t pins = {
        .bck_io_num   = PIN_MIC_SCK,
        .ws_io_num    = PIN_MIC_WS,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num  = PIN_MIC_SD,
    };
    err = i2s_set_pin(MIC_I2S_PORT, &pins);
    if (err != ESP_OK) { log_e("MicTask: i2s_set_pin failed (%d)", err); return false; }

    log_i("MicTask: INMP441 OK on SCK=%d WS=%d SD=%d", PIN_MIC_SCK, PIN_MIC_WS, PIN_MIC_SD);
    return true;
}

void MicTask(void* pvParams) {
    static int32_t buf[MIC_BUF_LEN];
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        size_t bytes_read = 0;
        i2s_read(MIC_I2S_PORT, buf, sizeof(buf), &bytes_read, pdMS_TO_TICKS(100));

        int samples = (int)(bytes_read / sizeof(int32_t));
        float db = 0.0f;
        if (samples > 0) {
            int64_t sum = 0;
            for (int i = 0; i < samples; i++) {
                int32_t s = buf[i] >> 14;
                sum += (int64_t)s * s;
            }
            float rms = sqrtf((float)sum / samples);
            db = 20.0f * log10f(rms + 1.0f);
        }

        WITH_STATE([&]{ g_state.motion.noise_db = db; });
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(200));
    }
}