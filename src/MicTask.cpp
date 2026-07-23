// ─────────────────────────────────────────────────────────────────────────────
//  MicTask.cpp  —  INMP441 MEMS microphone via I2S
//  Wiring: VDD→3.3V  GND→GND  SCK→GPIO38  WS→GPIO39  SD→GPIO40  L/R→GND
//
//  Also drives VoiceCommand: "hey deskmon" starts the session, "hey deskmon
//  stop" ends it — same effect as pressing the encoder button or hitting
//  start/stop from the web UI.
//
//  Serial commands (type into the PlatformIO monitor, 115200 baud):
//    enroll start   — arm enrollment, then say "hey deskmon" once
//    enroll stop    — arm enrollment, then say "hey deskmon stop" once
//    save           — persist enrolled templates to NVS
//    clear start    — wipe start-command templates
//    clear stop     — wipe stop-command templates
//    status         — print template counts + current thresholds
// ─────────────────────────────────────────────────────────────────────────────
#include <Arduino.h>
#include <driver/i2s.h>
#include <cmath>
#include "Config.h"
#include "NavState.h"
#include "VoiceCommand.h"

static const i2s_port_t MIC_I2S_PORT = I2S_NUM_1;
static VoiceCommand s_voice;

static void on_voice_command(VoiceCommand::Command cmd) {
    if (cmd == VoiceCommand::CMD_START) {
        WITH_STATE([&]{
            g_state.session.active      = true;
            g_state.session.elapsed_sec = 0;
        });
        nav_log_event(EventType::SESSION_START);
        log_i("VoiceCommand: 'hey deskmon' -> session started");
    } else if (cmd == VoiceCommand::CMD_STOP) {
        WITH_STATE([&]{ g_state.session.active = false; });
        nav_log_event(EventType::SESSION_END);
        log_i("VoiceCommand: 'hey deskmon stop' -> session stopped");
    }
}

// Very small line-based command parser over Serial for enrollment.
static void poll_serial_commands() {
    static String line;
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            line.trim();
            if (line.length()) {
                if (line == "enroll start") {
                    s_voice.armEnroll(VoiceCommand::CMD_START);
                } else if (line == "enroll stop") {
                    s_voice.armEnroll(VoiceCommand::CMD_STOP);
                } else if (line == "save") {
                    s_voice.saveTemplates();
                } else if (line == "clear start") {
                    s_voice.clearTemplates(VoiceCommand::CMD_START);
                } else if (line == "clear stop") {
                    s_voice.clearTemplates(VoiceCommand::CMD_STOP);
                } else if (line == "status") {
                    Serial.printf("VoiceCommand: start templates=%d stop templates=%d noiseDb=%.1f\n",
                                  s_voice.templateCount(VoiceCommand::CMD_START),
                                  s_voice.templateCount(VoiceCommand::CMD_STOP),
                                  s_voice.getNoiseDb());
                }
            }
            line = "";
        } else {
            line += c;
        }
    }
}

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

    s_voice.begin();
    s_voice.onCommand(on_voice_command);

    log_i("MicTask: INMP441 OK on SCK=%d WS=%d SD=%d", PIN_MIC_SCK, PIN_MIC_WS, PIN_MIC_SD);
    return true;
}

void MicTask(void* pvParams) {
    // NOTE: this task now reads I2S back-to-back (no outer vTaskDelayUntil
    // gap). The original 200ms-period / 100ms-timeout loop was fine for a
    // coarse noise_db sample, but it left ~170ms silent gaps between reads —
    // more than enough to chop "hey deskmon" into unrecognizable pieces.
    // i2s_read() still blocks on the DMA queue, so this doesn't spin the CPU.
    static int32_t buf[MIC_BUF_LEN];
    uint32_t last_publish_ms = 0;

    while (true) {
        size_t bytes_read = 0;
        i2s_read(MIC_I2S_PORT, buf, sizeof(buf), &bytes_read, pdMS_TO_TICKS(200));

        int samples = (int)(bytes_read / sizeof(int32_t));
        if (samples > 0) {
            int64_t sum = 0;
            for (int i = 0; i < samples; i++) {
                int32_t s = buf[i] >> 14;
                sum += (int64_t)s * s;
            }
            float rms = sqrtf((float)sum / samples);
            float db = 20.0f * log10f(rms + 1.0f);

            // Feed the same shifted samples into the wake-word detector.
            s_voice.feed(buf, samples, 14);

            // Publish noise_db to shared state at ~5Hz instead of every
            // ~32ms read — DistractTask only samples it at 10Hz anyway.
            uint32_t now = millis();
            if (now - last_publish_ms >= 200) {
                last_publish_ms = now;
                WITH_STATE([&]{ g_state.motion.noise_db = db; });
                poll_serial_commands();
            }
        }
    }
}
