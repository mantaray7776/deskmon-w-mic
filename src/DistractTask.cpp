// ─────────────────────────────────────────────────────────────────────────────
//  DistractTask.cpp  — computes distraction score from motion + EM + noise
// ─────────────────────────────────────────────────────────────────────────────
#include <cmath>
#include <algorithm>
#include "Config.h"
#include "NavState.h"
#include <Arduino.h>

void DistractTask(void* pvParams) {
    const float DECAY   = 0.7f;
    const float VIB_W   = 40.0f;
    const float EM_W    = 30.0f;
    const float NOISE_W = 30.0f;
    float score = 0;
    
   vTaskDelay(pdMS_TO_TICKS(3000));

    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        NavState st = state_snapshot();

        float vib_norm   = fminf(st.motion.vibration_rms / IMPACT_THRESHOLD, 1.0f);
        float em_norm    = fminf(st.motion.em_variance   / EM_SPIKE_THRESHOLD, 1.0f);
        float noise_norm = fminf(fmaxf((st.motion.noise_db - NOISE_QUIET_DB) /
                                       (NOISE_LOUD_DB - NOISE_QUIET_DB), 0.0f), 1.0f);

        float raw = vib_norm * VIB_W + em_norm * EM_W + noise_norm * NOISE_W;
        // raw الان بین 0 تا 100 هست
        score = score * DECAY + raw * (1.0f - DECAY);
        score = fminf(fmaxf(score, 0.0f), 100.0f);
        score = fminf(fmaxf(score, 0), 100);
        Serial.printf("noise=%.1f dB  distract=%d\n", st.motion.noise_db, (int)score);
        Serial.printf("vib=%.4f  em=%.4f  noise=%.1f\n", st.motion.vibration_rms, st.motion.em_variance, st.motion.noise_db);
        WITH_STATE([&]{
            g_state.session.distraction_score = (uint8_t)score;
        });

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100));
    }
}