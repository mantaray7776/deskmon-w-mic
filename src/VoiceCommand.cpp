#include "VoiceCommand.h"
#include <Preferences.h>
#include <cmath>
#include <arduinoFFT.h>

// ─────────────────────────────────────────────────────────────────────────────
// arduinoFFT works on a fixed pair of buffers bound at construction. Kept
// static since there's only ever one VoiceCommand instance in this project.
// NOTE: API shown here is for kosme/arduinoFFT v2.x (ArduinoFFT<float>).
// If PlatformIO pulls v1.x instead, swap to the old free-function style:
//   FFT.Windowing(vReal, N, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
//   FFT.Compute(vReal, vImag, N, FFT_FORWARD);
//   FFT.ComplexToMagnitude(vReal, vImag, N);
// ─────────────────────────────────────────────────────────────────────────────
static float s_vReal[VoiceCommand::FRAME_SIZE];
static float s_vImag[VoiceCommand::FRAME_SIZE];
static ArduinoFFT<float> s_fft(s_vReal, s_vImag, VoiceCommand::FRAME_SIZE, (float)VoiceCommand::SAMPLE_RATE);

void VoiceCommand::begin() {
    loadTemplates();
    log_i("VoiceCommand: ready (start templates=%d, stop templates=%d)",
          templateCount(CMD_START), templateCount(CMD_STOP));
}

void VoiceCommand::feed(const int32_t* samples, size_t count, int shift_bits) {
    for (size_t i = 0; i < count; i++) {
        hist_[writeIdx_] = (float)(samples[i] >> shift_bits);
        writeIdx_ = (writeIdx_ + 1) % FRAME_SIZE;
        if (filled_ < FRAME_SIZE) filled_++;
        pending_++;

        if (pending_ >= HOP_SIZE && filled_ >= FRAME_SIZE) {
            // Unroll the ring buffer into chronological order for this frame.
            static float frame[FRAME_SIZE];
            int idx = writeIdx_; // oldest sample position
            for (int k = 0; k < FRAME_SIZE; k++) {
                frame[k] = hist_[idx];
                idx = (idx + 1) % FRAME_SIZE;
            }
            processFrame(frame);
            pending_ -= HOP_SIZE;
        }
    }
}

void VoiceCommand::processFrame(const float* frame) {
    for (int i = 0; i < FRAME_SIZE; i++) {
        s_vReal[i] = frame[i];
        s_vImag[i] = 0.0f;
    }

    // Raw RMS/dB before windowing — same formula as MicTask's noise_db calc,
    // used here purely to drive VAD.
    double sumSq = 0;
    for (int i = 0; i < FRAME_SIZE; i++) sumSq += (double)s_vReal[i] * s_vReal[i];
    float rms = sqrtf((float)(sumSq / FRAME_SIZE));
    float frameDb = 20.0f * log10f(rms + 1.0f);
    noiseDb_ = noiseDb_ * 0.8f + frameDb * 0.2f;

    // Enrollment timeout — if armed too long with no speech, disarm quietly.
    if (enrollArmed_ && millis() - enrollArmedAtMs_ > ENROLL_TIMEOUT_MS && !inSpeech_) {
        enrollArmed_ = false;
        log_w("VoiceCommand: enroll timed out, no speech detected");
    }

    Feature feat = extractFeatures();

    if (!inSpeech_) {
        if (frameDb > vadOnDb_) {
            inSpeech_ = true;
            silenceRun_ = 0;
            currentUtterance_.clear();
            currentUtterance_.push_back(feat);
        }
        return;
    }

    // In speech: accumulate, watch for end-of-utterance.
    if ((int)currentUtterance_.size() < MAX_FRAMES) {
        currentUtterance_.push_back(feat);
    }
    if (frameDb < vadOffDb_) {
        silenceRun_++;
    } else {
        silenceRun_ = 0;
    }
    if (silenceRun_ >= SILENCE_FRAMES_TO_END || (int)currentUtterance_.size() >= MAX_FRAMES) {
        finishUtterance();
    }
}

VoiceCommand::Feature VoiceCommand::extractFeatures() {
    s_fft.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    s_fft.compute(FFTDirection::Forward);
    s_fft.complexToMagnitude();

    Feature feat{};
    const int usableBins = FRAME_SIZE / 2;
    for (int b = 0; b < NUM_BANDS; b++) {
        int lo = 1 + (int)powf((float)usableBins, (float)b / NUM_BANDS);
        int hi = 1 + (int)powf((float)usableBins, (float)(b + 1) / NUM_BANDS);
        if (hi <= lo) hi = lo + 1;
        if (hi > usableBins) hi = usableBins;
        float e = 0.0f;
        for (int k = lo; k < hi; k++) e += s_vReal[k];
        feat[b] = log10f(e + 1.0f);
    }
    return feat;
}

void VoiceCommand::finishUtterance() {
    inSpeech_ = false;
    silenceRun_ = 0;
    int frames = (int)currentUtterance_.size();

    if (frames < MIN_FRAMES) {
        currentUtterance_.clear();
        return; // too short — noise blip / tap, not a phrase
    }

    if (enrollArmed_) {
        auto& slots = templates_[enrollCmd_];
        if ((int)slots.size() >= MAX_TEMPLATES) slots.erase(slots.begin()); // drop oldest
        slots.push_back(currentUtterance_);
        enrollArmed_ = false;
        log_i("VoiceCommand: enrolled template for cmd=%d (now %d stored, %d frames)",
              (int)enrollCmd_, (int)slots.size(), frames);
        currentUtterance_.clear();
        return;
    }

    float bestDist = 1e9f;
    Command bestCmd = CMD_NONE;
    for (int c = 0; c < NUM_COMMANDS; c++) {
        for (auto& tmpl : templates_[c]) {
            float d = dtw(currentUtterance_, tmpl);
            if (d < bestDist) { bestDist = d; bestCmd = (Command)c; }
        }
    }

    log_i("VoiceCommand: utterance (%d frames) best=%d dist=%.2f (threshold %.2f)",
          frames, (int)bestCmd, bestDist, matchThreshold_);

    if (bestCmd != CMD_NONE && bestDist < matchThreshold_ && callback_) {
        callback_(bestCmd);
    }
    currentUtterance_.clear();
}

// DTW with rolling two-row DP — O(n*m) time, O(m) memory regardless of
// MAX_FRAMES, so this is cheap even though it only runs once per utterance.
float VoiceCommand::dtw(const std::vector<Feature>& a, const std::vector<Feature>& b) const {
    int n = (int)a.size();
    int m = (int)b.size();
    if (n == 0 || m == 0) return 1e9f;

    std::vector<float> prev(m + 1, 1e9f), curr(m + 1, 1e9f);
    prev[0] = 0.0f;

    for (int i = 1; i <= n; i++) {
        curr[0] = 1e9f;
        for (int j = 1; j <= m; j++) {
            float dEuclid = 0.0f;
            for (int k = 0; k < NUM_BANDS; k++) {
                float d = a[i - 1][k] - b[j - 1][k];
                dEuclid += d * d;
            }
            dEuclid = sqrtf(dEuclid);
            float best = fminf(prev[j], fminf(curr[j - 1], prev[j - 1]));
            curr[j] = dEuclid + best;
        }
        std::swap(prev, curr);
    }
    // Normalize by path length so longer/shorter utterances are comparable.
    return prev[m] / (float)(n + m);
}

void VoiceCommand::armEnroll(Command cmd) {
    enrollArmed_ = true;
    enrollCmd_ = cmd;
    enrollArmedAtMs_ = millis();
    log_i("VoiceCommand: enrollment armed for cmd=%d — speak the phrase now", (int)cmd);
}

void VoiceCommand::clearTemplates(Command cmd) {
    templates_[cmd].clear();
    log_i("VoiceCommand: cleared templates for cmd=%d", (int)cmd);
}

// ── NVS persistence ─────────────────────────────────────────────────────────
// Each template is stored as: "c{cmd}n{idx}f" (frame count, u16) and
// "c{cmd}n{idx}d" (raw float blob, frames*NUM_BANDS floats).
// Counts per command stored under "c{cmd}cnt".
void VoiceCommand::saveTemplates() {
    Preferences prefs;
    prefs.begin("voicecmd", false);

    for (int c = 0; c < NUM_COMMANDS; c++) {
        char cntKey[16];
        snprintf(cntKey, sizeof(cntKey), "c%dcnt", c);
        prefs.putUChar(cntKey, (uint8_t)templates_[c].size());

        for (size_t t = 0; t < templates_[c].size(); t++) {
            const auto& tmpl = templates_[c][t];
            char fKey[16], dKey[16];
            snprintf(fKey, sizeof(fKey), "c%dn%df", c, (int)t);
            snprintf(dKey, sizeof(dKey), "c%dn%dd", c, (int)t);
            prefs.putUShort(fKey, (uint16_t)tmpl.size());
            prefs.putBytes(dKey, tmpl.data(), tmpl.size() * sizeof(Feature));
        }
    }
    prefs.end();
    log_i("VoiceCommand: templates saved to NVS");
}

void VoiceCommand::loadTemplates() {
    Preferences prefs;
    prefs.begin("voicecmd", true);

    for (int c = 0; c < NUM_COMMANDS; c++) {
        templates_[c].clear();
        char cntKey[16];
        snprintf(cntKey, sizeof(cntKey), "c%dcnt", c);
        uint8_t n = prefs.getUChar(cntKey, 0);

        for (int t = 0; t < n; t++) {
            char fKey[16], dKey[16];
            snprintf(fKey, sizeof(fKey), "c%dn%df", c, t);
            snprintf(dKey, sizeof(dKey), "c%dn%dd", c, t);
            uint16_t frames = prefs.getUShort(fKey, 0);
            if (frames == 0) continue;

            std::vector<Feature> tmpl(frames);
            size_t got = prefs.getBytes(dKey, tmpl.data(), frames * sizeof(Feature));
            if (got == frames * sizeof(Feature)) {
                templates_[c].push_back(std::move(tmpl));
            }
        }
    }
    prefs.end();
}
