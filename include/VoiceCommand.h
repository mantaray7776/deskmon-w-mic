#pragma once
// ─────────────────────────────────────────────────────────────────────────────
//  VoiceCommand — on-device keyword spotting for "hey deskmon" / "hey deskmon
//  stop", with NO Edge Impulse / TFLite / cloud dependency.
//
//  How it works:
//    1. VAD (voice activity detection): each 16ms frame's RMS is compared to
//       an on/off dB threshold (hysteresis) to find where an utterance starts
//       and ends in the audio stream.
//    2. Each frame inside the utterance becomes a small feature vector
//       (log-energy in NUM_BANDS log-spaced FFT bins) — cheap to compute, and
//       stable enough for template matching.
//    3. The finished utterance (a short sequence of feature vectors) is
//       compared against templates *you* record yourself with armEnroll(),
//       using Dynamic Time Warping (DTW) — this tolerates you saying the
//       phrase a bit faster/slower each time.
//    4. Best match under setMatchThreshold() fires onCommand().
//
//  This is NOT as robust as a trained neural wake-word model. It works best:
//    - in a mostly-quiet room,
//    - with 3+ enrolled templates per command (say it a few different ways),
//    - after tuning setVadThresholdDb() / setMatchThreshold() against real
//      Serial output (see README).
//  Treat false accepts/rejects as a tuning problem, not a bug.
// ─────────────────────────────────────────────────────────────────────────────
#include <Arduino.h>
#include <array>
#include <vector>
#include <functional>

class VoiceCommand {
public:
    enum Command : int8_t { CMD_NONE = -1, CMD_START = 0, CMD_STOP = 1 };
    static constexpr int NUM_COMMANDS = 2;

    static constexpr int   SAMPLE_RATE  = 16000; // must match Config.h MIC_SAMPLE_RATE
    static constexpr int   FRAME_SIZE   = 256;   // ~16ms/frame @16kHz, power of 2 for FFT
    static constexpr int   HOP_SIZE     = 128;   // 50% overlap
    static constexpr int   NUM_BANDS    = 10;    // feature dims per frame
    static constexpr int   MAX_FRAMES   = 150;   // ~1.2s max utterance length
    static constexpr int   MIN_FRAMES   = 15;    // ~120ms min utterance (rejects blips/taps)
    static constexpr int   MAX_TEMPLATES = 3;    // enrolled recordings kept per command

    using Feature = std::array<float, NUM_BANDS>;
    using CommandCallback = std::function<void(Command)>;

    void begin();  // call once from mic_init(); loads saved templates from NVS

    // Feed raw I2S samples straight from your existing i2s_read() buffer.
    // shift_bits should match whatever MicTask already uses to bring the
    // 32-bit I2S word into a sane range (MicTask.cpp currently uses 14).
    void feed(const int32_t* samples, size_t count, int shift_bits = 14);

    // Smoothed frame dB, same formula MicTask uses for noise_db. Exposed in
    // case you want it, but MicTask's own noise_db calc already feeds the
    // distraction score — no need to duplicate that wiring.
    float getNoiseDb() const { return noiseDb_; }

    void onCommand(CommandCallback cb) { callback_ = cb; }

    // ── Enrollment ──────────────────────────────────────────────────────────
    // Arm enrollment, then speak the phrase once within ~2.5s. The next
    // completed utterance is stored as a new template for `cmd` (oldest
    // dropped once MAX_TEMPLATES is reached). Call saveTemplates() after
    // you're happy with what's enrolled so it survives a reboot.
    void armEnroll(Command cmd);
    bool isEnrollArmed() const { return enrollArmed_; }
    void clearTemplates(Command cmd);
    int  templateCount(Command cmd) const { return (int)templates_[cmd].size(); }

    void saveTemplates();  // persist all templates to NVS (namespace "voicecmd")
    void loadTemplates();  // load from NVS — called by begin()

    void setMatchThreshold(float t) { matchThreshold_ = t; }
    void setVadThresholdDb(float onDb, float offDb) { vadOnDb_ = onDb; vadOffDb_ = offDb; }

private:
    void      processFrame(const float* frame); // frame already shift-adjusted
    Feature   extractFeatures();
    void      finishUtterance();
    float     dtw(const std::vector<Feature>& a, const std::vector<Feature>& b) const;

    CommandCallback callback_;

    bool     enrollArmed_ = false;
    Command  enrollCmd_   = CMD_NONE;
    uint32_t enrollArmedAtMs_ = 0;
    static constexpr uint32_t ENROLL_TIMEOUT_MS = 2500;

    bool  inSpeech_  = false;
    int   silenceRun_ = 0;
    static constexpr int SILENCE_FRAMES_TO_END = 15; // ~120ms of quiet ends an utterance

    float noiseDb_        = 0.0f;
    float matchThreshold_ = 4.5f;   // avg per-frame DTW cost; lower = stricter. Tune on hw.
    float vadOnDb_         = 78.0f; // start-of-speech trigger; tune above your room's noise floor
    float vadOffDb_        = 72.0f; // end-of-speech trigger (hysteresis, must be < vadOnDb_)

    std::vector<Feature> currentUtterance_;
    std::vector<std::vector<Feature>> templates_[NUM_COMMANDS];

    // sliding sample history for hop-based framing
    float hist_[FRAME_SIZE] = {0};
    int   writeIdx_ = 0;
    int   filled_   = 0;
    int   pending_  = 0; // new samples since last frame was processed
};
