#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <vector>

namespace esphome {
namespace chirphappens {

// Detection core without any hardware dependency, so that the code running on
// the ESP can also be tested on a PC (see tests/).
//
// Works in three steps:
// 1. Frame:   Is this spectrum a clean sine in the band? (level, purity)
// 2. Tone:    Tonal frames with a stable pitch are merged into one tone
// 3. Pattern: The last tones are compared with the sequence as a whole.
//             This happens as soon as the last tone is long enough, so there
//             is no delay after the last tone.
//
// Time is counted in FFT frames, so it does not depend on loop() jitter.

struct TrackerConfig {
    // Audio
    float sample_rate = 8000.0f;
    int fft_size = 256;

    // Frame
    float start_freq = 1000.0f;     // Band of interest in Hz
    float end_freq = 2800.0f;
    float peak_threshold = 3e6f;    // Minimum peak amplitude after FFT
    float peak_to_mean = 4.0f;      // Minimum peak amplitude / mean amplitude in band
    float min_purity = 0.35f;       // Minimum share of total energy in the peak (0..1)

    // Tone
    int min_frames = 3;             // Minimum number of frames of one tone
    int max_tone_ms = 1200;         // Longer tones are not part of a pattern
    float max_spread = 0.06f;       // Maximum pitch wobble relative to the tone's pitch

    // Pattern
    std::vector<float> sequence = { 1750.0f, -400.0f, 400.0f };
    float max_delta_base = 500.0f;  // Tolerance of the first tone in Hz
    float max_delta_freq = 180.0f;  // Tolerance of the following tones in Hz
    int pause_ms = 420;             // Time from the start of a tone to the start of the next
    int max_delta_time = 300;       // Tolerance of pause_ms
    float max_rhythm = 2.0f;        // Longest / shortest time between tones
    int isolation_ms = 400;         // No other tone this close before the pattern
    int cooldown_ms = 1000;         // Minimum time between two patterns
};

struct Frame {
    float frequency = 0.0f;         // Interpolated peak frequency in Hz
    float amplitude = 0.0f;         // Interpolated peak amplitude after FFT
    float purity = 0.0f;            // Share of the energy in the peak
};

struct Tone {
    uint32_t start = 0;             // First frame
    uint32_t last = 0;              // Last frame
    float frequency = 0.0f;         // Median frequency in Hz
    float spread = 0.0f;            // Pitch wobble relative to frequency
    float level = 0.0f;             // Mean peak level in dBFS
    float purity = 0.0f;            // Mean share of the energy in the peak
    bool valid = false;             // Usable as part of a pattern
};

class ToneTracker {
public:
    void configure(const TrackerConfig &config);

    // Feed the magnitudes of one FFT (fft_size / 2 values are used)
    // Returns true when a pattern was completed
    bool process_frame(const float *magnitudes);

    // Optional diagnostics
    std::function<void(const Tone &tone, float ms)> on_tone;
    std::function<void(const char *reason)> on_reject;

protected:
    struct Track {
        uint32_t start = 0;
        uint32_t last = 0;
        std::vector<float> frequencies;
        float amplitude = 0.0f;     // Sum over all frames
        float purity = 0.0f;        // Sum over all frames
        bool active() const { return !frequencies.empty(); }
        void begin(uint32_t frame, const Frame &frame_data);
        void add(uint32_t frame, const Frame &frame_data);
        void clear() { frequencies.clear(); amplitude = purity = 0.0f; }
    };

    bool analyze(const float *magnitudes, Frame *frame) const;
    void track(bool tonal, const Frame &frame);
    Tone make_tone(Track &track) const;
    void close_track();
    bool match_pattern(const Tone &last);
    void reject(const char *reason);
    float median(const std::vector<float> &values) const;
    float frames_to_ms(uint32_t frames) const { return frames * frame_ms_; }

    TrackerConfig c_;
    std::vector<float> targets_;    // Absolute frequencies of the sequence
    float frame_ms_ = 32.0f;
    int start_index_ = 0;
    int end_index_ = 0;
    int broad_start_index_ = 0;
    int broad_end_index_ = 0;

    uint32_t frame_ = 0;
    Track track_;
    Track candidate_;
    std::deque<Tone> history_;
    bool has_fired_ = false;
    uint32_t last_fired_ = 0;
};

}  // namespace chirphappens
}  // namespace esphome
