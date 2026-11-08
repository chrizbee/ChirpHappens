#include "tone_tracker.h"
#include <algorithm>
#include <cmath>

namespace esphome {
namespace chirphappens {

// Frames a tone may drop out without being split
static const uint32_t MAX_GAP_FRAMES = 1;
// Frames a different pitch needs to take over from the current tone
static const size_t TAKEOVER_FRAMES = 2;
// Frames with a pitch this close (relative) belong to the same tone
static const float SAME_TONE = 0.06f;
// A Hann window spreads a sine over the peak bin +- 2
static const int PEAK_WIDTH = 2;
// Lower limit of the energy reference, below is mostly hum and rumble
static const float BROAD_START_FREQ = 300.0f;

void ToneTracker::Track::begin(uint32_t frame, float frequency)
{
    start = last = frame;
    frequencies.assign(1, frequency);
}

void ToneTracker::configure(const TrackerConfig &config)
{
    c_ = config;
    const float resolution = c_.sample_rate / c_.fft_size;
    const int bins = c_.fft_size / 2;
    frame_ms_ = 1000.0f * c_.fft_size / c_.sample_rate;
    start_index_ = std::max(1, static_cast<int>(std::ceil(c_.start_freq / resolution)));
    end_index_ = std::min(bins - 1, static_cast<int>(std::floor(c_.end_freq / resolution)));
    broad_start_index_ = static_cast<int>(std::ceil(BROAD_START_FREQ / resolution));
    broad_end_index_ = bins;

    // First value is absolute, all following are relative to their previous
    targets_.clear();
    float frequency = 0.0f;
    for (float step : c_.sequence) {
        frequency += step;
        targets_.push_back(frequency);
    }
}

uint32_t ToneTracker::ms_to_frames(float ms) const
{
    return static_cast<uint32_t>(std::ceil(ms / frame_ms_));
}

bool ToneTracker::process_frame(const float *magnitudes)
{
    float frequency = 0.0f;
    const bool tonal = analyze(magnitudes, &frequency);
    track(tonal, frequency);
    ++frame_;

    // A pattern only counts after the required silence following it
    if (fire_pending_) {
        // Any tonal frame counts, as there is not enough time to wait for a whole tone
        if (track_.active()) {
            fire_pending_ = false;
            reject("followed by another tone");
        } else if (static_cast<int32_t>(frame_ - fire_frame_) >= 0) {
            fire_pending_ = false;
            has_fired_ = true;
            last_fired_ = frame_;
            history_.clear();
            return true;
        }
    }
    return false;
}

bool ToneTracker::analyze(const float *m, float *frequency) const
{
    if (start_index_ >= end_index_)
        return false;

    // Find the peak and the mean within the band
    int peak = start_index_;
    float sum = 0.0f;
    for (int i = start_index_; i < end_index_; ++i) {
        sum += m[i];
        if (m[i] > m[peak])
            peak = i;
    }

    // A maximum at the edge of the band is the flank of something outside
    if (peak == start_index_ || peak == end_index_ - 1)
        return false;

    // Parabolic interpolation for better accuracy
    const float y0 = m[peak - 1], y1 = m[peak], y2 = m[peak + 1];
    const float denominator = y0 - 2.0f * y1 + y2;
    if (denominator >= 0.0f)
        return false;
    const float delta = 0.5f * (y0 - y2) / denominator;
    const float amplitude = y1 - 0.25f * (y0 - y2) * delta;

    // Loud enough and sticking out of the band?
    const float mean = sum / (end_index_ - start_index_);
    if (amplitude < c_.peak_threshold || amplitude < c_.peak_to_mean * mean)
        return false;

    // A whistle is almost a pure sine: nearly all of the energy is in the peak.
    // This is what tells it apart from speech and music, which spread their
    // energy over many harmonics, also outside of the band.
    float peak_energy = 0.0f, total_energy = 0.0f;
    for (int i = broad_start_index_; i < broad_end_index_; ++i) {
        const float energy = m[i] * m[i];
        total_energy += energy;
        if (i >= peak - PEAK_WIDTH && i <= peak + PEAK_WIDTH)
            peak_energy += energy;
    }
    if (peak_energy < c_.min_purity * total_energy)
        return false;

    *frequency = (peak + delta) * c_.sample_rate / c_.fft_size;
    return true;
}

void ToneTracker::track(bool tonal, float frequency)
{
    if (tonal) {
        if (!track_.active()) {
            track_.begin(frame_, frequency);

        // Same pitch: Continue the tone
        } else if (std::fabs(frequency - median(track_.frequencies)) <= SAME_TONE * frequency) {
            track_.frequencies.push_back(frequency);
            track_.last = frame_;
            candidate_.clear();

        // Different pitch: Could be an outlier or already the next tone
        } else if (candidate_.active() &&
                   std::fabs(frequency - median(candidate_.frequencies)) <= SAME_TONE * frequency) {
            candidate_.frequencies.push_back(frequency);
            candidate_.last = frame_;
        } else {
            candidate_.begin(frame_, frequency);
        }
    } else {
        candidate_.clear();
    }

    // The next tone has taken over or the current tone has ended
    const bool takeover = candidate_.frequencies.size() >= TAKEOVER_FRAMES;
    if (track_.active() && (takeover || frame_ - track_.last > MAX_GAP_FRAMES)) {
        close_track();
        track_ = candidate_;
        candidate_.clear();
    }
}

void ToneTracker::close_track()
{
    std::vector<float> &frequencies = track_.frequencies;
    const bool long_enough = static_cast<int>(frequencies.size()) >= c_.min_frames;
    if (long_enough) {
        Tone tone;
        tone.start = track_.start;
        tone.last = track_.last;
        tone.frequency = median(frequencies);

        // Wobble without the outermost frames (start and end of a tone)
        std::sort(frequencies.begin(), frequencies.end());
        const size_t cut = frequencies.size() / 10;
        const float range = frequencies[frequencies.size() - 1 - cut] - frequencies[cut];
        tone.spread = range / tone.frequency;

        const float ms = frames_to_ms(tone.last - tone.start + 1);
        tone.valid = ms <= c_.max_tone_ms && tone.spread <= c_.max_spread;
        if (on_tone)
            on_tone(tone, ms);

        history_.push_back(tone);
        while (history_.size() > targets_.size() + 1)
            history_.pop_front();
        match_pattern();
    }
    track_.clear();
}

void ToneTracker::match_pattern()
{
    // Is the last tone even the end of a sequence?
    const size_t count = targets_.size();
    if (count == 0 || history_.size() < count)
        return;
    const size_t first = history_.size() - count;
    const Tone &base = history_[first];

    // Pitch: Whistling higher or lower keeps the interval, not the distance
    // in Hz, so all tones are scaled to the pitch of the sequence.
    if (std::fabs(base.frequency - targets_[0]) > c_.max_delta_base)
        return;
    const float scale = targets_[0] / base.frequency;
    for (size_t i = 0; i < count; ++i) {
        if (!history_[first + i].valid)
            return;
        if (i > 0 && std::fabs(history_[first + i].frequency * scale - targets_[i]) > c_.max_delta_freq)
            return;
    }

    // From here on it sounds right, so the reason for a rejection is of interest
    // Rhythm: The time between the tones has to be plausible and steady
    float shortest = 0.0f, longest = 0.0f;
    for (size_t i = 1; i < count; ++i) {
        const float pause = frames_to_ms(history_[first + i].start - history_[first + i - 1].start);
        if (std::fabs(pause - c_.pause_ms) > c_.max_delta_time)
            return reject("pause out of range");
        shortest = i == 1 ? pause : std::min(shortest, pause);
        longest = i == 1 ? pause : std::max(longest, pause);
    }
    if (longest > c_.max_rhythm * shortest)
        return reject("uneven rhythm");

    // Part of a melody?
    if (first > 0 && frames_to_ms(base.start - history_[first - 1].last) < c_.isolation_ms)
        return reject("preceded by another tone");

    if (has_fired_ && frames_to_ms(base.start - last_fired_) < c_.cooldown_ms)
        return reject("cooldown");

    // Wait for the silence after the pattern
    fire_pending_ = true;
    fire_frame_ = history_.back().last + ms_to_frames(c_.trailing_ms);
}

void ToneTracker::reject(const char *reason)
{
    if (on_reject)
        on_reject(reason);
}

float ToneTracker::median(const std::vector<float> &values) const
{
    std::vector<float> copy(values);
    std::nth_element(copy.begin(), copy.begin() + copy.size() / 2, copy.end());
    return copy[copy.size() / 2];
}

}  // namespace chirphappens
}  // namespace esphome
