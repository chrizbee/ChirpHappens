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

void ToneTracker::Track::begin(uint32_t frame, const Frame &frame_data)
{
    clear();
    start = frame;
    add(frame, frame_data);
}

void ToneTracker::Track::add(uint32_t frame, const Frame &frame_data)
{
    last = frame;
    frequencies.push_back(frame_data.frequency);
    amplitude += frame_data.amplitude;
    purity += frame_data.purity;
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

bool ToneTracker::process_frame(const float *magnitudes)
{
    Frame frame;
    const bool tonal = analyze(magnitudes, &frame);
    track(tonal, frame);

    // Check the pattern as soon as the current tone is long enough
    bool detected = false;
    if (track_.active() && static_cast<int>(track_.frequencies.size()) == c_.min_frames)
        detected = match_pattern(make_tone(track_));
    ++frame_;
    return detected;
}

bool ToneTracker::analyze(const float *m, Frame *frame) const
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

    frame->frequency = (peak + delta) * c_.sample_rate / c_.fft_size;
    frame->amplitude = amplitude;
    frame->purity = peak_energy / total_energy;
    return true;
}

void ToneTracker::track(bool tonal, const Frame &frame)
{
    const float frequency = frame.frequency;
    if (tonal) {
        if (!track_.active()) {
            track_.begin(frame_, frame);

        // Same pitch: Continue the tone
        } else if (std::fabs(frequency - median(track_.frequencies)) <= SAME_TONE * frequency) {
            track_.add(frame_, frame);
            candidate_.clear();

        // Different pitch: Could be an outlier or already the next tone
        } else if (candidate_.active() &&
                   std::fabs(frequency - median(candidate_.frequencies)) <= SAME_TONE * frequency) {
            candidate_.add(frame_, frame);
        } else {
            candidate_.begin(frame_, frame);
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

Tone ToneTracker::make_tone(Track &track) const
{
    std::vector<float> frequencies(track.frequencies);
    const size_t count = frequencies.size();
    Tone tone;
    tone.start = track.start;
    tone.last = track.last;
    tone.frequency = median(frequencies);

    // Wobble without the outermost frames (start and end of a tone)
    std::sort(frequencies.begin(), frequencies.end());
    const size_t cut = count / 10;
    tone.spread = (frequencies[count - 1 - cut] - frequencies[cut]) / tone.frequency;

    // Level relative to a full scale sine (Hann window: amplitude * N / 4)
    const float full_scale = 2147483648.0f * c_.fft_size / 4.0f;
    tone.level = 20.0f * std::log10(track.amplitude / count / full_scale);
    tone.purity = track.purity / count;

    const float ms = frames_to_ms(tone.last - tone.start + 1);
    tone.valid = ms <= c_.max_tone_ms && tone.spread <= c_.max_spread;
    return tone;
}

void ToneTracker::close_track()
{
    if (static_cast<int>(track_.frequencies.size()) >= c_.min_frames) {
        Tone tone = make_tone(track_);
        if (on_tone)
            on_tone(tone, frames_to_ms(tone.last - tone.start + 1));

        // After a pattern start from scratch: The tone that completed it and
        // tones right after it must not be part of the next one. They are kept
        // as invalid, so that a following pattern is not isolated.
        const int32_t since_pattern = static_cast<int32_t>(tone.start - last_fired_);
        if (track_consumed_ || (has_fired_ && frames_to_ms(since_pattern) < c_.cooldown_ms)) {
            reject(track_consumed_ ? "tone already used" : "tone during cooldown");
            tone.valid = false;
        }
        history_.push_back(tone);
        while (history_.size() > targets_.size())
            history_.pop_front();
    }
    track_consumed_ = false;
    track_.clear();
}

bool ToneTracker::match_pattern(const Tone &last)
{
    // The tones before the current one plus the current one
    const size_t count = targets_.size();
    if (count == 0 || history_.size() < count - 1)
        return false;
    std::vector<const Tone *> tones;
    for (size_t i = history_.size() - (count - 1); i < history_.size(); ++i)
        tones.push_back(&history_[i]);
    tones.push_back(&last);
    const Tone &base = *tones[0];

    // Pitch: Whistling higher or lower keeps the interval, not the distance
    // in Hz, so all tones are scaled to the pitch of the sequence.
    if (std::fabs(base.frequency - targets_[0]) > c_.max_delta_base)
        return false;
    const float scale = targets_[0] / base.frequency;
    for (size_t i = 0; i < count; ++i) {
        if (!tones[i]->valid)
            return false;
        if (i > 0 && std::fabs(tones[i]->frequency * scale - targets_[i]) > c_.max_delta_freq)
            return false;
    }

    // From here on it sounds right, so the reason for a rejection is of interest
    // Rhythm: The time between the tones has to be plausible and steady
    float shortest = 0.0f, longest = 0.0f;
    for (size_t i = 1; i < count; ++i) {
        const float pause = frames_to_ms(tones[i]->start - tones[i - 1]->start);
        if (std::fabs(pause - c_.pause_ms) > c_.max_delta_time) {
            reject("pause out of range");
            return false;
        }
        shortest = i == 1 ? pause : std::min(shortest, pause);
        longest = i == 1 ? pause : std::max(longest, pause);
    }
    if (longest > c_.max_rhythm * shortest) {
        reject("uneven rhythm");
        return false;
    }

    // Part of a melody?
    if (history_.size() >= count && frames_to_ms(base.start - history_[history_.size() - count].last) < c_.isolation_ms) {
        reject("preceded by another tone");
        return false;
    }

    has_fired_ = true;
    track_consumed_ = true;
    last_fired_ = frame_;
    history_.clear();
    return true;
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
