// Tests the detection with synthetic signals, see README for how to run it.

#include "../components/chirphappens/tone_tracker.h"
#include "spectrum.h"
#include <cstdio>
#include <initializer_list>
#include <vector>

using namespace esphome::chirphappens;

static const float RATE = 8000.0f;
static const float LEVEL = 0.01f;   // -40 dBFS

// Sum of the given harmonics, relative to the fundamental
static void add_tone(std::vector<float> &signal, float at, float length, float frequency,
                     std::initializer_list<float> harmonics = { 1.0f })
{
    const size_t start = static_cast<size_t>(at * RATE);
    const size_t count = static_cast<size_t>(length * RATE);
    if (signal.size() < start + count)
        signal.resize(start + count);
    for (size_t i = 0; i < count; ++i) {
        const float t = i / RATE;
        const float fade = std::min(1.0f, std::min(t, length - t) / 0.02f);
        int harmonic = 1;
        for (float amplitude : harmonics) {
            signal[start + i] += fade * LEVEL * amplitude * std::sin(2.0f * M_PI * frequency * harmonic * t);
            ++harmonic;
        }
    }
}

static std::vector<float> sequence(std::initializer_list<float> frequencies, float pause = 0.3f, float length = 0.2f)
{
    std::vector<float> signal;
    float at = 0.5f;
    for (float frequency : frequencies) {
        add_tone(signal, at, length, frequency);
        at += pause;
    }
    return signal;
}

static int patterns(std::vector<float> signal)
{
    signal.resize(signal.size() + static_cast<size_t>(2 * RATE));
    TrackerConfig config;
    ToneTracker tracker;
    tracker.configure(config);
    Spectrum spectrum(config.fft_size);
    int count = 0;
    for (size_t i = 0; i + config.fft_size <= signal.size(); i += config.fft_size)
        count += tracker.process_frame(spectrum.compute(&signal[i]));
    return count;
}

static int failures = 0;

static void expect(const char *name, int actual, int expected)
{
    const bool ok = actual == expected;
    failures += !ok;
    std::printf("%s %s (%d patterns)\n", ok ? "ok  " : "FAIL", name, actual);
}

int main()
{
    // Has to be detected
    expect("exact sequence", patterns(sequence({ 1750, 1350, 1750 })), 1);
    expect("whistled lower", patterns(sequence({ 1500, 1157, 1500 })), 1);
    expect("whistled higher", patterns(sequence({ 2100, 1620, 2100 })), 1);
    expect("second tone a bit off", patterns(sequence({ 1750, 1480, 1750 })), 1);
    expect("fast", patterns(sequence({ 1750, 1350, 1750 }, 0.2f, 0.15f)), 1);
    expect("slow", patterns(sequence({ 1750, 1350, 1750 }, 0.7f, 0.5f)), 1);
    expect("no gap between tones", patterns(sequence({ 1750, 1350, 1750 }, 0.3f, 0.3f)), 1);

    // Repeating after the cooldown triggers again
    std::vector<float> twice = sequence({ 1750, 1350, 1750 });
    add_tone(twice, 3.0f, 0.2f, 1750);
    add_tone(twice, 3.3f, 0.2f, 1350);
    add_tone(twice, 3.6f, 0.2f, 1750);
    expect("repeated", patterns(twice), 2);

    // Must not be detected
    expect("silence", patterns(std::vector<float>(static_cast<size_t>(RATE))), 0);
    expect("reversed", patterns(sequence({ 1350, 1750, 1350 })), 0);
    expect("rising", patterns(sequence({ 1350, 1750, 2150 })), 0);
    expect("same tone", patterns(sequence({ 1750, 1750, 1750 })), 0);
    expect("too few tones", patterns(sequence({ 1750, 1350 })), 0);
    expect("too short tones", patterns(sequence({ 1750, 1350, 1750 }, 0.3f, 0.05f)), 0);
    expect("too slow", patterns(sequence({ 1750, 1350, 1750 }, 0.9f, 0.5f)), 0);
    expect("followed by a tone", patterns(sequence({ 1750, 1350, 1750, 1350 })), 1);
    expect("preceded by a tone", patterns(sequence({ 1500, 1750, 1350, 1750 })), 0);

    std::vector<float> uneven;
    add_tone(uneven, 0.5f, 0.15f, 1750);
    add_tone(uneven, 0.7f, 0.15f, 1350);
    add_tone(uneven, 1.3f, 0.15f, 1750);
    expect("uneven rhythm", patterns(uneven), 0);

    // Harmonics of a voice or an instrument hitting the sequence's pitches
    std::vector<float> rich;
    for (int i = 0; i < 3; ++i)
        add_tone(rich, 0.5f + 0.3f * i, 0.2f, (i == 1 ? 1350 : 1750) / 4.0f, { 0.8f, 0.8f, 0.8f, 1.0f, 0.8f, 0.8f, 0.8f });
    expect("not a pure sine", patterns(rich), 0);

    std::printf("%s\n", failures ? "FAILED" : "All tests passed");
    return failures != 0;
}
