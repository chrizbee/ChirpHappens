// Runs the detection on a recording and prints every tone and pattern.
//
// Convert a recording and run it, optionally overriding settings:
//   ffmpeg -i whistle.wav -ac 1 -ar 8000 -f f32le whistle.f32
//   ./simulate whistle.f32 min_purity=0.4 pause_ms=400 sequence=1750,-400,400
//
// Pass -q to print the pattern times only.

#include "../components/chirphappens/tone_tracker.h"
#include "spectrum.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>

using namespace esphome::chirphappens;

int main(int argc, char **argv)
{
    bool quiet = false;
    const char *path = nullptr;
    std::map<std::string, float> set;
    std::vector<float> sequence;
    for (int i = 1; i < argc; ++i) {
        const char *equals = std::strchr(argv[i], '=');
        if (!std::strcmp(argv[i], "-q"))
            quiet = true;
        else if (!std::strncmp(argv[i], "sequence=", 9))
            for (char *value = std::strtok(argv[i] + 9, ","); value; value = std::strtok(nullptr, ","))
                sequence.push_back(std::atof(value));
        else if (equals)
            set[std::string(argv[i], equals - argv[i])] = std::atof(equals + 1);
        else
            path = argv[i];
    }
    FILE *file = !path || !std::strcmp(path, "-") ? stdin : std::fopen(path, "rb");
    if (!file) {
        std::perror(path);
        return 1;
    }

    TrackerConfig c;
    const std::map<std::string, float *> floats = {
        { "start_freq", &c.start_freq }, { "end_freq", &c.end_freq },
        { "peak_threshold", &c.peak_threshold }, { "peak_to_mean", &c.peak_to_mean },
        { "min_purity", &c.min_purity }, { "max_spread", &c.max_spread },
        { "max_delta_base", &c.max_delta_base }, { "max_delta_freq", &c.max_delta_freq },
        { "max_rhythm", &c.max_rhythm },
    };
    const std::map<std::string, int *> ints = {
        { "min_frames", &c.min_frames }, { "max_tone_ms", &c.max_tone_ms },
        { "pause_ms", &c.pause_ms }, { "max_delta_time", &c.max_delta_time },
        { "isolation_ms", &c.isolation_ms }, { "trailing_ms", &c.trailing_ms }, { "cooldown_ms", &c.cooldown_ms },
    };
    for (const auto &entry : set) {
        if (floats.count(entry.first))
            *floats.at(entry.first) = entry.second;
        else if (ints.count(entry.first))
            *ints.at(entry.first) = static_cast<int>(entry.second);
        else {
            std::fprintf(stderr, "Unknown setting %s\n", entry.first.c_str());
            return 1;
        }
    }

    if (!sequence.empty())
        c.sequence = sequence;

    ToneTracker tracker;
    tracker.configure(c);
    const float frame_s = c.fft_size / c.sample_rate;
    if (!quiet) {
        tracker.on_tone = [&](const Tone &tone, float ms) {
            std::printf("%8.2f s  tone %6.0f Hz  %5.0f ms  spread %.3f%s\n", tone.start * frame_s,
                        tone.frequency, ms, tone.spread, tone.valid ? "" : "  (invalid)");
        };
        tracker.on_reject = [](const char *reason) { std::printf("           rejected: %s\n", reason); };
    }

    Spectrum spectrum(c.fft_size);
    std::vector<float> samples(c.fft_size);
    unsigned long frame = 0;
    while (std::fread(samples.data(), sizeof(float), samples.size(), file) == samples.size()) {
        ++frame;
        if (tracker.process_frame(spectrum.compute(samples.data())))
            std::printf(quiet ? "%.2f\n" : "%8.2f s  PATTERN\n", frame * frame_s);
    }
    return 0;
}
