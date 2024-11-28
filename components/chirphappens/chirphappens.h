#pragma once

#include "esphome/core/component.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "arduinoFFT.h"
#include <driver/i2s.h>
#include <vector>
#include <numeric>

// Use port 0 no matter what if we have only one
// This is the case for ESP32-C3
#if SOC_I2S_NUM <= 1
    #define I2S_NUM_1  I2S_NUM_0
#endif

namespace esphome {
namespace chirphappens {

class ChirpHappens : public binary_sensor::BinarySensor, public Component {
public:
    // Configuration methods
    void set_sequence(const std::vector<float> &sequence) { sequence_ = sequence; }
    void set_pause_ms(int pause_ms) { pause_ms_ = pause_ms; }
    void set_max_delta_freq(int max_delta_freq) { max_delta_freq_ = max_delta_freq; }
    void set_max_delta_time(int max_delta_time) { max_delta_time_ = max_delta_time; }
    void set_peak_threshold(float peak_threshold) { peak_threshold_ = peak_threshold; }
    void set_peak_to_mean(float peak_to_mean) { peak_to_mean_ = peak_to_mean; }
    void set_start_freq(float start_freq) { start_freq_ = start_freq; }
    void set_end_freq(float end_freq) { end_freq_ = end_freq; }
    void set_reset_time(int reset_time) { reset_time_ = reset_time; }
    void set_sample_rate(uint32_t sample_rate) { sample_rate_ = sample_rate; }
    void set_buffer_size(int buffer_size) { buffer_size_ = buffer_size; }
    void set_i2s_pins(int ws, int sck, int sd, int lr) { pin_i2s_ws_ = ws; pin_i2s_sck_ = sck; pin_i2s_sd_ = sd; pin_i2s_lr_ = lr; }
    void set_i2s_port(const std::string &port) { i2s_port_ = port == "I2S_NUM_1" ? I2S_NUM_1 : I2S_NUM_0; }
    void set_i2s_channel(const std::string &channel) { i2s_channel_ = channel == "left" ? 0 : 1; }

    // ESPHome component methods
    void setup() override;
    void loop() override;
    void dump_config() override;

protected:
    void calculate_peak(
        const std::vector<float> &data,
        float sample_rate,
        float start_hz, float end_hz,
        float *frequency, float *amplitude
    );

    bool pattern_detected_;

    // Sequence configuration and thresholds
    std::vector<float> sequence_;
    int pause_ms_;
    int max_delta_freq_;
    int max_delta_time_;
    float peak_threshold_;
    float peak_to_mean_;
    float start_freq_;
    float end_freq_;
    int reset_time_;

    // Audio and I2S configuration
    uint32_t sample_rate_;
    int buffer_size_;
    int pin_i2s_ws_;
    int pin_i2s_sck_;
    int pin_i2s_sd_;
    int pin_i2s_lr_;
    i2s_port_t i2s_port_;
    int i2s_channel_;

    // Buffers and global variables
    ArduinoFFT<float> fft_;
    std::vector<int32_t> sample_buffer_;
    std::vector<float> v_real_, v_imag_;
    uint32_t sequence_index_;
    float last_frequency_;
    unsigned long last_time_;
    unsigned long last_detection_time_;
};

}  // namespace chirphappens
}  // namespace esphome
