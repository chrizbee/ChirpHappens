#include "chirphappens.h"
#include "esphome/core/log.h"

namespace esphome {
namespace chirphappens {

static const char *TAG = "chirphappens";

void ChirpHappens::setup()
{
     ESP_LOGCONFIG(TAG, "Setting up ChirpHappens...");

    // Pullup / -down for channel select
    pinMode(pin_i2s_lr_, OUTPUT);
    digitalWrite(pin_i2s_lr_, i2s_channel_);

    // I2S config
    i2s_config_t i2s_config = {
        .mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = sample_rate_,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = i2s_channel_ ? I2S_CHANNEL_FMT_ONLY_RIGHT : I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = buffer_size_,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0,
    };

    // I2S pins
    i2s_pin_config_t pin_config = {
        .bck_io_num = pin_i2s_sck_,
        .ws_io_num = pin_i2s_ws_,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = pin_i2s_sd_,
    };

    // Configuring the I2S driver and pins
    // This function must be called before any I2S driver read/write operations
    ESP_ERROR_CHECK(i2s_driver_install(i2s_port_, &i2s_config, 0, nullptr));
    ESP_ERROR_CHECK(i2s_set_pin(i2s_port_, &pin_config));

    // Initialize FFT buffers
    v_real_.resize(buffer_size_);
    v_imag_.resize(buffer_size_);
    sample_buffer_.resize(buffer_size_);
    // Last parameter (false) = windowingFactors: internal storage of the windowing factors?
    fft_ = ArduinoFFT<float>(v_real_.data(), v_imag_.data(), buffer_size_, sample_rate_, false);

    // Initialize variables
    pattern_detected_ = false;
    sequence_index_ = 0;
    consecutive_count_ = 0;
    last_frequency_ = 0.0f;
    last_time_ = 0;
    last_detection_time_ = 0;
    publish_initial_state(false);
}

void ChirpHappens::loop()
{
    // Read samples from I2S microphone
    size_t bytes_read = 0;
    i2s_read(i2s_port_, sample_buffer_.data(), sample_buffer_.size() * sizeof(int32_t), &bytes_read, portMAX_DELAY);
    int samples_read = bytes_read / sizeof(int32_t);
    if (samples_read < buffer_size_)
        return;

    // Reset switch state after some time
    unsigned long current_time = millis();
    if (pattern_detected_ && (current_time - last_detection_time_ > reset_time_)) {
        pattern_detected_ = false;
        this->publish_state(false);
    }
    
    // Check if enough time has passed already
    unsigned long elapsed = current_time - last_time_;
    if (elapsed < pause_ms_ - max_delta_time_)
        return;

    // Reset if too much time has passed
    // This won't matter if it's the first frequency in sequence
    if (elapsed > pause_ms_ + max_delta_time_) {
        last_frequency_ = 0.0;

        // Reset consecutive detections only if it's not the first frequency in sequence
        // This is because elapsed always greater than pause_ms_ since there was no last_time_
        if (sequence_index_ > 0)
            consecutive_count_ = 0;
        sequence_index_ = 0;
    }


    // Fill real and complex arrays
    for (int i = 0; i < buffer_size_; ++i) {
        v_real_[i] = static_cast<float>(sample_buffer_[i]);
        v_imag_[i] = 0.0f;
    }

    // Calculate fft
    // This will operate on given buffers (v_real_ and v_imag_)
    fft_.dcRemoval();
    fft_.windowing(FFT_WIN_TYP_HANN, FFT_FORWARD);
    fft_.compute(FFT_FORWARD);
    fft_.complexToMagnitude();

    // Get frequency, peak / mean amplitude and budget snr
    // majorPeak() always uses the whole range, so I wrote calculate_peak()
    float frequency, peak_amplitude, mean_amplitude;
    calculate_peak_mean(v_real_, sample_rate_, start_freq_, end_freq_, &frequency, &peak_amplitude, &mean_amplitude);
    // float mean_amplitude = std::accumulate(v_real_.begin(), v_real_.end(), 0.0f) / buffer_size_;
    float peak_to_mean = peak_amplitude / mean_amplitude;

    // Check if thresholds are exceeded and reset consecutive counter if not
    if (peak_amplitude < peak_threshold_ || peak_to_mean < peak_to_mean_) {
        consecutive_count_ = 0;
        return;
    }

    // Check for index in range
    if (sequence_index_ >= sequence_.size())
        return;

    // Get delta values
    float expected_frequency = last_frequency_ + sequence_[sequence_index_];
    float delta_frequency = abs(frequency - expected_frequency);
    int delta_time = abs(pause_ms_ - (int)elapsed);

    // First frequency will be more forgiving
    if ((sequence_index_ == 0 && delta_frequency <= max_delta_freq_ * 1.5) ||
        (delta_frequency <= max_delta_freq_ && delta_time <= max_delta_time_)) {

        // Move to the next frequency only if enough consecutive detections
        if (++consecutive_count_ >= consecutive_detections_) {

            // Reset consecutive counter for next frequency
            consecutive_count_ = 0;

            // Set last frequency to this one
            // First measured frequency will be the base for all upcoming
            last_frequency_ = sequence_index_ == 0 ? frequency : expected_frequency;

            // Check if end of pattern is reached
            if (++sequence_index_ >= sequence_.size()) {
                this->publish_state(true);
                last_detection_time_ = current_time;
                pattern_detected_ = true;
                sequence_index_ = 0;
            
            // Else store current time
            } else last_time_ = current_time;
        }
    
    // Reset consecutive counter if the current frequency is invalid
    } else consecutive_count_ = 0;
}

void ChirpHappens::dump_config() {
    ESP_LOGCONFIG(TAG, "ChirpHappens");
}

void ChirpHappens::calculate_peak_mean(const std::vector<float> &data, float sample_rate, float start_hz, float end_hz, float *peak_frequency, float *peak_amplitude, float *mean_amplitude)
{
    // Calculate frequency resolution
    int sample_count = data.size();
    float frequency_resolution = sample_rate / sample_count;

    // Define the range in terms of indices
    int start_index = (start_hz == -1) ? 0 : std::max(0, static_cast<int>(std::ceil(start_hz / frequency_resolution)));
    int end_index = (end_hz == -1) ? (sample_count / 2) : std::min(sample_count / 2, static_cast<int>(std::floor(end_hz / frequency_resolution)));

    // Ensure valid range
    if (start_index >= end_index) {
        *peak_frequency = 0;
        *peak_amplitude = 0;
        return;
    }

    // Find the maximum in the specified range
    auto max_element_iter = std::max_element(data.begin() + start_index, data.begin() + end_index);
    int max_index = std::distance(data.begin(), max_element_iter);

    // Calculate mean amplitude for the specified range
    *mean_amplitude = std::accumulate(data.begin() + start_index, data.begin() + end_index, 0.0f) / (end_index - start_index + 1);

    // Calculate interpolated peak (parabolic interpolation for better accuracy)
    if (max_index > 0 && max_index < (sample_count / 2) - 1) {
        float y0 = data[max_index - 1];
        float y1 = data[max_index];
        float y2 = data[max_index + 1];
        
        // Calculate the parabolic offset
        float delta = 0.5f * (y0 - y2) / (y0 - 2.0f * y1 + y2);

        // Interpolated frequency
        *peak_frequency = (max_index + delta) * frequency_resolution;

        // Interpolated amplitude (magnitude)
        *peak_amplitude = std::abs(y1 - (y0 - y2) * delta / 2.0f);
    
    // Edge case: no interpolation possible
    } else {
        *peak_frequency = max_index * frequency_resolution;
        *peak_amplitude = std::abs(*max_element_iter);
    }
}

}  // namespace chirphappens
}  // namespace esphome
