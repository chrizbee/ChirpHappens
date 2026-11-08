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
    last_detection_time_ = 0;
    publish_initial_state(false);

    // Initialize detection
    tracker_config_.sample_rate = sample_rate_;
    tracker_config_.fft_size = buffer_size_;
    tracker_.configure(tracker_config_);

    // Log every tone, this is what you need to adapt the configuration to your whistle
    tracker_.on_tone = [](const Tone &tone, float ms) {
        ESP_LOGD(TAG, "Tone: %.0f Hz, %.0f ms, spread %.3f%s", tone.frequency, ms, tone.spread, tone.valid ? "" : " (invalid)");
    };
    tracker_.on_reject = [](const char *reason) {
        ESP_LOGD(TAG, "Pattern rejected: %s", reason);
    };
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

    // Check if this frame completes the pattern
    if (tracker_.process_frame(v_real_.data())) {
        ESP_LOGI(TAG, "Pattern detected");
        this->publish_state(true);
        last_detection_time_ = current_time;
        pattern_detected_ = true;
    }
}

void ChirpHappens::dump_config() {
    ESP_LOGCONFIG(TAG, "ChirpHappens");
    ESP_LOGCONFIG(TAG, "  Band: %.0f - %.0f Hz", tracker_config_.start_freq, tracker_config_.end_freq);
    ESP_LOGCONFIG(TAG, "  Tolerance: first tone %.0f Hz, others %.0f Hz", tracker_config_.max_delta_base, tracker_config_.max_delta_freq);
    ESP_LOGCONFIG(TAG, "  Pause: %d +- %d ms", tracker_config_.pause_ms, tracker_config_.max_delta_time);
    ESP_LOGCONFIG(TAG, "  Minimum purity: %.2f", tracker_config_.min_purity);
}

}  // namespace chirphappens
}  // namespace esphome
