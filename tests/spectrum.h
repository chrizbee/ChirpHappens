#pragma once

// Same processing as ChirpHappens::loop() does with arduinoFFT:
// DC removal, Hann window, FFT, magnitude. Input is scaled like the int32
// samples of the I2S microphone, so thresholds are the same as on the device.

#include <cmath>
#include <complex>
#include <vector>

class Spectrum {
public:
    explicit Spectrum(int size) : size_(size), window_(size), data_(size), magnitudes_(size / 2)
    {
        for (int i = 0; i < size; ++i)
            window_[i] = 0.5 * (1.0 - std::cos(2.0 * M_PI * i / (size - 1)));
    }

    // Samples in the range -1.0 .. 1.0
    const float *compute(const float *samples)
    {
        double mean = 0.0;
        for (int i = 0; i < size_; ++i)
            mean += samples[i];
        mean /= size_;
        for (int i = 0; i < size_; ++i)
            data_[i] = (samples[i] - mean) * 2147483648.0 * window_[i];
        fft();
        for (int i = 0; i < size_ / 2; ++i)
            magnitudes_[i] = static_cast<float>(std::abs(data_[i]));
        return magnitudes_.data();
    }

private:
    void fft()
    {
        for (int i = 1, j = 0; i < size_; ++i) {
            int bit = size_ >> 1;
            for (; j & bit; bit >>= 1)
                j ^= bit;
            j ^= bit;
            if (i < j)
                std::swap(data_[i], data_[j]);
        }
        for (int length = 2; length <= size_; length <<= 1) {
            const std::complex<double> step = std::polar(1.0, -2.0 * M_PI / length);
            for (int i = 0; i < size_; i += length) {
                std::complex<double> w(1.0);
                for (int j = 0; j < length / 2; ++j) {
                    const std::complex<double> a = data_[i + j];
                    const std::complex<double> b = data_[i + j + length / 2] * w;
                    data_[i + j] = a + b;
                    data_[i + j + length / 2] = a - b;
                    w *= step;
                }
            }
        }
    }

    int size_;
    std::vector<double> window_;
    std::vector<std::complex<double>> data_;
    std::vector<float> magnitudes_;
};
