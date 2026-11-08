# ChirpHappens
ESPHome component for detecting whistle patterns.

## Hardware

To make the device as small as possible, the following hardware was used:

- Microcontroller: [NoLogo ESP32-C3 SuperMini](https://www.nologo.tech/product/esp32/esp32c3/esp32c3supermini/esp32C3SuperMini.html), [Pinout](https://www.sudo.is/docs/esphome/boards/esp32c3supermini/)
- Microphone: [INMP441 MEMS I2S Microphone](https://www.berrybase.de/en/inmp441-mems-omnidirektionales-mikrofonmodul-i2s-interface)
- Case: [ChirpHappens on Printables](https://www.printables.com/model/1090007-chirphappens)

![](images/esp32c3_supermini.png) ![](images/inmp441.png) ![](images/hat.png)

## Software

Build dependencies: [ESPHome](https://esphome.io/guides/installing_esphome)

An example YAML configuration for the mentioned hardware can be found [here](example.yaml).

### Building & Uploading

Run the following commands to build and run the example configuration.

```bash
esphome compile example.yaml # Build only
esphome run example.yaml     # Build & run
```

### Configuration

When using the hardware and pin configuration specified above, there is no need to change anything in the example configuration. Otherwise you might want to adapt to your hardware.

Enter your WiFi credentials here or just connect to the fallback hotspot and enter your credentials from there.

```yaml
# Default wifi configuration
wifi:
  # ssid: "Your WiFi SSID"
  # password: "VerySafePassword"

  # Enable fallback hotspot with captive portal in case wifi connection fails
  ap:
    ssid: "chirphappens"
    password: ""
```

Either clone the chirphappens repository and edit & run the [example.yaml](example,yaml) file from there to use the component locally or pull the component from GitHub.

```yaml
# External chirphappens component
# Either source from cloned repository or pull from GitHub
external_components:
  # - source: components
  - source:
      type: git
      url: https://github.com/chrizbee/ChirpHappens
      ref: main
    components: [ chirphappens ]
```

The default pin configuration for the hardware specified above is:

```yaml
# I2S pin configuration
pin_i2s_ws: 7        # I2S WS pin
pin_i2s_sck: 8       # I2S SCK pin
pin_i2s_sd: 4        # I2S SD pin
pin_i2s_lr: 6        # I2S L/R pin
i2s_channel: left    # I2S channel
i2s_port: I2S_NUM_0  # I2S port
```

## Detection

Detection works in three steps, so that a sloppy whistle is accepted while everything else is not:

1. **Frame**: Only a clean sine counts. Nearly all of the energy has to be in one peak (`min_purity`), which is not the case for speech and most music.
2. **Tone**: Frames with a stable pitch are merged into one tone with a median frequency. Slides between tones and outliers are ignored.
3. **Pattern**: The last tones are compared as a whole. Only the intervals have to match, so you can whistle higher or lower. The tones also have to be evenly spaced with silence before the sequence (`isolation_ms`), which rejects melodies and sirens.

The sensor triggers as soon as the last tone has lasted `consecutive_detections` frames.

### Tuning

With the default log level, every recognized tone is logged. Whistle your sequence and compare with your configuration:

```
[D][chirphappens]: Tone: 1762 Hz, 224 ms, -58 dBFS, purity 0.81, spread 0.012
```

If there are no tones at all, the frames are rejected: lower `peak_threshold` or `min_purity`. If there are tones but no detection, adapt `sequence`, `pause_ms` or the tolerances. Raise tolerances one at a time, as each of them lets in more false positives.

### Testing

The detection in [tone_tracker.cpp](components/chirphappens/tone_tracker.cpp) has no hardware dependencies and can be tested on a PC.

```bash
g++ -std=c++14 -O2 -o test_tone_tracker tests/test_tone_tracker.cpp components/chirphappens/tone_tracker.cpp
./test_tone_tracker
```

To try settings on a recording of your own, use the simulator. It prints all tones and patterns.

```bash
g++ -std=c++14 -O2 -o simulate tests/simulate.cpp components/chirphappens/tone_tracker.cpp
ffmpeg -i whistle.wav -ac 1 -ar 8000 -f f32le whistle.f32
./simulate whistle.f32 min_purity=0.4
```
