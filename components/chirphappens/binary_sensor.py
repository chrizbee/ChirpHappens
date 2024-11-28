import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import CONF_ID

CODEOWNERS = ["chrizbee"]
DEPENDENCIES = ["esp32"]

CONF_SEQUENCE = "sequence"
CONF_PAUSE_MS = "pause_ms"
CONF_MAX_DELTA_FREQ = "max_delta_freq"
CONF_MAX_DELTA_TIME = "max_delta_time"
CONF_PEAK_THRESHOLD = "peak_threshold"
CONF_PEAK_TO_MEAN = "peak_to_mean"
CONF_START_FREQ = "start_freq"
CONF_END_FREQ = "end_freq"
CONF_RESET_TIME = "reset_time"
CONF_SAMPLE_RATE = "sample_rate"
CONF_BUFFER_SIZE = "buffer_size"
CONF_PIN_I2S_WS = "pin_i2s_ws"
CONF_PIN_I2S_SCK = "pin_i2s_sck"
CONF_PIN_I2S_SD = "pin_i2s_sd"
CONF_PIN_I2S_LR = "pin_i2s_lr"
CONF_I2S_PORT = "i2s_port"
CONF_I2S_CHANNEL = "i2s_channel"

chirphappens_ns = cg.esphome_ns.namespace("chirphappens")
ChirpHappens = chirphappens_ns.class_("ChirpHappens", binary_sensor.BinarySensor, cg.Component)

CONFIG_SCHEMA = binary_sensor.binary_sensor_schema(ChirpHappens).extend({
    cv.Optional(CONF_SEQUENCE, default=[1800, -400, 400]): cv.ensure_list(cv.float_),
    cv.Required(CONF_PAUSE_MS): cv.int_,
    cv.Required(CONF_MAX_DELTA_FREQ): cv.int_,
    cv.Required(CONF_MAX_DELTA_TIME): cv.int_,
    cv.Required(CONF_PEAK_THRESHOLD): cv.float_,
    cv.Required(CONF_PEAK_TO_MEAN): cv.float_,
    cv.Required(CONF_START_FREQ): cv.float_,
    cv.Required(CONF_END_FREQ): cv.float_,
    cv.Required(CONF_RESET_TIME): cv.int_,
    cv.Optional(CONF_SAMPLE_RATE, default=8000): cv.int_,
    cv.Optional(CONF_BUFFER_SIZE, default=512): cv.int_,
    cv.Optional(CONF_PIN_I2S_WS, default=7): cv.int_,
    cv.Optional(CONF_PIN_I2S_SCK, default=8): cv.int_,
    cv.Optional(CONF_PIN_I2S_SD, default=4): cv.int_,
    cv.Optional(CONF_PIN_I2S_LR, default=6): cv.int_,
    cv.Optional(CONF_I2S_PORT, default="I2S_NUM_0"): cv.string,
    cv.Optional(CONF_I2S_CHANNEL, default="left"): cv.string,
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    cg.add_library("kosme/arduinoFFT", "^2.0.3")
    var = cg.new_Pvariable(config[CONF_ID])
    cg.add(var.set_sequence(config[CONF_SEQUENCE]))
    cg.add(var.set_pause_ms(config[CONF_PAUSE_MS]))
    cg.add(var.set_max_delta_freq(config[CONF_MAX_DELTA_FREQ]))
    cg.add(var.set_max_delta_time(config[CONF_MAX_DELTA_TIME]))
    cg.add(var.set_peak_threshold(config[CONF_PEAK_THRESHOLD]))
    cg.add(var.set_peak_to_mean(config[CONF_PEAK_TO_MEAN]))
    cg.add(var.set_start_freq(config[CONF_START_FREQ]))
    cg.add(var.set_end_freq(config[CONF_END_FREQ]))
    cg.add(var.set_reset_time(config[CONF_RESET_TIME]))
    cg.add(var.set_sample_rate(config[CONF_SAMPLE_RATE]))
    cg.add(var.set_buffer_size(config[CONF_BUFFER_SIZE]))
    cg.add(var.set_i2s_pins(config[CONF_PIN_I2S_WS], config[CONF_PIN_I2S_SCK], config[CONF_PIN_I2S_SD], config[CONF_PIN_I2S_LR]))
    cg.add(var.set_i2s_port(config[CONF_I2S_PORT]))
    cg.add(var.set_i2s_channel(config[CONF_I2S_CHANNEL]))
    await cg.register_component(var, config)
    await binary_sensor.register_binary_sensor(var, config)
