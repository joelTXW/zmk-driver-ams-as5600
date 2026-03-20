#define DT_DRV_COMPAT zmk_input_ams_as5600

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#if IS_ENABLED(CONFIG_ZMK_INPUT_AMS_AS5600_SET_HID_RESOLUTION_MULTIPLIER)
/*
 * This module is compiled with ZMK's module include path, which may not expose
 * app headers such as zmk/endpoints.h across all ZMK versions. Keep minimal
 * local declarations to stay source-compatible.
 */
enum zmk_transport {
    ZMK_TRANSPORT_NONE = 0,
    ZMK_TRANSPORT_USB = 1,
    ZMK_TRANSPORT_BLE = 2,
};

struct zmk_transport_usb_data {};

struct zmk_transport_ble_data {
    int profile_index;
};

struct zmk_endpoint_instance {
    enum zmk_transport transport;
    union {
        struct zmk_transport_usb_data usb;
        struct zmk_transport_ble_data ble;
    };
};

struct zmk_pointing_resolution_multipliers {
    uint8_t wheel;
    uint8_t hor_wheel;
};

int zmk_ble_active_profile_index(void);
void zmk_pointing_resolution_multipliers_set_profile(
    struct zmk_pointing_resolution_multipliers multipliers, struct zmk_endpoint_instance endpoint);
#endif

#include "zmk_input_ams_as5600/zmk_input_ams_as5600_config.h"

LOG_MODULE_REGISTER(zmk_input_ams_as5600, CONFIG_INPUT_LOG_LEVEL);

#define ZMK_INPUT_AMS_AS5600_CONF_REGISTER 0x07
#define ZMK_INPUT_AMS_AS5600_STATUS_REGISTER 0x0B
#define ZMK_INPUT_AMS_AS5600_RAW_ANGLE_REGISTER 0x0C
#define ZMK_INPUT_AMS_AS5600_AGC_REGISTER 0x1A

#define ZMK_INPUT_AMS_AS5600_STATUS_REGISTER_AGC_UNDERFLOW_BIT 3
#define ZMK_INPUT_AMS_AS5600_STATUS_REGISTER_AGC_OVERFLOW_BIT 4
#define ZMK_INPUT_AMS_AS5600_STATUS_REGISTER_MAGNET_DETECTED_BIT 5

#define ZMK_INPUT_AMS_AS5600_PULSES_PER_REV 4096

#define ZMK_INPUT_AMS_AS5600_LOG_PREFIX "AS5600: "

#if IS_ENABLED(CONFIG_ZMK_INPUT_AMS_AS5600_STATUS_REGISTER_MONITORING)
#define ZMK_INPUT_AMS_AS5600_READ_BUFFER_SIZE 3
#define ZMK_INPUT_AMS_AS5600_READ_OFFSET_STATUS 0
#define ZMK_INPUT_AMS_AS5600_READ_OFFSET_ANGLE_HI 1
#define ZMK_INPUT_AMS_AS5600_READ_OFFSET_ANGLE_LO 2
#define ZMK_INPUT_AMS_AS5600_READ_REGISTER ZMK_INPUT_AMS_AS5600_STATUS_REGISTER
#else
#define ZMK_INPUT_AMS_AS5600_READ_BUFFER_SIZE 2
#define ZMK_INPUT_AMS_AS5600_READ_OFFSET_ANGLE_HI 0
#define ZMK_INPUT_AMS_AS5600_READ_OFFSET_ANGLE_LO 1
#define ZMK_INPUT_AMS_AS5600_READ_REGISTER ZMK_INPUT_AMS_AS5600_RAW_ANGLE_REGISTER
#endif

#define GET_BIT(reg, pos) ((reg) & (1 << (pos)))

struct zmk_input_ams_as5600_config {
    struct i2c_dt_spec i2c_port;
    struct gpio_dt_spec power_gpio;
};

enum zmk_input_ams_as5600_mode {
    ZMK_INPUT_AMS_AS5600_MODE_IDLE,
    ZMK_INPUT_AMS_AS5600_MODE_ACTIVE,
};

struct zmk_input_ams_as5600_data {
    const struct device *dev;
    uint16_t last_angle;
    bool last_angle_initialized;
    bool sensor_powered;
    bool sensor_configured;
    enum zmk_input_ams_as5600_mode mode;
    int64_t last_movement_ms;
    int32_t scaled_remainder;
    struct k_work_delayable work;
};

#if IS_ENABLED(CONFIG_ZMK_INPUT_AMS_AS5600_SET_HID_RESOLUTION_MULTIPLIER)
static inline void zmk_input_ams_as5600_apply_resolution_multiplier(void) {
    struct zmk_pointing_resolution_multipliers multipliers = {
        .wheel = CONFIG_ZMK_INPUT_AMS_AS5600_HID_WHEEL_RESOLUTION,
        .hor_wheel = CONFIG_ZMK_INPUT_AMS_AS5600_HID_WHEEL_RESOLUTION,
    };

#if IS_ENABLED(CONFIG_ZMK_BLE)
    int profile = zmk_ble_active_profile_index();
    if (profile >= 0) {
        struct zmk_endpoint_instance ble_endpoint = {
            .transport = ZMK_TRANSPORT_BLE,
            .ble = {.profile_index = profile},
        };
        zmk_pointing_resolution_multipliers_set_profile(multipliers, ble_endpoint);
    }
#endif

#if IS_ENABLED(CONFIG_ZMK_USB)
    struct zmk_endpoint_instance usb_endpoint = {
        .transport = ZMK_TRANSPORT_USB,
    };
    zmk_pointing_resolution_multipliers_set_profile(multipliers, usb_endpoint);
#endif
}
#endif

#if IS_ENABLED(CONFIG_ZMK_INPUT_AMS_AS5600_LOG_AGC)
static void zmk_input_ams_as5600_log_agc(const struct device *dev) {
    const struct zmk_input_ams_as5600_config *config = dev->config;

    int err;
    uint8_t agc;

    err = i2c_burst_read_dt(&(config->i2c_port),
                            ZMK_INPUT_AMS_AS5600_AGC_REGISTER,
                            &agc,
                            sizeof(agc));
    if (err) {
        LOG_ERR(ZMK_INPUT_AMS_AS5600_LOG_PREFIX "I2C read agc failed: %d", err);
        return;
    }
    LOG_INF(ZMK_INPUT_AMS_AS5600_LOG_PREFIX "AGC: %d", agc);
}
#endif

static int zmk_input_ams_as5600_write_configuration(const struct device *dev) {
    const struct zmk_input_ams_as5600_config *config = dev->config;

    return i2c_burst_write_dt(&(config->i2c_port), ZMK_INPUT_AMS_AS5600_CONF_REGISTER,
                              (const uint8_t *)&ZMK_INPUT_AMS_AS5600_CONFIG,
                              sizeof(ZMK_INPUT_AMS_AS5600_CONFIG));
}

static int zmk_input_ams_as5600_set_power(const struct device *dev, bool on) {
    const struct zmk_input_ams_as5600_config *config = dev->config;
    struct zmk_input_ams_as5600_data *data = dev->data;

    if (config->power_gpio.port == NULL) {
        data->sensor_powered = true;
        return 0;
    }

    if (data->sensor_powered == on) {
        return 0;
    }

    int err = gpio_pin_set_dt(&config->power_gpio, on ? 1 : 0);
    if (err) {
        LOG_ERR(ZMK_INPUT_AMS_AS5600_LOG_PREFIX "Failed to set sensor power: %d", err);
        return err;
    }

    data->sensor_powered = on;
    if (!on) {
        data->sensor_configured = false;
        data->last_angle_initialized = false;
        data->scaled_remainder = 0;
    }

    return 0;
}

static int zmk_input_ams_as5600_power_on_and_prepare(const struct device *dev) {
    struct zmk_input_ams_as5600_data *data = dev->data;
    int err;

    if (!data->sensor_powered) {
        err = zmk_input_ams_as5600_set_power(dev, true);
        if (err) {
            return err;
        }
        k_sleep(K_MSEC(CONFIG_ZMK_INPUT_AMS_AS5600_SENSOR_STARTUP_DELAY_MS));
        data->sensor_configured = false;
        data->last_angle_initialized = false;
        data->scaled_remainder = 0;
    }

    if (!data->sensor_configured) {
        err = zmk_input_ams_as5600_write_configuration(dev);
        if (err) {
            LOG_ERR(ZMK_INPUT_AMS_AS5600_LOG_PREFIX "Failed to write configuration: %d", err);
            return err;
        }
        data->sensor_configured = true;
    }

    return 0;
}

static int zmk_input_ams_as5600_scale_pulses(struct zmk_input_ams_as5600_data *data, int32_t pulses,
                                             int32_t *scaled) {
    int32_t accumulated =
        pulses * CONFIG_ZMK_INPUT_AMS_AS5600_SCROLL_SCALE_MULTIPLIER + data->scaled_remainder;

    *scaled = accumulated / CONFIG_ZMK_INPUT_AMS_AS5600_SCROLL_SCALE_DIVISOR;
    data->scaled_remainder = accumulated % CONFIG_ZMK_INPUT_AMS_AS5600_SCROLL_SCALE_DIVISOR;

    return 0;
}

static int zmk_input_ams_as5600_process(const struct device *dev, int32_t *reported_pulses,
                                        bool *movement_detected) {
    const struct zmk_input_ams_as5600_config *config = dev->config;
    struct zmk_input_ams_as5600_data *data = dev->data;

    int err;
    uint8_t read_buffer[ZMK_INPUT_AMS_AS5600_READ_BUFFER_SIZE];
#if IS_ENABLED(CONFIG_ZMK_INPUT_AMS_AS5600_STATUS_REGISTER_MONITORING)
    uint8_t status;
#endif
    uint16_t angle;
    int32_t pulses;

#if IS_ENABLED(CONFIG_ZMK_INPUT_AMS_AS5600_LOG_AGC)
    /* Log AGC value to console */
    zmk_input_ams_as5600_log_agc(dev);
#endif

    /* Read status and angle */
    err = i2c_burst_read_dt(&(config->i2c_port),
                            ZMK_INPUT_AMS_AS5600_READ_REGISTER,
                            &read_buffer[0],
                            sizeof(read_buffer));
    if (err) {
        LOG_ERR(ZMK_INPUT_AMS_AS5600_LOG_PREFIX "I2C read data failed: %d", err);
        return err;
    }

#if IS_ENABLED(CONFIG_ZMK_INPUT_AMS_AS5600_STATUS_REGISTER_MONITORING)
    /* Check status bits */
    status = read_buffer[ZMK_INPUT_AMS_AS5600_READ_OFFSET_STATUS];
    if (GET_BIT(status, ZMK_INPUT_AMS_AS5600_STATUS_REGISTER_AGC_OVERFLOW_BIT)) {
        LOG_ERR(ZMK_INPUT_AMS_AS5600_LOG_PREFIX "AGC maximum gain overflow, magnet too weak");
        return -1;
    }
    if (GET_BIT(status, ZMK_INPUT_AMS_AS5600_STATUS_REGISTER_AGC_UNDERFLOW_BIT)) {
        LOG_ERR(ZMK_INPUT_AMS_AS5600_LOG_PREFIX "AGC minimum gain overflow, magnet too strong");
        return -1;
    }
    if (!GET_BIT(status, ZMK_INPUT_AMS_AS5600_STATUS_REGISTER_MAGNET_DETECTED_BIT)) {
        LOG_ERR(ZMK_INPUT_AMS_AS5600_LOG_PREFIX "Magnet not detected");
        return -1;
    }
#endif

    /* Calculate angle and pulses */
    angle = ((uint16_t)read_buffer[ZMK_INPUT_AMS_AS5600_READ_OFFSET_ANGLE_HI] << 8) | read_buffer[ZMK_INPUT_AMS_AS5600_READ_OFFSET_ANGLE_LO];
    pulses = angle - data->last_angle;
    if (pulses > (ZMK_INPUT_AMS_AS5600_PULSES_PER_REV / 2)) {
        pulses -= ZMK_INPUT_AMS_AS5600_PULSES_PER_REV;
    } else if (pulses < (-ZMK_INPUT_AMS_AS5600_PULSES_PER_REV / 2)) {
        pulses += ZMK_INPUT_AMS_AS5600_PULSES_PER_REV;
    }

    LOG_DBG(ZMK_INPUT_AMS_AS5600_LOG_PREFIX "Angle: %d, Last Angle: %d, Pulses: %d", angle, data->last_angle, pulses);

    /* Skip the first value to avoid a large initial jump */
    if (!data->last_angle_initialized) {
        data->last_angle = angle;
        data->last_angle_initialized = true;
        *reported_pulses = 0;
        *movement_detected = false;
        return 0;
    }

    data->last_angle = angle;

    if (pulses < 0) {
        *movement_detected = (-pulses) >= CONFIG_ZMK_INPUT_AMS_AS5600_MOVEMENT_THRESHOLD;
    } else {
        *movement_detected = pulses >= CONFIG_ZMK_INPUT_AMS_AS5600_MOVEMENT_THRESHOLD;
    }

    if (!*movement_detected) {
        *reported_pulses = 0;
        return 0;
    }

    zmk_input_ams_as5600_scale_pulses(data, pulses, reported_pulses);

    return 0;
}

static int zmk_input_ams_as5600_report_scroll(const struct device *dev, int32_t pulses) {
    int err;

    if (pulses != 0) {
#if IS_ENABLED(CONFIG_ZMK_INPUT_AMS_AS5600_SET_HID_RESOLUTION_MULTIPLIER)
        zmk_input_ams_as5600_apply_resolution_multiplier();
#endif
        err = input_report_rel(dev, INPUT_REL_WHEEL, pulses, true, K_FOREVER);
        if (err) {
            LOG_ERR(ZMK_INPUT_AMS_AS5600_LOG_PREFIX "Failed to report input value: %d", err);
            return err;
        }
    }

    return 0;
}

static void zmk_input_ams_as5600_schedule(struct zmk_input_ams_as5600_data *data, int interval_ms) {
    k_work_reschedule(&data->work, K_MSEC(interval_ms));
}

static void zmk_input_ams_as5600_work_handler(struct k_work *work) {
    struct zmk_input_ams_as5600_data *data =
        CONTAINER_OF(work, struct zmk_input_ams_as5600_data, work.work);
    const struct device *dev = data->dev;

    bool movement_detected = false;
    int32_t reported_pulses = 0;
    int err;

    if (data->mode == ZMK_INPUT_AMS_AS5600_MODE_IDLE) {
        err = zmk_input_ams_as5600_power_on_and_prepare(dev);
        if (err) {
            (void)zmk_input_ams_as5600_set_power(dev, false);
            zmk_input_ams_as5600_schedule(data, CONFIG_ZMK_INPUT_AMS_AS5600_IDLE_POLL_INTERVAL);
            return;
        }

        err = zmk_input_ams_as5600_process(dev, &reported_pulses, &movement_detected);
        if (err) {
            (void)zmk_input_ams_as5600_set_power(dev, false);
            zmk_input_ams_as5600_schedule(data, CONFIG_ZMK_INPUT_AMS_AS5600_IDLE_POLL_INTERVAL);
            return;
        }

        if (movement_detected) {
            data->mode = ZMK_INPUT_AMS_AS5600_MODE_ACTIVE;
            data->last_movement_ms = k_uptime_get();
            (void)zmk_input_ams_as5600_report_scroll(dev, reported_pulses);
            zmk_input_ams_as5600_schedule(data, CONFIG_ZMK_INPUT_AMS_AS5600_ACTIVE_POLL_INTERVAL);
            return;
        }

        (void)zmk_input_ams_as5600_set_power(dev, false);
        zmk_input_ams_as5600_schedule(data, CONFIG_ZMK_INPUT_AMS_AS5600_IDLE_POLL_INTERVAL);
        return;
    }

    err = zmk_input_ams_as5600_power_on_and_prepare(dev);
    if (err) {
        zmk_input_ams_as5600_schedule(data, CONFIG_ZMK_INPUT_AMS_AS5600_ACTIVE_POLL_INTERVAL);
        return;
    }

    err = zmk_input_ams_as5600_process(dev, &reported_pulses, &movement_detected);
    if (!err && movement_detected) {
        data->last_movement_ms = k_uptime_get();
        (void)zmk_input_ams_as5600_report_scroll(dev, reported_pulses);
    }

    if ((k_uptime_get() - data->last_movement_ms) >=
        CONFIG_ZMK_INPUT_AMS_AS5600_INACTIVITY_TIMEOUT_MS) {
        data->mode = ZMK_INPUT_AMS_AS5600_MODE_IDLE;
        (void)zmk_input_ams_as5600_set_power(dev, false);
        zmk_input_ams_as5600_schedule(data, CONFIG_ZMK_INPUT_AMS_AS5600_IDLE_POLL_INTERVAL);
        return;
    }

    zmk_input_ams_as5600_schedule(data, CONFIG_ZMK_INPUT_AMS_AS5600_ACTIVE_POLL_INTERVAL);
}

static int zmk_input_ams_as5600_initialize(const struct device *dev) {
    const struct zmk_input_ams_as5600_config *config = dev->config;
    struct zmk_input_ams_as5600_data *data = dev->data;

    int err;

    data->dev = dev;
    data->last_angle = 0;
    data->last_angle_initialized = false;
    data->scaled_remainder = 0;
    data->mode = ZMK_INPUT_AMS_AS5600_MODE_IDLE;
    data->last_movement_ms = k_uptime_get();
    data->sensor_powered = false;
    data->sensor_configured = false;

    if (config->power_gpio.port != NULL) {
        if (!device_is_ready(config->power_gpio.port)) {
            LOG_ERR(ZMK_INPUT_AMS_AS5600_LOG_PREFIX "Power GPIO is not ready");
            return -ENODEV;
        }

        err = gpio_pin_configure_dt(&config->power_gpio, GPIO_OUTPUT_INACTIVE);
        if (err) {
            LOG_ERR(ZMK_INPUT_AMS_AS5600_LOG_PREFIX "Failed to configure power GPIO: %d", err);
            return err;
        }
    } else {
        data->sensor_powered = true;
        data->sensor_configured = false;
    }

    k_work_init_delayable(&data->work, &zmk_input_ams_as5600_work_handler);
    zmk_input_ams_as5600_schedule(data, 0);

    LOG_INF(ZMK_INPUT_AMS_AS5600_LOG_PREFIX "Device %s initialized", dev->name);

    return 0;
}

#define ZMK_INPUT_AMS_AS5600_INIT(n)                               \
    static struct zmk_input_ams_as5600_data zmk_input_ams_as5600_data##n;                      \
    static const struct zmk_input_ams_as5600_config zmk_input_ams_as5600_cfg##n = {            \
        .i2c_port = I2C_DT_SPEC_INST_GET(n), \
        .power_gpio = GPIO_DT_SPEC_INST_GET_OR(n, power_gpios, {0})};                      \
                                                                   \
    DEVICE_DT_INST_DEFINE(n, zmk_input_ams_as5600_initialize, NULL,              \
                          &zmk_input_ams_as5600_data##n, &zmk_input_ams_as5600_cfg##n,         \
                          POST_KERNEL, CONFIG_ZMK_INPUT_AMS_AS5600_INIT_PRIORITY, \
                          NULL);

DT_INST_FOREACH_STATUS_OKAY(ZMK_INPUT_AMS_AS5600_INIT)
