/*
 * High Resolution Scroll Dial AS5600 driver with transport-aware power management.
 *
 * USB mode  – 1 ms polling, NOM power mode, sensor always on, no idle.
 * BLE mode  – LPM1 in active mode; after 300 ms inactivity switch AS5600 to
 *             LPM2; after a further 1700 ms (2 s total) power-off the sensor
 *             (idle).  Any detected movement promotes back to active/LPM1.
 *
 * The AS5600 can be switched off via a MOSFET controlled by a GPIO.
 */

#define DT_DRV_COMPAT zmk_input_ams_as5600

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>

/* ─────────── ZMK transport type declarations (always needed) ─────────── */
/*
 * Minimal forward declarations for ZMK endpoint / transport types.
 * Kept local to avoid hard dependency on app-level headers that may
 * not be on the include path for out-of-tree modules.
 */
enum zmk_transport {
    ZMK_TRANSPORT_NONE = 0,
    ZMK_TRANSPORT_USB  = 1,
    ZMK_TRANSPORT_BLE  = 2,
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

/*
 * Returns the currently selected endpoint (USB or BLE profile).
 * Support both historical and current ZMK symbol names so this
 * out-of-tree module stays buildable across ZMK revisions.
 */
extern struct zmk_endpoint_instance zmk_endpoint_get_selected(void)
    __attribute__((weak));
extern struct zmk_endpoint_instance zmk_endpoints_selected(void)
    __attribute__((weak));

static inline struct zmk_endpoint_instance get_selected_endpoint(void) {
    if (zmk_endpoint_get_selected != NULL) {
        return zmk_endpoint_get_selected();
    }
    if (zmk_endpoints_selected != NULL) {
        return zmk_endpoints_selected();
    }

    /* Fallback if neither symbol is linked in this build variant. */
    return (struct zmk_endpoint_instance){.transport = ZMK_TRANSPORT_NONE};
}

#if IS_ENABLED(CONFIG_ZMK_INPUT_AMS_AS5600_SET_HID_RESOLUTION_MULTIPLIER)
struct zmk_pointing_resolution_multipliers {
    uint8_t wheel;
    uint8_t hor_wheel;
};

int  zmk_ble_active_profile_index(void);
void zmk_pointing_resolution_multipliers_set_profile(
    struct zmk_pointing_resolution_multipliers multipliers,
    struct zmk_endpoint_instance endpoint);
#endif

#include "zmk_input_ams_as5600/zmk_input_ams_as5600_config.h"

LOG_MODULE_REGISTER(zmk_input_ams_as5600, CONFIG_INPUT_LOG_LEVEL);

/* ───────────────────────── AS5600 register map ────────────────────────── */
#define AS5600_REG_CONF       0x07
#define AS5600_REG_STATUS     0x0B
#define AS5600_REG_RAW_ANGLE  0x0C
#define AS5600_REG_AGC        0x1A

#define AS5600_STATUS_AGC_UNDERFLOW  3
#define AS5600_STATUS_AGC_OVERFLOW   4
#define AS5600_STATUS_MAGNET_DETECT  5

#define AS5600_PULSES_PER_REV  4096
#define AS5600_ANGLE_MASK      0x0FFF  /* 12-bit angle */

/* AS5600 power-mode field (bits [1:0] of CONF low byte, i.e. byte index 1). */
#define AS5600_PM_NOM   0x00
#define AS5600_PM_LPM1  0x01
#define AS5600_PM_LPM2  0x02
#define AS5600_PM_LPM3  0x03
#define AS5600_PM_MASK  0x03

/* ──────────────────── Timing / polling constants ──────────────────────── */

/* USB: always-on, fastest polling, no idle */
#define USB_POLL_INTERVAL_MS  1

/*
 * BLE active – use the user's Kconfig-tuned active poll interval.
 * AS5600 LPM1 samples every ~4.4 ms; polling the MCU faster just re-reads
 * the same value but keeps latency minimal for the first new sample.
 */
#define BLE_ACTIVE_POLL_MS  CONFIG_ZMK_INPUT_AMS_AS5600_ACTIVE_POLL_INTERVAL

/* BLE low-power (LPM2 – AS5600 samples every ~18 ms) */
#ifndef CONFIG_ZMK_INPUT_AMS_AS5600_BLE_LP_POLL_MS
#define CONFIG_ZMK_INPUT_AMS_AS5600_BLE_LP_POLL_MS  20
#endif
#define BLE_LP_POLL_MS  CONFIG_ZMK_INPUT_AMS_AS5600_BLE_LP_POLL_MS

/* BLE idle (sensor off, wake-check interval) */
#define BLE_IDLE_POLL_MS  CONFIG_ZMK_INPUT_AMS_AS5600_IDLE_POLL_INTERVAL

/* BLE: ms of inactivity before ACTIVE (LPM1) → LOW_POWER (LPM2) */
#define BLE_ACTIVE_TO_LP_MS   300

/* BLE: ms of TOTAL inactivity before LOW_POWER → IDLE (power off) */
#define BLE_ACTIVE_TO_IDLE_MS 2000

/* ────────────────────────── Log prefix ────────────────────────────────── */
#define LOG_PFX  "AS5600: "

/* ────────────────── Read-buffer layout (compile-time) ─────────────────── */
#if IS_ENABLED(CONFIG_ZMK_INPUT_AMS_AS5600_STATUS_REGISTER_MONITORING)
#define READ_BUF_SIZE       3
#define READ_OFF_STATUS     0
#define READ_OFF_ANGLE_HI   1
#define READ_OFF_ANGLE_LO   2
#define READ_START_REG      AS5600_REG_STATUS
#else
#define READ_BUF_SIZE       2
#define READ_OFF_ANGLE_HI   0
#define READ_OFF_ANGLE_LO   1
#define READ_START_REG      AS5600_REG_RAW_ANGLE
#endif

#define GET_BIT(reg, pos) ((reg) & (1U << (pos)))

/* ──────────────────────── Data structures ─────────────────────────────── */

struct zmk_input_ams_as5600_config {
    struct i2c_dt_spec  i2c_port;
    struct gpio_dt_spec power_gpio;
};

enum zmk_input_ams_as5600_mode {
    ZMK_INPUT_AMS_AS5600_MODE_IDLE,       /* Sensor powered off (BLE only)       */
    ZMK_INPUT_AMS_AS5600_MODE_LOW_POWER,  /* LPM2, reduced polling (BLE only)    */
    ZMK_INPUT_AMS_AS5600_MODE_ACTIVE,     /* LPM1 (BLE) / NOM (USB), fast poll   */
};

struct zmk_input_ams_as5600_data {
    const struct device           *dev;
    uint16_t                       last_angle;
    bool                           last_angle_initialized;
    bool                           sensor_powered;
    bool                           sensor_configured;
    uint8_t                        current_pm;       /* Cached AS5600 power mode  */
    enum zmk_input_ams_as5600_mode mode;
    int64_t                        last_movement_ms;
    int32_t                        scaled_remainder;
    struct k_work_delayable        work;
};

/* ─────────────────── Transport detection helper ──────────────────────── */

/**
 * Return true when the *selected* transport is USB HID.
 *
 * Uses selected endpoint transport so that plugging in USB for charging
 * while BLE is the active transport correctly returns false.
 *
 * Build configs:
 *   - USB only  → always true
 *   - BLE only  → always false
 *   - USB + BLE → runtime check of selected endpoint
 */
static inline bool is_usb_mode(void) {
#if IS_ENABLED(CONFIG_ZMK_USB) && IS_ENABLED(CONFIG_ZMK_BLE)
    return get_selected_endpoint().transport == ZMK_TRANSPORT_USB;
#elif IS_ENABLED(CONFIG_ZMK_USB)
    return true;
#else
    return false;
#endif
}

/* ──────── HID resolution multiplier (optional, compile-gated) ────────── */

#if IS_ENABLED(CONFIG_ZMK_INPUT_AMS_AS5600_SET_HID_RESOLUTION_MULTIPLIER)
static inline void apply_resolution_multiplier(void) {
    struct zmk_pointing_resolution_multipliers mults = {
        .wheel     = CONFIG_ZMK_INPUT_AMS_AS5600_HID_WHEEL_RESOLUTION,
        .hor_wheel = CONFIG_ZMK_INPUT_AMS_AS5600_HID_WHEEL_RESOLUTION,
    };

#if IS_ENABLED(CONFIG_ZMK_BLE)
    int profile = zmk_ble_active_profile_index();
    if (profile >= 0) {
        struct zmk_endpoint_instance ep = {
            .transport = ZMK_TRANSPORT_BLE,
            .ble       = {.profile_index = profile},
        };
        zmk_pointing_resolution_multipliers_set_profile(mults, ep);
    }
#endif
#if IS_ENABLED(CONFIG_ZMK_USB)
    {
        struct zmk_endpoint_instance ep = {.transport = ZMK_TRANSPORT_USB};
        zmk_pointing_resolution_multipliers_set_profile(mults, ep);
    }
#endif
}
#endif

/* ──────────────────── AGC logging (optional) ─────────────────────────── */

#if IS_ENABLED(CONFIG_ZMK_INPUT_AMS_AS5600_LOG_AGC)
static void log_agc(const struct device *dev) {
    const struct zmk_input_ams_as5600_config *cfg = dev->config;
    uint8_t agc;
    int err = i2c_burst_read_dt(&cfg->i2c_port, AS5600_REG_AGC, &agc, sizeof(agc));
    if (err) {
        LOG_ERR(LOG_PFX "I2C read AGC failed: %d", err);
        return;
    }
    LOG_INF(LOG_PFX "AGC: %d", agc);
}
#endif

/* ───────────── AS5600 configuration / power-mode helpers ─────────────── */

/**
 * Write the full CONF register with the specified AS5600 power-mode bits.
 * Caches current_pm so redundant I2C writes are skipped.
 */
static int as5600_write_conf(const struct device *dev, uint8_t pm) {
    const struct zmk_input_ams_as5600_config *cfg  = dev->config;
    struct zmk_input_ams_as5600_data         *data = dev->data;
    uint8_t buf[sizeof(ZMK_INPUT_AMS_AS5600_CONFIG)];

    if (data->sensor_configured && data->current_pm == pm) {
        return 0;
    }

    memcpy(buf, &ZMK_INPUT_AMS_AS5600_CONFIG, sizeof(buf));
    buf[sizeof(buf) - 1] = (buf[sizeof(buf) - 1] & (uint8_t)~AS5600_PM_MASK)
                         | (pm & AS5600_PM_MASK);

    int err = i2c_burst_write_dt(&cfg->i2c_port, AS5600_REG_CONF, buf, sizeof(buf));
    if (err) {
        LOG_ERR(LOG_PFX "CONF write failed: %d", err);
        return err;
    }

    data->sensor_configured = true;
    data->current_pm        = pm;
    return 0;
}

/* ────────────────────── Power (MOSFET) control ───────────────────────── */

static int set_power(const struct device *dev, bool on) {
    const struct zmk_input_ams_as5600_config *cfg  = dev->config;
    struct zmk_input_ams_as5600_data         *data = dev->data;

    if (cfg->power_gpio.port == NULL) {
        data->sensor_powered = true;
        return 0;
    }
    if (data->sensor_powered == on) {
        return 0;
    }

    int err = gpio_pin_set_dt(&cfg->power_gpio, on ? 1 : 0);
    if (err) {
        LOG_ERR(LOG_PFX "Power GPIO set failed: %d", err);
        return err;
    }

    data->sensor_powered = on;
    if (!on) {
        data->sensor_configured = false;
        data->scaled_remainder  = 0;
        /*
         * IMPORTANT: Do NOT reset last_angle_initialized here.
         * Keeping the stale last_angle lets the idle-wake check detect
         * that the dial moved while the sensor was powered off.
         * The movement-threshold filter already rejects noise-level
         * differences.
         */
    }
    return 0;
}

/* ──────────── Power-on + configure for a given power mode ────────────── */

static int power_on_and_configure(const struct device *dev, uint8_t pm) {
    struct zmk_input_ams_as5600_data *data = dev->data;
    int err;

    if (!data->sensor_powered) {
        err = set_power(dev, true);
        if (err) {
            return err;
        }
        k_sleep(K_MSEC(CONFIG_ZMK_INPUT_AMS_AS5600_SENSOR_STARTUP_DELAY_MS));
    }

    err = as5600_write_conf(dev, pm);
    if (err) {
        return err;
    }
    return 0;
}

/* ──────────────── Angle read + pulse calculation ─────────────────────── */

static int scale_pulses(struct zmk_input_ams_as5600_data *data, int32_t pulses,
                        int32_t *scaled) {
    int32_t acc = pulses * CONFIG_ZMK_INPUT_AMS_AS5600_SCROLL_SCALE_MULTIPLIER
                + data->scaled_remainder;

    *scaled = acc / CONFIG_ZMK_INPUT_AMS_AS5600_SCROLL_SCALE_DIVISOR;
    data->scaled_remainder = acc % CONFIG_ZMK_INPUT_AMS_AS5600_SCROLL_SCALE_DIVISOR;
    return 0;
}

static int process_angle(const struct device *dev, int32_t *reported_pulses,
                         bool *movement_detected) {
    const struct zmk_input_ams_as5600_config *cfg  = dev->config;
    struct zmk_input_ams_as5600_data         *data = dev->data;
    uint8_t buf[READ_BUF_SIZE];
    int err;

#if IS_ENABLED(CONFIG_ZMK_INPUT_AMS_AS5600_LOG_AGC)
    log_agc(dev);
#endif

    err = i2c_burst_read_dt(&cfg->i2c_port, READ_START_REG, buf, sizeof(buf));
    if (err) {
        LOG_ERR(LOG_PFX "I2C read failed: %d", err);
        return err;
    }

#if IS_ENABLED(CONFIG_ZMK_INPUT_AMS_AS5600_STATUS_REGISTER_MONITORING)
    {
        uint8_t st = buf[READ_OFF_STATUS];
        if (GET_BIT(st, AS5600_STATUS_AGC_OVERFLOW)) {
            LOG_ERR(LOG_PFX "AGC overflow – magnet too weak");
            return -EIO;
        }
        if (GET_BIT(st, AS5600_STATUS_AGC_UNDERFLOW)) {
            LOG_ERR(LOG_PFX "AGC underflow – magnet too strong");
            return -EIO;
        }
        if (!GET_BIT(st, AS5600_STATUS_MAGNET_DETECT)) {
            LOG_ERR(LOG_PFX "Magnet not detected");
            return -EIO;
        }
    }
#endif

    /* Mask to 12 bits – upper nibble of high byte is undefined */
    uint16_t angle = (((uint16_t)buf[READ_OFF_ANGLE_HI] << 8)
                    | buf[READ_OFF_ANGLE_LO]) & AS5600_ANGLE_MASK;

    /* First valid sample after boot: seed last_angle, report nothing */
    if (!data->last_angle_initialized) {
        data->last_angle             = angle;
        data->last_angle_initialized = true;
        *reported_pulses  = 0;
        *movement_detected = false;
        return 0;
    }

    int32_t pulses = (int32_t)angle - (int32_t)data->last_angle;
    if (pulses > (AS5600_PULSES_PER_REV / 2)) {
        pulses -= AS5600_PULSES_PER_REV;
    } else if (pulses < -(AS5600_PULSES_PER_REV / 2)) {
        pulses += AS5600_PULSES_PER_REV;
    }

    LOG_DBG(LOG_PFX "angle=%u last=%u pulses=%d", angle, data->last_angle, pulses);
    data->last_angle = angle;

    int32_t abs_pulses = (pulses < 0) ? -pulses : pulses;
    *movement_detected = (abs_pulses >= CONFIG_ZMK_INPUT_AMS_AS5600_MOVEMENT_THRESHOLD);

    if (!*movement_detected) {
        *reported_pulses = 0;
        return 0;
    }

    scale_pulses(data, pulses, reported_pulses);
    return 0;
}

/* ──────────────────────── Report to HID ──────────────────────────────── */

static int report_scroll(const struct device *dev, int32_t pulses) {
    if (pulses == 0) {
        return 0;
    }
#if IS_ENABLED(CONFIG_ZMK_INPUT_AMS_AS5600_SET_HID_RESOLUTION_MULTIPLIER)
    apply_resolution_multiplier();
#endif
    int err = input_report_rel(dev, INPUT_REL_WHEEL, pulses, true, K_FOREVER);
    if (err) {
        LOG_ERR(LOG_PFX "input_report_rel failed: %d", err);
    }
    return err;
}

/* ─────────────────────── Scheduling helper ───────────────────────────── */

static void schedule(struct zmk_input_ams_as5600_data *data, int ms) {
    k_work_reschedule(&data->work, K_MSEC(ms));
}

/* ══════════════════════ USB work handler ═════════════════════════════════
 *
 * USB: sensor always on, NOM power mode, 1 ms poll, never idle.
 * ════════════════════════════════════════════════════════════════════════ */

static void work_handler_usb(struct zmk_input_ams_as5600_data *data) {
    const struct device *dev = data->dev;
    int32_t pulses = 0;
    bool    moved  = false;

    int err = power_on_and_configure(dev, AS5600_PM_NOM);
    if (err) {
        schedule(data, USB_POLL_INTERVAL_MS);
        return;
    }

    err = process_angle(dev, &pulses, &moved);
    if (!err && moved) {
        (void)report_scroll(dev, pulses);
    }

    /* Always reschedule at 1 ms – no idle in USB mode */
    schedule(data, USB_POLL_INTERVAL_MS);
}

/* ══════════════════════ BLE work handler ═════════════════════════════════
 *
 * Three-tier power management:
 *   ACTIVE     – LPM1, poll at user-configured active interval
 *   LOW_POWER  – LPM2, poll every ~20 ms  (after 300 ms inactivity)
 *   IDLE       – sensor off, poll at idle interval (after 2 s total)
 *
 * Any detected movement promotes back to ACTIVE / LPM1.
 * ════════════════════════════════════════════════════════════════════════ */

static void ble_enter_active(struct zmk_input_ams_as5600_data *data) {
    data->mode             = ZMK_INPUT_AMS_AS5600_MODE_ACTIVE;
    data->last_movement_ms = k_uptime_get();
}

static void work_handler_ble(struct zmk_input_ams_as5600_data *data) {
    const struct device *dev = data->dev;
    int32_t pulses = 0;
    bool    moved  = false;
    int     err;
    int64_t now;
    int64_t idle_ms;

    switch (data->mode) {

    /* ── IDLE: power on briefly, check for movement, power off ────────── */
    case ZMK_INPUT_AMS_AS5600_MODE_IDLE:
        err = power_on_and_configure(dev, AS5600_PM_LPM1);
        if (err) {
            (void)set_power(dev, false);
            schedule(data, BLE_IDLE_POLL_MS);
            return;
        }

        err = process_angle(dev, &pulses, &moved);
        if (err) {
            (void)set_power(dev, false);
            schedule(data, BLE_IDLE_POLL_MS);
            return;
        }

        if (moved) {
            /* Wake up – stay powered on, switch to active */
            ble_enter_active(data);
            (void)report_scroll(dev, pulses);
            schedule(data, BLE_ACTIVE_POLL_MS);
        } else {
            /* No movement – power off, check again later */
            (void)set_power(dev, false);
            schedule(data, BLE_IDLE_POLL_MS);
        }
        return;

    /* ── LOW_POWER (LPM2): sensor on at reduced rate ─────────────────── */
    case ZMK_INPUT_AMS_AS5600_MODE_LOW_POWER:
        err = power_on_and_configure(dev, AS5600_PM_LPM2);
        if (err) {
            schedule(data, BLE_LP_POLL_MS);
            return;
        }

        err = process_angle(dev, &pulses, &moved);
        if (!err && moved) {
            /* Promote back to active */
            ble_enter_active(data);
            (void)as5600_write_conf(dev, AS5600_PM_LPM1);
            (void)report_scroll(dev, pulses);
            schedule(data, BLE_ACTIVE_POLL_MS);
            return;
        }

        now     = k_uptime_get();
        idle_ms = now - data->last_movement_ms;

        if (idle_ms >= BLE_ACTIVE_TO_IDLE_MS) {
            /* Transition to IDLE – power off sensor */
            data->mode = ZMK_INPUT_AMS_AS5600_MODE_IDLE;
            (void)set_power(dev, false);
            schedule(data, BLE_IDLE_POLL_MS);
        } else {
            schedule(data, BLE_LP_POLL_MS);
        }
        return;

    /* ── ACTIVE (LPM1): fast polling ─────────────────────────────────── */
    case ZMK_INPUT_AMS_AS5600_MODE_ACTIVE:
        err = power_on_and_configure(dev, AS5600_PM_LPM1);
        if (err) {
            schedule(data, BLE_ACTIVE_POLL_MS);
            return;
        }

        err = process_angle(dev, &pulses, &moved);
        if (!err && moved) {
            data->last_movement_ms = k_uptime_get();
            (void)report_scroll(dev, pulses);
        }

        now     = k_uptime_get();
        idle_ms = now - data->last_movement_ms;

        if (idle_ms >= BLE_ACTIVE_TO_IDLE_MS) {
            /* Straight to idle (edge case: very long single poll gap) */
            data->mode = ZMK_INPUT_AMS_AS5600_MODE_IDLE;
            (void)set_power(dev, false);
            schedule(data, BLE_IDLE_POLL_MS);
        } else if (idle_ms >= BLE_ACTIVE_TO_LP_MS) {
            /* Demote to low-power */
            data->mode = ZMK_INPUT_AMS_AS5600_MODE_LOW_POWER;
            (void)as5600_write_conf(dev, AS5600_PM_LPM2);
            schedule(data, BLE_LP_POLL_MS);
        } else {
            schedule(data, BLE_ACTIVE_POLL_MS);
        }
        return;
    }
}

/* ═══════════════════ Unified work handler dispatch ══════════════════════ */

static void work_handler(struct k_work *work) {
    struct zmk_input_ams_as5600_data *data =
        CONTAINER_OF(work, struct zmk_input_ams_as5600_data, work.work);

    if (is_usb_mode()) {
        work_handler_usb(data);
    } else {
        work_handler_ble(data);
    }
}

/* ═══════════════════════ Initialisation ═════════════════════════════════ */

static int zmk_input_ams_as5600_init(const struct device *dev) {
    const struct zmk_input_ams_as5600_config *cfg  = dev->config;
    struct zmk_input_ams_as5600_data         *data = dev->data;
    int err;

    data->dev                    = dev;
    data->last_angle             = 0;
    data->last_angle_initialized = false;
    data->scaled_remainder       = 0;
    data->last_movement_ms       = k_uptime_get();
    data->sensor_powered         = false;
    data->sensor_configured      = false;
    data->current_pm             = AS5600_PM_NOM;

    /* USB starts ACTIVE; BLE starts IDLE to save power on boot */
    data->mode = is_usb_mode() ? ZMK_INPUT_AMS_AS5600_MODE_ACTIVE
                               : ZMK_INPUT_AMS_AS5600_MODE_IDLE;

    if (cfg->power_gpio.port != NULL) {
        if (!device_is_ready(cfg->power_gpio.port)) {
            LOG_ERR(LOG_PFX "Power GPIO not ready");
            return -ENODEV;
        }
        err = gpio_pin_configure_dt(&cfg->power_gpio, GPIO_OUTPUT_INACTIVE);
        if (err) {
            LOG_ERR(LOG_PFX "Power GPIO configure failed: %d", err);
            return err;
        }
    } else {
        data->sensor_powered = true;
    }

    k_work_init_delayable(&data->work, work_handler);
    schedule(data, 0);

    LOG_INF(LOG_PFX "Initialised %s (mode=%s)", dev->name,
            is_usb_mode() ? "USB" : "BLE");
    return 0;
}

/* ═══════════════════ Device instantiation macro ═════════════════════════ */

#define ZMK_INPUT_AMS_AS5600_INST(n)                                           \
    static struct zmk_input_ams_as5600_data  as5600_data_##n;                  \
    static const struct zmk_input_ams_as5600_config as5600_cfg_##n = {         \
        .i2c_port   = I2C_DT_SPEC_INST_GET(n),                                \
        .power_gpio = GPIO_DT_SPEC_INST_GET_OR(n, power_gpios, {0}),          \
    };                                                                         \
                                                                               \
    DEVICE_DT_INST_DEFINE(n, zmk_input_ams_as5600_init, NULL,                  \
                          &as5600_data_##n, &as5600_cfg_##n,                   \
                          POST_KERNEL,                                         \
                          CONFIG_ZMK_INPUT_AMS_AS5600_INIT_PRIORITY,           \
                          NULL);

DT_INST_FOREACH_STATUS_OKAY(ZMK_INPUT_AMS_AS5600_INST)
