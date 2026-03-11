#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

static const struct gpio_dt_spec mosfet =
    GPIO_DT_SPEC_GET(DT_NODELABEL(gpio0), gpios);

static const struct gpio_dt_spec led = {
    .port = DEVICE_DT_GET(DT_NODELABEL(gpio0)),
    .pin = 30,
    .dt_flags = GPIO_ACTIVE_LOW,
};

static int power_init(void)
{
    if (!device_is_ready(led.port)) {
        return 0;
    }

    gpio_pin_configure(led.port, 30, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure(led.port, 2, GPIO_OUTPUT_ACTIVE);

    return 0;
}

SYS_INIT(power_init, APPLICATION, 0);