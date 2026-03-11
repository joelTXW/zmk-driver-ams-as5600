#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

static int power_init(void)
{
    const struct device *gpio = device_get_binding("GPIO_0");

    if (!gpio) {
        return 0;
    }

    /* P0.02 = MOSFET gate */
    gpio_pin_configure(gpio, 2, GPIO_OUTPUT_ACTIVE);

    return 0;
}

SYS_INIT(power_init, APPLICATION, 0);