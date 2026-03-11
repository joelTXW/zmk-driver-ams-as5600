#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

static int power_init(void)
{
    const struct device *gpio = device_get_binding("GPIO_0");

    if (!gpio) {
        return 0;
    }

    /* Configure pins as outputs */
    gpio_pin_configure(gpio, 2, GPIO_OUTPUT);
    gpio_pin_configure(gpio, 30, GPIO_OUTPUT);

    /* Drive them LOW */
    gpio_pin_set(gpio, 2, 0);
    gpio_pin_set(gpio, 30, 0);

    return 0;
}

SYS_INIT(power_init, PRE_KERNEL_1, 0);