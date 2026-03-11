#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>

#define AS5600_POWER_NODE DT_NODELABEL(as5600_power)

static int power_init(void)
{
    const struct gpio_dt_spec mosfet = GPIO_DT_SPEC_GET(AS5600_POWER_NODE, gpios);

    if (!device_is_ready(mosfet.port)) {
        return 0;
    }

    /* Drive MOSFET gate active (LOW) to power AS5600 */
    gpio_pin_configure(mosfet.port, mosfet.pin, GPIO_OUTPUT_ACTIVE);

    return 0;
}

/* Run at application boot */
SYS_INIT(power_init, APPLICATION, 0);