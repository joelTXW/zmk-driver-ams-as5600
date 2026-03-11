#include <zephyr/kernel.h>
#include <hal/nrf_gpio.h>

static int power_init(void)
{
    /* Configure pins as outputs */
    nrf_gpio_cfg_output(2);   // P0.02 MOSFET
    nrf_gpio_cfg_output(30);  // P0.30 LED

    /* Drive LOW */
    nrf_gpio_pin_clear(2);
    nrf_gpio_pin_clear(30);

    return 0;
}

SYS_INIT(power_init, PRE_KERNEL_1, 0);