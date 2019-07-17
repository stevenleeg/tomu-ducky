#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/vector.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/efm32/wdog.h>
#include <libopencm3/efm32/gpio.h>
#include <libopencm3/efm32/cmu.h>

#include <stdbool.h>
#include <stdio.h>

#include <toboot.h>

TOBOOT_CONFIGURATION(0);

#define SYSTICK_FREQUENCY 1000
#define USB_CLK_FREQUENCY 24000000

#define LED_GREEN_PORT GPIOA
#define LED_GREEN_PIN  GPIO0
#define LED_RED_PORT   GPIOB
#define LED_RED_PIN    GPIO7

uint32_t millis = 0;
bool prev_value = false;

void sys_tick_handler(void) {
    // Start with the green LED turned on, so the two toggle
    if (millis == 0) {
        gpio_toggle(LED_GREEN_PORT, LED_GREEN_PIN);
    }

    millis++;
    
    if (millis % 1000 == 0 && prev_value) {
        gpio_toggle(LED_RED_PORT, LED_RED_PIN);
        prev_value = !prev_value;
    } else if (millis % 1000 == 0 && !prev_value) {
        gpio_toggle(LED_GREEN_PORT, LED_GREEN_PIN);
        prev_value = !prev_value;
    }
}

int main(int argc, char **argv) {
    (void) argc;
    (void) argv;

    // Disable watchdog, letting the Tomu know that the program is running and
    // it doesn't need to reboot
    WDOG_CTRL = 0;

    cmu_periph_clock_enable(CMU_GPIO);

    // Setup our ports
    gpio_mode_setup(LED_RED_PORT, GPIO_MODE_WIRED_AND, LED_RED_PIN);
    gpio_mode_setup(LED_GREEN_PORT, GPIO_MODE_WIRED_AND, LED_GREEN_PIN);

    // I have no idea what's going on here
    cmu_osc_on(USHFRCO);
    cmu_wait_for_osc_ready(USHFRCO);
    CMU_USBCRCTRL = CMU_USBCRCTRL_EN;
    CMU_CMD = CMU_CMD_HFCLKSEL(5);
    while (!(CMU_STATUS & CMU_STATUS_USHFRCODIV2SEL));

    systick_set_frequency(SYSTICK_FREQUENCY, USB_CLK_FREQUENCY);
    systick_counter_enable();
    systick_interrupt_enable();

    while (1);
}
