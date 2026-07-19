/*
 * QTest testcase for STM32F103 GPIO (Rust device "stm32f1xx-gpio-rust")
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Copyright (c) 2026 QEMU contributors
 *
 * The STM32F1 GPIO differs from the L4 family: each pin is configured by a
 * 4-bit field MODE[1:0]+CNF[1:0] packed into CRL (pins 0-7) and CRH (pins
 * 8-15), instead of the separate MODER/OTYPER/PUPDR registers of the L4.
 * See ST RM0008 for the register map and reset values.
 */

#include "qemu/osdep.h"
#include "libqtest-single.h"

#define GPIO_BASE_ADDR 0x40010800
#define GPIO_SIZE      0x400
#define NUM_GPIOS      7
#define NUM_GPIO_PINS  16

#define GPIO_A 0x40010800
#define GPIO_B 0x40010C00
#define GPIO_C 0x40011000
#define GPIO_D 0x40011400
#define GPIO_E 0x40011800
#define GPIO_F 0x40011C00
#define GPIO_G 0x40012000

/* Register offsets (RM0008). */
#define CRL  0x00
#define CRH  0x04
#define IDR  0x08
#define ODR  0x0C
#define BSRR 0x10
#define BRR  0x14
#define LCKR 0x18

/* 4-bit MODE[1:0]+CNF[1:0] pin configurations. */
#define CFG_INPUT_FLOATING   0x4  /* MODE=00 CNF=01 (reset default) */
#define CFG_INPUT_PULL       0x8  /* MODE=00 CNF=10, ODR selects up/down */
#define CFG_OUTPUT_PP        0x1  /* MODE=01 CNF=00, general push-pull */
#define CFG_OUTPUT_OD        0x5  /* MODE=01 CNF=01, general open-drain */

/* CRL/CRH reset value: every pin floating input. */
#define CR_RESET 0x44444444

static uint32_t gpio_readl(unsigned int gpio, unsigned int offset)
{
    return readl(gpio + offset);
}

static void gpio_writel(unsigned int gpio, unsigned int offset, uint32_t value)
{
    writel(gpio + offset, value);
}

/* Set the 4-bit config nibble for `pin` in CRL/CRH. */
static void gpio_set_config(unsigned int gpio, unsigned int pin, uint8_t cfg)
{
    unsigned int reg = (pin < 8) ? CRL : CRH;
    unsigned int shift = (pin % 8) * 4;
    uint32_t mask = ~(0xFu << shift);
    uint32_t val = (gpio_readl(gpio, reg) & mask) | ((uint32_t)cfg << shift);
    gpio_writel(gpio, reg, val);
}

static void gpio_set_odr_bit(unsigned int gpio, unsigned int pin,
                             uint32_t value)
{
    uint32_t mask = ~(0x1u << pin);
    gpio_writel(gpio, ODR, (gpio_readl(gpio, ODR) & mask) | (value << pin));
}

static unsigned int get_gpio_id(uint32_t gpio_addr)
{
    return (gpio_addr - GPIO_BASE_ADDR) / GPIO_SIZE;
}

static const char *gpio_path(uint32_t gpio)
{
    static char buf[32];
    snprintf(buf, sizeof(buf), "/machine/soc/gpio%c", 'a' + get_gpio_id(gpio));
    return buf;
}

/* Drive pin `num` of `gpio` to `level` from an external source. */
static void gpio_set_irq(unsigned int gpio, int num, int level)
{
    qtest_set_irq_in(global_qtest, gpio_path(gpio), NULL, num, level);
}

/*
 * Intercept this port's own gpio-out lines so get_irq(pin) observes them.
 *
 * qtest only supports intercepting one device's out-GPIOs per session (the
 * interception device is global), so every test that observes gpio-out via
 * get_irq() must target the SAME port. We use GPIO_A for all of them. Tests
 * that only read IDR/ODR over MMIO can use any port and skip interception.
 */
static void intercept_out(unsigned int gpio)
{
    qtest_irq_intercept_out(global_qtest, gpio_path(gpio));
}

static void reset_gpio(unsigned int gpio)
{
    gpio_writel(gpio, CRL, CR_RESET);
    gpio_writel(gpio, CRH, CR_RESET);
    gpio_writel(gpio, ODR, 0);
}

static void test_reset_values(void)
{
    /*
     * Scribble over the config/data registers, issue a system reset, and
     * check the RM0008 documented reset state is restored: CRL/CRH back to
     * 0x44444444 (floating input) and IDR/ODR back to 0.
     */
    gpio_writel(GPIO_A, CRL, 0xDEADBEEF);
    gpio_writel(GPIO_A, CRH, 0xDEADBEEF);
    gpio_writel(GPIO_A, ODR, 0xDEADBEEF);

    gpio_writel(GPIO_C, CRL, 0x12345678);
    gpio_writel(GPIO_C, ODR, 0x0000FFFF);

    qtest_system_reset(global_qtest);

    g_assert_cmphex(gpio_readl(GPIO_A, CRL), ==, CR_RESET);
    g_assert_cmphex(gpio_readl(GPIO_A, CRH), ==, CR_RESET);
    g_assert_cmphex(gpio_readl(GPIO_A, IDR), ==, 0);
    g_assert_cmphex(gpio_readl(GPIO_A, ODR), ==, 0);

    g_assert_cmphex(gpio_readl(GPIO_C, CRL), ==, CR_RESET);
    g_assert_cmphex(gpio_readl(GPIO_C, IDR), ==, 0);
    g_assert_cmphex(gpio_readl(GPIO_C, ODR), ==, 0);
}

static void test_output_mode(const void *data)
{
    /*
     * Configure a pin as push-pull output and check ODR drives IDR and the
     * gpio-out line. Also check ODR writes made while the pin is still an
     * input are latched (not discarded), taking effect once switched.
     */
    unsigned int pin = (uintptr_t)data & 0xF;
    uint32_t gpio = (uintptr_t)data & ~(GPIO_SIZE - 1);
    bool observe = (gpio == GPIO_A);

    reset_gpio(gpio);
    if (observe) {
        intercept_out(gpio);
    }

    /* Set ODR while floating input: IDR unaffected, no output edge. */
    gpio_set_odr_bit(gpio, pin, 1);
    g_assert_cmphex(gpio_readl(gpio, IDR), ==, 0);
    if (observe) {
        g_assert_false(get_irq(pin));
    }

    /* Switch to push-pull output: pin now follows ODR (high). */
    gpio_set_config(gpio, pin, CFG_OUTPUT_PP);
    g_assert_cmphex(gpio_readl(gpio, IDR), ==, (1u << pin));
    if (observe) {
        g_assert_true(get_irq(pin));
    }

    /* Clear ODR bit: pin goes low. */
    gpio_set_odr_bit(gpio, pin, 0);
    g_assert_cmphex(gpio_readl(gpio, IDR), ==, 0);
    if (observe) {
        g_assert_false(get_irq(pin));
    }

    reset_gpio(gpio);
}

static void test_input_mode(const void *data)
{
    /*
     * Configure a pin as floating input and check that an external drive
     * (via the gpio-in line) is reflected in IDR and on the gpio-out line.
     */
    unsigned int pin = (uintptr_t)data & 0xF;
    uint32_t gpio = (uintptr_t)data & ~(GPIO_SIZE - 1);
    bool observe = (gpio == GPIO_A);

    reset_gpio(gpio);
    if (observe) {
        intercept_out(gpio);
    }

    gpio_set_config(gpio, pin, CFG_INPUT_FLOATING);

    gpio_set_irq(gpio, pin, 1);
    g_assert_cmphex(gpio_readl(gpio, IDR), ==, (1u << pin));
    if (observe) {
        g_assert_true(get_irq(pin));
    }

    gpio_set_irq(gpio, pin, 0);
    g_assert_cmphex(gpio_readl(gpio, IDR), ==, 0);
    if (observe) {
        g_assert_false(get_irq(pin));
    }

    reset_gpio(gpio);
}

static void test_pull_up_down(const void *data)
{
    /*
     * In input pull mode (CNF=10), ODR selects the resistor: ODR=1 pulls the
     * pin up (IDR=1) and ODR=0 pulls it down (IDR=0), even with no external
     * driver connected.
     */
    unsigned int pin = (uintptr_t)data & 0xF;
    uint32_t gpio = (uintptr_t)data & ~(GPIO_SIZE - 1);

    reset_gpio(gpio);
    intercept_out(gpio);

    /* Pull-up. */
    gpio_set_odr_bit(gpio, pin, 1);
    gpio_set_config(gpio, pin, CFG_INPUT_PULL);
    g_assert_cmphex(gpio_readl(gpio, IDR), ==, (1u << pin));
    g_assert_true(get_irq(pin));

    /* Pull-down. */
    gpio_set_odr_bit(gpio, pin, 0);
    g_assert_cmphex(gpio_readl(gpio, IDR), ==, 0);
    g_assert_false(get_irq(pin));

    reset_gpio(gpio);
}

static void test_bsrr_brr(const void *data)
{
    /*
     * BSRR: low 16 bits set ODR bits, high 16 bits reset them, with set
     * taking priority. BRR resets ODR bits.
     */
    unsigned int pin = (uintptr_t)data & 0xF;
    uint32_t gpio = (uintptr_t)data & ~(GPIO_SIZE - 1);

    reset_gpio(gpio);

    gpio_writel(gpio, BSRR, (1u << pin));
    g_assert_cmphex(gpio_readl(gpio, ODR), ==, (1u << pin));

    gpio_writel(gpio, BSRR, (1u << (pin + 16)));
    g_assert_cmphex(gpio_readl(gpio, ODR), ==, 0);

    gpio_writel(gpio, BSRR, (1u << pin));
    g_assert_cmphex(gpio_readl(gpio, ODR), ==, (1u << pin));

    gpio_writel(gpio, BRR, (1u << pin));
    g_assert_cmphex(gpio_readl(gpio, ODR), ==, 0);

    /* Set has priority over reset in BSRR. */
    gpio_writel(gpio, BSRR, (1u << pin) | (1u << (pin + 16)));
    g_assert_cmphex(gpio_readl(gpio, ODR), ==, (1u << pin));

    reset_gpio(gpio);
}

static void test_push_pull_disconnect(const void *data)
{
    /*
     * A pin driven externally, then reconfigured as push-pull output, must be
     * disconnected from the external driver: its IDR follows ODR, not the
     * stale external level.
     */
    unsigned int pin = (uintptr_t)data & 0xF;
    uint32_t gpio = (uintptr_t)data & ~(GPIO_SIZE - 1);

    reset_gpio(gpio);

    /* Drive high externally as input. */
    gpio_set_config(gpio, pin, CFG_INPUT_FLOATING);
    gpio_set_irq(gpio, pin, 1);
    g_assert_cmphex(gpio_readl(gpio, IDR), ==, (1u << pin));

    /* Reconfigure as push-pull output with ODR=0: pin must drop to 0. */
    gpio_set_odr_bit(gpio, pin, 0);
    gpio_set_config(gpio, pin, CFG_OUTPUT_PP);
    g_assert_cmphex(gpio_readl(gpio, IDR), ==, 0);

    /* Further external drives are ignored while output push-pull. */
    gpio_set_irq(gpio, pin, 1);
    g_assert_cmphex(gpio_readl(gpio, IDR), ==, 0);

    reset_gpio(gpio);
}

int main(int argc, char **argv)
{
    int ret;

    g_test_init(&argc, &argv, NULL);
    g_test_set_nonfatal_assertions();

    qtest_add_func("stm32f103/gpio/reset_values", test_reset_values);
    qtest_add_data_func("stm32f103/gpio/output_mode_a5",
                        (void *)(uintptr_t)(GPIO_A | 5), test_output_mode);
    qtest_add_data_func("stm32f103/gpio/output_mode_c13",
                        (void *)(uintptr_t)(GPIO_C | 13), test_output_mode);
    qtest_add_data_func("stm32f103/gpio/input_mode_b6",
                        (void *)(uintptr_t)(GPIO_B | 6), test_input_mode);
    qtest_add_data_func("stm32f103/gpio/input_mode_d10",
                        (void *)(uintptr_t)(GPIO_D | 10), test_input_mode);
    qtest_add_data_func("stm32f103/gpio/pull_up_down_a0",
                        (void *)(uintptr_t)(GPIO_A | 0), test_pull_up_down);
    qtest_add_data_func("stm32f103/gpio/bsrr_brr_a1",
                        (void *)(uintptr_t)(GPIO_A | 1), test_bsrr_brr);
    qtest_add_data_func("stm32f103/gpio/bsrr_brr_e12",
                        (void *)(uintptr_t)(GPIO_E | 12), test_bsrr_brr);
    qtest_add_data_func("stm32f103/gpio/push_pull_disconnect_g7",
                        (void *)(uintptr_t)(GPIO_G | 7),
                        test_push_pull_disconnect);

    qtest_start("-machine stm32f103");
    ret = g_test_run();
    qtest_end();

    return ret;
}
