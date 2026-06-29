/*
 * STM32F103 SoC
 *
 * Copyright (c) 2014 Alistair Francis <alistair@alistair23.me>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "system/address-spaces.h"
#include "system/system.h"
#include "hw/arm/stm32f103_soc.h"
#include "hw/core/qdev-clock.h"
#include "hw/misc/unimp.h"

#define RCC_ADDR 0x40021000

/* USART addresses: USART1, USART2, USART3, UART4, UART5 */
static const uint32_t usart_addr[] = {
    0x40013800, 0x40004400, 0x40004800, 0x40004C00, 0x40005000
};

/* SPI addresses: SPI1, SPI2, SPI3 */
static const uint32_t spi_addr[] = {
    0x40013000, 0x40003800, 0x40003C00
};

/* ADC addresses: ADC1, ADC2, ADC3 */
static const uint32_t adc_addr[] = {
    0x40012400, 0x40012800, 0x40013C00
};

static const int usart_irq[] = { 37, 38, 39, 52, 53 };
static const int spi_irq[]   = { 35, 36, 51 };
#define ADC_IRQ 18


static void stm32f103_soc_initfn(Object *obj)
{
    STM32F103State *s = STM32F103_SOC(obj);
    int i;

    object_initialize_child(obj, "armv7m", &s->armv7m, TYPE_ARMV7M);

    object_initialize_child(obj, "rcc", &s->rcc, TYPE_STM32F1XX_RCC);

    for (i = 0; i < STM32F103_NUM_USARTS; i++) {
        object_initialize_child(obj, "usart[*]", &s->usart[i],
                                TYPE_STM32F2XX_USART);
    }

    for (i = 0; i < STM32F103_NUM_SPIS; i++) {
        object_initialize_child(obj, "spi[*]", &s->spi[i],
                                TYPE_STM32F2XX_SPI);
    }

    for (i = 0; i < STM32F103_NUM_ADCS; i++) {
        object_initialize_child(obj, "adc[*]", &s->adc[i],
                                TYPE_STM32F1XX_ADC);
    }

    s->sysclk = qdev_init_clock_in(DEVICE(s), "sysclk", NULL, NULL, 0);
    s->refclk = qdev_init_clock_in(DEVICE(s), "refclk", NULL, NULL, 0);
}

static void stm32f103_soc_realize(DeviceState *dev_soc, Error **errp)
{
    STM32F103State *s = STM32F103_SOC(dev_soc);
    MemoryRegion *system_memory = get_system_memory();
    DeviceState *dev, *armv7m;
    SysBusDevice *busdev;
    Error *err = NULL;
    int i;

    if (clock_has_source(s->refclk)) {
        error_setg(errp, "refclk clock must not be wired up by the board code");
        return;
    }

    if (!clock_has_source(s->sysclk)) {
        error_setg(errp, "sysclk clock must be wired up by the board code");
        return;
    }

    /* The refclk always runs at frequency HCLK / 8 */
    clock_set_mul_div(s->refclk, 8, 1);
    clock_set_source(s->refclk, s->sysclk);

    /* Flash: 512 KB (high-density STM32F103) */
    memory_region_init_rom(&s->flash, OBJECT(dev_soc), "STM32F103.flash",
                           STM32F103_FLASH_SIZE, &err);
    if (err != NULL) {
        error_propagate(errp, err);
        return;
    }
    memory_region_init_alias(&s->flash_alias, OBJECT(dev_soc),
                             "STM32F103.flash.alias", &s->flash, 0,
                             STM32F103_FLASH_SIZE);

    memory_region_add_subregion(system_memory, STM32F103_FLASH_BASE_ADDRESS,
                                &s->flash);
    memory_region_add_subregion(system_memory, 0, &s->flash_alias);

    /* SRAM: 64 KB */
    memory_region_init_ram(&s->sram, NULL, "STM32F103.sram",
                           STM32F103_SRAM_SIZE, &err);
    if (err != NULL) {
        error_propagate(errp, err);
        return;
    }
    memory_region_add_subregion(system_memory, STM32F103_SRAM_BASE_ADDRESS,
                                &s->sram);

    /* Init ARMv7m (Cortex-M3) */
    armv7m = DEVICE(&s->armv7m);
    qdev_prop_set_uint32(armv7m, "num-irq", 67);
    qdev_prop_set_uint8(armv7m, "num-prio-bits", 4);
    qdev_prop_set_string(armv7m, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m3"));
    qdev_prop_set_bit(armv7m, "enable-bitband", true);
    qdev_connect_clock_in(armv7m, "cpuclk", s->sysclk);
    qdev_connect_clock_in(armv7m, "refclk", s->refclk);
    object_property_set_link(OBJECT(&s->armv7m), "memory",
                             OBJECT(system_memory), &error_abort);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->armv7m), errp)) {
        return;
    }

    /* Reset and clock controller */
    dev = DEVICE(&s->rcc);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->rcc), errp)) {
        return;
    }
    busdev = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(busdev, 0, RCC_ADDR);

    /* USART controllers */
    for (i = 0; i < STM32F103_NUM_USARTS; i++) {
        dev = DEVICE(&(s->usart[i]));
        qdev_prop_set_chr(dev, "chardev", serial_hd(i));
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->usart[i]), errp)) {
            return;
        }
        busdev = SYS_BUS_DEVICE(dev);
        sysbus_mmio_map(busdev, 0, usart_addr[i]);
        sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(armv7m, usart_irq[i]));
    }

    /* SPI controllers */
    for (i = 0; i < STM32F103_NUM_SPIS; i++) {
        dev = DEVICE(&(s->spi[i]));
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->spi[i]), errp)) {
            return;
        }
        busdev = SYS_BUS_DEVICE(dev);
        sysbus_mmio_map(busdev, 0, spi_addr[i]);
        sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(armv7m, spi_irq[i]));
    }

    /* ADC device, the IRQs are ORed together */
    if (!object_initialize_child_with_props(OBJECT(s), "adc-orirq",
                                            &s->adc_irqs, sizeof(s->adc_irqs),
                                            TYPE_OR_IRQ, errp, NULL)) {
        return;
    }
    object_property_set_int(OBJECT(&s->adc_irqs), "num-lines",
                            STM32F103_NUM_ADCS, &error_abort);
    if (!qdev_realize(DEVICE(&s->adc_irqs), NULL, errp)) {
        return;
    }
    qdev_connect_gpio_out(DEVICE(&s->adc_irqs), 0,
                          qdev_get_gpio_in(armv7m, ADC_IRQ));

    for (i = 0; i < STM32F103_NUM_ADCS; i++) {
        dev = DEVICE(&(s->adc[i]));
        s->adc[i].adc_index = i;
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->adc[i]), errp)) {
            return;
        }
        busdev = SYS_BUS_DEVICE(dev);
        sysbus_mmio_map(busdev, 0, adc_addr[i]);
        sysbus_connect_irq(busdev, 0,
                           qdev_get_gpio_in(DEVICE(&s->adc_irqs), i));
    }

    /*
     * Unimplemented peripherals -- STM32F103 devices that do not yet
     * have QEMU models.
     */

    /* APB1 peripherals */
    create_unimplemented_device("timer[2]",    0x40000000, 0x400);
    create_unimplemented_device("timer[3]",    0x40000400, 0x400);
    create_unimplemented_device("timer[4]",    0x40000800, 0x400);
    create_unimplemented_device("timer[5]",    0x40000C00, 0x400);
    create_unimplemented_device("timer[6]",    0x40001000, 0x400);
    create_unimplemented_device("timer[7]",    0x40001400, 0x400);
    create_unimplemented_device("timer[12]",   0x40001800, 0x400);
    create_unimplemented_device("timer[13]",   0x40001C00, 0x400);
    create_unimplemented_device("timer[14]",   0x40002000, 0x400);
    create_unimplemented_device("RTC",         0x40002800, 0x400);
    create_unimplemented_device("WWDG",        0x40002C00, 0x400);
    create_unimplemented_device("IWDG",        0x40003000, 0x400);
    create_unimplemented_device("SPI2/I2S",    0x40003800, 0x400);
    create_unimplemented_device("I2C1",        0x40005400, 0x400);
    create_unimplemented_device("I2C2",        0x40005800, 0x400);
    create_unimplemented_device("USB device",  0x40005C00, 0x400);
    create_unimplemented_device("CAN shared",  0x40006000, 0x400);
    create_unimplemented_device("bxCAN1",      0x40006400, 0x400);
    create_unimplemented_device("bxCAN2",      0x40006800, 0x400);
    create_unimplemented_device("BKP",         0x40006C00, 0x400);
    create_unimplemented_device("PWR",         0x40007000, 0x400);
    create_unimplemented_device("DAC",         0x40007400, 0x400);

    /* APB2 peripherals */
    create_unimplemented_device("AFIO",        0x40010000, 0x400);
    create_unimplemented_device("EXTI",        0x40010400, 0x400);
    create_unimplemented_device("GPIOA",       0x40010800, 0x400);
    create_unimplemented_device("GPIOB",       0x40010C00, 0x400);
    create_unimplemented_device("GPIOC",       0x40011000, 0x400);
    create_unimplemented_device("GPIOD",       0x40011400, 0x400);
    create_unimplemented_device("GPIOE",       0x40011800, 0x400);
    create_unimplemented_device("GPIOF",       0x40011C00, 0x400);
    create_unimplemented_device("GPIOG",       0x40012000, 0x400);
    create_unimplemented_device("timer[1]",    0x40012C00, 0x400);
    create_unimplemented_device("timer[8]",    0x40013400, 0x400);
    create_unimplemented_device("timer[9]",    0x40014C00, 0x400);
    create_unimplemented_device("timer[10]",   0x40015000, 0x400);
    create_unimplemented_device("timer[11]",   0x40015400, 0x400);
    create_unimplemented_device("SDIO",        0x40018000, 0x400);

    /* AHB peripherals */
    create_unimplemented_device("DMA1",        0x40020000, 0x400);
    create_unimplemented_device("DMA2",        0x40020400, 0x400);
    /* Flash ACR (0x40022000): minimal stub so HAL_RCC_ClockConfig can set LATENCY */
    memory_region_init_ram(&s->flash_acr, NULL, "stm32f103.flash_acr", 0x400, &err);
    memory_region_add_subregion(system_memory, 0x40022000, &s->flash_acr);
    create_unimplemented_device("CRC",         0x40023000, 0x400);
    create_unimplemented_device("Ethernet",    0x40028000, 0x2000);

    /* FSMC (high-density STM32F103) */
    create_unimplemented_device("FSMC",        0xA0000000, 0x1000);

    /* USB OTG FS */
    create_unimplemented_device("USB OTG FS",  0x50000000, 0x40000);
}

static void stm32f103_soc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = stm32f103_soc_realize;
    /* No vmstate or reset required: device has no internal state */
}

static const TypeInfo stm32f103_soc_info = {
    .name          = TYPE_STM32F103_SOC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(STM32F103State),
    .instance_init = stm32f103_soc_initfn,
    .class_init    = stm32f103_soc_class_init,
};

static void stm32f103_soc_types(void)
{
    type_register_static(&stm32f103_soc_info);
}

type_init(stm32f103_soc_types)
