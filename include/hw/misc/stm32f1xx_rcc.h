/*
 * STM32F1xx RCC
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef HW_MISC_STM32F1XX_RCC_H
#define HW_MISC_STM32F1XX_RCC_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_STM32F1XX_RCC "stm32f1xx-rcc"
OBJECT_DECLARE_SIMPLE_TYPE(STM32F1XXRccState, STM32F1XX_RCC)

/* Register offsets */
#define RCC_CR      0x00
#define RCC_CFGR    0x04
#define RCC_CIR     0x08
#define RCC_APB2RSTR 0x0C
#define RCC_APB1RSTR 0x10
#define RCC_AHBENR  0x14
#define RCC_APB2ENR 0x18
#define RCC_APB1ENR 0x1C
#define RCC_BDCR    0x20
#define RCC_CSR     0x24

#define RCC_NREGS  10

struct STM32F1XXRccState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;

    /* Register storage */
    uint32_t regs[RCC_NREGS];
};

#endif /* HW_MISC_STM32F1XX_RCC_H */
