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

#ifndef HW_ARM_STM32F103_SOC_H
#define HW_ARM_STM32F103_SOC_H

#include "hw/misc/stm32f1xx_rcc.h"
#include "hw/char/stm32f2xx_usart.h"
#include "hw/ssi/stm32f2xx_spi.h"
#include "hw/adc/stm32f1xx_adc.h"
#include "hw/core/or-irq.h"
#include "hw/arm/armv7m.h"
#include "qom/object.h"
#include "hw/core/clock.h"

#define TYPE_STM32F103_SOC "stm32f103-soc"
OBJECT_DECLARE_SIMPLE_TYPE(STM32F103State, STM32F103_SOC)

#define STM32F103_NUM_USARTS 5
#define STM32F103_NUM_SPIS 3
#define STM32F103_NUM_ADCS 3

/* High-density STM32F103: 512 KB Flash, 64 KB SRAM */
#define STM32F103_FLASH_BASE_ADDRESS 0x08000000
#define STM32F103_FLASH_SIZE (512 * 1024)
#define STM32F103_SRAM_BASE_ADDRESS 0x20000000
#define STM32F103_SRAM_SIZE (64 * 1024)

struct STM32F103State {
    SysBusDevice parent_obj;

    ARMv7MState armv7m;

    STM32F1XXRccState rcc;
    STM32F2XXUsartState usart[STM32F103_NUM_USARTS];
    STM32F2XXSPIState spi[STM32F103_NUM_SPIS];
    OrIRQState adc_irqs;
    STM32F1XXADCState adc[STM32F103_NUM_ADCS];

    MemoryRegion sram;
    MemoryRegion flash;
    MemoryRegion flash_alias;
    MemoryRegion flash_acr;

    Clock *sysclk;
    Clock *refclk;
};

#endif
