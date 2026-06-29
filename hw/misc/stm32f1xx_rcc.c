/*
 * STM32F1xx RCC - Implementation
 *
 * SPDX-License-Identifier: MIT
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/misc/stm32f1xx_rcc.h"

/* RCC_CR bits */
#define RCC_CR_HSION    (1u << 0)
#define RCC_CR_HSIRDY   (1u << 1)
#define RCC_CR_HSEON    (1u << 16)
#define RCC_CR_HSERDY   (1u << 17)
#define RCC_CR_PLLON    (1u << 24)
#define RCC_CR_PLLRDY   (1u << 25)

/* RCC_CFGR bits */
#define RCC_CFGR_SW_MASK    0x3
#define RCC_CFGR_SWS_MASK   0x3
#define RCC_CFGR_SWS_SHIFT  2

/* RCC_CSR bits */
#define RCC_CSR_LSION   (1u << 0)
#define RCC_CSR_LSIRDY  (1u << 1)
#define RCC_CSR_RMVF    (1u << 24)
#define RCC_CSR_PINRSTF (1u << 26)
#define RCC_CSR_PORRSTF (1u << 27)

/* ============================================================================
 * MMIO read handler
 * ============================================================================ */

static uint64_t stm32f1xx_rcc_read(void *opaque, hwaddr addr,
                                    unsigned int size)
{
    STM32F1XXRccState *s = opaque;

    if (addr >= RCC_NREGS * 4) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%"HWADDR_PRIx"\n",
                      __func__, addr);
        return 0;
    }

    uint32_t value = s->regs[addr >> 2];

    /* RCC_CR: set RDY flags based on ON bits */
    if (addr == RCC_CR) {
        if (value & RCC_CR_HSION) {
            value |= RCC_CR_HSIRDY;
        }
        if (value & RCC_CR_HSEON) {
            value |= RCC_CR_HSERDY;
        }
        if (value & RCC_CR_PLLON) {
            value |= RCC_CR_PLLRDY;
        }
        /* HSICAL: default calibration value */
        value |= (0x80 << 8);
    }

    /* RCC_CFGR: set SWS to match SW */
    if (addr == RCC_CFGR) {
        uint8_t sw = value & RCC_CFGR_SW_MASK;
        value = (value & ~(RCC_CFGR_SWS_MASK << RCC_CFGR_SWS_SHIFT))
                | ((uint32_t)sw << RCC_CFGR_SWS_SHIFT);
    }

    /* RCC_CSR: set LSIRDY based on LSION */
    if (addr == RCC_CSR) {
        if (value & RCC_CSR_LSION) {
            value |= RCC_CSR_LSIRDY;
        }
    }

    return value;
}

/* ============================================================================
 * MMIO write handler
 * ============================================================================ */

static void stm32f1xx_rcc_write(void *opaque, hwaddr addr,
                                 uint64_t val64, unsigned int size)
{
    STM32F1XXRccState *s = opaque;
    uint32_t value = (uint32_t)val64;

    if (addr >= RCC_NREGS * 4) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%"HWADDR_PRIx"\n",
                      __func__, addr);
        return;
    }

    switch (addr) {
    case RCC_CR: {
        uint32_t old_cr = s->regs[RCC_CR >> 2];

        /* Preserve read-only RDY bits from old value, set new ones */
        value &= ~(RCC_CR_HSIRDY | RCC_CR_HSERDY | RCC_CR_PLLRDY);
        value |= (old_cr & (RCC_CR_HSIRDY | RCC_CR_HSERDY | RCC_CR_PLLRDY));

        if (value & RCC_CR_HSION) {
            value |= RCC_CR_HSIRDY;
        }
        if (value & RCC_CR_HSEON) {
            value |= RCC_CR_HSERDY;
        }
        if (value & RCC_CR_PLLON) {
            value |= RCC_CR_PLLRDY;
        } else {
            value &= ~RCC_CR_PLLRDY;
        }

        s->regs[RCC_CR >> 2] = value;
        break;
    }

    case RCC_CFGR: {
        /* Clear read-only SWS bits, set SWS = SW immediately */
        value &= ~(RCC_CFGR_SWS_MASK << RCC_CFGR_SWS_SHIFT);
        uint8_t sw = value & RCC_CFGR_SW_MASK;
        value |= ((uint32_t)sw << RCC_CFGR_SWS_SHIFT);
        s->regs[RCC_CFGR >> 2] = value;
        break;
    }

    case RCC_CSR: {
        uint32_t csr = s->regs[RCC_CSR >> 2];

        /* LSION / LSIRDY */
        if (value & RCC_CSR_LSION) {
            csr |= RCC_CSR_LSION | RCC_CSR_LSIRDY;
        } else {
            csr &= ~(RCC_CSR_LSION | RCC_CSR_LSIRDY);
        }

        /* RMVF: clear all reset flags */
        if (value & RCC_CSR_RMVF) {
            csr &= ~(RCC_CSR_PINRSTF | RCC_CSR_PORRSTF | (1u << 28) |
                      (1u << 29) | (1u << 30) | (1u << 31));
        }

        s->regs[RCC_CSR >> 2] = csr;
        break;
    }

    default:
        /* All other registers: store raw value */
        s->regs[addr >> 2] = value;
        break;
    }
}

/* ============================================================================
 * Memory region ops
 * ============================================================================ */

static const MemoryRegionOps stm32f1xx_rcc_ops = {
    .read = stm32f1xx_rcc_read,
    .write = stm32f1xx_rcc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

/* ============================================================================
 * Device lifecycle
 * ============================================================================ */

static void stm32f1xx_rcc_init(Object *obj)
{
    STM32F1XXRccState *s = STM32F1XX_RCC(obj);

    memory_region_init_io(&s->mmio, obj, &stm32f1xx_rcc_ops, s,
                          TYPE_STM32F1XX_RCC, RCC_NREGS * 4);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
}

static void stm32f1xx_rcc_reset(DeviceState *dev)
{
    STM32F1XXRccState *s = STM32F1XX_RCC(dev);

    memset(s->regs, 0, sizeof(s->regs));

    /* RCC_CR: HSI ON and ready after reset, HSITRIM = 16 */
    s->regs[RCC_CR >> 2] = RCC_CR_HSION | RCC_CR_HSIRDY | (0x80 << 8);

    /* RCC_CSR: PINRSTF and PORRSTF set after POR */
    s->regs[RCC_CSR >> 2] = RCC_CSR_PINRSTF | RCC_CSR_PORRSTF;
}

static void stm32f1xx_rcc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, stm32f1xx_rcc_reset);
}

static const TypeInfo stm32f1xx_rcc_info = {
    .name          = TYPE_STM32F1XX_RCC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(STM32F1XXRccState),
    .instance_init = stm32f1xx_rcc_init,
    .class_init    = stm32f1xx_rcc_class_init,
};

static void stm32f1xx_rcc_register_types(void)
{
    type_register_static(&stm32f1xx_rcc_info);
}

type_init(stm32f1xx_rcc_register_types)
