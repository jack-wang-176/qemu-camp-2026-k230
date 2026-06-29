/*
 * STM32F1XX ADC - Full implementation for STM32F103
 *
 * SPDX-License-Identifier: MIT
 */

#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/adc/stm32f1xx_adc.h"

#ifndef STM_F1_ADC_ERR_DEBUG
#define STM_F1_ADC_ERR_DEBUG 0
#endif

#define DB_PRINT(fmt, ...) do { \
    if (STM_F1_ADC_ERR_DEBUG >= 1) { \
        qemu_log("%s: " fmt, __func__, ##__VA_ARGS__); \
    } \
} while (0)

/* Timer interval for conversion simulation (nanoseconds) */
#define ADC_CONVERSION_NS  100

/* ============================================================================
 * Forward declarations
 * ============================================================================ */

static void stm32f1xx_adc_update_irq(STM32F1XXADCState *s);
static void stm32f1xx_adc_conversion_tick(void *opaque);

/* ============================================================================
 * Helper functions
 * ============================================================================ */

static uint32_t extract_bits(uint32_t value, int high, int low)
{
    uint32_t mask = (1U << (high - low + 1)) - 1;
    return (value >> low) & mask;
}

static uint8_t get_regular_seq_length(STM32F1XXADCState *s)
{
    return extract_bits(s->adc_sqr1, 23, 20) + 1;
}

static uint8_t get_injected_seq_length(STM32F1XXADCState *s)
{
    return extract_bits(s->adc_jsqr, 21, 20) + 1;
}

static uint8_t get_regular_channel(STM32F1XXADCState *s, int index)
{
    switch (index) {
    case 0: return extract_bits(s->adc_sqr3, 4, 0) & 0x1F;
    case 1: return extract_bits(s->adc_sqr3, 9, 5) & 0x1F;
    case 2: return extract_bits(s->adc_sqr3, 14, 10) & 0x1F;
    case 3: return extract_bits(s->adc_sqr3, 19, 15) & 0x1F;
    case 4: return extract_bits(s->adc_sqr3, 24, 20) & 0x1F;
    case 5: return extract_bits(s->adc_sqr3, 29, 25) & 0x1F;
    case 6: return extract_bits(s->adc_sqr2, 4, 0) & 0x1F;
    case 7: return extract_bits(s->adc_sqr2, 9, 5) & 0x1F;
    case 8: return extract_bits(s->adc_sqr2, 14, 10) & 0x1F;
    case 9: return extract_bits(s->adc_sqr2, 19, 15) & 0x1F;
    case 10: return extract_bits(s->adc_sqr2, 24, 20) & 0x1F;
    case 11: return extract_bits(s->adc_sqr2, 29, 25) & 0x1F;
    case 12: return extract_bits(s->adc_sqr1, 4, 0) & 0x1F;
    case 13: return extract_bits(s->adc_sqr1, 9, 5) & 0x1F;
    case 14: return extract_bits(s->adc_sqr1, 14, 10) & 0x1F;
    case 15: return extract_bits(s->adc_sqr1, 19, 15) & 0x1F;
    default: return 0;
    }
}

static uint8_t get_injected_channel(STM32F1XXADCState *s, int index)
{
    switch (index) {
    case 0: return extract_bits(s->adc_jsqr, 4, 0) & 0x1F;
    case 1: return extract_bits(s->adc_jsqr, 9, 5) & 0x1F;
    case 2: return extract_bits(s->adc_jsqr, 14, 10) & 0x1F;
    case 3: return extract_bits(s->adc_jsqr, 19, 15) & 0x1F;
    default: return 0;
    }
}

static uint32_t apply_alignment(STM32F1XXADCState *s, uint16_t raw)
{
    if (s->adc_cr2 & ADC_CR2_ALIGN) {
        return ((uint32_t)raw << 4) & 0xFFF0;
    } else {
        return raw;
    }
}

static bool adc_is_enabled(STM32F1XXADCState *s)
{
    return (s->adc_cr2 & ADC_CR2_ADON) != 0;
}

/* ============================================================================
 * IRQ management
 * ============================================================================ */

static void stm32f1xx_adc_update_irq(STM32F1XXADCState *s)
{
    bool pending = false;

    if ((s->adc_sr & ADC_SR_EOC) && (s->adc_cr1 & ADC_CR1_EOCIE)) {
        pending = true;
    }
    if ((s->adc_sr & ADC_SR_JEOC) && (s->adc_cr1 & ADC_CR1_JEOCIE)) {
        pending = true;
    }
    if ((s->adc_sr & ADC_SR_AWD) && (s->adc_cr1 & ADC_CR1_AWDIE)) {
        pending = true;
    }

    qemu_set_irq(s->irq, pending ? 1 : 0);
}

/* ============================================================================
 * Analog watchdog check
 * ============================================================================ */

static void stm32f1xx_adc_check_analog_watchdog(STM32F1XXADCState *s,
                                                  uint16_t value)
{
    bool awden = (s->adc_cr1 & ADC_CR1_AWDEN) != 0;
    bool jawden = (s->adc_cr1 & ADC_CR1_JAWDEN) != 0;

    if (!awden && !jawden) {
        return;
    }

    uint16_t ht = s->adc_htr & 0xFFF;
    uint16_t lt = s->adc_ltr & 0xFFF;

    if (value > ht || value < lt) {
        s->adc_sr |= ADC_SR_AWD;
        stm32f1xx_adc_update_irq(s);
    }
}

/* ============================================================================
 * CDR update for dual mode (F103: ADC2 data in DR[31:16] of ADC1)
 * ============================================================================ */

static void stm32f1xx_adc_update_dr_dual(STM32F1XXADCState *s, uint16_t data)
{
    /* In dual mode, ADC1.DR contains ADC2 data in upper 16 bits */
    if (s->adc_index == 0 && (s->adc_cr1 & ADC_CR1_DUALMOD_MASK) != 0) {
        s->adc2_data = data;
    }
}

/* ============================================================================
 * Conversion state machine
 * ============================================================================ */

static void stm32f1xx_adc_start_regular(STM32F1XXADCState *s)
{
    if (!adc_is_enabled(s)) {
        return;
    }
    if (s->conversion_active) {
        return;
    }

    DB_PRINT("Starting regular conversion\n");
    s->conversion_active = true;
    s->seq_index = 0;
    s->seq_length = get_regular_seq_length(s);
    s->adc_sr |= ADC_SR_STRT;
    timer_mod(&s->conversion_timer, ADC_CONVERSION_NS);
}

static void stm32f1xx_adc_start_injected(STM32F1XXADCState *s)
{
    if (!adc_is_enabled(s)) {
        return;
    }
    if (s->injected_active) {
        return;
    }

    DB_PRINT("Starting injected conversion\n");
    s->injected_active = true;
    s->inj_index = 0;
    s->inj_length = get_injected_seq_length(s);
    s->adc_sr |= ADC_SR_JSTRT;
    timer_mod(&s->conversion_timer, ADC_CONVERSION_NS);
}

static void stm32f1xx_adc_convert_regular(STM32F1XXADCState *s)
{
    if (s->seq_index >= s->seq_length) {
        /* Sequence complete */
        s->conversion_active = false;
        s->adc_sr |= ADC_SR_EOC;
        stm32f1xx_adc_update_irq(s);

        /* Continuous mode: restart */
        if (s->adc_cr2 & ADC_CR2_CONT) {
            s->seq_index = 0;
            timer_mod(&s->conversion_timer, ADC_CONVERSION_NS);
            return;
        }

        /* JAUTO: auto-start injected */
        if (s->adc_cr1 & ADC_CR1_JAUTO) {
            stm32f1xx_adc_start_injected(s);
            return;
        }
        return;
    }

    uint8_t channel = get_regular_channel(s, s->seq_index);
    DB_PRINT("Converting channel %d (index %d/%d)\n",
             channel, s->seq_index, s->seq_length);

    /* Simulate conversion */
    uint16_t raw = s->analog_input[channel & 0x1F];

    /* Store result */
    s->adc_dr = raw;

    /* Update dual mode data */
    stm32f1xx_adc_update_dr_dual(s, raw);

    /* Set EOC */
    s->adc_sr |= ADC_SR_EOC;
    stm32f1xx_adc_update_irq(s);

    /* Check analog watchdog */
    stm32f1xx_adc_check_analog_watchdog(s, raw);

    s->seq_index++;

    /* Schedule next conversion if more channels */
    if (s->seq_index < s->seq_length) {
        timer_mod(&s->conversion_timer, ADC_CONVERSION_NS);
    }
}

static void stm32f1xx_adc_convert_injected(STM32F1XXADCState *s)
{
    if (s->inj_index >= s->inj_length) {
        s->injected_active = false;
        s->adc_sr |= ADC_SR_JEOC;
        stm32f1xx_adc_update_irq(s);
        return;
    }

    uint8_t channel = get_injected_channel(s, s->inj_index);
    DB_PRINT("Converting injected channel %d\n", channel);

    uint16_t raw = s->analog_input[channel & 0x1F];

    /* Apply injected offset */
    uint16_t offset = s->adc_jofr[s->inj_index] & 0xFFF;
    if (raw >= offset) {
        raw -= offset;
    } else {
        raw = 0;
    }

    /* Store in JDR register */
    if (s->inj_index < 4) {
        s->adc_jdr[s->inj_index] = raw;
    }

    stm32f1xx_adc_check_analog_watchdog(s, raw);

    s->inj_index++;
    if (s->inj_index < s->inj_length) {
        timer_mod(&s->conversion_timer, ADC_CONVERSION_NS);
    }
}

static void stm32f1xx_adc_conversion_tick(void *opaque)
{
    STM32F1XXADCState *s = opaque;

    if (s->conversion_active) {
        stm32f1xx_adc_convert_regular(s);
    } else if (s->injected_active) {
        stm32f1xx_adc_convert_injected(s);
    }
}

/* ============================================================================
 * MMIO read handler
 * ============================================================================ */

static uint64_t stm32f1xx_adc_read(void *opaque, hwaddr addr,
                                    unsigned int size)
{
    STM32F1XXADCState *s = opaque;

    switch (addr) {
    case ADC_F1_SR:
        return s->adc_sr;
    case ADC_F1_CR1:
        return s->adc_cr1;
    case ADC_F1_CR2:
        return s->adc_cr2;
    case ADC_F1_SMPR1:
        return s->adc_smpr1;
    case ADC_F1_SMPR2:
        return s->adc_smpr2;
    case ADC_F1_JOFR1:
    case ADC_F1_JOFR2:
    case ADC_F1_JOFR3:
    case ADC_F1_JOFR4:
        return s->adc_jofr[(addr - ADC_F1_JOFR1) / 4];
    case ADC_F1_HTR:
        return s->adc_htr;
    case ADC_F1_LTR:
        return s->adc_ltr;
    case ADC_F1_SQR1:
        return s->adc_sqr1;
    case ADC_F1_SQR2:
        return s->adc_sqr2;
    case ADC_F1_SQR3:
        return s->adc_sqr3;
    case ADC_F1_JSQR:
        return s->adc_jsqr;
    case ADC_F1_JDR1:
    case ADC_F1_JDR2:
    case ADC_F1_JDR3:
    case ADC_F1_JDR4: {
        int idx = (addr - ADC_F1_JDR1) / 4;
        s->adc_sr &= ~ADC_SR_JEOC;
        stm32f1xx_adc_update_irq(s);
        return s->adc_jdr[idx];
    }
    case ADC_F1_DR:
        /* In dual mode, DR[31:16] contains ADC2 data */
        s->adc_sr &= ~ADC_SR_EOC;
        stm32f1xx_adc_update_irq(s);
        if ((s->adc_cr1 & ADC_CR1_DUALMOD_MASK) != 0 && s->adc_index == 0) {
            return ((uint32_t)s->adc2_data << 16) |
                   (apply_alignment(s, s->adc_dr) & 0xFFFF);
        }
        return apply_alignment(s, s->adc_dr);
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad offset 0x%" HWADDR_PRIx "\n", __func__, addr);
        break;
    }
    return 0;
}

/* ============================================================================
 * MMIO write handler
 * ============================================================================ */

static void stm32f1xx_adc_write(void *opaque, hwaddr addr,
                                 uint64_t val64, unsigned int size)
{
    STM32F1XXADCState *s = opaque;
    uint32_t value = (uint32_t)val64;

    switch (addr) {
    case ADC_F1_SR:
        /* rc_w0: writing 0 clears the flag */
        s->adc_sr &= (value | ~ADC_SR_MASK);
        stm32f1xx_adc_update_irq(s);
        break;

    case ADC_F1_CR1:
        s->adc_cr1 = value;
        stm32f1xx_adc_update_irq(s);
        break;

    case ADC_F1_CR2: {
        uint32_t old_cr2 = s->adc_cr2;
        s->adc_cr2 = value;

        /* ADON transitions */
        bool old_adon = (old_cr2 & ADC_CR2_ADON) != 0;
        bool new_adon = (value & ADC_CR2_ADON) != 0;

        if (!old_adon && new_adon) {
            DB_PRINT("ADC power on\n");
        } else if (old_adon && !new_adon) {
            DB_PRINT("ADC power down\n");
            s->conversion_active = false;
            s->injected_active = false;
            timer_del(&s->conversion_timer);
            s->adc_sr = 0;
            stm32f1xx_adc_update_irq(s);
        }

        /* RSTCAL: reset calibration */
        if (value & ADC_CR2_RSTCAL) {
            DB_PRINT("RSTCAL: reset calibration\n");
            s->calibration_done = false;
            s->rstcal_pending = true;
            /* Hardware clears RSTCAL after calibration registers reset */
            s->adc_cr2 &= ~ADC_CR2_RSTCAL;
        }

        /* CAL: start calibration */
        if (value & ADC_CR2_CAL) {
            DB_PRINT("CAL: start calibration\n");
            s->cal_pending = true;
            /* Hardware clears CAL after calibration completes */
            s->adc_cr2 &= ~ADC_CR2_CAL;
            s->calibration_done = true;
        }

        /* SWSTART: start regular conversion */
        if (value & ADC_CR2_SWSTART) {
            if (adc_is_enabled(s) && !s->conversion_active) {
                DB_PRINT("SWSTART triggered\n");
                stm32f1xx_adc_start_regular(s);
            }
            s->adc_cr2 &= ~ADC_CR2_SWSTART;
        }

        /* JSWSTART: start injected conversion */
        if (value & ADC_CR2_JSWSTART) {
            if (adc_is_enabled(s) && !s->injected_active) {
                DB_PRINT("JSWSTART triggered\n");
                stm32f1xx_adc_start_injected(s);
            }
            s->adc_cr2 &= ~ADC_CR2_JSWSTART;
        }

        stm32f1xx_adc_update_irq(s);
        break;
    }

    case ADC_F1_SMPR1:
        s->adc_smpr1 = value;
        for (int i = 0; i < 8; i++) {
            s->sampling_time[10 + i] = extract_bits(value, i * 3 + 2, i * 3);
        }
        break;

    case ADC_F1_SMPR2:
        s->adc_smpr2 = value;
        for (int i = 0; i < 10; i++) {
            s->sampling_time[i] = extract_bits(value, i * 3 + 2, i * 3);
        }
        break;

    case ADC_F1_JOFR1:
    case ADC_F1_JOFR2:
    case ADC_F1_JOFR3:
    case ADC_F1_JOFR4:
        s->adc_jofr[(addr - ADC_F1_JOFR1) / 4] = value & 0xFFF;
        break;

    case ADC_F1_HTR:
        s->adc_htr = value;
        break;

    case ADC_F1_LTR:
        s->adc_ltr = value;
        break;

    case ADC_F1_SQR1:
        s->adc_sqr1 = value;
        s->seq_length = get_regular_seq_length(s);
        break;

    case ADC_F1_SQR2:
        s->adc_sqr2 = value;
        break;

    case ADC_F1_SQR3:
        s->adc_sqr3 = value;
        break;

    case ADC_F1_JSQR:
        s->adc_jsqr = value;
        s->inj_length = get_injected_seq_length(s);
        break;

    case ADC_F1_JDR1:
    case ADC_F1_JDR2:
    case ADC_F1_JDR3:
    case ADC_F1_JDR4:
        /* Read-only */
        break;

    case ADC_F1_DR:
        /* Read-only */
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad offset 0x%" HWADDR_PRIx "\n", __func__, addr);
        break;
    }
}

/* ============================================================================
 * Memory region ops
 * ============================================================================ */

static const MemoryRegionOps stm32f1xx_adc_ops = {
    .read = stm32f1xx_adc_read,
    .write = stm32f1xx_adc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

/* ============================================================================
 * VM state
 * ============================================================================ */

static const VMStateDescription vmstate_stm32f1xx_adc = {
    .name = TYPE_STM32F1XX_ADC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(adc_sr, STM32F1XXADCState),
        VMSTATE_UINT32(adc_cr1, STM32F1XXADCState),
        VMSTATE_UINT32(adc_cr2, STM32F1XXADCState),
        VMSTATE_UINT32(adc_smpr1, STM32F1XXADCState),
        VMSTATE_UINT32(adc_smpr2, STM32F1XXADCState),
        VMSTATE_UINT32_ARRAY(adc_jofr, STM32F1XXADCState, 4),
        VMSTATE_UINT32(adc_htr, STM32F1XXADCState),
        VMSTATE_UINT32(adc_ltr, STM32F1XXADCState),
        VMSTATE_UINT32(adc_sqr1, STM32F1XXADCState),
        VMSTATE_UINT32(adc_sqr2, STM32F1XXADCState),
        VMSTATE_UINT32(adc_sqr3, STM32F1XXADCState),
        VMSTATE_UINT32(adc_jsqr, STM32F1XXADCState),
        VMSTATE_UINT32_ARRAY(adc_jdr, STM32F1XXADCState, 4),
        VMSTATE_UINT32(adc_dr, STM32F1XXADCState),
        VMSTATE_BOOL(conversion_active, STM32F1XXADCState),
        VMSTATE_BOOL(injected_active, STM32F1XXADCState),
        VMSTATE_UINT8(seq_index, STM32F1XXADCState),
        VMSTATE_UINT8(inj_index, STM32F1XXADCState),
        VMSTATE_UINT8(seq_length, STM32F1XXADCState),
        VMSTATE_UINT8(inj_length, STM32F1XXADCState),
        VMSTATE_UINT8_ARRAY(sampling_time, STM32F1XXADCState, 18),
        VMSTATE_UINT16_ARRAY(analog_input, STM32F1XXADCState, 18),
        VMSTATE_UINT16(adc2_data, STM32F1XXADCState),
        VMSTATE_UINT8(adc_index, STM32F1XXADCState),
        VMSTATE_END_OF_LIST()
    }
};

/* ============================================================================
 * Device reset
 * ============================================================================ */

static void stm32f1xx_adc_reset(DeviceState *dev)
{
    STM32F1XXADCState *s = STM32F1XX_ADC(dev);

    s->adc_sr = 0x00000000;
    s->adc_cr1 = 0x00000000;
    s->adc_cr2 = 0x00000000;
    s->adc_smpr1 = 0x00000000;
    s->adc_smpr2 = 0x00000000;
    s->adc_jofr[0] = 0x00000000;
    s->adc_jofr[1] = 0x00000000;
    s->adc_jofr[2] = 0x00000000;
    s->adc_jofr[3] = 0x00000000;
    s->adc_htr = 0x00000FFF;
    s->adc_ltr = 0x00000000;
    s->adc_sqr1 = 0x00000000;
    s->adc_sqr2 = 0x00000000;
    s->adc_sqr3 = 0x00000000;
    s->adc_jsqr = 0x00000000;
    s->adc_jdr[0] = 0x00000000;
    s->adc_jdr[1] = 0x00000000;
    s->adc_jdr[2] = 0x00000000;
    s->adc_jdr[3] = 0x00000000;
    s->adc_dr = 0x00000000;

    s->conversion_active = false;
    s->injected_active = false;
    s->seq_index = 0;
    s->inj_index = 0;
    s->seq_length = 0;
    s->inj_length = 0;
    s->calibration_done = false;
    s->rstcal_pending = false;
    s->cal_pending = false;
    s->adc2_data = 0;

    timer_del(&s->conversion_timer);
    qemu_set_irq(s->irq, 0);
}

/* ============================================================================
 * Device initialization
 * ============================================================================ */

static void stm32f1xx_adc_init(Object *obj)
{
    STM32F1XXADCState *s = STM32F1XX_ADC(obj);

    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);

    memory_region_init_io(&s->mmio, obj, &stm32f1xx_adc_ops, s,
                          TYPE_STM32F1XX_ADC, ADC_F1_REG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
}

static void stm32f1xx_adc_realize(DeviceState *dev, Error **errp)
{
    STM32F1XXADCState *s = STM32F1XX_ADC(dev);

    timer_init_ns(&s->conversion_timer, QEMU_CLOCK_VIRTUAL,
                  stm32f1xx_adc_conversion_tick, s);

    /* Initialize analog inputs to mid-range */
    for (int i = 0; i < 18; i++) {
        s->analog_input[i] = 0x800;
    }
}

/* ============================================================================
 * Device class
 * ============================================================================ */

static void stm32f1xx_adc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, stm32f1xx_adc_reset);
    dc->vmsd = &vmstate_stm32f1xx_adc;
    dc->realize = stm32f1xx_adc_realize;
}

static const TypeInfo stm32f1xx_adc_info = {
    .name          = TYPE_STM32F1XX_ADC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(STM32F1XXADCState),
    .instance_init = stm32f1xx_adc_init,
    .class_init    = stm32f1xx_adc_class_init,
};

static void stm32f1xx_adc_register_types(void)
{
    type_register_static(&stm32f1xx_adc_info);
}

type_init(stm32f1xx_adc_register_types)
