/*
 * STM32F1XX ADC - Full implementation for STM32F103
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef HW_STM32F1XX_ADC_H
#define HW_STM32F1XX_ADC_H

#include "hw/core/sysbus.h"
#include "qom/object.h"
#include "qemu/timer.h"

/* ============================================================================
 * Register offset definitions
 * ============================================================================ */

#define ADC_F1_SR     0x00
#define ADC_F1_CR1    0x04
#define ADC_F1_CR2    0x08
#define ADC_F1_SMPR1  0x0C
#define ADC_F1_SMPR2  0x10
#define ADC_F1_JOFR1  0x14
#define ADC_F1_JOFR2  0x18
#define ADC_F1_JOFR3  0x1C
#define ADC_F1_JOFR4  0x20
#define ADC_F1_HTR    0x24
#define ADC_F1_LTR    0x28
#define ADC_F1_SQR1   0x2C
#define ADC_F1_SQR2   0x30
#define ADC_F1_SQR3   0x34
#define ADC_F1_JSQR   0x38
#define ADC_F1_JDR1   0x3C
#define ADC_F1_JDR2   0x40
#define ADC_F1_JDR3   0x44
#define ADC_F1_JDR4   0x48
#define ADC_F1_DR     0x4C

/* Register size */
#define ADC_F1_REG_SIZE 0x100

/* ============================================================================
 * Bit field definitions
 * ============================================================================ */

/* ADC_SR bits */
#define ADC_SR_AWD    (1 << 0)
#define ADC_SR_EOC    (1 << 1)
#define ADC_SR_JEOC   (1 << 2)
#define ADC_SR_JSTRT  (1 << 3)
#define ADC_SR_STRT   (1 << 4)
#define ADC_SR_MASK   0x1F

/* ADC_CR1 bits */
#define ADC_CR1_AWDCH_MASK   0x1F
#define ADC_CR1_EOCIE        (1 << 5)
#define ADC_CR1_AWDIE        (1 << 6)
#define ADC_CR1_JEOCIE       (1 << 7)
#define ADC_CR1_SCAN         (1 << 8)
#define ADC_CR1_AWDSGL       (1 << 9)
#define ADC_CR1_JAUTO        (1 << 10)
#define ADC_CR1_DISCEN       (1 << 11)
#define ADC_CR1_JDISCEN      (1 << 12)
#define ADC_CR1_DISCNUM_MASK 0x7
#define ADC_CR1_DISCNUM_SHIFT 13
#define ADC_CR1_DUALMOD_MASK 0xF
#define ADC_CR1_DUALMOD_SHIFT 16
#define ADC_CR1_JAWDEN       (1 << 22)
#define ADC_CR1_AWDEN        (1 << 23)

/* ADC_CR2 bits */
#define ADC_CR2_ADON         (1 << 0)
#define ADC_CR2_CONT         (1 << 1)
#define ADC_CR2_CAL          (1 << 2)
#define ADC_CR2_RSTCAL       (1 << 3)
#define ADC_CR2_DMA          (1 << 8)
#define ADC_CR2_ALIGN        (1 << 11)
#define ADC_CR2_JEXTSEL_MASK 0x7
#define ADC_CR2_JEXTSEL_SHIFT 12
#define ADC_CR2_JEXTTRIG     (1 << 15)
#define ADC_CR2_EXTSEL_MASK  0x7
#define ADC_CR2_EXTSEL_SHIFT 17
#define ADC_CR2_EXTTRIG      (1 << 20)
#define ADC_CR2_JSWSTART     (1 << 21)
#define ADC_CR2_SWSTART      (1 << 22)
#define ADC_CR2_TSVREFE      (1 << 23)

/* Dual mode selection (CR1.DUALMOD) */
#define ADC_DUALMOD_INDEPENDENT           0x00
#define ADC_DUALMOD_REG_INJ_SIMULT        0x01
#define ADC_DUALMOD_REG_SIMULT_ALT        0x02
#define ADC_DUALMOD_INJ_SIMULT_FAST_ILV   0x03
#define ADC_DUALMOD_INJ_SIMULT_SLOW_ILV   0x04
#define ADC_DUALMOD_INJ_SIMULT_ONLY       0x05
#define ADC_DUALMOD_REG_SIMULT_ONLY       0x06
#define ADC_DUALMOD_FAST_INTERLEAVED      0x07
#define ADC_DUALMOD_SLOW_INTERLEAVED      0x08
#define ADC_DUALMOD_ALT_TRIGGER           0x09

/* ============================================================================
 * Type definitions
 * ============================================================================ */

#define TYPE_STM32F1XX_ADC "stm32f1xx-adc"
OBJECT_DECLARE_SIMPLE_TYPE(STM32F1XXADCState, STM32F1XX_ADC)

struct STM32F1XXADCState {
    /* <private> */
    SysBusDevice parent_obj;

    /* <public> */
    MemoryRegion mmio;

    /* Register storage */
    uint32_t adc_sr;
    uint32_t adc_cr1;
    uint32_t adc_cr2;
    uint32_t adc_smpr1;
    uint32_t adc_smpr2;
    uint32_t adc_jofr[4];
    uint32_t adc_htr;
    uint32_t adc_ltr;
    uint32_t adc_sqr1;
    uint32_t adc_sqr2;
    uint32_t adc_sqr3;
    uint32_t adc_jsqr;
    uint32_t adc_jdr[4];
    uint32_t adc_dr;

    /* Internal state */
    bool conversion_active;
    bool injected_active;
    uint8_t seq_index;
    uint8_t inj_index;
    uint8_t seq_length;
    uint8_t inj_length;

    /* Sampling time per channel (0-17) */
    uint8_t sampling_time[18];

    /* Analog input model (fuzzable) */
    uint16_t analog_input[18];

    /* F103 calibration state */
    bool calibration_done;
    bool rstcal_pending;
    bool cal_pending;

    /* F103 dual mode: ADC2 data in DR[31:16] of ADC1 */
    uint16_t adc2_data;

    /* ADC index (0=ADC1, 1=ADC2, 2=ADC3) */
    uint8_t adc_index;

    /* Conversion timer */
    QEMUTimer conversion_timer;

    /* IRQ output */
    qemu_irq irq;
};

#endif /* HW_STM32F1XX_ADC_H */
