// Native Rust unit tests for the STM32F1xx GPIO register logic.
//
// Copyright 2026, Jack Wang
// Author(s): Jack Wang <163wangjack@gmail.com>
// SPDX-License-Identifier: GPL-2.0-or-later
//
// Unlike tests/qtest/stm32f103_gpio-test.c (which drives the device over a
// socket from an external qemu-system-arm process), these tests link the
// device crate directly and exercise `GpioRegisters` in-process. This gives
// fast, native Rust coverage of the CRL/CRH decode + IDR algorithm without
// booting a machine or pulling in the softmmu memory subsystem.
//
// `GpioRegisters` deliberately has no QOM object, no MemoryRegion and no
// InterruptSource, so it can be constructed with `GpioRegisters::new()` and
// called directly. Only the BQL mock (bql::start_test) is required, because
// the register cells are `BqlCell`s.
//
// SPDX-License-Identifier: GPL-2.0-or-later

use stm32f1xx_gpio::gpio::{
    GpioRegisters, PinUpdate, REG_BRR, REG_BSRR, REG_CRH, REG_CRL, REG_IDR, REG_ODR, RESET_CRH,
    RESET_CRL,
};

/// Take the mock BQL so `BqlCell` accesses are permitted. Integration tests
/// run with `--test-threads=1`, so this is safe to call from every test.
fn regs() -> GpioRegisters {
    bql::start_test();
    GpioRegisters::new()
}

// 4-bit MODE[1:0]+CNF[1:0] pin configurations (RM0008).
const CFG_INPUT_FLOATING: u32 = 0x4; // MODE=00 CNF=01 (reset default)
const CFG_INPUT_PULL: u32 = 0x8; // MODE=00 CNF=10, ODR selects up/down
const CFG_OUTPUT_PP: u32 = 0x1; // MODE=01 CNF=00, general push-pull

/// Set the 4-bit config nibble for `pin` in CRL/CRH via an MMIO write.
fn set_config(r: &GpioRegisters, pin: usize, cfg: u32) {
    let reg = if pin < 8 { REG_CRL } else { REG_CRH };
    let shift = ((pin % 8) * 4) as u32;
    let old = r.read(reg);
    r.write(reg, (old & !(0xF << shift)) | (cfg << shift));
}

fn set_odr_bit(r: &GpioRegisters, pin: usize, value: u32) {
    let old = r.read(REG_ODR);
    r.write(REG_ODR, (old & !(1 << pin)) | (value << pin));
}

/// True if pin `pin` is set in a PinUpdate and its reported level is `level`.
fn pin_changed_to(upd: PinUpdate, pin: usize, level: bool) -> bool {
    (upd.mask & (1 << pin)) != 0 && (((upd.levels >> pin) & 1) != 0) == level
}

#[test]
/// A freshly created register block holds the RM0008 reset values.
fn test_reset_values() {
    let r = regs();
    assert_eq!(r.read(REG_CRL), RESET_CRL);
    assert_eq!(r.read(REG_CRH), RESET_CRH);
    assert_eq!(r.read(REG_IDR), 0);
    assert_eq!(r.read(REG_ODR), 0);
}

#[test]
/// `reset` restores the reset state after the registers are dirtied.
fn test_reset_restores() {
    let r = regs();
    r.write(REG_CRL, 0xDEAD_BEEF);
    r.write(REG_ODR, 0x0000_FFFF);
    assert_ne!(r.read(REG_CRL), RESET_CRL);

    r.reset();

    assert_eq!(r.read(REG_CRL), RESET_CRL);
    assert_eq!(r.read(REG_CRH), RESET_CRH);
    assert_eq!(r.read(REG_IDR), 0);
    assert_eq!(r.read(REG_ODR), 0);
}

#[test]
/// Push-pull output: IDR follows ODR; ODR written while input is latched,
/// and the out-pin change is reported.
fn test_output_mode() {
    let r = regs();
    let pin = 5;

    // Set ODR while still a floating input: IDR unaffected, no out change.
    let upd = set_odr_and_report(&r, pin, 1);
    assert_eq!(r.read(REG_IDR), 0);
    assert_eq!(upd.mask & (1 << pin), 0);

    // Switch to push-pull output: pin now follows ODR (high), and flips.
    let old = r.read(REG_CRL);
    let upd = r.write(
        REG_CRL,
        (old & !(0xF << (pin * 4))) | (CFG_OUTPUT_PP << (pin * 4)),
    );
    assert_eq!(r.read(REG_IDR), 1 << pin);
    assert!(pin_changed_to(upd, pin, true));

    // Clear ODR bit: pin goes low, and flips back.
    let upd = set_odr_and_report(&r, pin, 0);
    assert_eq!(r.read(REG_IDR), 0);
    assert!(pin_changed_to(upd, pin, false));
}

/// Helper: write one ODR bit and return the resulting PinUpdate.
fn set_odr_and_report(r: &GpioRegisters, pin: usize, value: u32) -> PinUpdate {
    let old = r.read(REG_ODR);
    r.write(REG_ODR, (old & !(1 << pin)) | (value << pin))
}

#[test]
/// Floating input: an external drive is reflected in IDR and reported.
fn test_input_mode() {
    let r = regs();
    let pin = 6usize;

    set_config(&r, pin, CFG_INPUT_FLOATING);

    let upd = r.gpio_set(pin, 1);
    assert_eq!(r.read(REG_IDR), 1 << pin);
    assert!(pin_changed_to(upd, pin, true));

    let upd = r.gpio_set(pin, 0);
    assert_eq!(r.read(REG_IDR), 0);
    assert!(pin_changed_to(upd, pin, false));
}

#[test]
/// Input pull mode: ODR selects pull-up (IDR=1) vs pull-down (IDR=0),
/// with no external driver connected.
fn test_pull_up_down() {
    let r = regs();
    let pin = 0;

    set_odr_bit(&r, pin, 1);
    set_config(&r, pin, CFG_INPUT_PULL);
    assert_eq!(r.read(REG_IDR), 1 << pin);

    set_odr_bit(&r, pin, 0);
    assert_eq!(r.read(REG_IDR), 0);
}

#[test]
/// BSRR sets/resets ODR bits (set has priority); BRR resets ODR bits.
fn test_bsrr_brr() {
    let r = regs();
    let pin = 1;

    r.write(REG_BSRR, 1 << pin);
    assert_eq!(r.read(REG_ODR), 1 << pin);

    r.write(REG_BSRR, 1 << (pin + 16));
    assert_eq!(r.read(REG_ODR), 0);

    r.write(REG_BSRR, 1 << pin);
    r.write(REG_BRR, 1 << pin);
    assert_eq!(r.read(REG_ODR), 0);

    // Set has priority over reset within one BSRR write.
    r.write(REG_BSRR, (1 << pin) | (1 << (pin + 16)));
    assert_eq!(r.read(REG_ODR), 1 << pin);
}

#[test]
/// A pin driven externally, then reconfigured as push-pull output, is
/// disconnected: IDR follows ODR, not the stale external level.
fn test_push_pull_disconnect() {
    let r = regs();
    let pin = 7usize;

    // Drive high externally as input.
    set_config(&r, pin, CFG_INPUT_FLOATING);
    r.gpio_set(pin, 1);
    assert_eq!(r.read(REG_IDR), 1 << pin);
    assert_eq!(r.disconnected_pins() & (1 << pin), 0); // now connected

    // Reconfigure as push-pull output with ODR=0: pin must drop to 0,
    // and the external driver must be disconnected.
    set_odr_bit(&r, pin, 0);
    set_config(&r, pin, CFG_OUTPUT_PP);
    assert_eq!(r.read(REG_IDR), 0);
    assert_ne!(r.disconnected_pins() & (1 << pin), 0); // disconnected again

    // Further external drives are ignored while output push-pull.
    r.gpio_set(pin, 1);
    assert_eq!(r.read(REG_IDR), 0);
}
