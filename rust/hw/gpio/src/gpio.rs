// STM32F1xx GPIO port device model (Rust)
// Copyright 2026, Jack Wang
// Author(s): Jack Wang <163wangjack@gmail.com>
// SPDX-License-Identifier: GPL-2.0-or-later
//
// Reference: ST RM0008, GPIO and AFIO registers.
// The register model deliberately follows the C stm32l4x5_gpio.c IDR
// algorithm, adapted to the F1 CRL/CRH (MODE[1:0]+CNF[1:0]) layout.
//
// The register state and all of its logic live in `GpioRegisters`, a plain
// struct with no QOM/MMIO/IRQ coupling. `Stm32f1xxGpioState` is a thin QOM
// device shell that owns a `GpioRegisters` plus the GPIO-out lines, forwards
// MMIO/pin callbacks into it, and propagates the resulting pin-level changes
// onto the out lines. Keeping the logic free of C dependencies lets the unit
// tests in tests/tests.rs exercise it natively, without booting a machine.

use std::ffi::CStr;

use bql::prelude::*;
use common::prelude::*;
use hwcore::prelude::*;
use migration::prelude::*;
use qom::prelude::*;
use system::prelude::*;
use util::prelude::*;

pub const GPIO_NUM_PINS: usize = 16;

pub const TYPE_STM32F1XX_GPIO_RUST: &CStr = c"stm32f1xx-gpio-rust";
qom_isa!(Stm32f1xxGpioState: SysBusDevice, DeviceState, Object);

unsafe impl ObjectType for Stm32f1xxGpioState {
    type Class = <SysBusDevice as ObjectType>::Class;
    const TYPE_NAME: &'static CStr = TYPE_STM32F1XX_GPIO_RUST;
}

// Trace bindings are generated from hw/gpio/trace-events (group "hw_gpio").
// Each `stm32f1xx_gpio_<name>` event yields a
// `trace::trace_stm32f1xx_gpio_<name>` function whose argument types match the
// trace-events declaration verbatim.
::trace::include_trace!("hw_gpio");

/// Register offsets (RM0008), shared by the device and its tests.
pub const REG_CRL: u64 = 0x00;
pub const REG_CRH: u64 = 0x04;
pub const REG_IDR: u64 = 0x08;
pub const REG_ODR: u64 = 0x0C;
pub const REG_BSRR: u64 = 0x10;
pub const REG_BRR: u64 = 0x14;
pub const REG_LCKR: u64 = 0x18;

/// RM0008 reset values: CRL/CRH = 0x44444444 (all pins floating input).
pub const RESET_CRL: u32 = 0x4444_4444;
pub const RESET_CRH: u32 = 0x4444_4444;

/// Pure register state + behaviour of one GPIO port.
///
/// This holds no QOM object, no `MemoryRegion` and no `InterruptSource`, so it
/// can be constructed and driven directly from unit tests. Methods that can
/// change pin levels return a [`PinUpdate`] describing which output lines
/// flipped; the QOM shell turns those into `InterruptSource` edges.
#[repr(C)]
#[derive(Debug)]
pub struct GpioRegisters {
    crl: BqlCell<u32>,
    crh: BqlCell<u32>,
    idr: BqlCell<u32>,
    odr: BqlCell<u32>,
    lckr: BqlCell<u32>,

    // Bit i set  => pin i is not driven by an external source.
    // Bit i clear => pin i is driven, and `pins_connected_high` bit i
    //                gives the level driven.
    disconnected_pins: BqlCell<u16>,
    pins_connected_high: BqlCell<u16>,
}

impl Default for GpioRegisters {
    fn default() -> Self {
        Self::new()
    }
}

/// Which output pins changed level as a result of an operation.
///
/// `mask` bit i set means pin i has a determined level in `levels` bit i and
/// that level differs from before the operation.
#[derive(Debug, Default, Clone, Copy)]
pub struct PinUpdate {
    pub mask: u32,
    pub levels: u32,
}

impl GpioRegisters {
    pub const fn new() -> Self {
        Self {
            crl: BqlCell::new(RESET_CRL),
            crh: BqlCell::new(RESET_CRH),
            idr: BqlCell::new(0),
            odr: BqlCell::new(0),
            lckr: BqlCell::new(0),
            disconnected_pins: BqlCell::new(0xFFFF),
            pins_connected_high: BqlCell::new(0),
        }
    }

    // ---- register accessors (for tests / vmstate / shell) ----

    pub fn idr(&self) -> u32 {
        self.idr.get()
    }
    pub fn odr(&self) -> u32 {
        self.odr.get()
    }
    pub fn crl(&self) -> u32 {
        self.crl.get()
    }
    pub fn crh(&self) -> u32 {
        self.crh.get()
    }
    pub fn disconnected_pins(&self) -> u16 {
        self.disconnected_pins.get()
    }

    // ---- pin configuration helpers (decode CRL/CRH nibbles) ----

    fn extract(value: u32, start: u32, length: u32) -> u32 {
        assert!(length > 0 && length <= 32 - start);
        (value >> start) & (!0u32 >> (32 - length))
    }

    // 4-bit MODE[1:0]+CNF[1:0] field for `pin`.
    fn get_modecnf(&self, pin: usize) -> u8 {
        let reg = if pin < 8 {
            self.crl.get()
        } else {
            self.crh.get()
        };
        let shift = ((pin % 8) * 4) as u32;
        Self::extract(reg, shift, 4) as u8
    }

    // MODE != 0 means output (any speed).
    fn is_output(&self, pin: usize) -> bool {
        (self.get_modecnf(pin) & 0x03) != 0
    }

    fn is_alternate_open_drain(&self, pin: usize) -> bool {
        let modecnf = self.get_modecnf(pin);
        ((modecnf & 0x03) != 0) && ((modecnf >> 2) == 0x03)
    }

    fn is_alternate_push_pull(&self, pin: usize) -> bool {
        let modecnf = self.get_modecnf(pin);
        ((modecnf & 0x03) != 0) && ((modecnf >> 2) == 0x02)
    }

    fn is_general_push_pull(&self, pin: usize) -> bool {
        let modecnf = self.get_modecnf(pin);
        ((modecnf & 0x03) != 0) && ((modecnf >> 2) == 0x00)
    }

    fn is_general_open_drain(&self, pin: usize) -> bool {
        let modecnf = self.get_modecnf(pin);
        ((modecnf & 0x03) != 0) && ((modecnf >> 2) == 0x01)
    }

    // Input with pull-up/pull-down selected: MODE=00, CNF=10 (0b1000).
    fn is_input_pull_up_down_mode(&self, pin: usize) -> bool {
        self.get_modecnf(pin) == 0b1000
    }

    // In pull mode, ODR bit selects pull-up (1) vs pull-down (0).
    fn is_input_pullup(&self, pin: usize) -> bool {
        self.is_input_pull_up_down_mode(pin) && ((self.odr.get() & (1 << pin)) != 0)
    }

    fn is_input_pulldown(&self, pin: usize) -> bool {
        self.is_input_pull_up_down_mode(pin) && ((self.odr.get() & (1 << pin)) == 0)
    }

    fn check_and_warn_af(&self, start_pin: usize) {
        for i in 0..8 {
            let pin = start_pin + i;
            if self.is_alternate_push_pull(pin) || self.is_alternate_open_drain(pin) {
                log_mask_ln!(
                    Log::Unimp,
                    "stm32f1xx_gpio: AF mode configured on pin {}. Downgrading to General Output \
                     to keep simulation running.",
                    pin
                );
            }
        }
    }

    // ---- operations (all return which out pins changed) ----

    /// Restore the RM0008 documented reset state.
    pub fn reset(&self) -> PinUpdate {
        self.crl.set(RESET_CRL);
        self.crh.set(RESET_CRH);
        self.idr.set(0);
        self.odr.set(0);
        self.lckr.set(0);
        self.disconnected_pins.set(0xFFFF);
        self.pins_connected_high.set(0);
        self.update_gpio_idr()
    }

    /// External source drives pin `line` to `level`.
    pub fn gpio_set(&self, line: usize, level: u32) -> PinUpdate {
        // A push-pull output (or an open-drain output driven high) cannot be
        // driven externally without a short-circuit; refuse it. An open-drain
        // output driven low is allowed.
        if self.is_output(line)
            && !((self.is_alternate_open_drain(line) || self.is_general_open_drain(line))
                && (level == 0))
        {
            log_mask_ln!(
                Log::GuestError,
                "stm32f1xx_gpio: Line {} can't be driven externally",
                line
            );
            return PinUpdate::default();
        }

        self.disconnected_pins
            .set(self.disconnected_pins.get() & !(1 << line));

        if level != 0 {
            self.pins_connected_high
                .set(self.pins_connected_high.get() | (1 << line));
        } else {
            self.pins_connected_high
                .set(self.pins_connected_high.get() & !(1 << line));
        }

        self.update_gpio_idr()
    }

    // Return mask of pins that are both configured in output mode and
    // externally driven (except open-drain pins externally set to 0), which
    // must be disconnected to avoid a short circuit.
    fn get_gpio_pinmask_to_disconnect(&self) -> u16 {
        let mut pins_to_disconnect: u16 = 0;
        for i in 0..GPIO_NUM_PINS {
            if (self.disconnected_pins.get() & (1 << i) == 0) && self.is_output(i) {
                if self.is_general_push_pull(i)
                    || self.is_alternate_push_pull(i)
                    || (self.pins_connected_high.get() & (1 << i) != 0)
                {
                    pins_to_disconnect |= 1 << i;
                    log_mask_ln!(
                        Log::GuestError,
                        "stm32f1xx_gpio: Line {} can't be driven externally",
                        i
                    );
                }
            }
        }
        pins_to_disconnect
    }

    fn disconnect_gpio_pins(&self, lines: u16) -> PinUpdate {
        self.disconnected_pins
            .set(self.disconnected_pins.get() | lines);
        self.update_gpio_idr()
    }

    /// MMIO write. Returns the resulting output-pin changes.
    pub fn write(&self, offset: u64, data: u32) -> PinUpdate {
        match offset {
            REG_CRL => {
                self.crl.set(data);
                self.check_and_warn_af(0);
                // A pin reconfigured to output can no longer be driven.
                self.disconnect_gpio_pins(self.get_gpio_pinmask_to_disconnect())
            }
            REG_CRH => {
                self.crh.set(data);
                self.check_and_warn_af(8);
                self.disconnect_gpio_pins(self.get_gpio_pinmask_to_disconnect())
            }
            REG_ODR => {
                self.odr.set(data & 0xFFFF);
                self.update_gpio_idr()
            }
            REG_BRR => {
                let bits_to_reset = data & 0xFFFF;
                self.odr.set(self.odr.get() & !bits_to_reset);
                self.update_gpio_idr()
            }
            REG_BSRR => {
                let bits_to_set = data & 0xFFFF;
                let bits_to_reset = (data >> 16) & 0xFFFF;

                // If both BSx and BRx are set, BSx has priority.
                let mut current_odr = self.odr.get();
                current_odr &= !bits_to_reset;
                current_odr |= bits_to_set;
                self.odr.set(current_odr);

                self.update_gpio_idr()
            }
            REG_LCKR => {
                self.lckr.set(data);
                log_mask_ln!(
                    Log::Unimp,
                    "stm32f1xx_gpio: Locking port bits configuration isn't supported"
                );
                PinUpdate::default()
            }
            _ => {
                log_mask_ln!(
                    Log::GuestError,
                    "stm32f1xx_gpio_write: Bad offset 0x{:x}",
                    offset
                );
                PinUpdate::default()
            }
        }
    }

    /// MMIO read.
    pub fn read(&self, offset: u64) -> u32 {
        match offset {
            REG_CRL => self.crl.get(),
            REG_CRH => self.crh.get(),
            REG_IDR => self.idr.get(),
            REG_ODR => self.odr.get(),
            REG_BSRR => 0,
            REG_BRR => 0,
            REG_LCKR => self.lckr.get(),
            _ => {
                log_mask_ln!(
                    Log::GuestError,
                    "stm32f1xx_gpio_read: Bad offset 0x{:x}",
                    offset
                );
                0
            }
        }
    }

    /// Recompute IDR from the pin configuration and external drive state.
    /// Returns which determined output pins changed level.
    fn update_gpio_idr(&self) -> PinUpdate {
        // new_idr_mask bit i => pin i has a determined level in new_idr.
        let mut new_idr_mask = 0u32;
        let mut new_idr = self.odr.get();
        let old_idr = self.idr.get();

        for i in 0..GPIO_NUM_PINS {
            if self.is_output(i) {
                if self.is_general_push_pull(i) || self.is_alternate_push_pull(i) {
                    // push-pull output: pin follows ODR
                    new_idr_mask |= 1 << i;
                } else if (self.odr.get() & (1 << i)) == 0 {
                    // open-drain, ODR 0: pin driven low
                    new_idr_mask |= 1 << i;
                } else if (self.disconnected_pins.get() & (1 << i)) == 0
                    && (self.pins_connected_high.get() & (1 << i)) == 0
                {
                    // open-drain, ODR 1, externally pulled low
                    new_idr &= !(1 << i);
                    new_idr_mask |= 1 << i;
                } else if self.is_input_pullup(i) {
                    new_idr_mask |= 1 << i;
                } else if self.is_input_pulldown(i) {
                    new_idr &= !(1 << i);
                    new_idr_mask |= 1 << i;
                }
                // otherwise open-drain ODR 1 floating: keep old value
            } else if (self.disconnected_pins.get() & (1 << i)) == 0 {
                // input/analog, externally driven
                if (self.pins_connected_high.get() & (1 << i)) != 0 {
                    new_idr_mask |= 1 << i;
                    new_idr |= 1 << i;
                } else {
                    new_idr_mask |= 1 << i;
                    new_idr &= !(1 << i);
                }
            } else {
                // input/analog, not driven: pull-up/down or floating
                if self.is_input_pullup(i) {
                    new_idr |= 1 << i;
                    new_idr_mask |= 1 << i;
                } else if self.is_input_pulldown(i) {
                    new_idr &= !(1 << i);
                    new_idr_mask |= 1 << i;
                }
            }
        }

        self.idr
            .set((old_idr & !new_idr_mask) | (new_idr & new_idr_mask));

        // Report which determined pins actually flipped level.
        let mut changed = 0u32;
        for i in 0..GPIO_NUM_PINS {
            if (new_idr_mask & (1 << i)) != 0
                && ((new_idr & (1 << i)) != 0) != ((old_idr & (1 << i)) != 0)
            {
                changed |= 1 << i;
            }
        }
        PinUpdate {
            mask: changed,
            levels: new_idr,
        }
    }
}

impl_vmstate_struct!(
    GpioRegisters,
    VMStateDescriptionBuilder::<GpioRegisters>::new()
        .name(c"stm32f1xx-gpio/regs")
        .version_id(1)
        .minimum_version_id(1)
        .fields(vmstate_fields! {
            vmstate_of!(GpioRegisters, crl),
            vmstate_of!(GpioRegisters, crh),
            vmstate_of!(GpioRegisters, idr),
            vmstate_of!(GpioRegisters, odr),
            vmstate_of!(GpioRegisters, lckr),
            vmstate_of!(GpioRegisters, disconnected_pins),
            vmstate_of!(GpioRegisters, pins_connected_high),
        })
        .build()
);

#[repr(C)]
#[derive(qom::Object, hwcore::Device)]
pub struct Stm32f1xxGpioState {
    parent_obj: ParentField<SysBusDevice>,
    mmio: MemoryRegion,

    regs: GpioRegisters,

    // One output line per pin, exposed as GPIO-out (to EXTI/board).
    outlines: [InterruptSource; GPIO_NUM_PINS],
}

impl Stm32f1xxGpioState {
    unsafe fn init(mut this: ParentInit<Self>) {
        static STM32F1XX_GPIO_OPS: MemoryRegionOps<Stm32f1xxGpioState> =
            MemoryRegionOpsBuilder::<Stm32f1xxGpioState>::new()
                .read(&Stm32f1xxGpioState::read)
                .write(&Stm32f1xxGpioState::write)
                .little_endian()
                .impl_sizes(4, 4)
                .build();

        MemoryRegion::init_io(
            &mut uninit_field_mut!(*this, mmio),
            &STM32F1XX_GPIO_OPS,
            "stm32f1xx-gpio-rust",
            0x400,
        );

        uninit_field_mut!(*this, regs).write(GpioRegisters::new());
        uninit_field_mut!(*this, outlines).write(Default::default());

        trace::trace_stm32f1xx_gpio_instance_init();
    }

    fn post_init(&self) {
        self.init_mmio(&self.mmio);
        // Pin-change interrupts on F1 are delivered through EXTI, not a
        // dedicated NVIC line, so the port only exposes GPIO in/out lines
        // (no sysbus IRQ).
        self.init_gpio_in(GPIO_NUM_PINS as u32, Self::gpio_set);
        self.init_gpio_out(&self.outlines);
    }

    /// Drive the out lines according to a [`PinUpdate`].
    fn apply_pin_update(&self, upd: PinUpdate) {
        for i in 0..GPIO_NUM_PINS {
            if (upd.mask & (1 << i)) != 0 {
                let level = (upd.levels & (1 << i)) != 0;
                self.outlines[i].set(level);
                trace::trace_stm32f1xx_gpio_irq(i as u32, level as i32);
            }
        }
    }

    fn reset_hold(&self, _type: ResetType) {
        trace::trace_stm32f1xx_gpio_reset();
        let upd = self.regs.reset();
        self.apply_pin_update(upd);
    }

    // GPIO-in callback: external source drives a pin.
    fn gpio_set(&self, line: u32, level: u32) {
        let upd = self.regs.gpio_set(line as usize, level);
        self.apply_pin_update(upd);
    }

    fn write(&self, offset: hwaddr, data: u64, size: u32) {
        trace::trace_stm32f1xx_gpio_write(offset, data, size);
        let upd = self.regs.write(offset, data as u32);
        self.apply_pin_update(upd);
    }

    fn read(&self, offset: hwaddr, size: u32) -> u64 {
        let value = self.regs.read(offset);
        trace::trace_stm32f1xx_gpio_read(offset, value as u64, size);
        value as u64
    }
}

impl ObjectImpl for Stm32f1xxGpioState {
    type ParentType = SysBusDevice;

    const CLASS_INIT: fn(&mut Self::Class) = Self::Class::class_init::<Self>;
    const INSTANCE_INIT: Option<unsafe fn(ParentInit<Self>)> = Some(Self::init);
    const INSTANCE_POST_INIT: Option<fn(&Self)> = Some(Self::post_init);
}

impl DeviceImpl for Stm32f1xxGpioState {
    const VMSTATE: Option<VMStateDescription<Self>> = Some(VMSTATE_STM32F1XX_GPIO);
}

impl SysBusDeviceImpl for Stm32f1xxGpioState {}

impl ResettablePhasesImpl for Stm32f1xxGpioState {
    const HOLD: Option<fn(&Self, ResetType)> = Some(Self::reset_hold);
}

// Migration: register block only; IRQ links are reconstructed on wire-up.
const VMSTATE_STM32F1XX_GPIO: VMStateDescription<Stm32f1xxGpioState> =
    VMStateDescriptionBuilder::<Stm32f1xxGpioState>::new()
        .name(c"stm32f1xx-gpio")
        .version_id(1)
        .minimum_version_id(1)
        .fields(vmstate_fields! {
            vmstate_of!(Stm32f1xxGpioState, regs),
        })
        .build();
