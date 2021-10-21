use pio::SideSet;
use rp2040_pac::{PIO0, PIO1};

use super::{InstalledProgram, PIOExt};

/// State machine identifier (without a specified PIO block).
pub trait StateMachineIndex {
    /// Numerical index of the state machine (0 to 3).
    fn id() -> usize;
}

/// First state machine.
pub struct SM0;
/// Second state machine.
pub struct SM1;
/// Third state machine.
pub struct SM2;
/// Fourth state machine.
pub struct SM3;

impl StateMachineIndex for SM0 {
    fn id() -> usize {
        0
    }
}
impl StateMachineIndex for SM1 {
    fn id() -> usize {
        1
    }
}
impl StateMachineIndex for SM2 {
    fn id() -> usize {
        2
    }
}
impl StateMachineIndex for SM3 {
    fn id() -> usize {
        3
    }
}

/// Trait to identify a single state machine, as a generic type parameter to `UninitStateMachine`,
/// `InitStateMachine`, etc.
pub trait ValidStateMachine {
    /// The PIO block to which this state machine belongs.
    type PIO: PIOExt;

    /// The index of this state machine (between 0 and 3).
    fn id() -> usize;
}

/// First state machine of the first PIO block.
pub type PIO0SM0 = (PIO0, SM0);
/// Second state machine of the first PIO block.
pub type PIO0SM1 = (PIO0, SM1);
/// Third state machine of the first PIO block.
pub type PIO0SM2 = (PIO0, SM2);
/// Fourth state machine of the first PIO block.
pub type PIO0SM3 = (PIO0, SM3);
/// First state machine of the second PIO block.
pub type PIO1SM0 = (PIO1, SM0);
/// Second state machine of the second PIO block.
pub type PIO1SM1 = (PIO1, SM1);
/// Third state machine of the second PIO block.
pub type PIO1SM2 = (PIO1, SM2);
/// Fourth state machine of the second PIO block.
pub type PIO1SM3 = (PIO1, SM3);

impl<P: PIOExt, SM: StateMachineIndex> ValidStateMachine for (P, SM) {
    type PIO = P;
    fn id() -> usize {
        SM::id()
    }
}

/// PIO State Machine (uninitialized, without a program).
#[derive(Debug)]
pub struct UninitStateMachine<SM: ValidStateMachine> {
    pub(super) block: *const rp2040_pac::pio0::RegisterBlock,
    pub(super) sm: *const rp2040_pac::pio0::SM,
    pub(super) _phantom: core::marker::PhantomData<SM>,
}

// Safety: `UninitStateMachine` only uses atomic accesses to shared registers.
unsafe impl<SM: ValidStateMachine + Send> Send for UninitStateMachine<SM> {}

impl<SM: ValidStateMachine> UninitStateMachine<SM> {
    /// Start and stop the state machine.
    pub(super) fn set_enabled(&mut self, enabled: bool) {
        // Bits 3:0 are SM_ENABLE.
        let mask = 1 << SM::id();
        if enabled {
            self.set_ctrl_bits(mask);
        } else {
            self.clear_ctrl_bits(mask);
        }
    }

    pub(super) fn restart(&mut self) {
        // Bits 7:4 are SM_RESTART.
        self.set_ctrl_bits(1 << (SM::id() + 4));
    }

    pub(super) fn reset_clock(&mut self) {
        // Bits 11:8 are CLKDIV_RESTART.
        self.set_ctrl_bits(1 << (SM::id() + 8));
    }

    fn set_ctrl_bits(&mut self, bits: u32) {
        const ATOMIC_SET_OFFSET: usize = 0x2000;
        // Safety: We only use the atomic alias of the register.
        unsafe {
            (*self.block)
                .ctrl
                .as_ptr()
                .add(ATOMIC_SET_OFFSET / 4)
                .write_volatile(bits);
        }
    }

    fn clear_ctrl_bits(&mut self, bits: u32) {
        const ATOMIC_CLEAR_OFFSET: usize = 0x3000;
        // Safety: We only use the atomic alias of the register.
        unsafe {
            (*self.block)
                .ctrl
                .as_ptr()
                .add(ATOMIC_CLEAR_OFFSET / 4)
                .write_volatile(bits);
        }
    }

    pub(super) fn set_clock_divisor(&self, divisor: f32) {
        // sm frequency = clock freq / (CLKDIV_INT + CLKDIV_FRAC / 256)
        let int = divisor as u16;
        let frac = ((divisor - int as f32) * 256.0) as u8;

        self.sm().sm_clkdiv.write(|w| {
            unsafe {
                w.int().bits(int);
                w.frac().bits(frac);
            }

            w
        });
    }

    /// Set the current instruction.
    pub(super) fn set_instruction(&mut self, instruction: u16) {
        self.sm()
            .sm_instr
            .write(|w| unsafe { w.sm0_instr().bits(instruction) })
    }

    pub(super) fn sm(&self) -> &rp2040_pac::pio0::SM {
        unsafe { &*self.sm }
    }
}

/// PIO State Machine with an associated program.
pub struct StateMachine<SM: ValidStateMachine, State> {
    pub(super) sm: UninitStateMachine<SM>,
    pub(super) program: InstalledProgram<SM::PIO>,
    pub(super) _phantom: core::marker::PhantomData<State>,
}

/// Marker for an initialized, but stopped state machine.
pub struct Stopped;
/// Marker for an initialized and running state machine.
pub struct Running;

impl<SM: ValidStateMachine, State> StateMachine<SM, State> {
    /// Stops the state machine if it is still running and returns its program.
    ///
    /// The program can be uninstalled to free space once it is no longer used by any state
    /// machine.
    pub fn uninit(
        mut self,
        _rx: Rx<SM>,
        _tx: Tx<SM>,
    ) -> (UninitStateMachine<SM>, InstalledProgram<SM::PIO>) {
        self.sm.set_enabled(false);
        (self.sm, self.program)
    }

    /// The address of the instruction currently being executed.
    pub fn instruction_address(&self) -> u32 {
        self.sm.sm().sm_addr.read().bits()
    }

    /// Set the current instruction.
    pub fn set_instruction(&mut self, instruction: u16) {
        // TODO: Check if this function is safe to call while the state machine is running.
        self.sm.set_instruction(instruction);
    }

    /// Check if the current instruction is stalled.
    pub fn stalled(&self) -> bool {
        self.sm.sm().sm_execctrl.read().exec_stalled().bits()
    }
}

impl<SM: ValidStateMachine> StateMachine<SM, Stopped> {
    /// Starts execution of the selected program.
    pub fn start(mut self) -> StateMachine<SM, Running> {
        // Enable SM
        self.sm.set_enabled(true);

        StateMachine {
            sm: self.sm,
            program: self.program,
            _phantom: core::marker::PhantomData,
        }
    }

    /// Sets the pin directions for the specified pins.
    ///
    /// The `pins` parameter specifies a set of pins as a mask, and `pindir` contains the
    /// directions that are configured for these pins. The bits in both masks correspond to the pin
    /// number. The user has to make sure that they do not select any pins that are in use by any
    /// other state machines of the same PIO block.
    ///
    /// This function needs to be called for sideset pins if they are supposed to be used as
    /// output pins.
    pub fn set_pindirs_with_mask(&mut self, mut pins: u32, pindir: u32) {
        let mut pin = 0;
        let prev_pinctrl = self.sm.sm().sm_pinctrl.read().bits();
        // For each pin in the mask, we select the pin as a SET pin and then execute "set PINDIRS,
        // <direction>".
        while pins != 0 {
            if (pins & 1) != 0 {
                self.sm.sm().sm_pinctrl.write(|w| {
                    unsafe {
                        w.set_count().bits(1);
                        w.set_base().bits(pin as u8);
                    }
                    w
                });
                let set_pindirs = pio::Instruction {
                    operands: pio::InstructionOperands::SET {
                        destination: pio::SetDestination::PINDIRS,
                        data: ((pindir >> pin) & 0x1) as u8,
                    },
                    delay: 0,
                    side_set: None,
                }
                .encode(SideSet::new(false, 0, false));
                self.sm.sm().sm_instr.write(|w| {
                    unsafe {
                        w.sm0_instr().bits(set_pindirs);
                    }
                    w
                });
            }
            pin += 1;
            pins >>= 1;
        }
        // We modified PINCTRL, yet the program assumes a certain configuration, so restore the
        // previous value.
        self.sm.sm().sm_pinctrl.write(|w| {
            unsafe {
                w.bits(prev_pinctrl);
            }
            w
        });
    }
}

impl<P: PIOExt, SM: StateMachineIndex> StateMachine<(P, SM), Stopped> {
    /// Restarts the clock dividers for the specified state machines.
    ///
    /// As a result, the clock will be synchronous for the state machines, which is a precondition
    /// for synchronous operation.
    ///
    /// The function returns an object that, once destructed, restarts the clock dividers. This
    /// object allows further state machines to be added if more than two shall be synchronized.
    ///
    /// # Example
    ///
    /// ```ignore
    /// sm0.synchronize_with(sm1).and_with(sm2);
    /// ```
    pub fn synchronize_with<'sm, SM2: StateMachineIndex>(
        &'sm mut self,
        _other_sm: &'sm mut StateMachine<(P, SM2), Stopped>,
    ) -> Synchronize<'sm, (P, SM)> {
        let sm_mask = (1 << SM::id()) | (1 << SM2::id());
        Synchronize { sm: self, sm_mask }
    }
}

/// Type which, once destructed, restarts the clock dividers for all selected state machines,
/// effectively synchronizing them.
pub struct Synchronize<'sm, SM: ValidStateMachine> {
    sm: &'sm mut StateMachine<SM, Stopped>,
    sm_mask: u32,
}

impl<'sm, P: PIOExt, SM: StateMachineIndex> Synchronize<'sm, (P, SM)> {
    /// Adds another state machine to be synchronized.
    pub fn and_with<SM2: StateMachineIndex>(
        mut self,
        _other_sm: &'sm mut StateMachine<(P, SM2), Stopped>,
    ) -> Self {
        // Add another state machine index to the mask.
        self.sm_mask |= 1 << SM2::id();
        self
    }
}

impl<'sm, SM: ValidStateMachine> Drop for Synchronize<'sm, SM> {
    fn drop(&mut self) {
        // Restart the clocks of all state machines specified by the mask.
        // Bits 11:8 of CTRL contain CLKDIV_RESTART.
        let sm_mask = self.sm_mask << 8;
        const ATOMIC_SET_OFFSET: usize = 0x2000;
        // Safety: We only use the atomic alias of the register.
        unsafe {
            (*self.sm.sm.block)
                .ctrl
                .as_ptr()
                .add(ATOMIC_SET_OFFSET / 4)
                .write_volatile(sm_mask as u32);
        }
    }
}

impl<SM: ValidStateMachine> StateMachine<SM, Running> {
    /// Stops execution of the selected program.
    pub fn stop(mut self) -> StateMachine<SM, Stopped> {
        // Enable SM
        self.sm.set_enabled(false);

        StateMachine {
            sm: self.sm,
            program: self.program,
            _phantom: core::marker::PhantomData,
        }
    }
}

/// PIO RX FIFO handle.
pub struct Rx<SM: ValidStateMachine> {
    pub(super) block: *const rp2040_pac::pio0::RegisterBlock,
    pub(super) _phantom: core::marker::PhantomData<SM>,
}

impl<SM: ValidStateMachine> Rx<SM> {
    /// Get the next element from RX FIFO.
    ///
    /// Returns `None` if the FIFO is empty.
    pub fn read(&mut self) -> Option<u32> {
        // Safety: The register is never written by software.
        let is_empty = unsafe { &*self.block }.fstat.read().rxempty().bits() & (1 << SM::id()) != 0;

        if is_empty {
            return None;
        }

        // Safety: The register is unique to this Rx instance.
        Some(unsafe { &*self.block }.rxf[SM::id() as usize].read().bits())
    }
}

/// PIO TX FIFO handle.
pub struct Tx<SM: ValidStateMachine> {
    pub(super) block: *const rp2040_pac::pio0::RegisterBlock,
    pub(super) _phantom: core::marker::PhantomData<SM>,
}

impl<SM: ValidStateMachine> Tx<SM> {
    /// Write an element to TX FIFO.
    ///
    /// Returns `true` if the value was written to FIFO, `false` otherwise.
    pub fn write(&mut self, value: u32) -> bool {
        // Safety: The register is never written by software.
        let is_full = unsafe { &*self.block }.fstat.read().txfull().bits() & (1 << SM::id()) != 0;

        if is_full {
            return false;
        }

        // Safety: The register is unique to this Tx instance.
        unsafe { &*self.block }.txf[SM::id()].write(|w| unsafe { w.bits(value) });

        true
    }
}
