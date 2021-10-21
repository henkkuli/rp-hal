//! Programmable IO (PIO)
/// See [Chapter 3](https://rptl.io/pico-datasheet) for more details.
use crate::resets::SubsystemReset;
use pio::Program;
use rp2040_pac::{PIO0, PIO1};

mod builder;
pub use builder::*;
mod interrupt;
pub use interrupt::*;
mod program;
pub use program::*;
mod state_machine;
pub use state_machine::*;

const PIO_INSTRUCTION_COUNT: usize = 32;

/// PIO Instance
pub trait PIOExt:
    core::ops::Deref<Target = rp2040_pac::pio0::RegisterBlock> + SubsystemReset + Sized
{
    /// Create a new PIO wrapper and split the state machines into individual objects.
    #[allow(clippy::type_complexity)] // Required for symmetry with PIO::free().
    fn split(
        self,
        resets: &mut pac::RESETS,
    ) -> (
        PIO<Self>,
        UninitStateMachine<(Self, SM0)>,
        UninitStateMachine<(Self, SM1)>,
        UninitStateMachine<(Self, SM2)>,
        UninitStateMachine<(Self, SM3)>,
    ) {
        self.reset_bring_up(resets);

        let sm0 = UninitStateMachine {
            block: self.deref(),
            sm: &self.deref().sm[0],
            _phantom: core::marker::PhantomData,
        };
        let sm1 = UninitStateMachine {
            block: self.deref(),
            sm: &self.deref().sm[1],
            _phantom: core::marker::PhantomData,
        };
        let sm2 = UninitStateMachine {
            block: self.deref(),
            sm: &self.deref().sm[2],
            _phantom: core::marker::PhantomData,
        };
        let sm3 = UninitStateMachine {
            block: self.deref(),
            sm: &self.deref().sm[3],
            _phantom: core::marker::PhantomData,
        };
        (
            PIO {
                used_instruction_space: 0,
                interrupts: [
                    Interrupt {
                        id: 0,
                        block: self.deref(),
                        _phantom: core::marker::PhantomData,
                    },
                    Interrupt {
                        id: 1,
                        block: self.deref(),
                        _phantom: core::marker::PhantomData,
                    },
                ],
                pio: self,
            },
            sm0,
            sm1,
            sm2,
            sm3,
        )
    }
}

impl PIOExt for PIO0 {}
impl PIOExt for PIO1 {}

/// Programmable IO Block
pub struct PIO<P: PIOExt> {
    used_instruction_space: u32, // bit for each PIO_INSTRUCTION_COUNT
    pio: P,
    interrupts: [Interrupt<P>; 2],
}

impl<P: PIOExt> core::fmt::Debug for PIO<P> {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        f.debug_struct("PIO")
            .field("used_instruction_space", &self.used_instruction_space)
            .field("pio", &"PIO { .. }")
            .finish()
    }
}

// Safety: `PIO` only provides access to those registers which are not directly used by
// `StateMachine`.
unsafe impl<P: PIOExt + Send> Send for PIO<P> {}

impl<P: PIOExt> PIO<P> {
    /// Free this instance.
    ///
    /// All output pins are left in their current state.
    pub fn free(
        self,
        _sm0: UninitStateMachine<(P, SM0)>,
        _sm1: UninitStateMachine<(P, SM1)>,
        _sm2: UninitStateMachine<(P, SM2)>,
        _sm3: UninitStateMachine<(P, SM3)>,
    ) -> P {
        // All state machines have already been stopped.
        self.pio
    }

    /// This PIO's interrupts.
    pub fn interrupts(&self) -> &[Interrupt<P>; 2] {
        &self.interrupts
    }

    /// Clear PIO's IRQ flags indicated by the bits.
    ///
    /// The PIO has 8 IRQ flags, of which 4 are visible to the host processor. Each bit of `flags` corresponds to one of
    /// the IRQ flags.
    pub fn clear_irq(&self, flags: u8) {
        self.pio.irq.write(|w| unsafe { w.irq().bits(flags) });
    }

    /// Force PIO's IRQ flags indicated by the bits.
    ///
    /// The PIO has 8 IRQ flags, of which 4 are visible to the host processor. Each bit of `flags` corresponds to one of
    /// the IRQ flags.
    pub fn force_irq(&self, flags: u8) {
        self.pio
            .irq_force
            .write(|w| unsafe { w.irq_force().bits(flags) });
    }

    fn find_offset_for_instructions(&self, i: &[u16], origin: Option<u8>) -> Option<usize> {
        if i.len() > PIO_INSTRUCTION_COUNT {
            None
        } else {
            let mask = (1 << i.len()) - 1;
            if let Some(origin) = origin {
                if origin as usize > PIO_INSTRUCTION_COUNT - i.len()
                    || self.used_instruction_space & (mask << origin) != 0
                {
                    None
                } else {
                    Some(origin as usize)
                }
            } else {
                for i in (0..=32 - i.len()).rev() {
                    if self.used_instruction_space & (mask << i) == 0 {
                        return Some(i);
                    }
                }
                None
            }
        }
    }

    /// Allocates space in instruction memory and installs the program.
    ///
    /// The function returns a handle to the installed program that can be used to configure a
    /// `StateMachine` via `PIOBuilder`. The program can be uninstalled to free instruction memory
    /// via `uninstall()` once the state machine using the program has been uninitialized.
    pub fn install(
        &mut self,
        p: &Program<{ pio::RP2040_MAX_PROGRAM_SIZE }>,
    ) -> Result<InstalledProgram<P>, InstallError> {
        if let Some(offset) = self.find_offset_for_instructions(&p.code, p.origin) {
            for (i, instr) in p
                .code
                .iter()
                .map(|instr| {
                    let mut instr = pio::Instruction::decode(*instr, p.side_set).unwrap();

                    instr.operands = match instr.operands {
                        pio::InstructionOperands::JMP { condition, address } => {
                            // JMP instruction. We need to apply offset here
                            let address = address + offset as u8;
                            assert!(
                                address < pio::RP2040_MAX_PROGRAM_SIZE as u8,
                                "Invalid JMP out of the program after offset addition"
                            );
                            pio::InstructionOperands::JMP { condition, address }
                        }
                        _ => instr.operands,
                    };

                    instr.encode(p.side_set)
                })
                .enumerate()
            {
                self.pio.instr_mem[i + offset].write(|w| unsafe { w.bits(instr as u32) })
            }
            self.used_instruction_space |= ((1 << p.code.len()) - 1) << offset;
            Ok(InstalledProgram {
                offset: offset as u8,
                length: p.code.len() as u8,
                side_set: p.side_set,
                wrap: p.wrap,
                _phantom: core::marker::PhantomData,
            })
        } else {
            Err(InstallError::NoSpace)
        }
    }

    /// Removes the specified program from instruction memory, freeing the allocated space.
    pub fn uninstall(&mut self, p: InstalledProgram<P>) {
        let instr_mask = ((1 << p.length as u32) - 1) << p.offset as u32;
        self.used_instruction_space &= !instr_mask;
    }
}
