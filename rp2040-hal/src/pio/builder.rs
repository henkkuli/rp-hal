use super::{
    InstalledProgram, PIOExt, Rx, StateMachine, StateMachineIndex, Stopped, Tx, UninitStateMachine,
};

/// Comparison used for `mov x, status` instruction.
#[derive(Debug, Clone, Copy)]
pub enum MovStatusConfig {
    /// The `mov x, status` instruction returns all ones if TX FIFO level is below the set status, otherwise all zeros.
    Tx(u8),
    /// The `mov x, status` instruction returns all ones if RX FIFO level is below the set status, otherwise all zeros.
    Rx(u8),
}

/// Shift direction for input and output shifting.
#[derive(Debug, Clone, Copy)]
pub enum ShiftDirection {
    /// Shift register to left.
    Left,
    /// Shift register to right.
    Right,
}

impl ShiftDirection {
    fn bit(self) -> bool {
        match self {
            Self::Left => false,
            Self::Right => true,
        }
    }
}
/// Builder to deploy a fully configured PIO program on one of the state
/// machines.
#[derive(Debug)]
pub struct PIOBuilder<P> {
    /// Clock divisor.
    clock_divisor: f32,

    /// Program location and configuration.
    program: InstalledProgram<P>,
    /// GPIO pin used by `jmp pin` instruction.
    jmp_pin: u8,

    /// Continuously assert the most recent OUT/SET to the pins.
    out_sticky: bool,
    /// Use a bit of OUT data as an auxilary write enable.
    ///
    /// When [`out_sticky`](Self::out_sticky) is enabled, setting the bit to 0 deasserts for that instr.
    inline_out: Option<u8>,
    /// Config for `mov x, status` instruction.
    mov_status: MovStatusConfig,

    /// Config for FIFO joining.
    fifo_join: Buffers,

    /// Number of bits shifted out of `OSR` before autopull or conditional pull will take place.
    pull_threshold: u8,
    /// Number of bits shifted into `ISR` before autopush or conditional push will take place.
    push_threshold: u8,
    /// Shift direction for `OUT` instruction.
    out_shiftdir: ShiftDirection,
    /// Shift direction for `IN` instruction.
    in_shiftdir: ShiftDirection,
    /// Enable autopull.
    autopull: bool,
    /// Enable autopush.
    autopush: bool,

    /// Number of pins asserted by a `SET`.
    set_count: u8,
    /// Number of pins asserted by an `OUT PINS`, `OUT PINDIRS` or `MOV PINS` instruction.
    out_count: u8,
    /// The first pin that is assigned in state machine's `IN` data bus.
    in_base: u8,
    /// The first pin that is affected by side-set operations.
    side_set_base: u8,
    /// The first pin that is affected by `SET PINS` or `SET PINDIRS` instructions.
    set_base: u8,
    /// The first pin that is affected by `OUT PINS`, `OUT PINDIRS` or `MOV PINS` instructions.
    out_base: u8,
}

/// Buffer sharing configuration.
#[derive(Debug, Clone, Copy)]
pub enum Buffers {
    /// No sharing.
    RxTx,
    /// The memory of the RX FIFO is given to the TX FIFO to double its depth.
    OnlyTx,
    /// The memory of the TX FIFO is given to the RX FIFO to double its depth.
    OnlyRx,
}

/// Errors that occurred during `PIO::install`.
#[derive(Debug)]
pub enum InstallError {
    /// There was not enough space for the instructions on the selected PIO.
    NoSpace,
}

impl<P: PIOExt> PIOBuilder<P> {
    /// Set config settings based on information from the given [`pio::Program`].
    /// Additional configuration may be needed in addition to this.
    pub fn from_program(p: InstalledProgram<P>) -> Self {
        PIOBuilder {
            clock_divisor: 1.0,
            program: p,
            jmp_pin: 0,
            out_sticky: false,
            inline_out: None,
            mov_status: MovStatusConfig::Tx(0),
            fifo_join: Buffers::RxTx,
            pull_threshold: 0,
            push_threshold: 0,
            out_shiftdir: ShiftDirection::Left,
            in_shiftdir: ShiftDirection::Left,
            autopull: false,
            autopush: false,
            set_count: 5,
            out_count: 0,
            in_base: 0,
            side_set_base: 0,
            set_base: 0,
            out_base: 0,
        }
    }

    /// Set the pins asserted by `SET` instruction.
    ///
    /// The least-significant bit of `SET` instruction asserts the state of the pin indicated by `base`, the next bit
    /// asserts the state of the next pin, and so on up to `count` pins. The pin numbers are considered modulo 32.
    pub fn set_pins(mut self, base: u8, count: u8) -> Self {
        assert!(count <= 5);
        self.set_base = base;
        self.set_count = count;
        self
    }

    /// Set the pins asserted by `OUT` instruction.
    ///
    /// The least-significant bit of `OUT` instruction asserts the state of the pin indicated by `base`, the next bit
    /// asserts the state of the next pin, and so on up to `count` pins. The pin numbers are considered modulo 32.
    pub fn out_pins(mut self, base: u8, count: u8) -> Self {
        assert!(count <= 5);
        self.out_base = base;
        self.out_count = count;
        self
    }

    /// Set the pins used by `IN` instruction.
    ///
    /// The `IN` instruction reads the least significant bit from the pin indicated by `base`, the next bit from the
    /// next pin, and so on. The pin numbers are considered modulo 32.
    pub fn in_pin_base(mut self, base: u8) -> Self {
        self.in_base = base;
        self
    }

    /// Set the pin used by `JMP PIN` instruction.
    ///
    /// When the pin set by this function is high, the jump is taken, otherwise not.
    pub fn jmp_pin(mut self, pin: u8) -> Self {
        self.jmp_pin = pin;
        self
    }

    /// Set the pins used by side-set instructions.
    ///
    /// The least-significant side-set bit asserts the state of the pin indicated by `base`, the next bit asserts the
    /// state of the next pin, and so on up to number of bits set using [`Self::side_set`] function.
    pub fn side_set_pin_base(mut self, base: u8) -> Self {
        self.side_set_base = base;
        self
    }

    /// Set buffer sharing.
    ///
    /// See [`Buffers`] for more information.
    pub fn buffers(mut self, buffers: Buffers) -> Self {
        self.fifo_join = buffers;
        self
    }

    /// Set the clock divisor.
    ///
    /// The is based on the sys_clk. Set 1 for full speed. A clock divisor of `n` will cause the state machine to run 1
    /// cycle every `n` clock cycles. For small values of `n`, a fractional divisor may introduce unacceptable jitter.
    pub fn clock_divisor(mut self, divisor: f32) -> Self {
        self.clock_divisor = divisor;
        self
    }

    /// Set the output sticky state.
    ///
    /// When the output is set to be sticky, the PIO hardware continuously asserts the most recent `OUT`/`SET` to the
    /// pins.
    pub fn out_sticky(mut self, out_sticky: bool) -> Self {
        self.out_sticky = out_sticky;
        self
    }

    /// Set the inline `OUT` enable bit.
    ///
    /// When set to value, the given bit of `OUT` instruction's data is used as an auxiliary write enable. When used
    /// with [`Self::out_sticky`], writes with enable 0 will deassert the latest pin write.
    pub fn inline_out(mut self, inline_out: Option<u8>) -> Self {
        self.inline_out = inline_out;
        self
    }

    /// Set the autopush state.
    ///
    /// When autopush is enabled, the `IN` instruction automatically pushes the data once the number of bits reaches
    /// threshold set by [`Self::push_threshold`].
    pub fn autopush(mut self, autopush: bool) -> Self {
        self.autopush = autopush;
        self
    }

    /// Set the number of bits pushed into ISR before autopush or conditional push will take place.
    pub fn push_threshold(mut self, threshold: u8) -> Self {
        self.push_threshold = threshold;
        self
    }

    /// Set the autopull state.
    ///
    /// When autopull is enabled, the `OUT` instruction automatically pulls the data once the number of bits reaches
    /// threshold set by [`Self::pull_threshold`].
    pub fn autopull(mut self, autopull: bool) -> Self {
        self.autopull = autopull;
        self
    }

    /// Set the number of bits pulled from out of OSR before autopull or conditional pull will take place.
    pub fn pull_threshold(mut self, threshold: u8) -> Self {
        self.pull_threshold = threshold;
        self
    }

    /// Set the ISR shift direction for `IN` instruction.
    ///
    /// For example `ShiftDirection::Right` means that ISR is shifted to right, i.e. data enters from left.
    pub fn in_shift_direction(mut self, direction: ShiftDirection) -> Self {
        self.in_shiftdir = direction;
        self
    }

    /// Set the OSR shift direction for `OUT` instruction.
    ///
    /// For example `ShiftDirection::Right` means that OSR is shifted to right, i.e. data is taken from the right side.
    pub fn out_shift_direction(mut self, direction: ShiftDirection) -> Self {
        self.out_shiftdir = direction;
        self
    }

    /// Build the config and deploy it to a StateMachine.
    #[allow(clippy::type_complexity)] // The return type cannot really be simplified.
    pub fn build<SM: StateMachineIndex>(
        self,
        mut sm: UninitStateMachine<(P, SM)>,
    ) -> (StateMachine<(P, SM), Stopped>, Rx<(P, SM)>, Tx<(P, SM)>) {
        let offset = self.program.offset;

        // Stop the SM
        sm.set_enabled(false);

        // Write all configuration bits
        sm.set_clock_divisor(self.clock_divisor);

        sm.sm().sm_execctrl.write(|w| {
            w.side_en().bit(self.program.side_set.optional());
            w.side_pindir().bit(self.program.side_set.pindirs());

            unsafe {
                w.jmp_pin().bits(self.jmp_pin);
            }

            if let Some(inline_out) = self.inline_out {
                w.inline_out_en().bit(true);
                unsafe {
                    w.out_en_sel().bits(inline_out);
                }
            } else {
                w.inline_out_en().bit(false);
            }

            w.out_sticky().bit(self.out_sticky);

            unsafe {
                w.wrap_top().bits(offset as u8 + self.program.wrap.source);
                w.wrap_bottom()
                    .bits(offset as u8 + self.program.wrap.target);
            }

            let n = match self.mov_status {
                MovStatusConfig::Tx(n) => {
                    w.status_sel().bit(false);
                    n
                }
                MovStatusConfig::Rx(n) => {
                    w.status_sel().bit(true);
                    n
                }
            };
            unsafe {
                w.status_n().bits(n);
            }

            w
        });

        sm.sm().sm_shiftctrl.write(|w| {
            let (fjoin_rx, fjoin_tx) = match self.fifo_join {
                Buffers::RxTx => (false, false),
                Buffers::OnlyTx => (false, true),
                Buffers::OnlyRx => (true, false),
            };
            w.fjoin_rx().bit(fjoin_rx);
            w.fjoin_tx().bit(fjoin_tx);

            unsafe {
                // TODO: Encode 32 as zero, and error on 0
                w.pull_thresh().bits(self.pull_threshold);
                w.push_thresh().bits(self.push_threshold);
            }

            w.out_shiftdir().bit(self.out_shiftdir.bit());
            w.in_shiftdir().bit(self.in_shiftdir.bit());

            w.autopull().bit(self.autopull);
            w.autopush().bit(self.autopush);

            w
        });

        sm.sm().sm_pinctrl.write(|w| {
            unsafe {
                w.sideset_count().bits(self.program.side_set.bits());
                w.set_count().bits(self.set_count);
                w.out_count().bits(self.out_count);

                w.in_base().bits(self.in_base);
                w.sideset_base().bits(self.side_set_base);
                w.set_base().bits(self.set_base);
                w.out_base().bits(self.out_base);
            }

            w
        });

        // Restart SM and its clock
        sm.restart();
        sm.reset_clock();

        // Set starting location by setting the state machine to execute a jmp
        // to the beginning of the program we loaded in.
        sm.set_instruction(
            pio::InstructionOperands::JMP {
                condition: pio::JmpCondition::Always,
                address: offset as u8,
            }
            .encode(),
        );

        let rx = Rx {
            block: sm.block,
            _phantom: core::marker::PhantomData,
        };
        let tx = Tx {
            block: sm.block,
            _phantom: core::marker::PhantomData,
        };
        (
            StateMachine {
                sm,
                program: self.program,
                _phantom: core::marker::PhantomData,
            },
            rx,
            tx,
        )
    }
}
