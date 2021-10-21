use super::PIOExt;

/// PIO Interrupt controller.
#[derive(Debug)]
pub struct Interrupt<P: PIOExt> {
    pub(super) id: u8,
    pub(super) block: *const rp2040_pac::pio0::RegisterBlock,
    pub(super) _phantom: core::marker::PhantomData<P>,
}

// Safety: `Interrupt` provides exclusive access to interrupt registers.
unsafe impl<P: PIOExt + Send> Send for Interrupt<P> {}

impl<P: PIOExt> Interrupt<P> {
    /// Enable interrupts raised by state machines.
    ///
    /// The PIO peripheral has 4 outside visible interrupts that can be raised by the state machines. Note that this
    /// don't correspond with the state machine index; any state machine can raise any one of the four interrupts.
    pub fn enable_sm_interrupt(&self, id: u8) {
        match id {
            0 => self.irq().irq_inte.modify(|_, w| w.sm0().set_bit()),
            1 => self.irq().irq_inte.modify(|_, w| w.sm1().set_bit()),
            2 => self.irq().irq_inte.modify(|_, w| w.sm2().set_bit()),
            3 => self.irq().irq_inte.modify(|_, w| w.sm3().set_bit()),
            _ => panic!("invalid state machine interrupt number"),
        }
    }

    /// Disable interrupts raised by state machines.
    ///
    /// See [`Self::enable_sm_interrupt`] for info about the index.
    pub fn disable_sm_interrupt(&self, id: u8) {
        match id {
            0 => self.irq().irq_inte.modify(|_, w| w.sm0().clear_bit()),
            1 => self.irq().irq_inte.modify(|_, w| w.sm1().clear_bit()),
            2 => self.irq().irq_inte.modify(|_, w| w.sm2().clear_bit()),
            3 => self.irq().irq_inte.modify(|_, w| w.sm3().clear_bit()),
            _ => panic!("invalid state machine interrupt number"),
        }
    }

    /// Force state machine interrupt.
    ///
    /// Note that this doesn't affect the state seen by the state machine. For that, see [`PIO::force_irq`].
    ///
    /// See [`Self::enable_sm_interrupt`] for info about the index.
    pub fn force_sm_interrupt(&self, id: u8) {
        match id {
            0 => self.irq().irq_intf.modify(|_, w| w.sm0().set_bit()),
            1 => self.irq().irq_intf.modify(|_, w| w.sm1().set_bit()),
            2 => self.irq().irq_intf.modify(|_, w| w.sm2().set_bit()),
            3 => self.irq().irq_intf.modify(|_, w| w.sm3().set_bit()),
            _ => panic!("invalid state machine interrupt number"),
        }
    }

    /// Enable TX FIFO not full interrupt.
    ///
    /// Each of the 4 state machines have their own TX FIFO. This interrupt is raised when the TX FIFO is not full, i.e.
    /// one could push more data to it.
    pub fn enable_tx_not_full_interrupt(&self, id: u8) {
        match id {
            0 => self.irq().irq_inte.modify(|_, w| w.sm0_txnfull().set_bit()),
            1 => self.irq().irq_inte.modify(|_, w| w.sm1_txnfull().set_bit()),
            2 => self.irq().irq_inte.modify(|_, w| w.sm2_txnfull().set_bit()),
            3 => self.irq().irq_inte.modify(|_, w| w.sm3_txnfull().set_bit()),
            _ => panic!("invalid state machine interrupt number"),
        }
    }

    /// Disable TX FIFO not full interrupt.
    ///
    /// See [`Self::enable_tx_not_full_interrupt`] for info about the index.
    pub fn disable_tx_not_full_interrupt(&self, id: u8) {
        match id {
            0 => self
                .irq()
                .irq_inte
                .modify(|_, w| w.sm0_txnfull().clear_bit()),
            1 => self
                .irq()
                .irq_inte
                .modify(|_, w| w.sm1_txnfull().clear_bit()),
            2 => self
                .irq()
                .irq_inte
                .modify(|_, w| w.sm2_txnfull().clear_bit()),
            3 => self
                .irq()
                .irq_inte
                .modify(|_, w| w.sm3_txnfull().clear_bit()),
            _ => panic!("invalid state machine interrupt number"),
        }
    }

    /// Force TX FIFO not full interrupt.
    ///
    /// See [`Self::enable_tx_not_full_interrupt`] for info about the index.
    pub fn force_tx_not_full_interrupt(&self, id: u8) {
        match id {
            0 => self.irq().irq_intf.modify(|_, w| w.sm0_txnfull().set_bit()),
            1 => self.irq().irq_intf.modify(|_, w| w.sm1_txnfull().set_bit()),
            2 => self.irq().irq_intf.modify(|_, w| w.sm2_txnfull().set_bit()),
            3 => self.irq().irq_intf.modify(|_, w| w.sm3_txnfull().set_bit()),
            _ => panic!("invalid state machine interrupt number"),
        }
    }

    /// Enable RX FIFO not empty interrupt.
    ///
    /// Each of the 4 state machines have their own RX FIFO. This interrupt is raised when the RX FIFO is not empty,
    /// i.e. one could read more data from it.
    pub fn enable_rx_not_empty_interrupt(&self, id: u8) {
        match id {
            0 => self
                .irq()
                .irq_inte
                .modify(|_, w| w.sm0_rxnempty().set_bit()),
            1 => self
                .irq()
                .irq_inte
                .modify(|_, w| w.sm1_rxnempty().set_bit()),
            2 => self
                .irq()
                .irq_inte
                .modify(|_, w| w.sm2_rxnempty().set_bit()),
            3 => self
                .irq()
                .irq_inte
                .modify(|_, w| w.sm3_rxnempty().set_bit()),
            _ => panic!("invalid state machine interrupt number"),
        }
    }

    /// Disable RX FIFO not empty interrupt.
    ///
    /// See [`Self::enable_rx_not_empty_interrupt`] for info about the index.
    pub fn disable_rx_not_empty_interrupt(&self, id: u8) {
        match id {
            0 => self
                .irq()
                .irq_inte
                .modify(|_, w| w.sm0_rxnempty().clear_bit()),
            1 => self
                .irq()
                .irq_inte
                .modify(|_, w| w.sm1_rxnempty().clear_bit()),
            2 => self
                .irq()
                .irq_inte
                .modify(|_, w| w.sm2_rxnempty().clear_bit()),
            3 => self
                .irq()
                .irq_inte
                .modify(|_, w| w.sm3_rxnempty().clear_bit()),
            _ => panic!("invalid state machine interrupt number"),
        }
    }

    /// Force RX FIFO not empty interrupt.
    ///
    /// See [`Self::enable_rx_not_empty_interrupt`] for info about the index.
    pub fn force_rx_not_empty_interrupt(&self, id: u8) {
        match id {
            0 => self
                .irq()
                .irq_intf
                .modify(|_, w| w.sm0_rxnempty().set_bit()),
            1 => self
                .irq()
                .irq_intf
                .modify(|_, w| w.sm1_rxnempty().set_bit()),
            2 => self
                .irq()
                .irq_intf
                .modify(|_, w| w.sm2_rxnempty().set_bit()),
            3 => self
                .irq()
                .irq_intf
                .modify(|_, w| w.sm3_rxnempty().set_bit()),
            _ => panic!("invalid state machine interrupt number"),
        }
    }

    /// Get the raw interrupt state.
    ///
    /// This is the state of the interrupts without interrupt masking and forcing.
    pub fn raw(&self) -> InterruptState {
        InterruptState(self.block().intr.read().bits())
    }

    /// Get the interrupt state.
    ///
    /// This is the state of the interrupts after interrupt masking and forcing.
    pub fn state(&self) -> InterruptState {
        InterruptState(self.irq().irq_ints.read().bits())
    }

    fn block(&self) -> &rp2040_pac::pio0::RegisterBlock {
        unsafe { &*self.block }
    }

    fn irq(&self) -> &rp2040_pac::pio0::SM_IRQ {
        &self.block().sm_irq[self.id as usize]
    }
}

/// Provides easy access for decoding PIO's interrupt state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct InterruptState(u32);

macro_rules! raw_interrupt_accessor {
    ($name:ident, $doc:literal, $idx:expr) => {
        #[doc = concat!("Check whether interrupt ", $doc, " has been raised.")]
        pub fn $name(self) -> bool {
            self.0 & (1 << $idx) != 0
        }
    };
}
impl InterruptState {
    raw_interrupt_accessor!(sm0_rx_not_empty, "SM0_RXNEMPTY", 0);
    raw_interrupt_accessor!(sm1_rx_not_empty, "SM1_RXNEMPTY", 1);
    raw_interrupt_accessor!(sm2_rx_not_empty, "SM2_RXNEMPTY", 2);
    raw_interrupt_accessor!(sm3_rx_not_empty, "SM3_RXNEMPTY", 3);

    raw_interrupt_accessor!(sm0_tx_not_full, "SM0_TXNFULL", 4);
    raw_interrupt_accessor!(sm1_tx_not_full, "SM1_TXNFULL", 5);
    raw_interrupt_accessor!(sm2_tx_not_full, "SM2_TXNFULL", 6);
    raw_interrupt_accessor!(sm3_tx_not_full, "SM3_TXNFULL", 7);

    raw_interrupt_accessor!(sm0, "SM0", 8);
    raw_interrupt_accessor!(sm1, "SM1", 9);
    raw_interrupt_accessor!(sm2, "SM2", 10);
    raw_interrupt_accessor!(sm3, "SM3", 11);
}
