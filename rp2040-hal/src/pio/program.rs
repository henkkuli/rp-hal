use super::PIOExt;
use pio::{SideSet, Wrap};

/// Handle to a program that was placed in the PIO's instruction memory.
///
/// Objects of this type can be reused for multiple state machines of the same PIO block to save
/// memory if multiple state machines are supposed to perform the same function (for example, if
/// one PIO block is used to implement multiple I2C busses).
///
/// `PIO::uninstall(program)` can be used to free the space occupied by the program once it is no
/// longer used.
///
/// # Examples
///
/// ```no_run
/// use rp2040_hal::{pac, pio::PIOBuilder, pio::PIOExt};
/// let mut peripherals = pac::Peripherals::take().unwrap();
/// let (mut pio, sm0, _, _, _) = peripherals.PIO0.split(&mut peripherals.RESETS);
/// // Install a program in instruction memory.
/// let program = pio_proc::pio!(
///     32,
///     ".wrap_target
///     set pins, 1 [31]
///     set pins, 0 [31]
/// .wrap
///     "
/// ).program;
/// let installed = pio.install(&program).unwrap();
/// // Configure a state machine to use the program.
/// let (sm, rx, tx) = PIOBuilder::from_program(installed).build(sm0);
/// // Uninitialize the state machine again, freeing the program.
/// let (sm, installed) = sm.uninit(rx, tx);
/// // Uninstall the program to free instruction memory.
/// pio.uninstall(installed);
/// ```
///
/// # Safety
///
/// Objects of this type can outlive their `PIO` object. If the PIO block is reinitialized, the API
/// does not prevent the user from calling `uninstall()` when the PIO block does not actually hold
/// the program anymore. The user must therefore make sure that `uninstall()` is only called on the
/// PIO object which was used to install the program.
///
/// ```ignore
/// let (mut pio, sm0, sm1, sm2, sm3) = pac.PIO0.split(&mut pac.RESETS);
/// // Install a program in instruction memory.
/// let installed = pio.install(&program).unwrap();
/// // Reinitialize PIO.
/// let pio0 = pio.free(sm0, sm1, sm2, sm3);
/// let (mut pio, _, _, _, _) = pio0.split(&mut pac.RESETS);
/// // Do not do the following, the program is not in instruction memory anymore!
/// pio.uninstall(installed);
/// ```
#[derive(Debug)]
pub struct InstalledProgram<P> {
    pub(super) offset: u8,
    pub(super) length: u8,
    pub(super) side_set: SideSet,
    pub(super) wrap: Wrap,
    pub(super) _phantom: core::marker::PhantomData<P>,
}

impl<P: PIOExt> InstalledProgram<P> {
    /// Clones this program handle so that it can be executed by two state machines at the same
    /// time.
    ///
    /// # Safety
    ///
    /// This function is marked as unsafe because, once this function has been called, the
    /// resulting handle can be used to call `PIO::uninstall()` while the program is still running.
    ///
    /// The user has to make sure to call `PIO::uninstall()` only once and only after all state
    /// machines using the program have been uninitialized.
    pub unsafe fn share(&self) -> InstalledProgram<P> {
        InstalledProgram {
            offset: self.offset,
            length: self.length,
            side_set: self.side_set,
            wrap: self.wrap,
            _phantom: core::marker::PhantomData,
        }
    }
}
