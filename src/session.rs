//! Shared hardware ownership and cancellation-safe lifecycle transitions.

use crate::{
    config::{Config, SampleMode},
    device::DeviceInfo,
    errors::{Error, Result},
    maybe_future::{Either, MaybeFutureExt, ready},
    protocol::{self, Command, OutRequest, ReceiveSettings, ReceiverMode, Sequence},
    usb::{ControlBackend, NusbTransport},
};
use nusb::MaybeFuture;
use std::sync::{Arc, Mutex, MutexGuard, OnceLock, PoisonError};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum DeviceLifecycle {
    Open,
    Closing,
    Closed,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ReceiverState {
    Stopped,
    CleanupRequired,
    Running,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum ClaimAvailability {
    Available,
    Held,
    CleanupRequired,
}

#[derive(Debug)]
pub(crate) struct SharedDeviceState {
    pub(crate) device: DeviceLifecycle,
    pub(crate) claim: ClaimAvailability,
}

impl Default for SharedDeviceState {
    fn default() -> Self {
        Self {
            device: DeviceLifecycle::Open,
            claim: ClaimAvailability::Available,
        }
    }
}

/// Hardware lifetime and the exclusive receiver claim are shared by all owners.
#[derive(Debug)]
pub(crate) struct Session<C: ControlBackend + 'static = NusbTransport> {
    pub(crate) control: Arc<C>,
    pub(crate) info: OnceLock<DeviceInfo>,
    state: Mutex<SharedDeviceState>,
    receive: Mutex<ReceiveSettings>,
}

impl<C: ControlBackend + 'static> Session<C> {
    pub(crate) fn lock(&self) -> MutexGuard<'_, SharedDeviceState> {
        self.state.lock().unwrap_or_else(PoisonError::into_inner)
    }

    pub(crate) fn ensure_open(&self) -> Result<()> {
        if self.lock().device == DeviceLifecycle::Open {
            Ok(())
        } else {
            Err(Error::DeviceClosed)
        }
    }

    fn begin_shutdown(&self) -> Result<bool> {
        let mut state = self.lock();
        match state.device {
            DeviceLifecycle::Closed => Ok(false),
            DeviceLifecycle::Closing => Ok(true),
            DeviceLifecycle::Open => {
                if state.claim == ClaimAvailability::Held {
                    return Err(Error::Busy);
                }
                state.device = DeviceLifecycle::Closing;
                Ok(true)
            }
        }
    }

    pub(crate) fn shutdown(self: &Arc<Self>) -> impl MaybeFuture<Output = Result<()>> + use<C> {
        let shared = Arc::clone(self);
        ready(()).map(move |()| shared.begin_shutdown()).and_then({
            let shared = Arc::clone(self);
            move |run| {
                if run {
                    Either::left(
                        shutdown_hardware(shared.control.as_ref()).map(move |result| {
                            if result.is_ok() {
                                let mut state = shared.lock();
                                state.device = DeviceLifecycle::Closed;
                                state.claim = ClaimAvailability::Available;
                            }
                            result
                        }),
                    )
                } else {
                    Either::right(ready(Ok(())))
                }
            }
        })
    }
}

impl<C: ControlBackend + 'static> Drop for Session<C> {
    fn drop(&mut self) {
        if self.lock().device == DeviceLifecycle::Closed {
            return;
        }
        let cleanup = shutdown_hardware(self.control.as_ref());
        #[cfg(not(target_arch = "wasm32"))]
        let _ = cleanup.wait();
        #[cfg(target_arch = "wasm32")]
        wasm_bindgen_futures::spawn_local(async move {
            let _ = cleanup.await;
        });
    }
}

impl<C: ControlBackend + 'static> Session<C> {
    pub(crate) fn new(control: C) -> Arc<Self> {
        Arc::new(Self {
            control: Arc::new(control),
            info: OnceLock::new(),
            state: Mutex::new(SharedDeviceState::default()),
            receive: Mutex::new(ReceiveSettings {
                packing: false,
                decimation: 1,
            }),
        })
    }

    pub(crate) fn receive_settings(&self) -> ReceiveSettings {
        *self.receive.lock().unwrap_or_else(PoisonError::into_inner)
    }

    pub(crate) fn apply(
        self: &Arc<Self>,
        commands: Vec<Command>,
        receive: Option<ReceiveSettings>,
    ) -> impl MaybeFuture<Output = Result<()>> + use<C> {
        let shared = Arc::clone(self);
        ready(()).continue_with(move |_| {
            let validation = shared.ensure_open();
            ready(validation).and_then(move |_| {
                Sequence::new(Arc::clone(&shared.control), commands).map(move |result| {
                    result?;
                    if let Some(receive) = receive {
                        *shared
                            .receive
                            .lock()
                            .unwrap_or_else(PoisonError::into_inner) = receive;
                    }
                    Ok(())
                })
            })
        })
    }

    pub(crate) fn configure<M: SampleMode>(
        self: &Arc<Self>,
        config: &Config<M>,
    ) -> impl MaybeFuture<Output = Result<()>> + use<C, M> {
        let (commands, receive) = protocol::configuration(config);
        self.apply(commands, Some(receive))
    }

    /// Arm final-owner cleanup before the first metadata or configuration request.
    pub(crate) fn initialize<M: SampleMode>(
        control: C,
        config: Config<M>,
    ) -> impl MaybeFuture<Output = Result<Arc<Self>>> + use<C, M> {
        let shared = Self::new(control);
        let metadata = protocol::device_info(shared.control.as_ref())
            .map_err(|e| e.at("reading HydraSDR device metadata"));
        let setup = metadata.and_then({
            let shared = Arc::clone(&shared);
            move |info| {
                shared.info.set(info).expect("metadata initialized once");
                shared
                    .configure(&config)
                    .map_err(|e| e.at("applying initial HydraSDR configuration"))
            }
        });
        setup.continue_with(move |result| match result {
            Ok(()) => Either::left(ready(Ok(shared))),
            Err(error) => Either::right(shared.shutdown().map(move |_| {
                drop(shared);
                Err(error)
            })),
        })
    }
}

pub(crate) fn shutdown_hardware<C: ControlBackend>(
    control: &C,
) -> impl MaybeFuture<Output = Result<()>> + use<C> {
    let receiver_off = control
        .control_out(OutRequest::receiver_mode(ReceiverMode::Off))
        .map_err(|error| error.at("turning off the receiver during shutdown"));
    let bias_off = control
        .control_out(OutRequest::set_rf_bias(false))
        .map_err(|error| error.at("turning off RF bias power during shutdown"));

    receiver_off
        .map(Ok::<_, Error>)
        .and_then(move |receiver_result| {
            bias_off.map(move |bias_result| receiver_result.and(bias_result))
        })
}

/// Keeps both the device and its exclusive stream claim alive through cleanup.
#[derive(Debug)]
pub(crate) struct RxStreamClaim<C: ControlBackend + 'static> {
    pub(crate) shared: Arc<Session<C>>,
    receiver: ReceiverState,
    cleanup_armed: bool,
}

impl<C: ControlBackend + 'static> RxStreamClaim<C> {
    pub(crate) fn receiver_state(&self) -> ReceiverState {
        self.receiver
    }

    pub(crate) fn set_receiver_state(&mut self, receiver: ReceiverState) {
        self.receiver = receiver;
    }

    pub(crate) fn begin_start(&mut self) -> Result<bool> {
        match self.receiver {
            ReceiverState::Running => Ok(false),
            ReceiverState::CleanupRequired => Err(Error::Busy),
            ReceiverState::Stopped => {
                self.receiver = ReceiverState::CleanupRequired;
                Ok(true)
            }
        }
    }

    pub(crate) fn begin_stop(&mut self) -> bool {
        if self.receiver == ReceiverState::Stopped {
            return false;
        }
        // Cancellation or failure must not leave a stream apparently running.
        self.receiver = ReceiverState::CleanupRequired;
        true
    }

    pub(crate) fn acquire(shared: &Arc<Session<C>>) -> Result<Self> {
        let mut state = shared.lock();
        if state.device != DeviceLifecycle::Open {
            return Err(Error::DeviceClosed);
        }
        if state.claim != ClaimAvailability::Available {
            return Err(Error::Busy);
        }
        state.claim = ClaimAvailability::Held;
        Ok(Self {
            shared: Arc::clone(shared),
            receiver: ReceiverState::Stopped,
            cleanup_armed: true,
        })
    }

    fn release(&mut self) {
        self.shared.lock().claim = ClaimAvailability::Available;
        self.cleanup_armed = false;
    }

    pub(crate) fn close(&mut self) -> impl MaybeFuture<Output = Result<()>> + '_ {
        ready(()).continue_with(move |()| {
            let operation = if self.begin_stop() {
                Either::left(
                    self.shared
                        .control
                        .control_out(OutRequest::receiver_mode(ReceiverMode::Off)),
                )
            } else {
                Either::right(ready(Ok(())))
            };
            operation.map(move |result| {
                if result.is_ok() {
                    self.receiver = ReceiverState::Stopped;
                    self.release();
                }
                result
            })
        })
    }

    // The owned operation keeps the claim reserved even before a WebUSB
    // background task is polled. Drop runs it to completion best-effort.
    pub(crate) fn cleanup(&mut self) -> impl MaybeFuture<Output = ()> + use<C> {
        self.cleanup_armed = false;
        let shared = Arc::clone(&self.shared);
        let operation = if self.begin_stop() {
            Either::left(
                shared
                    .control
                    .control_out(OutRequest::receiver_mode(ReceiverMode::Off)),
            )
        } else {
            Either::right(ready(Ok(())))
        };
        operation.map(move |result| {
            let mut state = shared.lock();
            state.claim = if result.is_ok() {
                ClaimAvailability::Available
            } else {
                ClaimAvailability::CleanupRequired
            };
        })
    }
}

impl<C: ControlBackend + 'static> Drop for RxStreamClaim<C> {
    fn drop(&mut self) {
        if !self.cleanup_armed {
            return;
        }
        let cleanup = self.cleanup();
        #[cfg(not(target_arch = "wasm32"))]
        cleanup.wait();
        #[cfg(target_arch = "wasm32")]
        wasm_bindgen_futures::spawn_local(async move {
            cleanup.await;
        });
    }
}
