//! Lifecycle regression tests using the shared typed receiver.
use super::{RxStream, StreamingStats, receiver::Receiver};
use crate::{
    Complex32, Config,
    config::{F32Iq, RawAdc, SampleMode},
    errors::{Error, Result},
    maybe_future::Either,
    protocol::{InRequest, OutRequest, ReceiveSettings, ReceiverMode},
    session::{ClaimAvailability, DeviceLifecycle, ReceiverState, RxStreamClaim, Session},
    test_support::{FakeEvent, FakeState, FakeTransport},
};
use futures_lite::future::block_on;
use nusb::MaybeFuture;
use std::{
    future::IntoFuture,
    sync::{Arc, atomic::Ordering},
    time::Duration,
};
fn fake_device(control: FakeTransport) -> Arc<Session<FakeTransport>> {
    Session::new(control)
}
fn owned_stream_claim(device: Arc<Session<FakeTransport>>) -> RxStreamClaim<FakeTransport> {
    RxStreamClaim::acquire(&device).unwrap()
}
fn requests(state: &FakeState) -> Vec<OutRequest> {
    state.control_out_requests.lock().unwrap().clone()
}
enum TestReceiver {
    Raw(Box<Receiver<FakeTransport, RawAdc>>),
    Iq(Box<Receiver<FakeTransport, F32Iq>>),
}
impl TestReceiver {
    fn close(self) -> impl MaybeFuture<Output = Result<StreamingStats>> {
        match self {
            Self::Raw(rx) => Either::left(rx.close()),
            Self::Iq(rx) => Either::right(rx.close()),
        }
    }
    fn stop(&mut self) -> impl MaybeFuture<Output = Result<StreamingStats>> + '_ {
        match self {
            Self::Raw(rx) => Either::left(rx.stop()),
            Self::Iq(rx) => Either::right(rx.stop()),
        }
    }
}
#[test]
fn dormant_claim_keeps_device_alive_until_its_drop() {
    let control = FakeTransport::default();
    let state = Arc::clone(&control.state);
    let device = fake_device(control);
    let owner = Arc::downgrade(&device);
    let claim = RxStreamClaim::acquire(&device).unwrap();
    assert!(matches!(RxStreamClaim::acquire(&device), Err(Error::Busy)));

    drop(device);
    assert!(owner.upgrade().is_some());
    assert!(requests(&state).is_empty());

    drop(claim);
    assert!(owner.upgrade().is_none());
    assert_eq!(
        requests(&state),
        [
            OutRequest::receiver_mode(ReceiverMode::Off),
            OutRequest::set_rf_bias(false),
        ]
    );
}

#[test]
fn dropping_a_dormant_claim_allows_a_replacement_without_usb_commands() {
    let control = FakeTransport::default();
    let state = Arc::clone(&control.state);
    let device = fake_device(control);
    drop(RxStreamClaim::acquire(&device).unwrap());
    let _replacement = RxStreamClaim::acquire(&device).unwrap();
    assert!(requests(&state).is_empty());
}

#[test]
fn async_stream_can_start_read_and_restart_after_device_drop() {
    block_on(async {
        // Drop the public owner before starting, while running, or after stop.
        for phase in 0..3 {
            let control = FakeTransport::default();
            let state = Arc::clone(&control.state);
            let device = fake_device(control);
            device
                .apply(vec![OutRequest::set_rf_bias(true).into()], None)
                .await
                .unwrap();
            let owner = Arc::downgrade(&device);
            let mut stream = Receiver::<_, F32Iq>::new(RxStreamClaim::acquire(&device).unwrap());
            if phase >= 1 {
                stream.start().await.unwrap();
            }
            if phase >= 2 {
                stream.stop().await.unwrap();
            }
            let before_drop = requests(&state);
            drop(device);
            assert_eq!(requests(&state), before_drop);

            let mut sample = [Complex32::default(); 1];
            for _ in 0..2 {
                stream.start().await.unwrap();
                assert_eq!(stream.read(&mut sample, None).await.unwrap(), 1);
                stream.stop().await.unwrap();
            }
            assert!(owner.upgrade().is_some());
            assert!(!requests(&state).contains(&OutRequest::set_rf_bias(false)));
            drop(stream);
            assert!(owner.upgrade().is_none());
            assert!(requests(&state).ends_with(&[
                OutRequest::receiver_mode(ReceiverMode::Off),
                OutRequest::set_rf_bias(false),
            ]));
        }
    });
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn blocking_raw_stream_can_restart_after_device_drop() {
    for phase in 0..3 {
        let control = FakeTransport::default();
        let state = Arc::clone(&control.state);
        let device = fake_device(control);
        let mut stream = Receiver::<_, RawAdc>::new(RxStreamClaim::acquire(&device).unwrap());
        if phase >= 1 {
            stream.start().wait().unwrap();
        }
        if phase >= 2 {
            stream.stop().wait().unwrap();
        }
        let before_drop = requests(&state);
        drop(device);
        assert_eq!(requests(&state), before_drop);
        for _ in 0..2 {
            stream.start().wait().unwrap();
            assert!(stream.next_block(None).wait().unwrap().is_some());
            stream.stop().wait().unwrap();
        }
        assert!(!requests(&state).contains(&OutRequest::set_rf_bias(false)));
        stream.start().wait().unwrap();
        drop(stream);
        assert!(state.events.lock().unwrap().ends_with(&[
            FakeEvent::QueueDropped,
            FakeEvent::Control(OutRequest::receiver_mode(ReceiverMode::Off)),
            FakeEvent::Control(OutRequest::receiver_mode(ReceiverMode::Off)),
            FakeEvent::Control(OutRequest::set_rf_bias(false)),
        ]));
        assert_eq!(
            requests(&state).last(),
            Some(&OutRequest::set_rf_bias(false))
        );
    }
}

#[test]
fn dropping_an_active_stream_allows_a_replacement_and_leaves_device_open() {
    block_on(async {
        let control = FakeTransport::default();
        let state = Arc::clone(&control.state);
        let device = fake_device(control);
        let mut stream = Receiver::<_, RawAdc>::new(RxStreamClaim::acquire(&device).unwrap());
        stream.start().await.unwrap();
        drop(stream);
        assert!(state.events.lock().unwrap().ends_with(&[
            FakeEvent::QueueDropped,
            FakeEvent::Control(OutRequest::receiver_mode(ReceiverMode::Off)),
        ]));
        assert_eq!(state.cancel_count.load(Ordering::SeqCst), 1);
        assert_eq!(
            requests(&state),
            [
                OutRequest::receiver_mode(ReceiverMode::Off),
                OutRequest::receiver_mode(ReceiverMode::Rx),
                OutRequest::receiver_mode(ReceiverMode::Off),
            ]
        );
        device.ensure_open().unwrap();
        let mut replacement = Receiver::<_, RawAdc>::new(RxStreamClaim::acquire(&device).unwrap());
        replacement.start().await.unwrap();
        replacement.stop().await.unwrap();
        replacement.retire_stream();
        replacement.claim.close().await.unwrap();
        device.shutdown().await.unwrap();
        let after_shutdown = requests(&state);
        drop(device);
        drop(replacement);
        assert_eq!(requests(&state), after_shutdown);
    });
}

async fn close_test_stream(
    device: &Arc<Session<FakeTransport>>,
    mode: usize,
    phase: usize,
) -> TestReceiver {
    let claim = RxStreamClaim::acquire(device).unwrap();
    let mut rx = if mode.is_multiple_of(2) {
        TestReceiver::Raw(Box::new(Receiver::new(claim)))
    } else {
        TestReceiver::Iq(Box::new(Receiver::new(claim)))
    };
    if phase != 0 {
        match &mut rx {
            TestReceiver::Raw(inner) => {
                if mode < 2 {
                    inner.start().wait().unwrap();
                    inner
                        .next_block(Some(Duration::from_secs(1)))
                        .wait()
                        .unwrap();
                } else {
                    inner.start().await.unwrap();
                    inner.next_block(None).await.unwrap();
                }
            }
            TestReceiver::Iq(inner) => {
                let mut out = [Complex32::default(); 1];
                if mode < 2 {
                    inner.start().wait().unwrap();
                    inner
                        .read(&mut out, Some(Duration::from_secs(1)))
                        .wait()
                        .unwrap();
                } else {
                    inner.start().await.unwrap();
                    inner.read(&mut out, None).await.unwrap();
                }
            }
        }
        if phase == 2 {
            rx.stop().await.unwrap();
        }
    }
    rx
}

#[test]
fn close_all_modes_releases_queue_claim_and_owner_and_returns_statistics() {
    block_on(async {
        for mode in 0..4 {
            for phase in 0..3 {
                for wait in [false, true] {
                    let control = FakeTransport::default();
                    let state = Arc::clone(&control.state);
                    let device = fake_device(control);
                    let rx = close_test_stream(&device, mode, phase).await;
                    assert!(matches!(device.shutdown().await, Err(Error::Busy)));
                    let before = requests(&state);
                    let operation = rx.close();
                    assert_eq!(requests(&state), before);
                    let mut completed = None;
                    let stats = if wait {
                        operation.wait().unwrap()
                    } else {
                        let mut future = Box::pin(operation.into_future());
                        let stats = future.as_mut().await.unwrap();
                        completed = Some(future);
                        stats
                    };
                    assert_eq!(Arc::strong_count(&device), 1);
                    assert_eq!(
                        requests(&state).len(),
                        before.len() + usize::from(phase == 1)
                    );
                    if phase == 0 {
                        assert_eq!(stats, StreamingStats::default());
                    } else {
                        assert!(stats.buffers_received > 0);
                        assert_eq!(
                            state
                                .events
                                .lock()
                                .unwrap()
                                .iter()
                                .filter(|event| **event == FakeEvent::QueueDropped)
                                .count(),
                            1
                        );
                    }
                    let after = requests(&state);
                    let replacement = RxStreamClaim::acquire(&device).unwrap();
                    // Even a retained completed future has released its ownership.
                    drop(completed);
                    assert!(matches!(RxStreamClaim::acquire(&device), Err(Error::Busy)));
                    assert_eq!(requests(&state), after);
                    drop(replacement);
                    device.shutdown().await.unwrap();
                }
            }
        }
    });
}

#[test]
fn dropping_unpolled_close_operation_or_future_cleans_up_its_owned_stream() {
    block_on(async {
        for mode in 0..4 {
            for phase in 0..3 {
                for into_future in [false, true] {
                    let control = FakeTransport::default();
                    let state = Arc::clone(&control.state);
                    let device = fake_device(control);
                    let rx = close_test_stream(&device, mode, phase).await;
                    let before = state.events.lock().unwrap().clone();
                    let operation = rx.close();
                    assert_eq!(*state.events.lock().unwrap(), before);
                    assert!(matches!(device.shutdown().await, Err(Error::Busy)));
                    if into_future {
                        let future = operation.into_future();
                        assert_eq!(*state.events.lock().unwrap(), before);
                        assert!(matches!(device.shutdown().await, Err(Error::Busy)));
                        drop(future);
                    } else {
                        drop(operation);
                    }
                    assert_eq!(Arc::strong_count(&device), 1);
                    if phase == 1 {
                        assert!(state.events.lock().unwrap().ends_with(&[
                            FakeEvent::QueueDropped,
                            FakeEvent::Control(OutRequest::receiver_mode(ReceiverMode::Off)),
                        ]));
                    }
                    let replacement = RxStreamClaim::acquire(&device).unwrap();
                    drop(replacement);
                    device.shutdown().await.unwrap();
                }
            }
        }
    });
}

#[test]
fn canceled_close_retains_claim_until_drop_fallback_finishes() {
    block_on(async {
        for mode in 0..4 {
            for fail_fallback in [false, true] {
                let control = FakeTransport::default();
                let state = Arc::clone(&control.state);
                let device = fake_device(control);
                let rx = close_test_stream(&device, mode, 1).await;
                state
                    .pause_control_out_at
                    .store(requests(&state).len() + 1, Ordering::SeqCst);
                let mut close = Box::pin(rx.close().into_future());
                assert!(futures_lite::future::poll_once(&mut close).await.is_none());
                assert!(matches!(device.shutdown().await, Err(Error::Busy)));
                assert!(matches!(RxStreamClaim::acquire(&device), Err(Error::Busy)));
                assert!(state.events.lock().unwrap().ends_with(&[
                    FakeEvent::QueueDropped,
                    FakeEvent::Control(OutRequest::receiver_mode(ReceiverMode::Off)),
                ]));
                state
                    .fail_control_out
                    .store(fail_fallback, Ordering::SeqCst);
                drop(close);
                assert_eq!(Arc::strong_count(&device), 1);
                assert_eq!(
                    device.lock().claim,
                    if fail_fallback {
                        ClaimAvailability::CleanupRequired
                    } else {
                        ClaimAvailability::Available
                    }
                );
                if fail_fallback {
                    assert!(matches!(RxStreamClaim::acquire(&device), Err(Error::Busy)));
                } else {
                    drop(RxStreamClaim::acquire(&device).unwrap());
                }
                state.fail_control_out.store(false, Ordering::SeqCst);
                state.pause_control_out_at.store(0, Ordering::SeqCst);
                device.shutdown().await.unwrap();
            }
        }
    });
}

#[test]
fn failed_close_returns_original_error_and_runs_drop_fallback() {
    block_on(async {
        for mode in 0..4 {
            for wait in [false, true] {
                for fail_fallback in [false, true] {
                    let control = FakeTransport::default();
                    let state = Arc::clone(&control.state);
                    let device = fake_device(control);
                    let rx = close_test_stream(&device, mode, 1).await;
                    let before = requests(&state).len();
                    state
                        .fail_control_out_at
                        .store(before + 1, Ordering::SeqCst);
                    state
                        .fail_control_out
                        .store(fail_fallback, Ordering::SeqCst);
                    let operation = rx.close();
                    let error = if wait {
                        operation.wait().unwrap_err()
                    } else {
                        operation.await.unwrap_err()
                    };
                    assert!(matches!(
                        error,
                        Error::Transfer(nusb::transfer::TransferError::Fault)
                    ));
                    assert_eq!(requests(&state).len(), before + 2);
                    assert_eq!(Arc::strong_count(&device), 1);
                    assert_eq!(
                        device.lock().claim,
                        if fail_fallback {
                            ClaimAvailability::CleanupRequired
                        } else {
                            ClaimAvailability::Available
                        }
                    );
                    state.fail_control_out.store(false, Ordering::SeqCst);
                    device.shutdown().await.unwrap();
                }
            }
        }
    });
}

#[test]
fn close_after_failed_stop_and_device_drop_completes_final_owner_cleanup() {
    block_on(async {
        for mode in 0..4 {
            let control = FakeTransport::default();
            let state = Arc::clone(&control.state);
            let device = fake_device(control);
            let owner = Arc::downgrade(&device);
            let mut rx = close_test_stream(&device, mode, 1).await;
            state.fail_control_out.store(true, Ordering::SeqCst);
            let result = rx.stop().await;
            assert!(result.is_err());
            state.fail_control_out.store(false, Ordering::SeqCst);
            drop(device);
            assert!(owner.upgrade().is_some());
            let stats = rx.close().await.unwrap();
            assert!(stats.buffers_received > 0);
            assert!(owner.upgrade().is_none());
            assert!(requests(&state).ends_with(&[
                OutRequest::receiver_mode(ReceiverMode::Off),
                OutRequest::receiver_mode(ReceiverMode::Off),
                OutRequest::set_rf_bias(false),
            ]));
            assert_eq!(Arc::strong_count(&state), 1);
        }
    });
}

#[test]
fn public_close_operation_and_future_are_send_and_static() {
    fn assert_owned<T: Send + 'static>(_: &T) {}
    fn check<M: SampleMode>(rx: RxStream<M>) {
        let operation = rx.close();
        assert_owned(&operation);
        assert_owned(&operation.into_future());
    }
    let _ = check::<RawAdc>;
    let _ = check::<F32Iq>;
}

#[test]
fn shutdown_requires_release_even_for_dormant_and_stopped_streams() {
    block_on(async {
        for started in [false, true] {
            let control = FakeTransport::default();
            let state = Arc::clone(&control.state);
            let device = fake_device(control);
            let mut stream = Receiver::<_, F32Iq>::new(RxStreamClaim::acquire(&device).unwrap());
            let before = requests(&state);
            drop(device.shutdown());
            device.ensure_open().unwrap();
            assert_eq!(requests(&state), before);
            assert!(matches!(device.shutdown().await, Err(Error::Busy)));
            if started {
                stream.start().await.unwrap();
                assert!(matches!(device.shutdown().await, Err(Error::Busy)));
                stream.stop().await.unwrap();
            }
            assert!(matches!(device.shutdown().await, Err(Error::Busy)));
            stream.retire_stream();
            stream.claim.close().await.unwrap();
            device.shutdown().await.unwrap();
            device.shutdown().await.unwrap();
            let after = requests(&state);
            drop(stream);
            drop(device);
            assert_eq!(requests(&state), after);
        }
    });
}

#[test]
fn failed_shutdown_remains_closing_and_can_be_retried() {
    block_on(async {
        for failed_command in [1, 2] {
            let control = FakeTransport::default();
            let state = Arc::clone(&control.state);
            state
                .fail_control_out_at
                .store(failed_command, Ordering::SeqCst);
            let device = fake_device(control);
            assert!(device.shutdown().await.is_err());
            assert_eq!(device.lock().device, DeviceLifecycle::Closing);
            assert_eq!(requests(&state).len(), 2);
            assert!(matches!(
                RxStreamClaim::acquire(&device),
                Err(Error::DeviceClosed)
            ));
            device.shutdown().await.unwrap();
            assert_eq!(device.lock().device, DeviceLifecycle::Closed);
            drop(device);
            assert_eq!(requests(&state).len(), 4);
        }
    });
}

#[test]
fn failed_stream_drop_cleanup_blocks_replacement_until_device_shutdown() {
    block_on(async {
        let control = FakeTransport::default();
        let state = Arc::clone(&control.state);
        let device = fake_device(control);
        let mut stream = Receiver::<_, F32Iq>::new(RxStreamClaim::acquire(&device).unwrap());
        stream.start().await.unwrap();
        state.fail_control_out_at.store(3, Ordering::SeqCst);
        drop(stream);
        // Only the claim tries receiver cleanup; there is no inner retry.
        assert_eq!(requests(&state).len(), 3);
        assert_eq!(state.cancel_count.load(Ordering::SeqCst), 1);
        assert_ne!(device.lock().claim, ClaimAvailability::Held);
        assert_eq!(device.lock().claim, ClaimAvailability::CleanupRequired);
        assert!(matches!(RxStreamClaim::acquire(&device), Err(Error::Busy)));
        device.shutdown().await.unwrap();
        assert_eq!(requests(&state).len(), 5);
        drop(device);
        assert_eq!(requests(&state).len(), 5);
    });
}

#[test]
fn cancelled_async_stop_requires_cleanup_before_read_restart_or_shutdown() {
    block_on(async {
        let control = FakeTransport::default();
        let state = Arc::clone(&control.state);
        let device = fake_device(control);
        let mut stream = Receiver::<_, F32Iq>::new(RxStreamClaim::acquire(&device).unwrap());
        stream.start().await.unwrap();
        state.pause_control_out_at.store(3, Ordering::SeqCst);
        let mut stop = Box::pin(stream.stop().into_future());
        assert!(futures_lite::future::poll_once(&mut stop).await.is_none());
        drop(stop);
        let mut sample = [Complex32::default(); 1];
        assert!(stream.read(&mut sample, None).await.is_err());
        assert!(matches!(stream.start().await, Err(Error::Busy)));
        assert!(matches!(device.shutdown().await, Err(Error::Busy)));
        assert_eq!(requests(&state).len(), 3);
        stream.stop().await.unwrap();
        stream.start().await.unwrap();
        assert_eq!(stream.read(&mut sample, None).await.unwrap(), 1);
        stream.stop().await.unwrap();
        drop(stream);
        device.shutdown().await.unwrap();
    });
}

#[test]
fn device_drop_during_async_start_does_not_interrupt_or_shut_down_the_stream() {
    block_on(async {
        // Pause during initial receiver-off, receiver-on, or queue setup.
        for phase in 1..=3 {
            let control = FakeTransport::default();
            let state = Arc::clone(&control.state);
            let device = fake_device(control);
            let owner = Arc::downgrade(&device);
            let mut stream = Receiver::<_, F32Iq>::new(RxStreamClaim::acquire(&device).unwrap());
            if phase < 3 {
                state.pause_control_out_at.store(phase, Ordering::SeqCst);
            } else {
                state.pause_clear_halt.store(true, Ordering::SeqCst);
            }
            let mut start = Box::pin(stream.start().into_future());
            assert!(futures_lite::future::poll_once(&mut start).await.is_none());
            let before_drop = requests(&state);
            drop(device);
            assert!(owner.upgrade().is_some());
            assert_eq!(requests(&state), before_drop);
            drop(start);
            assert!(matches!(stream.start().await, Err(Error::Busy)));
            stream.stop().await.unwrap();
            state.pause_control_out_at.store(0, Ordering::SeqCst);
            state.pause_clear_halt.store(false, Ordering::SeqCst);
            stream.start().await.unwrap();
            let mut sample = [Complex32::default(); 1];
            assert_eq!(stream.read(&mut sample, None).await.unwrap(), 1);
            drop(stream);
            assert!(owner.upgrade().is_none());
            assert!(state.events.lock().unwrap().ends_with(&[
                FakeEvent::QueueDropped,
                FakeEvent::Control(OutRequest::receiver_mode(ReceiverMode::Off)),
                FakeEvent::Control(OutRequest::receiver_mode(ReceiverMode::Off)),
                FakeEvent::Control(OutRequest::set_rf_bias(false)),
            ]));
        }
    });
}

#[test]
fn background_cleanup_reserves_the_claim_until_receiver_off_completes() {
    block_on(async {
        for fail in [false, true] {
            let control = FakeTransport::default();
            let state = Arc::clone(&control.state);
            let device = fake_device(control);
            let mut stream = Receiver::<_, RawAdc>::new(RxStreamClaim::acquire(&device).unwrap());
            stream.start().await.unwrap();
            // Drive the same owned operation that Drop schedules on WebUSB.
            stream.retire_stream();
            state.pause_control_out_at.store(3, Ordering::SeqCst);
            let mut cleanup = Box::pin(stream.claim.cleanup().into_future());
            assert!(matches!(device.shutdown().await, Err(Error::Busy)));
            assert!(
                futures_lite::future::poll_once(&mut cleanup)
                    .await
                    .is_none()
            );
            assert!(matches!(RxStreamClaim::acquire(&device), Err(Error::Busy)));
            assert!(matches!(device.shutdown().await, Err(Error::Busy)));
            state.fail_control_out.store(fail, Ordering::SeqCst);
            state.pause_control_out_at.store(0, Ordering::SeqCst);
            cleanup.await;
            assert_ne!(device.lock().claim, ClaimAvailability::Held);
            assert_eq!(
                device.lock().claim,
                if fail {
                    ClaimAvailability::CleanupRequired
                } else {
                    ClaimAvailability::Available
                }
            );
            if fail {
                assert!(matches!(RxStreamClaim::acquire(&device), Err(Error::Busy)));
            }
            // Retire the test's source claim before acquiring another one.
            drop(stream);
            state.fail_control_out.store(false, Ordering::SeqCst);
            if !fail {
                drop(RxStreamClaim::acquire(&device).unwrap());
            }
            device.shutdown().await.unwrap();
        }
    });
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn explicit_shutdown_turns_off_receiver_and_bias_power() {
    let control = FakeTransport::default();
    let state = Arc::clone(&control.state);
    let device = fake_device(control);

    device.shutdown().wait().expect("explicit shutdown");

    assert_eq!(
        *state
            .control_out_requests
            .lock()
            .expect("control request lock"),
        [
            OutRequest::receiver_mode(ReceiverMode::Off),
            OutRequest::set_rf_bias(false),
        ]
    );
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn shutdown_attempts_bias_off_after_receiver_off_fails() {
    let control = FakeTransport::default();
    let state = Arc::clone(&control.state);
    state.fail_control_out_at.store(1, Ordering::SeqCst);
    let device = fake_device(control);

    let error = device
        .shutdown()
        .wait()
        .expect_err("receiver shutdown should fail");

    assert!(matches!(
        error,
        Error::Operation {
            operation: "turning off the receiver during shutdown",
            ..
        }
    ));
    assert_eq!(state.control_out_count.load(Ordering::SeqCst), 2);
    assert_eq!(
        *state
            .control_out_requests
            .lock()
            .expect("control request lock"),
        [
            OutRequest::receiver_mode(ReceiverMode::Off),
            OutRequest::set_rf_bias(false),
        ]
    );

    drop(device);
    assert_eq!(state.control_out_count.load(Ordering::SeqCst), 4);
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn initial_packing_failure_does_not_enable_bias_power() {
    let control = FakeTransport::default();
    let state = Arc::clone(&control.state);
    state.fail_packing.store(true, Ordering::SeqCst);
    let config = Config::builder().bias_tee(true).build().unwrap();
    assert!(Session::initialize(control, config).wait().is_err());
    assert!(!requests(&state).contains(&OutRequest::set_rf_bias(true)));
    assert!(requests(&state).ends_with(&[
        OutRequest::receiver_mode(ReceiverMode::Off),
        OutRequest::set_rf_bias(false)
    ]));
}

#[test]
fn failed_initial_configuration_runs_full_hardware_cleanup() {
    block_on(async {
        let control = FakeTransport::default();
        let state = Arc::clone(&control.state);
        state.fail_packing.store(true, Ordering::SeqCst);
        let config = Config::builder().bias_tee(true).build().unwrap();
        assert!(Session::initialize(control, config).await.is_err());
        assert_eq!(
            requests(&state),
            [
                OutRequest::set_frequency(100_000_000),
                OutRequest::receiver_mode(ReceiverMode::Off),
                OutRequest::set_rf_bias(false)
            ]
        );
    });
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn dropping_unpolled_initialization_runs_hardware_cleanup() {
    let control = FakeTransport::default();
    let state = Arc::clone(&control.state);
    let operation = Session::initialize(control, Config::default());
    assert!(requests(&state).is_empty());
    drop(operation);
    assert_eq!(
        requests(&state),
        [
            OutRequest::receiver_mode(ReceiverMode::Off),
            OutRequest::set_rf_bias(false)
        ]
    );
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn cancelling_initial_configuration_runs_hardware_cleanup() {
    let control = FakeTransport::default();
    let state = Arc::clone(&control.state);
    state.pause_control_out_at.store(1, Ordering::SeqCst);
    let mut operation = Box::pin(Session::initialize(control, Config::default()).into_future());
    assert!(block_on(futures_lite::future::poll_once(operation.as_mut())).is_none());
    drop(operation);
    assert_eq!(
        requests(&state),
        [
            OutRequest::set_frequency(100_000_000),
            OutRequest::receiver_mode(ReceiverMode::Off),
            OutRequest::set_rf_bias(false)
        ]
    );
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn targeted_bias_tee_control_sends_one_request() {
    let control = FakeTransport::default();
    let state = Arc::clone(&control.state);
    let device = fake_device(control);

    device
        .apply(vec![OutRequest::set_rf_bias(true).into()], None)
        .wait()
        .expect("enable bias tee");

    assert_eq!(
        *state
            .control_out_requests
            .lock()
            .expect("control request lock"),
        [OutRequest::set_rf_bias(true)]
    );
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn targeted_packing_control_updates_streaming_state() {
    let device = fake_device(FakeTransport::default());
    let settings = ReceiveSettings {
        packing: true,
        ..device.receive_settings()
    };
    device
        .apply(vec![InRequest::set_packing(true).into()], Some(settings))
        .wait()
        .unwrap();
    assert!(device.receive_settings().packing);
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn targeted_decimation_policy_reapplies_the_requested_rate() {
    let device = fake_device(FakeTransport::default());
    let config = Config::builder()
        .sample_rate_hz(5_000_000)
        .decimation_policy(crate::DecimationPolicy::HighDefinition)
        .build()
        .unwrap();
    device.configure(&config).wait().unwrap();
    assert_eq!(device.receive_settings().decimation, 2);
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn dropping_unpolled_shutdown_keeps_native_fallback_armed() {
    let control = FakeTransport::default();
    let state = Arc::clone(&control.state);
    let device = fake_device(control);

    let shutdown = device.shutdown();
    assert_eq!(state.control_out_count.load(Ordering::SeqCst), 0);
    drop(shutdown);
    assert_eq!(state.control_out_count.load(Ordering::SeqCst), 0);
    drop(device);

    assert_eq!(
        *state
            .control_out_requests
            .lock()
            .expect("control request lock"),
        [
            OutRequest::receiver_mode(ReceiverMode::Off),
            OutRequest::set_rf_bias(false),
        ]
    );
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn canceling_polled_shutdown_keeps_native_fallback_armed() {
    use std::future::IntoFuture;

    let control = FakeTransport::default();
    let state = Arc::clone(&control.state);
    state.pause_control_out_at.store(1, Ordering::SeqCst);
    let device = fake_device(control);
    let mut shutdown = Box::pin(device.shutdown().into_future());

    assert!(block_on(futures_lite::future::poll_once(shutdown.as_mut())).is_none());
    drop(shutdown);
    assert_eq!(state.control_out_count.load(Ordering::SeqCst), 1);
    drop(device);

    assert_eq!(state.control_out_count.load(Ordering::SeqCst), 3);
    assert_eq!(
        *state
            .control_out_requests
            .lock()
            .expect("control request lock"),
        [
            OutRequest::receiver_mode(ReceiverMode::Off),
            OutRequest::receiver_mode(ReceiverMode::Off),
            OutRequest::set_rf_bias(false),
        ]
    );
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn native_device_drop_performs_best_effort_shutdown() {
    let control = FakeTransport::default();
    let state = Arc::clone(&control.state);
    let device = fake_device(control);

    drop(device);

    assert_eq!(
        *state
            .control_out_requests
            .lock()
            .expect("control request lock"),
        [
            OutRequest::receiver_mode(ReceiverMode::Off),
            OutRequest::set_rf_bias(false),
        ]
    );
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn owned_raw_stream_drop_retries_failed_receiver_off() {
    let control = FakeTransport::default();
    let state = Arc::clone(&control.state);
    let device = fake_device(control);

    {
        let mut stream = Receiver::<_, RawAdc>::new(owned_stream_claim(device));
        stream.start().wait().expect("start owned raw stream");
        state.fail_control_out_at.store(3, Ordering::SeqCst);
        stream
            .stop()
            .wait()
            .expect_err("first receiver-off request should fail");
    }

    assert_eq!(state.cancel_count.load(Ordering::SeqCst), 1);
    assert_eq!(
        *state
            .control_out_requests
            .lock()
            .expect("control request lock"),
        [
            OutRequest::receiver_mode(ReceiverMode::Off),
            OutRequest::receiver_mode(ReceiverMode::Rx),
            OutRequest::receiver_mode(ReceiverMode::Off),
            OutRequest::receiver_mode(ReceiverMode::Off),
            OutRequest::receiver_mode(ReceiverMode::Off),
            OutRequest::set_rf_bias(false),
        ]
    );
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn owned_synchronous_f32_stream_keeps_queue_between_reads() {
    let control = FakeTransport::default();
    let state = Arc::clone(&control.state);
    let device = fake_device(control);
    let mut stream = Receiver::<_, F32Iq>::new(owned_stream_claim(device));
    stream
        .start()
        .wait()
        .expect("start owned synchronous F32 stream");

    let mut first = [Complex32::default(); 1];
    let mut second = [Complex32::default(); 1];
    assert_eq!(
        stream
            .read(&mut first, Some(Duration::from_secs(1)))
            .wait()
            .expect("first read"),
        1
    );
    assert_eq!(
        stream
            .read(&mut second, Some(Duration::from_secs(1)))
            .wait()
            .expect("second read"),
        1
    );
    assert_eq!(state.bulk_in_count.load(Ordering::SeqCst), 1);
    assert_eq!(state.control_out_count.load(Ordering::SeqCst), 2);

    let stats = stream
        .stop()
        .wait()
        .expect("stop owned synchronous F32 stream");
    assert_eq!(stats.buffers_received, 1);
    assert_eq!(state.cancel_count.load(Ordering::SeqCst), 1);
    assert_eq!(state.control_out_count.load(Ordering::SeqCst), 3);
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn synchronous_read_error_requires_stop_before_restart() {
    let control = FakeTransport::default();
    let state = Arc::clone(&control.state);
    let device = fake_device(control);
    let mut stream = Receiver::<_, F32Iq>::new(owned_stream_claim(device));
    let mut sample = [Complex32::default(); 1];

    stream.start().wait().expect("start synchronous stream");
    state.fail_bulk_completion.store(true, Ordering::SeqCst);
    assert!(stream.read(&mut sample, None).wait().is_err());
    assert_eq!(
        stream.claim.receiver_state(),
        ReceiverState::CleanupRequired
    );
    assert!(
        stream
            .start()
            .wait()
            .is_err_and(|error| error.kind() == crate::ErrorKind::Busy)
    );

    stream.stop().wait().expect("clean up failed read");
    stream.start().wait().expect("restart after cleanup");
    stream.stop().wait().expect("stop restarted stream");
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn synchronous_raw_read_error_requires_stop_before_restart() {
    let control = FakeTransport::default();
    let state = Arc::clone(&control.state);
    let device = fake_device(control);
    let mut stream = Receiver::<_, RawAdc>::new(owned_stream_claim(device));

    stream.start().wait().expect("start synchronous raw stream");
    state.fail_bulk_completion.store(true, Ordering::SeqCst);
    assert!(stream.next_block(None).wait().is_err());
    assert_eq!(
        stream.claim.receiver_state(),
        ReceiverState::CleanupRequired
    );
    assert!(
        stream
            .start()
            .wait()
            .is_err_and(|error| error.kind() == crate::ErrorKind::Busy)
    );

    stream.stop().wait().expect("clean up failed raw block");
    stream
        .start()
        .wait()
        .expect("restart raw stream after cleanup");
    stream.stop().wait().expect("stop restarted raw stream");
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn owned_synchronous_stream_stats_accumulate_across_restarts() {
    let control = FakeTransport::default();
    let device = fake_device(control);
    let mut stream = Receiver::<_, F32Iq>::new(owned_stream_claim(device));
    let mut sample = [Complex32::default(); 1];

    stream.start().wait().expect("start first synchronous run");
    stream
        .read(&mut sample, Some(Duration::from_secs(1)))
        .wait()
        .expect("read first synchronous run");
    assert_eq!(
        stream
            .stop()
            .wait()
            .expect("stop first synchronous run")
            .buffers_received,
        1
    );

    stream.start().wait().expect("start second synchronous run");
    stream
        .read(&mut sample, Some(Duration::from_secs(1)))
        .wait()
        .expect("read second synchronous run");
    assert_eq!(
        stream
            .stop()
            .wait()
            .expect("stop second synchronous run")
            .buffers_received,
        2 + crate::rfone::RFONE_TRANSFER_COUNT as u64
    );
}

#[test]
fn owned_async_f32_stream_reuses_queue_and_closes_on_drop() {
    block_on(async {
        let control = FakeTransport::default();
        let state = Arc::clone(&control.state);
        let device = fake_device(control);
        let mut stream = Receiver::<_, F32Iq>::new(owned_stream_claim(device));
        stream.start().await.expect("start owned async F32 stream");

        let mut first = [Complex32::default(); 1];
        let mut second = [Complex32::default(); 1];
        assert_eq!(stream.read(&mut first, None).await.expect("first read"), 1);
        assert_eq!(
            stream.read(&mut second, None).await.expect("second read"),
            1
        );
        assert_eq!(state.control_out_count.load(Ordering::SeqCst), 2);

        let stats = stream.stop().await.expect("stop owned async F32 stream");
        drop(stream);
        assert_eq!(stats.buffers_received, 1);
        assert_eq!(state.control_out_count.load(Ordering::SeqCst), 5);
        assert_eq!(state.cancel_count.load(Ordering::SeqCst), 2);
    });
}

#[test]
fn owned_async_f32_stream_reuses_queue_across_restart() {
    block_on(async {
        let control = FakeTransport::default();
        let state = Arc::clone(&control.state);
        let device = fake_device(control);
        let mut stream = Receiver::<_, F32Iq>::new(owned_stream_claim(device));

        stream.start().await.expect("start owned async F32 stream");
        let mut first = [Complex32::default(); 1];
        assert_eq!(stream.read(&mut first, None).await.expect("first read"), 1);
        stream.stop().await.expect("stop owned async F32 stream");
        assert_eq!(state.bulk_in_count.load(Ordering::SeqCst), 1);
        assert_eq!(state.cancel_count.load(Ordering::SeqCst), 1);
        assert!(
            stream
                .read(&mut first, None)
                .await
                .is_err_and(|error| { error.kind() == crate::ErrorKind::StreamClosed })
        );
        stream.stop().await.expect("repeated stop is a no-op");
        assert_eq!(state.control_out_count.load(Ordering::SeqCst), 3);

        stream
            .start()
            .await
            .expect("restart owned async F32 stream");
        let mut second = [Complex32::default(); 1];
        assert_eq!(
            stream.read(&mut second, None).await.expect("second read"),
            1
        );
        let stats = stream.stop().await.expect("stop owned async F32 stream");
        assert_eq!(state.bulk_in_count.load(Ordering::SeqCst), 1);
        assert_eq!(state.cancel_count.load(Ordering::SeqCst), 2);
        assert_eq!(
            stats.buffers_discarded_on_restart,
            crate::rfone::RFONE_TRANSFER_COUNT as u64
        );

        drop(stream);
        assert_eq!(state.cancel_count.load(Ordering::SeqCst), 3);
    });
}

#[test]
fn owned_async_stream_stats_survive_queue_recreation() {
    block_on(async {
        let control = FakeTransport::default();
        let device = fake_device(control);
        let mut stream = Receiver::<_, F32Iq>::new(owned_stream_claim(device));
        let mut sample = [Complex32::default(); 1];

        stream.start().await.expect("start first async queue");
        stream
            .read(&mut sample, None)
            .await
            .expect("read first async queue");
        stream.stop().await.expect("stop first async queue");
        stream.retire_stream();

        stream.start().await.expect("start replacement async queue");
        stream
            .read(&mut sample, None)
            .await
            .expect("read replacement async queue");
        assert_eq!(
            stream
                .stop()
                .await
                .expect("stop replacement async queue")
                .buffers_received,
            2
        );
    });
}

#[test]
fn async_raw_read_error_requires_stop_before_restart() {
    block_on(async {
        let control = FakeTransport::default();
        let state = Arc::clone(&control.state);
        let device = fake_device(control);
        let mut stream = Receiver::<_, RawAdc>::new(owned_stream_claim(device));

        stream.start().await.expect("start async raw stream");
        state.fail_bulk_completion.store(true, Ordering::SeqCst);
        assert!(stream.next_block(None).await.is_err());
        assert_eq!(
            stream.claim.receiver_state(),
            ReceiverState::CleanupRequired
        );
        assert!(
            stream
                .start()
                .await
                .is_err_and(|error| error.kind() == crate::ErrorKind::Busy)
        );

        stream.stop().await.expect("clean up failed read");
        stream.start().await.expect("restart after cleanup");
        stream.stop().await.expect("stop restarted stream");
    });
}

#[test]
fn async_f32_read_error_requires_stop_before_restart() {
    block_on(async {
        let control = FakeTransport::default();
        let state = Arc::clone(&control.state);
        let device = fake_device(control);
        let mut stream = Receiver::<_, F32Iq>::new(owned_stream_claim(device));
        let mut sample = [Complex32::default(); 1];

        stream.start().await.expect("start async F32 stream");
        state.fail_bulk_completion.store(true, Ordering::SeqCst);
        assert!(stream.read(&mut sample, None).await.is_err());
        assert_eq!(
            stream.claim.receiver_state(),
            ReceiverState::CleanupRequired
        );
        assert!(
            stream
                .start()
                .await
                .is_err_and(|error| error.kind() == crate::ErrorKind::Busy)
        );

        stream.stop().await.expect("clean up failed F32 read");
        stream.start().await.expect("restart F32 after cleanup");
        stream.stop().await.expect("stop restarted F32 stream");
    });
}

#[test]
fn cancelled_owned_async_raw_start_can_be_stopped() {
    block_on(async {
        let control = FakeTransport::default();
        let state = Arc::clone(&control.state);
        state.pause_control_out_at.store(2, Ordering::SeqCst);
        let device = fake_device(control);
        let mut stream = Receiver::<_, RawAdc>::new(owned_stream_claim(device));

        let mut start = Box::pin(stream.start().into_future());
        assert!(futures_lite::future::poll_once(&mut start).await.is_none());
        assert_eq!(state.control_out_count.load(Ordering::SeqCst), 2);
        drop(start);

        stream
            .stop()
            .await
            .expect("stop cancelled owned async raw stream start");
        drop(stream);
        assert_eq!(state.control_out_count.load(Ordering::SeqCst), 5);
    });
}

#[test]
fn cancelled_owned_async_f32_start_can_be_stopped() {
    block_on(async {
        let control = FakeTransport::default();
        let state = Arc::clone(&control.state);
        state.pause_control_out_at.store(2, Ordering::SeqCst);
        let device = fake_device(control);
        let mut stream = Receiver::<_, F32Iq>::new(owned_stream_claim(device));

        let mut start = Box::pin(stream.start().into_future());
        assert!(futures_lite::future::poll_once(&mut start).await.is_none());
        assert_eq!(state.control_out_count.load(Ordering::SeqCst), 2);
        drop(start);

        stream
            .stop()
            .await
            .expect("stop cancelled owned async F32 stream start");
        drop(stream);
        assert_eq!(state.control_out_count.load(Ordering::SeqCst), 5);
    });
}

#[test]
fn owned_async_stream_stop_error_still_closes_on_drop() {
    block_on(async {
        let control = FakeTransport::default();
        let state = Arc::clone(&control.state);
        let device = fake_device(control);
        let mut stream = Receiver::<_, F32Iq>::new(owned_stream_claim(device));
        stream.start().await.expect("start owned async F32 stream");
        state.fail_control_out.store(true, Ordering::SeqCst);

        assert!(
            stream
                .stop()
                .await
                .is_err_and(|error| error.kind() == crate::ErrorKind::Usb)
        );
        drop(stream);
        assert_eq!(state.cancel_count.load(Ordering::SeqCst), 1);
    });
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn native_drop_stops_async_receiver_closes_queue_and_shuts_down_hardware() {
    block_on(async {
        let control = FakeTransport::default();
        let state = Arc::clone(&control.state);
        let device = fake_device(control);
        let mut stream = Receiver::<_, F32Iq>::new(owned_stream_claim(device));
        stream.start().await.expect("start owned async F32 stream");
        assert_eq!(state.control_out_count.load(Ordering::SeqCst), 2);

        drop(stream);
        assert_eq!(state.cancel_count.load(Ordering::SeqCst), 1);
        assert_eq!(state.control_out_count.load(Ordering::SeqCst), 5);
    });
}
