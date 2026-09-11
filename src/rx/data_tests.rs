//! Buffer ownership and sample processing across blocking and async operations.

use super::{
    queue::{DEFAULT_BUFFER_SIZE, MAX_F32_IQ_SAMPLES_PER_TRANSFER},
    receiver::Receiver,
};
use crate::{
    Complex32, Config, F32Iq, RawAdc, SampleMode,
    errors::Error,
    protocol::{InRequest, ReceiveSettings},
    rfone::RFONE_TRANSFER_COUNT,
    session::{RxStreamClaim, Session},
    test_support::{FakeBulkIn, FakeTransport},
};
use futures_lite::future::{block_on, poll_once};
use nusb::MaybeFuture;
use std::{
    future::IntoFuture,
    sync::{Arc, atomic::Ordering},
    time::Duration,
};

fn receiver<M: SampleMode>() -> Receiver<FakeTransport, M> {
    let device = Session::new(FakeTransport::default());
    Receiver::new(RxStreamClaim::acquire(&device).unwrap())
}
fn bulk<M: SampleMode>(rx: &mut Receiver<FakeTransport, M>) -> &mut FakeBulkIn {
    rx.queue.as_mut().unwrap().bulk_in.as_mut().unwrap()
}

#[test]
fn raw_timeout_preserves_queue_and_forwards_deadline() {
    let mut rx = receiver::<RawAdc>();
    rx.start().wait().unwrap();
    bulk(&mut rx).timeout_next = true;
    let timeout = Duration::from_millis(25);
    assert!(rx.next_block(Some(timeout)).wait().unwrap().is_none());
    let bulk = bulk(&mut rx);
    assert!(bulk.last_timeout.unwrap() <= timeout);
    assert!(bulk.last_timeout.unwrap() > Duration::ZERO);
    assert_eq!(bulk.submitted.len(), RFONE_TRANSFER_COUNT as usize);
    assert!(!bulk.cancelled);
}

#[test]
fn iq_timeout_and_empty_output_preserve_queue() {
    let mut rx = receiver::<F32Iq>();
    rx.start().wait().unwrap();
    assert_eq!(rx.read(&mut [], None).wait().unwrap(), 0);
    assert_eq!(rx.current_stats().buffers_received, 0);
    bulk(&mut rx).timeout_next = true;
    assert_eq!(
        rx.read(&mut [Complex32::default(); 1], Some(Duration::ZERO))
            .wait()
            .unwrap(),
        0
    );
    assert_eq!(rx.current_stats().buffers_received, 0);
    assert_eq!(bulk(&mut rx).submitted.len(), RFONE_TRANSFER_COUNT as usize);
}

#[test]
fn iq_partial_reads_retain_and_recycle_one_usb_buffer() {
    block_on(async {
        let mut rx = receiver::<F32Iq>();
        rx.start().await.unwrap();
        assert_eq!(bulk(&mut rx).submit_count, RFONE_TRANSFER_COUNT as usize);
        let mut first = [Complex32::default(); 1];
        assert_eq!(rx.read(&mut first, None).await.unwrap(), 1);
        assert!(rx.processor.has_pending());
        assert!(rx.queue.as_ref().unwrap().current.is_some());
        assert_eq!(bulk(&mut rx).submit_count, RFONE_TRANSFER_COUNT as usize);
        // Switch completion mechanisms without recreating the queue or losing the odd IQ sample.
        assert_eq!(rx.read(&mut first, None).wait().unwrap(), 1);
        assert_eq!(rx.current_stats().buffers_received, 1);
        let mut rest = vec![Complex32::default(); MAX_F32_IQ_SAMPLES_PER_TRANSFER - 2];
        assert_eq!(rx.read(&mut rest, None).await.unwrap(), rest.len());
        assert_eq!(
            bulk(&mut rx).submit_count,
            RFONE_TRANSFER_COUNT as usize + 1
        );
        assert!(rx.queue.as_ref().unwrap().current.is_none());
        assert!(!rx.processor.has_pending());
    });
}

#[test]
fn both_read_modes_process_at_most_one_new_usable_completion() {
    block_on(async {
        let mut rx = receiver::<F32Iq>();
        rx.start().wait().unwrap();
        let mut out = vec![Complex32::default(); MAX_F32_IQ_SAMPLES_PER_TRANSFER + 1];
        assert_eq!(
            rx.read(&mut out, None).wait().unwrap(),
            MAX_F32_IQ_SAMPLES_PER_TRANSFER
        );
        assert_eq!(rx.current_stats().buffers_received, 1);
        assert_eq!(
            rx.read(&mut out, None).await.unwrap(),
            MAX_F32_IQ_SAMPLES_PER_TRANSFER
        );
        assert_eq!(rx.current_stats().buffers_received, 2);
    });
}

#[test]
fn raw_blocks_borrow_and_reuse_the_fixed_pool_across_modes() {
    block_on(async {
        let mut rx = receiver::<RawAdc>();
        rx.start().await.unwrap();
        assert_eq!(
            rx.next_block(None)
                .await
                .unwrap()
                .unwrap()
                .raw_bytes()
                .len(),
            DEFAULT_BUFFER_SIZE
        );
        assert_eq!(bulk(&mut rx).submit_count, RFONE_TRANSFER_COUNT as usize);
        assert_eq!(
            rx.next_block(None).wait().unwrap().unwrap().sample_count(),
            DEFAULT_BUFFER_SIZE / 2
        );
        assert_eq!(
            bulk(&mut rx).submit_count,
            RFONE_TRANSFER_COUNT as usize + 1
        );
        assert_eq!(
            bulk(&mut rx).submitted.len(),
            RFONE_TRANSFER_COUNT as usize - 1
        );
    });
}

#[test]
fn canceling_pending_iq_read_leaves_completion_available_to_blocking_read() {
    block_on(async {
        let mut rx = receiver::<F32Iq>();
        rx.start().wait().unwrap();
        bulk(&mut rx).pending_once = true;
        let mut out = [Complex32::default(); 1];
        let mut read = Box::pin(rx.read(&mut out, None));
        assert!(poll_once(&mut read).await.is_none());
        drop(read);
        assert_eq!(rx.current_stats().buffers_received, 0);
        assert_eq!(rx.read(&mut out, None).wait().unwrap(), 1);
        assert_eq!(rx.current_stats().buffers_received, 1);
    });
}

#[test]
fn canceling_pending_raw_read_leaves_completion_available_to_blocking_read() {
    block_on(async {
        let mut rx = receiver::<RawAdc>();
        rx.start().wait().unwrap();
        bulk(&mut rx).pending_once = true;
        let mut read = Box::pin(rx.next_block(None).into_future());
        assert!(poll_once(&mut read).await.is_none());
        drop(read);
        assert_eq!(rx.current_stats().buffers_received, 0);
        assert_eq!(
            rx.next_block(None)
                .wait()
                .unwrap()
                .unwrap()
                .raw_bytes()
                .len(),
            DEFAULT_BUFFER_SIZE
        );
        assert_eq!(rx.current_stats().buffers_received, 1);
    });
}

#[test]
fn rejected_completions_close_queue_and_preserve_statistics_in_both_modes() {
    block_on(async {
        for short in [false, true] {
            for wait in [false, true] {
                let mut rx = receiver::<F32Iq>();
                rx.start().await.unwrap();
                bulk(&mut rx).short_next = short;
                bulk(&mut rx).fail_next = !short;
                let mut out = [Complex32::default(); 1];
                let error = if wait {
                    rx.read(&mut out, None).wait()
                } else {
                    rx.read(&mut out, None).await
                }
                .unwrap_err();
                if short {
                    assert!(matches!(error, Error::Protocol { .. }));
                } else {
                    assert!(matches!(error, Error::Transfer(_)));
                }
                assert_eq!(rx.current_stats().buffers_received, 1);
                assert_eq!(rx.current_stats().buffers_dropped, 1);
                assert!(rx.queue.is_none());
                rx.stop().wait().unwrap();
                rx.start().await.unwrap();
                assert_eq!(rx.read(&mut out, None).wait().unwrap(), 1);
            }
        }
    });
}

#[test]
fn alternating_start_stop_and_read_modes_retain_endpoint_and_discard_old_submissions() {
    block_on(async {
        let mut rx = receiver::<F32Iq>();
        let state = Arc::clone(&rx.claim.shared.control.state);
        rx.start().wait().unwrap();
        rx.read(&mut [Complex32::default(); 1], None).await.unwrap();
        rx.stop().wait().unwrap();
        rx.start().await.unwrap();
        rx.read(&mut [Complex32::default(); 1], None)
            .wait()
            .unwrap();
        let stats = rx.stop().await.unwrap();
        assert_eq!(state.bulk_in_count.load(Ordering::SeqCst), 1);
        assert_eq!(stats.buffers_processed, 2);
        assert_eq!(
            stats.buffers_discarded_on_restart,
            RFONE_TRANSFER_COUNT as u64
        );
        rx.close().wait().unwrap();
    });
}

#[test]
fn iq_processing_adopts_live_decimation_without_recreating_endpoint() {
    block_on(async {
        let mut rx = receiver::<F32Iq>();
        rx.start().wait().unwrap();
        rx.read(&mut [Complex32::default(); 1], None)
            .wait()
            .unwrap();
        let shared = Arc::clone(&rx.claim.shared);
        let config = Config::builder().sample_rate_hz(625_000).build().unwrap();
        shared.configure(&config).await.unwrap();
        rx.read(&mut [Complex32::default(); 1], None).await.unwrap();
        assert_eq!(rx.queue.as_ref().unwrap().settings.decimation, 4);
        assert_eq!(rx.current_stats().buffers_dropped, 1);
        assert_eq!(shared.control.state.bulk_in_count.load(Ordering::SeqCst), 1);
    });
}

#[test]
fn packing_changes_rebuild_only_raw_queue_and_keep_cumulative_statistics() {
    block_on(async {
        for wait in [false, true] {
            let mut rx = receiver::<RawAdc>();
            rx.start().await.unwrap();
            rx.next_block(None).await.unwrap();
            let shared = Arc::clone(&rx.claim.shared);
            shared
                .apply(
                    vec![InRequest::set_packing(true).into()],
                    Some(ReceiveSettings {
                        packing: true,
                        decimation: 1,
                    }),
                )
                .await
                .unwrap();
            let block = if wait {
                rx.next_block(None).wait()
            } else {
                rx.next_block(None).await
            }
            .unwrap()
            .unwrap();
            assert_eq!(block.raw_bytes().len(), 6_144 * 24);
            assert_eq!(block.sample_count(), (6_144 * 24 / 2) * 4 / 3);
            assert_eq!(rx.current_stats().buffers_processed, 2);
            assert_eq!(shared.control.state.bulk_in_count.load(Ordering::SeqCst), 2);
        }
    });
}

#[test]
fn canceled_packing_restart_requires_stop_before_reading_or_restarting() {
    block_on(async {
        let mut rx = receiver::<RawAdc>();
        rx.start().await.unwrap();
        let shared = Arc::clone(&rx.claim.shared);
        shared
            .apply(
                vec![InRequest::set_packing(true).into()],
                Some(ReceiveSettings {
                    packing: true,
                    decimation: 1,
                }),
            )
            .await
            .unwrap();
        shared
            .control
            .state
            .pause_clear_halt
            .store(true, Ordering::SeqCst);
        let mut read = Box::pin(rx.next_block(None).into_future());
        assert!(poll_once(&mut read).await.is_none());
        drop(read);
        assert!(rx.next_block(None).wait().is_err());
        assert!(rx.start().await.is_err());
        rx.stop().wait().unwrap();
        shared
            .control
            .state
            .pause_clear_halt
            .store(false, Ordering::SeqCst);
        rx.start().await.unwrap();
        assert_eq!(
            rx.next_block(None)
                .wait()
                .unwrap()
                .unwrap()
                .raw_bytes()
                .len(),
            6_144 * 24
        );
    });
}

#[test]
fn unpolled_start_and_stop_operations_have_no_lifecycle_effects() {
    block_on(async {
        let mut rx = receiver::<F32Iq>();
        let state = Arc::clone(&rx.claim.shared.control.state);
        drop(rx.start());
        drop(rx.start().into_future());
        assert_eq!(state.control_out_count.load(Ordering::SeqCst), 0);
        assert_eq!(state.bulk_in_count.load(Ordering::SeqCst), 0);
        rx.start().wait().unwrap();
        drop(rx.stop());
        drop(rx.stop().into_future());
        assert_eq!(state.control_out_count.load(Ordering::SeqCst), 2);
        assert_eq!(
            rx.read(&mut [Complex32::default(); 1], None).await.unwrap(),
            1
        );
        rx.close().wait().unwrap();
    });
}
