# HydraSDR C parity test matrix

This matrix tracks the no-hardware safety net for the direct C-to-Rust translation phase. The C reference is `/home/basti/src/hydrasdr-host/libhydrasdr/src/`; Rust coverage lives in `/home/basti/src/hydrasdr-rs/tests/`.

| C reference behavior | Rust coverage | Default hardware? | Notes |
| --- | --- | --- | --- |
| `hydrasdr.h` version macros and `HYDRASDR_MAKE_VERSION` | `tests/foundation.rs::version_constants_match_c_header` | no | Checks `1.1.2`, major/minor/revision, numeric version, auto-bandwidth sentinel. |
| `enum hydrasdr_error` values and `hydrasdr_error_name` strings | `tests/foundation.rs::status_codes_and_error_names_match_c_api` | no | Includes unknown-code fallback. |
| Board IDs, board names, sample type enum/bitmask, decimation enum | `tests/foundation.rs::board_and_sample_type_values_match_c_enums` | no | Mirrors public C enum discriminants. |
| Vendor request IDs, receiver modes, RF ports, capability/gain bits | `tests/foundation.rs::vendor_request_and_capability_values_match_c_commands` | no | Mirrors `hydrasdr_commands.h`. |
| RFOne VID/PID, endpoint, transfer count, buffer sizes, RF limits, GPIO count, sample-type mask | `tests/foundation.rs::rfone_static_spec_matches_c_driver_constants` | no | Mirrors `hydrasdr_rfone.c` and `hydrasdr_shared.h`. |
| USB control request packing for frequency, samplerate, bandwidth, GPIO, SPI flash, receiver mode, RF port, packing, gain | `tests/foundation.rs::no_hardware_helpers_pack_usb_fields_like_c_driver`, `tests/direct_api.rs::direct_helpers_send_c_style_control_requests`, `tests/no_hardware_parity.rs::control_request_builders_encode_vendor_device_packets_like_c` | no | Checks vendor/device recipient shape through `nusb` control packet conversion and C wValue/wIndex/data layout. |
| Board ID, version string, part/serial, capability word decoding | `tests/direct_api.rs::query_helpers_decode_board_version_serial_and_capabilities`, `tests/no_hardware_parity.rs::short_control_reads_are_libusb_errors` | no | Short reads map to `HYDRASDR_ERROR_LIBUSB`. |
| Samplerate/bandwidth count-then-list protocol and index selection | `tests/direct_api.rs::sample_rate_and_bandwidth_helpers_use_count_then_list_protocol` | no | Covers C `GET_*` count/list request sequence. |
| Legacy gain clamping and unsupported/invalid gain paths | `tests/direct_api.rs::direct_helpers_send_c_style_control_requests`, `tests/no_hardware_parity.rs::invalid_parameters_are_rejected_without_usb_side_effects` | no | LNA clamps to RFOne max; invalid `GainType::Count` is rejected locally. |
| RF port firmware status byte handling | `tests/no_hardware_parity.rs::rf_port_firmware_rejection_maps_to_invalid_param` | no | C treats return byte != 1 as invalid parameter. |
| Invalid parameter handling for frequency, sample type, GPIO, SPI flash | `tests/direct_api.rs::direct_helpers_send_c_style_control_requests`, `tests/no_hardware_parity.rs::invalid_parameters_are_rejected_without_usb_side_effects` | no | Ensures deterministic host-side errors where the C driver validates before USB. |
| Streaming start state machine: receiver OFF -> RX, endpoint open/clear, transfer queueing, callback transfer fields, stop cleanup | `tests/streaming.rs::start_rx_uses_c_style_receiver_modes_endpoint_and_callback_loop` | no | Uses a fake bulk-IN backend; no connected device needed. |
| Callback semantics: non-zero callback return stops streaming and leaves later completions unread | `tests/streaming.rs::callback_can_stop_streaming_before_more_completions_are_processed` | no | Mirrors C callback contract. |
| USB transfer error path: libusb-status propagation, cancellation, streaming flag reset | `tests/streaming.rs::transfer_error_stops_streaming_reports_libusb_and_cancels_pending` | no | Deterministic fake transfer error. |
| Stop-before-start idempotence and receiver OFF command | `tests/streaming.rs::stop_rx_is_idempotent_when_streaming_is_idle` | no | Mirrors C stop path being safe when no threads/transfers are active. |
| Packed streaming buffer size and sample count formula | `tests/streaming.rs::packed_streaming_uses_c_buffer_size_and_sample_count` | no | Mirrors `PACKED_BUFFER_SIZE` and `(((buffer_size / 2) * 4) / 3)`. |
| Real device open/query/configure/short stream smoke | `tests/hardware.rs` | ignored by default | Run explicitly with `cargo test --test hardware -- --ignored --nocapture` on a machine with a HydraSDR RFOne and permissions/udev access. |

## Default determinism

`cargo test` must pass with no HydraSDR connected. Hardware-dependent checks are `#[ignore]` and are only executed when the operator opts into ignored tests.

## Known phase boundary

The current Rust direct-port stream callback receives raw USB bytes plus C-parity sample-count metadata. The later C DSP/DDC conversion pipeline is intentionally out of scope for this safety-net card and should get its own parity matrix/tests when ported.
