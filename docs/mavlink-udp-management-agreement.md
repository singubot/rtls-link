# MAVLink UDP Management Implementation Agreement

## Goal

Remove the firmware and manager WebSocket management path and replace it with a native MAVLink v2 UDP management path. The change should simplify the architecture, reduce protocol duplication, and keep firmware/ground-station communication efficient on resource-constrained ESP32 targets.

## Scope

- Remove firmware WebSocket command/config handling and the manager WebSocket client.
- Keep UDP as the management transport, using dedicated UDP port `3333`.
- Keep the existing HTTP OTA upload path for this change so in-hardware verification can still update devices safely.
- Leave MAVLink OTA for a follow-up change, but keep the new code structure compatible with adding it.
- Keep firmware and `tools/rtls-link-manager` behavior in sync.

## MAVLink Dialect

- Use `lib/c_library_v2` as the generated C MAVLink library source.
- Add an RTLS-Link dialect that includes only `minimal.xml` plus the specific messages this firmware supports.
- Do not include the full `common.xml` message surface in the manager/backend dialect for this protocol.
- Reuse the standard PARAM_EXT message shapes for parameter read/list/set.
- Define RTLS-specific messages for device status, command request, and command response.
- The manager generates matching Rust bindings from the same RTLS-Link dialect XML.

## Parameters

- Expose stable MAVLink parameter IDs through a small registry at the protocol boundary.
- Keep existing LittleFS parameter group/name storage for this change to avoid risky migration while WebSocket removal is being validated.
- Map legacy manager config operations to MAVLink PARAM_EXT under the hood.
- Treat a future storage/key migration as a separate design decision after the MAVLink path is stable.

## Telemetry And Logging

- Keep status collection centralized behind a telemetry callback/DTO instead of scattering network/logging reads through UWB or app hot paths.
- Send periodic MAVLink heartbeat plus `RTLS_DEVICE_STATUS` for discovery and status.
- Preserve the useful existing status fields, including dynamic anchor positions when compiled and enabled.
- Keep high-rate or bulky diagnostics on explicit command/telemetry paths rather than bloating the periodic status message.
- Follow the ArduPilot-style direction of small typed messages, clear ownership, and explicit scheduling instead of ad hoc string/log plumbing.

## Validation Plan

1. Build manager core/CLI/Tauri backend and frontend.
2. Build firmware for `esp32_application` and `esp32s3_application`.
3. Review the diff for stale WebSocket code, old discovery protocol references, and unnecessary complexity.
4. Wait for the user's explicit signal before starting the 3-agent audit loop.
5. Run the 3-agent audit loop with distinct review focuses: protocol/functionality, simplification/maintainability, and bug/regression risk.
6. Incorporate legitimate audit findings and discard nitpicks.
7. Ask the user to start the in-hardware validation loop with 4 anchors and 1 tag.
8. Use the existing HTTP OTA path for hardware flashing in this change.
