# TDoA2 Anchor Performance And Tag Timing Notes

## Context

This work continues PR #44, which added diagnostics around the TDoA2 anchor path after removing the unused LPP service-packet wait. The goal is to reclaim TDMA frame time, increase update rate, and make the anchor side reliable enough to expose the next bottleneck.

The current branch adds:

- a pinned, notification-driven TDoA2 anchor radio task
- anchor-side timing and sync diagnostics
- periodic UDP anchor telemetry for live validation
- runtime UWB suspend/resume support used during OTA
- manager CLI/UI support for configuring and listening to anchor telemetry

The current practical anchor configuration is:

- `tdoaSlotCount=4`
- `tdoaSlotDurationUs=1300`
- frame duration: `5200 us`
- expected per-anchor TX cadence: about `192 Hz`

## Anchor Timing Findings

The original concern was that tighter TDMA slots could reproduce anchor stalls or resets. With the pinned anchor radio task and telemetry, the boundary is now clearer.

Short stepped sweep with four anchors:

| Slot duration | Frame duration | Expected anchor cadence | Result |
| ---: | ---: | ---: | --- |
| `1900 us` | `7600 us` | `~132 Hz` | clean baseline |
| `1800 us` | `7200 us` | `~139 Hz` | clean |
| `1700 us` | `6800 us` | `~147 Hz` | clean |
| `1600 us` | `6400 us` | `~156 Hz` | clean, lower slack observed |
| `1500 us` | `6000 us` | `~167 Hz` | clean |
| `1400 us` | `5600 us` | `~179 Hz` | clean |
| `1300 us` | `5200 us` | `~192 Hz` | clean |
| `1250 us` | `5000 us` | `~200 Hz` | clean in short soak, tight margin |
| `1200 us` | `4800 us` | `~208 Hz` | first observed stall boundary |

The `1200 us` failure reproduced after the anchors were separated into a more realistic geometry. One anchor reported a new stall reset and only single-digit microseconds of slot slack. In the separated layout, RF receive timeout counters also rose sharply at `1200 us`.

`1250 us` survived a roughly 75 second soak in both close and separated layouts, but the remaining margin was narrow enough that it should remain an experimental stress setting for now.

`1300 us` is the current recommended candidate. It gives about a 46 percent cadence increase over `1900 us` while remaining clean in the current validation runs.

## Tag-Side Impact

Tags do not directly consume `tdoaSlotDurationUs` for scheduling. The parameter is passed into the anchor algorithm, while the tag stays in receive mode and processes received TDoA2 packets.

The tag is still affected indirectly:

- UWB RX interrupt rate increases as anchors transmit more often.
- The TDoA producer path sees more packet callbacks.
- The estimator has more opportunities to receive fresh anchor-pair constraints.
- MAVLink or RTLSLink output may still cap the final position output rate.

At `1300 us`, the frame duration is `5.2 ms`, which is close to the tag estimator notification debounce of `5 ms`. This makes it a good practical target: it increases freshness and solve opportunities without clearly running below the current tag-side cadence assumptions.

The current tag implementation does not average multiple temporal samples for the same anchor pair inside one estimator wake. It stores one latest measurement per unique anchor pair. The benefit from a faster anchor schedule is therefore:

- more chances to have four to six fresh unique pair measurements ready per solve
- lower measurement age and skew
- faster recovery after missed packets
- potentially better overdetermined Newton-Raphson solves when more unique pairs are present

It does not currently provide:

- temporal averaging per pair
- weighted least squares by age, RSSI, or residual
- multiple samples from the same pair in one solve
- motion-aware fusion across fast frames

## Tag Test Observation

A quick flight test with the anchors at `1300 us` looked noticeably better in position hold. This is a useful positive signal, but it should be treated as qualitative until the tag side exposes enough telemetry to correlate the improvement with estimator behavior.

The connected tag observed during validation was:

- role: `tag_tdoa`
- board: `ESP32S3_UWB`
- output backend: MAVLink (`outputBackend=0`)
- UWB enabled
- four anchors seen
- position output enabled

The tag firmware was older than this branch, so the current branch's anchor-side changes were being evaluated against the existing tag architecture.

## Tag-Side Timing Rigor Proposal

The next step should be tag-side observability before major optimization.

Add tag telemetry for:

- IRQ-to-service latency
- DW1000 interrupt handling duration
- RX packet counts per anchor
- TDoA measurements produced per anchor pair
- fresh pair count per estimator solve
- samples accepted and rejected
- rejection reason counts: RMSE, NaN, insufficient measurements
- stale measurement drops
- producer mutex drops
- solver min/avg/max duration
- MAVLink or RTLSLink output rate

Then compare `1900 us`, `1300 us`, and `1250 us` on the tag. The key question is whether the tighter anchor loop gives the estimator more usable constraints per solve, or whether samples are dropped or overwritten before consumption.

After instrumentation, the likely tag architecture changes are:

- move UWB RX handling into a dedicated high-priority pinned radio task
- wake that task directly from the UWB ISR using task notifications
- keep packet ingest short and deterministic
- keep estimator work fully decoupled from the radio path
- keep MAVLink or RTLSLink output outside the radio-critical path
- measure and report latest-pair overwrites
- consider a small per-pair history or weighted snapshot only after measuring the current latest-pair behavior

The anchor-side work improved deterministic TDMA execution. The tag-side equivalent is a deterministic RX pipeline plus a measured estimator/output pipeline.
