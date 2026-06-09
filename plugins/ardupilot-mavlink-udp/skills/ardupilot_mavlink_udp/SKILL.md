---
name: ardupilot_mavlink_udp
description: Use when working with the RTLS Link indoor drone through ArduPilot MAVLink over UDP, especially tag discovery, OTA, heartbeat/log checks, MAVFTP smoke tests, bridge diagnostics, mission download, and flight-log analysis.
---

# ArduPilot MAVLink UDP Workflow

This is the working RTLS Link hardware workflow for the indoor drone over UDP.
It intentionally includes local lab paths used by the agents on the RTLS Link
development machine.

## Paths

- RTLS Link repo: `/home/singu/dev/fw/rtls-link`
- RTLS Link worktrees: `/home/singu/dev/fw/.worktrees/rtls-link`
- Local artifacts: `/home/singu/dev/fw/.worktrees/rtls-link/.agents`
- ArduPilot logs/artifacts: `/home/singu/dev/fw/.worktrees/rtls-link/.agents/ardupilot_logs`
- MAVProxy/pymavlink tools: `/home/singu/venv-ardupilot/bin/mavproxy.py`, `/home/singu/venv-ardupilot/bin/mavlogdump.py`
- Manager CLI: `/home/singu/dev/fw/rtls-link/tools/rtls-link-manager/target/release/rtls-link-cli`
- Axiovel ArduPilot checkout: `/home/singu/dev/fw/axiovel/ardupilot`

## Current Device Facts

- ArduPilot MAVLink bridge: passive UDP `udp:0.0.0.0:14550`
- Observed vehicle: ArduCopter `V4.6.3`, system id `8`
- Common tag IP during June 2026 tests: `192.168.0.109`
- DHCP can move devices. Always rediscover before assuming the IP.
- Avoid multiple clients bound to UDP `14550` at the same time.

Check stale MAVLink or CLI clients before opening a new session:

```bash
pgrep -af 'MAVProxy|mavproxy|mavlogdump|pymavlink|rtls-link-cli'
```

## Discover And Inspect Tag

```bash
CLI=/home/singu/dev/fw/rtls-link/tools/rtls-link-manager/target/release/rtls-link-cli

$CLI discover --json --duration 5
$CLI discover --json --filter-role tag-tdoa --duration 5
$CLI status --json --health 192.168.0.109
```

Useful config checks:

```bash
$CLI config read --group wifi --name gcsIp 192.168.0.109
$CLI config read --group wifi --name logUdpEnabled 192.168.0.109
$CLI config read --group wifi --name logUdpPort 192.168.0.109
$CLI config read --group uwb --name use2DEstimator 192.168.0.109
$CLI config read --group uwb --name tdoaEstimatorMode 192.168.0.109
$CLI config read --group uwb --name tdoaEstimatorDiag 192.168.0.109
$CLI config read --group uwb --name enableCovMatrix 192.168.0.109
$CLI --timeout 3000 cmd --json 192.168.0.109 tdoa-estimator-status
```

Expected robust-only test values:

- `use2DEstimator=0`
- `tdoaEstimatorMode=1`
- `tdoaEstimatorDiag=1` only while collecting diagnostics
- `enableCovMatrix=0`
- estimator status mode `robust_3d`
- `compareMode=false`
- `fallbackLegacy=false`

## OTA Firmware

Run from the firmware worktree that produced the build:

```bash
CLI=/home/singu/dev/fw/rtls-link/tools/rtls-link-manager/target/release/rtls-link-cli

pio run -e esp32s3_application
$CLI ota update 192.168.0.109 .pio/build/esp32s3_application/firmware.bin
sleep 8
$CLI discover --json --filter-role tag-tdoa --duration 8
```

## MAVLink Baseline Check

Run after OTA and before MAVFTP. This verifies that the bridge carries normal
MAVLink traffic and that ArduPilot advertises FTP support.

```bash
/home/singu/venv-ardupilot/bin/python - <<'PY'
from pymavlink import mavutil
import time

mav = mavutil.mavlink_connection(
    'udp:0.0.0.0:14550',
    source_system=255,
    source_component=191,
    autoreconnect=True,
    dialect='ardupilotmega',
    force_mavlink2=True,
)
hb = mav.wait_heartbeat(timeout=20)
if hb is None:
    raise SystemExit('NO_HEARTBEAT')
print('heartbeat sys', mav.target_system, 'comp', mav.target_component)

mav.mav.command_long_send(
    mav.target_system,
    mav.target_component,
    mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
    0,
    mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
    0, 0, 0, 0, 0, 0,
)
deadline = time.time() + 8
version = None
while time.time() < deadline:
    msg = mav.recv_match(type=['AUTOPILOT_VERSION'], blocking=True, timeout=1)
    if msg is not None:
        version = msg
        break
print('ftp_capable', bool(version and int(version.capabilities) & mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_FTP))

mav.mav.log_request_list_send(mav.target_system, mav.target_component, 0, 0xffff)
logs = {}
deadline = time.time() + 12
while time.time() < deadline:
    msg = mav.recv_match(type=['LOG_ENTRY'], blocking=True, timeout=1)
    if msg is not None:
        logs[msg.id] = msg
print('log_entries', len(logs))
for log_id in sorted(logs)[-8:]:
    entry = logs[log_id]
    print(f'id={entry.id} size={entry.size} last_log_num={entry.last_log_num}')
PY
```

## MAVFTP Smoke Test

As of the UART bridge buffer fix, MAVFTP over UDP should list directories and
download small files. Keep this as a smoke test, not a full log-recovery flow.

```bash
MAVLINK20=1 /home/singu/venv-ardupilot/bin/python - <<'PY'
from pathlib import Path
from pymavlink import mavutil
from pymavlink.mavftp import MAVFTP, MAVFTPSettings
import time

outdir = Path('/home/singu/dev/fw/.worktrees/rtls-link/.agents/mavftp_bridge_smoke')
outdir.mkdir(parents=True, exist_ok=True)

settings = MAVFTPSettings([
    ('debug', int, 0),
    ('pkt_loss_tx', int, 0),
    ('pkt_loss_rx', int, 0),
    ('max_backlog', int, 5),
    ('burst_read_size', int, 80),
    ('write_size', int, 80),
    ('write_qsize', int, 5),
    ('idle_detection_time', float, 3.7),
    ('read_retry_time', float, 1.0),
    ('retry_time', float, 0.5),
])

m = mavutil.mavlink_connection(
    'udp:0.0.0.0:14550',
    source_system=255,
    source_component=196,
    autoreconnect=True,
    dialect='ardupilotmega',
    force_mavlink2=True,
)
if m.wait_heartbeat(timeout=15) is None:
    raise SystemExit('NO_HEARTBEAT')
time.sleep(4.2)
ftp = MAVFTP(m, 8, 1, settings=settings)

for path in ['/', '/APM', '/APM/LOGS', '@SYS']:
    ftp.list_result = []
    ret = ftp.cmd_list([path])
    print('LIST', path, ret.error_code, getattr(ret.error_code, 'name', '?'), len(ftp.list_result))

for remote, local_name in [
    ('@SYS/uarts.txt', 'uarts_udp_bridge.txt'),
    ('/APM/LOGS/LASTLOG.TXT', 'LASTLOG_udp_bridge.TXT'),
]:
    local = outdir / local_name
    if local.exists():
        local.unlink()
    ret = ftp.cmd_get([remote, str(local)])
    ret = ftp.process_ftp_reply('GetFile', timeout=20)
    print('GET', remote, ret.error_code, getattr(ret.error_code, 'name', '?'), local.stat().st_size if local.exists() else -1)
PY
```

Known-good result after the bridge fix:

- `LIST /`, `/APM`, `/APM/LOGS`, and `@SYS` succeeded.
- `/APM/LOGS` returned 37 entries.
- `GET @SYS/uarts.txt` downloaded 669 bytes.
- `GET /APM/LOGS/LASTLOG.TXT` downloaded 4 bytes.

## Bridge Diagnostics

The bridge logs stats every 5 seconds when there is activity. The firmware log
tag is the source filename, `wifi_uart_bridge.cpp`, so use a glob:

```bash
timeout 20s $CLI logs --ndjson --level info --tag 'wifi_uart_bridge*' 192.168.0.109
```

The manager CLI fix makes `rtls-link-cli --timeout <ms> logs ...` exit cleanly
by timeout. With older manager builds, use shell `timeout`.

Healthy smoke-test signals:

- `fail=0`
- `short=0`
- `rxLimit=0`
- `full=0`
- `noTgt=0`
- `maxAvail` far below the configured UART RX buffer size

Example healthy output:

```text
UART bridge I/O: udpRx=0/0B uartTx=0B uartRx=11852B udpTx=250/11852B
UART bridge diag: fail=0 short=0 rxLimit=0 full=0 noTgt=0 maxAvail=145 maxPkt=145 upd=250 avgMaxUs=1106/1608
```

## Mission Download

Save mission artifacts beside flight logs:

```bash
/home/singu/venv-ardupilot/bin/python - <<'PY'
from pymavlink import mavutil
import json

out = '/home/singu/dev/fw/.worktrees/rtls-link/.agents/ardupilot_logs/mission_items_latest.json'
mav = mavutil.mavlink_connection('udp:0.0.0.0:14550', source_system=255, autoreconnect=True)
if mav.wait_heartbeat(timeout=20) is None:
    raise SystemExit('NO_HEARTBEAT')

mav.mav.mission_request_list_send(mav.target_system, mav.target_component)
count_msg = mav.recv_match(type=['MISSION_COUNT'], blocking=True, timeout=10)
if count_msg is None:
    raise SystemExit('NO_MISSION_COUNT')

items = []
for seq in range(count_msg.count):
    mav.mav.mission_request_int_send(mav.target_system, mav.target_component, seq)
    msg = mav.recv_match(type=['MISSION_ITEM_INT', 'MISSION_ITEM'], blocking=True, timeout=5)
    if msg is None:
        raise SystemExit(f'MISSING_MISSION_ITEM_{seq}')
    d = msg.to_dict()
    d['seq'] = seq
    items.append(d)

with open(out, 'w') as f:
    json.dump({'count': count_msg.count, 'items': items}, f, indent=2, sort_keys=True)
print(out)
PY
```

For `MISSION_ITEM_INT`, latitude and longitude are integer degrees scaled by
`1e7`.

## DataFlash Log Download Notes

Classic `LOG_ENTRY` listing works over UDP. Bulk log download over this WiFi
path has historically produced corrupt files, even when the file size matched.
Prefer USB/SD for flight analysis unless a new log-transfer fix is being tested.

Validate every `.BIN` before analysis:

```bash
/home/singu/venv-ardupilot/bin/mavlogdump.py --types MODE,CMD,RFND,VISP,POS,XKF1,XKF4 log.bin >/tmp/mavlog_check.txt 2>&1
tail -80 /tmp/mavlog_check.txt
```

Reject files with `bad header`, `Invalid length in FMT`, or similar parse
errors.

## Flight Log Analysis Checklist

Use a clean `.BIN`, preferably copied from USB/SD. Inspect:

- Mission and modes: `CMD`, `MODE`, `MISE`
- External nav / visual odometry: `VISP`
- Rangefinder: `RFND`, and `SURF` if present
- EKF state and innovations: `XKF1`, `XKF2`, `XKF3`, `XKF4`, `XKF5`, `XKFS`, `XKV1`, `XKV2`
- Position controller: `PSCN`, `PSCE`, `PSCD`
- Vehicle position/attitude: `POS`, `ATT`, `AHR2`
- Power and vibration: `BAT`, `POWR`, `VIBE`

For rangefinder vs UWB Z, compare relative movement, not absolute height. Fit a
median offset during a stable segment because the floor is below the lower
anchor plane.
