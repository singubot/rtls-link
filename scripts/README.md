# Scripts

## Device discovery and configuration

Use the manager CLI so scripts and the desktop app share the MAVLink UDP backend:

```bash
cd ../tools/rtls-link-manager
cargo build --release -p rtls-link-cli
./target/release/rtls-link-cli discover
./target/release/rtls-link-cli config apply <device-ip> ../../scripts/params/tagHome.txt
```

## Telemetry visualization

The old TCP debug visualizer was removed with the TCP debug backend. Use the
manager CLI and UDP/MAVLink telemetry paths for validation.
