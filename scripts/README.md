# Scripts

## Device discovery and configuration

Use the manager CLI so scripts and the desktop app share the MAVLink UDP backend:

```bash
cd ../tools/rtls-link-manager
cargo build --release -p rtls-link-cli
./target/release/rtls-link-cli discover
./target/release/rtls-link-cli config apply <device-ip> ../../scripts/params/tagHome.txt
```

## 3D Data visualizer

`python3 indoor_loc.py 192.168.4.1 23`
`python script.py 192.168.4.1 23 indoor_loc_config/config-3D.json`
`python script.py 192.168.4.1 23 indoor_loc_config/config-2D.json`
