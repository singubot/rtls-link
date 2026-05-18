# AGENTS.md

## Project Overview

This is an ESP32/ESP32S3 firmware project for real-time localization systems (RTLS) using Ultra-Wideband (UWB) technology. The firmware is intended for UWB-enabled drones and implements Time Difference of Arrival (TDoA) algorithms for 2D/3D positioning. These drones usually interface with ArduPilot through MAVLink by default, though some integrations may use custom protocols. In addition to that, it has a desktop application ( `tools/rtls-link-manager` , which is a git submodule with it's corresponding repository ) that it's used for configuring, monitoring and debugging this devices.

## ArduPilot Integration Context

RTLS Link firmware work may need to be coordinated with the Axiovel ArduPilot fork:

- Repository: https://github.com/Axiovel-tech/ardupilot
- Primary custom branch pattern: `AVCopter-x.x` (effectively `^AVCopter-[0-9]+\.[0-9]+$`, for example `AVCopter-4.5`)
- These `AVCopter-x.x` branches are where custom Axiovel changes are introduced and should receive extra attention when inspecting or coordinating ArduPilot-side work.
- The ArduPilot fork has SITL support, which is useful for testing firmware/protocol changes before involving real hardware.
- If a feature touches both RTLS Link and ArduPilot behavior, inspect both repositories when possible and consider SITL validation as part of the test plan.

Machine-specific local paths for the ArduPilot checkout and remote access details should not be committed. The Axiovel ArduPilot fork currently uses self-hosted GitHub Actions runners on a remote server, and RTLS Link may use the same pattern in the future. Keep host-specific details in the local-only skill at `.agents/skills/local_context/SKILL.md`, which is ignored by git. That skill may point to sibling local-only context files, such as remote CI host notes; read those only when the task needs that context.

## Build System

This project uses PlatformIO for build management with multiple environments:

### Build Commands
- `pio run` - Build all environments
- `pio run -e esp32_application` - Build for ESP32 (Makerfabs board)
- `pio run -e esp32s3_application` - Build for ESP32S3 (custom UWB board)
- `pio run -e native` - Build native tests with GoogleTest

### Testing
- `pio test` - Run all tests
- `pio test -e native` - Run native tests only
- `pio test -e esp32_application` - Run hardware tests on ESP32

### Upload
- `pio run -e esp32_application -t upload` - Upload to ESP32
- `pio run -e esp32s3_application -t upload` - Upload to ESP32S3

### OTA Upload (Preferred)
Most deployments update devices over WiFi (OTA), not USB serial. Use the CLI from the `tools/rtls-link-manager` submodule so automation and the desktop app share the same Rust backend implementation:

```bash
git submodule update --init tools/rtls-link-manager
cd tools/rtls-link-manager
cargo build --release -p rtls-link-cli
./target/release/rtls-link-cli discover
./target/release/rtls-link-cli ota update <ip> ../../.pio/build/<env>/firmware.bin
./target/release/rtls-link-cli ota update all ../../.pio/build/<env>/firmware.bin
```

### Compilation Testing Requirements
**CRITICAL**: Before pushing any changes to remote branches, ALWAYS verify compilation:
- `pio run -e esp32_application` - Must compile successfully
- `pio run -e esp32s3_application` - Must compile successfully
- Both targets must build without errors before any git push operation
- This prevents broken builds from being introduced to the codebase

## Desktop Application (rtls-link-manager)

The desktop application is located in `tools/rtls-link-manager` and is a Tauri app (Rust backend + React frontend).

### Build Commands
- `cd tools/rtls-link-manager && npm run tauri build` - Build production release
- `cd tools/rtls-link-manager && npm run dev` - Run in development mode
- `cd tools/rtls-link-manager && cargo check --manifest-path src-tauri/Cargo.toml` - Quick Rust type check (faster than full build)
- `cd tools/rtls-link-manager && cargo build --release -p rtls-link-cli` - Build the shared CLI tool

### Compilation Testing Requirements
**CRITICAL**: When modifying code in `tools/rtls-link-manager`, ALWAYS verify compilation before pushing:
- `cd tools/rtls-link-manager && cargo check --manifest-path src-tauri/Cargo.toml` - Rust backend must compile
- `cd tools/rtls-link-manager && npm run build` - Desktop app must build successfully
- Both checks must pass before any git push operation to the rtls-link-manager repository

### Board Support
- **ESP32 (Makerfabs)**: `MAKERFABS_ESP32_BOARD` flag
- **ESP32S3 (Konex UWB)**: `ESP32S3_UWB_BOARD` flag

### Main internal FreeRTOS tasks
Main tasks run at different frequencies:
- Application task: 10Hz
- WiFi task: 50Hz
- Console task: 50Hz
- UWB ranging: Main loop (continuous)

## Library Dependencies

ESP32 environments use:
- ETL (Embedded Template Library) for containers
- Eigen for matrix operations
- AsyncTCP/ESPAsyncWebServer for web interface
- SimpleCLI for command parsing
- Many others ...

## Important Files

- `src/main.cpp`: Application entry point and task setup
- `src/app.cpp`: Main application logic
- `src/bsp/`: Board support package definitions
- `src/uwb/`: UWB-related implementations
- `src/wifi/`: WiFi and web server implementations
- `scripts/`: Python utilities for testing and parameter management
- `tools/` : Tools like the `rtls-link-manager` desktop application for configuring and interacting with the system
- `platformio.ini`: Build configuration

# Important notes

- When adding new features to the code base, make sure to wrap it into a macro preprocessor feature switch (see features.hpp and user_defines.txt ). If there are any dependencies that need to be made ( like maybe feature X needs of feature Y, make sure to update the feature_validation.hpp ).
- When adding a new functionality from scratch. Always create a new branch called `feature/<title>` before starting to write the code.
- Always when adding a new feature to the firmware ( basically src folder ) you need to maintain in "sync" compatibility with the new feature of the desktop app that you can find inside `tools/rtls-link-manager`. The rtls-link-manager is a git submodule with it's own repository. You should create there the same corresponding branch. When opening the PRs, on the rtls-link repository, you should add a link with the matching feature PR of the `rtls-link-manager`.
- The repo has a special features file called `user_defines.txt` where it introduces all the feature flags included on the build. When adding a new feature make sure to add it to the `user_defines.txt`. All features should be there, if disables it should simply be commented out.
