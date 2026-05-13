# Refactor Baseline Checklist

## Goal
- Freeze current expected behavior before aggressive refactor.
- Catch regressions quickly after each refactor slice.

## Test Environment
- Device: ESP32 + TFT + MCP23017 + SI4703 + TDA7439
- RPi bridge: `raspberry_moode_bridge.py` or `raspberry_volumio_bridge.py`
- PC bridge: `windows_foobar_bridge.py`
- Client: one browser tab over IP (not multiple tabs)

## Baseline Scenarios

### 1) Boot and Web Availability
- Power on ESP32 and wait for WiFi connect.
- Open WebUI from IP.
- Confirm page loads fully (no partial HTML, no missing controls).
- Reload page 10 times quickly; each load must complete.

### 2) Source Switching
- Switch among `TUN`, `Bluetooth`, `Raspberry PI`, `PCD`, `PCA`.
- Verify selected source button state is always correct.
- Verify no accidental bounce (example: PCD not jumping back to RPI).

### 3) RPi Transport Stability
- On `Raspberry PI` source press `Prev/Next/PlayPause` rapidly (20-30 actions).
- Confirm UI remains responsive during spam.
- Confirm metadata eventually matches actual track/state.
- Repeat with RPi offline; WebUI must still stay responsive.

### 4) PCD/Foobar Transport Stability
- On `PCD`, run `Prev/Next/PlayPause` rapidly (20-30 actions).
- Verify no frozen UI and no stale "one-track-behind" persistence.
- Disconnect PC bridge and confirm offline handling works (controls disabled, metadata hidden).

### 5) Web Controls
- Volume slider drag fast from min to max and back.
- EQ sliders move quickly in sequence.
- Frequency set/seek operations on TUN.
- Power/mute toggles repeatedly.

### 6) Standby/Wakeup
- Enter standby from WebUI.
- Open standby page and verify `/systemInfo` refreshes.
- Exit standby and confirm full UI is restored.

### 7) Long-run Soak
- Keep source on RPi and leave WebUI open for 30-60 min.
- Perform periodic transport commands every 2-3 min.
- Confirm no progressive lag, no partial loads, no lockups.

## Acceptance Criteria Per Refactor Slice
- All scenarios above pass.
- No new compile errors.
- No route contract change (`/status` fields preserved).
- UART and HTTP bridge protocols unchanged.

## Regression Logging Template
- Refactor slice:
- Firmware commit/checkpoint:
- Python bridge checkpoint:
- Passed scenarios:
- Failed scenarios:
- Notes (latency, freeze symptoms, repro steps):

