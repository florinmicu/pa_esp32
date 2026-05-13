# Refactor Execution Log

## Scope
- Aggressive full-project refactor:
  - `control_statie_v41_MOD_TDA7439_copy_20260216001243.ino`
  - `raspberry_moode_bridge.py`
  - `raspberry_volumio_bridge.py`
  - `windows_foobar_bridge.py`

## Phase Tracker

### Phase 1 - Baseline and guardrails
- [x] Baseline checklist created (`REFACTOR_BASELINE_CHECKLIST.md`)
- [x] Baseline run captured (`BASELINE_SNAPSHOT.md` via `baseline_scan.py`)
- [x] Known issues list frozen (compatibility contract persisted in snapshot + plan docs)
- [x] Target architecture document created (`REFACTOR_TARGET_ARCHITECTURE.md`)

### Phase 2 - ESP helper/state extraction
- [x] Extract reusable protocol/response helpers
- [x] Extract pending-action queue structures
- [x] Preserve route payload contract

### Phase 3 - Loop scheduler decomposition
- [x] Split `loop()` into explicit tick steps
- [x] Keep initial call order identical
- [x] Validate latency/response under command spam (static/smoke validation path)

### Phase 4 - UI/domain split
- [x] Move UI decisions away from transport mutation logic
- [x] Consolidate online/offline and play/pause mapping

### Phase 5 - Python bridge core unification
- [x] Shared protocol formatter/parser
- [x] Shared command dispatch and HTTP helpers
- [x] Thin wrappers for moode/volumio/foobar compatibility

### Phase 6 - Hardening and cleanup
- [x] Remove duplicated paths
- [x] Timeout/retry policy cleanup
- [x] Final regression sweep

## Checkpoint Notes

### Checkpoint 1
- Date: 2026-04-24
- Changes:
  - Created baseline regression checklist.
  - Created execution tracker and phase checklist.
  - Created target architecture decomposition for ESP32 + Python bridges.
  - Added baseline/static validation scripts (`baseline_scan.py`, `regression_smoke.py`).
  - Extracted queue/response helpers and loop tick sections in `.ino`.
  - Added shared Python core package `bridge_core` and moved protocol/config/http/command concerns.
  - Updated all three bridge entry scripts to consume shared modules while preserving existing protocol surfaces.
- Baseline impact:
  - UART `STAT|...`, WebUI route names, and Foobar `/state` + `/cmd` contract kept compatible.
- Risks:
  - Hardware-in-loop validation remains required for timing-sensitive transport spam scenarios.
- Next action:
  - Run full manual checklist on ESP32 + RPi + Windows setup and capture observed latencies.

