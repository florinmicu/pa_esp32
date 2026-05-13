# Target Architecture (Aggressive Refactor)

## Objectives
- Decouple responsibilities in the `.ino` monolith.
- Keep external behavior/protocols stable while internals are reorganized.
- Unify Python bridges around one shared core.

## Compatibility Contract (must stay stable during migration)
- Web routes and payload field names currently consumed by WebUI.
- UART protocol with Pi bridge: `CMD|...`, `GET`, `STAT|state|title|artist|extra`.
- Foobar HTTP bridge endpoints: `/state`, `/cmd?c=...`.

## ESP32 Module Boundaries

### 1) AppController
- Owns deterministic scheduling order (tick functions).
- Applies pending actions and orchestrates services.

### 2) WebApi
- Owns HTTP routes and response helpers.
- Converts requests to intents/pending actions only.
- Must avoid direct heavy I2C/TFT operations.

### 3) SerialBridgeRpi
- Owns UART parsing and command TX queue/backoff.
- Publishes parsed player snapshots.

### 4) PcBridgeClient
- Owns foobar bridge polling and command dispatch.
- Publishes parsed player snapshots.

### 5) AudioService
- Owns TDA7439 input/volume/EQ/balance apply path.
- Central place for deferred audio apply.

### 6) FmService
- Owns SI4703 frequency/seek/RDS behavior.

### 7) UiService
- Owns TFT rendering for source panes, metadata, standby, indicators.
- Consumes read-only state snapshots.

### 8) InputService
- Owns touch, MCP buttons/encoder, IR decode.
- Emits intents only.

### 9) SettingsStore
- Owns Preferences keys, load/save, debounced writes.

### 10) NetworkTimeService
- Owns WiFi reconnect and NTP/RTC sync.

## Python Bridge Target Layout

```text
bridge_core/
  protocol.py      # STAT formatting/sanitization, state normalization
  config.py        # env/CLI config loading
  serial_loop.py   # reconnect/readline/write loop primitives
  commands.py      # command parse + dispatch mapping
  http_client.py   # shared timeout/retry wrappers
  backends/
    moode.py
    volumio.py
    beefweb.py
```

- Keep root scripts as thin wrappers for backward compatibility:
  - `raspberry_moode_bridge.py`
  - `raspberry_volumio_bridge.py`
  - `windows_foobar_bridge.py`

## Migration Sequencing Rules
- Move one concern at a time.
- Preserve call order initially when splitting `loop()`.
- Keep compatibility wrappers until full regression passes.
- After each slice, run baseline checklist.

