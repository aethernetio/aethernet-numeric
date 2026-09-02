# Numeric micro-benchmarks

Desktop host timings for `SegmentedNumber` and `CyclicCounter`.

**These results are desktop nanoseconds, not ESP32-C6 cycle counts.** Do not rescale them to MCU cycles.

## Method

| Item | Value |
|---|---|
| Architecture | Host desktop (Windows x86_64 when measured here) |
| Toolchain | Host CMake compiler for `numeric-bench` |
| Optimization | Release (`-O2` / MSVC `/O2` as configured) |
| CPU frequency | Host turbo frequency (not fixed MCU MHz) |
| Iterations | 200 000 timed ops after warm-up |
| Warm-up | ~5% of iterations before the timed loop |
| Anti-DCE | `volatile` sinks on encode/decode/wire results |
| Measured ops | Encode, Decode, Serialize, Deserialize, round-trip; CyclicCounter WireValue / TryRestore / TryAdvance / contextual deserialize |

Build and run:

```bash
cmake -S . -B build-bench -DCMAKE_BUILD_TYPE=Release -DAE_BUILD_TESTS=ON
cmake --build build-bench --target numeric-bench
./build-bench/tests/numeric-bench   # or build-bench\tests\Release\numeric-bench.exe
```

Machine-readable lines are prefixed `BENCH_SEG` / `BENCH_CYC`. Refresh the table below after a run (paste or feed into the doc generator when extended).

---

## Results

<!-- BEGIN GENERATED BENCHMARKS -->
`BENCH_HOST arch=desktop note=nanoseconds_not_esp32_cycles iters=200000`

| Name | Encode ns | Decode ns | Serialize ns | Deserialize ns | Round-trip ns | Notes |
|---|---:|---:|---:|---:|---:|---|
| Rssi | 24.170 | 0.238 | 474.880 | 0.268 | 440.944 | logical=-40 wire_bytes=1 |
| Temperature_center | 20.201 | 0.234 | 5676.991 | 33.132 | 5848.599 | logical=25 wire_bytes=1 |
| Temperature_low | 21.945 | 0.279 | 7578.979 | 226.782 | 7485.159 | logical=-35 wire_bytes=2 |
| Temperature_high | 22.858 | 0.258 | 7486.931 | 219.238 | 7515.242 | logical=100 wire_bytes=2 |
| Humidity | 23.458 | 0.259 | 636.130 | 0.259 | 633.419 | logical=55 wire_bytes=1 |
| Co2_1B | 23.430 | 0.224 | 6986.739 | 76.366 | 7125.114 | logical=600 wire_bytes=1 |
| Co2_2B | 26.120 | 0.252 | 6709.723 | 83.351 | 6619.372 | logical=2500 wire_bytes=2 |
| Co2_4B | 24.526 | 0.252 | 7105.739 | 78.898 | 7344.620 | logical=18000 wire_bytes=4 |
| Rx_1B | 23.081 | 0.252 | 5031.953 | 1.951 | 5078.962 | logical=1 wire_bytes=1 |
| Rx_2B | 23.729 | 0.239 | 9176.965 | 208.262 | 10699.008 | logical=120 wire_bytes=2 |
| Rx_4B | 30.999 | 0.218 | 10707.724 | 54.874 | 10635.228 | logical=7200 wire_bytes=4 |
| Battery | 53.407 | 0.418 | 1362.323 | 0.219 | 1267.943 | logical=3 wire_bytes=1 |
| ConnectDuration | 44.609 | 0.382 | 2543.144 | 1.254 | 2137.385 | logical=5 wire_bytes=1 |
| CyclicCounter WireValue | 0.627 | - | - | - | - | desktop ns |
| CyclicCounter TryRestore forward | 0.627 | - | - | - | - | desktop ns |
| CyclicCounter TryRestore backward | 0.627 | - | - | - | - | desktop ns |
| CyclicCounter TryAdvance | 0.598 | - | - | - | - | desktop ns |
| CyclicCounter contextual deserialize | 0.598 | - | - | - | - | desktop ns |
<!-- END GENERATED BENCHMARKS -->

### How to read SegmentedNumber rows

* **CO₂** and **RX** are split by approximate wire tier via sample selection (1 / 2 / 4-byte regimes where the format supports them).
* **Temperature** is split into central one-byte, lower tail, and upper tail samples.
* Encode/Decode are pure rank codecs; Serialize/Deserialize include `wire_traits` packing.

### CyclicCounter rows

| Operation | Meaning |
|---|---|
| WireValue | Truncate full value to wire bits |
| TryRestore forward | Restore a newer truncated sample without mutating |
| TryRestore backward | Restore an older truncated sample without mutating |
| TryAdvance | Restore and update base only when newer |
| Contextual deserialize | `TryDeserializeAndAdvance` from a wire buffer |
