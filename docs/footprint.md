# Numeric code footprint

This document records **object-only** flash and RAM cost for `SegmentedNumber` formats and `CyclicCounter` on **ESP32-C6** (RISC-V `rv32imac` / `ilp32`), plus instance sizes and stack usage.

Regenerate numbers after changing the library:

```bash
cmake -S . -B build-dev -DCMAKE_BUILD_TYPE=Release -DAE_BUILD_TESTS=ON
cmake --build build-dev --target segmented-footprint-obj
python tools/measure_esp32c6_footprint.py --repo .
python tools/generate_footprint_docs.py --repo .
```

`measure_esp32c6_footprint.py` writes `docs/footprint_results.json`.
`generate_footprint_docs.py` refreshes only the marked generated tables below.

---

## Metrics

| Section | Meaning |
|---|---|
| `.text` | Machine code |
| `.rodata` | Read-only constants |
| `.data` | Initialized static RAM (and its flash image) |
| `.bss` | Zero-initialized static RAM |

Formulas:

```text
Total flash = .text + .rodata + .data
Static RAM  = .data + .bss
```

Notes:

* Sizes come from **minimal object-only** targets (`tests/footprint/minimal_*.cpp`) compiled with Espressif `riscv32-esp-elf-g++`, `-ffunction-sections` / `-fdata-sections`, no exceptions/RTTI.
* CRT, Unity, `printf`, iostream, and application frameworks are **not** included.
* Harnesses keep encode/decode/serialize reachable (anti-DCE) via `volatile` sinks.
* Reported values are the **incremental footprint of the numeric implementation** pulled into that translation unit.
* A full application size also depends on linker deduplication and whether `Log2`/`Exp2`/FixedPoint helpers are already linked from elsewhere.

---

## `sizeof(CompiledSegment)` is not flash

```text
sizeof(CompiledSegment) == 68 bytes   # host / ESP ILP32 measurement via fp-dump-meta
```

That is the size of the **universal C++ compile-time descriptor type**, not bytes of flash per segment.

* The descriptor array is used at compile time and does **not** materialize as a per-segment `.rodata` table of `68 * segment_count`.
* Encode/decode paths are specialized per format and segment index.
* Real constant flash data are the shared FixedPoint `Log2` / `Exp2` tables when those curves are used; their size does **not** scale with code count or segment count.
* Do **not** treat `sizeof(CompiledSegment) * segments` as flash consumption.

---

## SegmentedNumber ESP32-C6 footprint

<!-- BEGIN GENERATED FOOTPRINT -->
Toolchain: `riscv32-esp-elf-g++ (esp-14.2.0_20241119)` (Espressif riscv32-esp-elf)  
Target: ESP32-C6 / riscv32 ilp32  

### ESP32-C6 `-Os`

| Format | Opt | .text | .rodata | .data | .bss | Flash | RAM | sizeof(runtime) | sizeof(number) | sizeof(wire) | Codes | Segments |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| RSSI | -Os | 964 | 0 | 0 | 0 | 964 | 0 | 1 | 1 | 1 | 128 | 1 |
| Temperature | -Os | 5126 | 208 | 0 | 0 | 5334 | 0 | 2 | 2 | 2 | 1021 | 3 |
| Humidity | -Os | 2962 | 0 | 0 | 0 | 2962 | 0 | 2 | 2 | 1 | 256 | 3 |
| CO2 | -Os | 3658 | 208 | 0 | 0 | 3866 | 0 | 2 | 2 | 4 | 822 | 3 |
| RX window | -Os | 6596 | 208 | 0 | 0 | 6804 | 0 | 4 | 4 | 4 | 3046 | 4 |
| Battery | -Os | 4398 | 208 | 0 | 0 | 4606 | 0 | 2 | 2 | 1 | 256 | 2 |
| ConnectDuration | -Os | 3068 | 208 | 0 | 0 | 3276 | 0 | 4 | 4 | 1 | 256 | 2 |
| Thermometer | -Os | 9124 | 208 | 0 | 0 | 9332 | 0 | - | - | - | - | - |
| All seven | -Os | 12080 | 208 | 0 | 0 | 12288 | 0 | - | - | - | - | - |

### ESP32-C6 `-O2`

| Format | Opt | .text | .rodata | .data | .bss | Flash | RAM | sizeof(runtime) | sizeof(number) | sizeof(wire) | Codes | Segments |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| RSSI | -O2 | 1136 | 0 | 0 | 0 | 1136 | 0 | 1 | 1 | 1 | 128 | 1 |
| Temperature | -O2 | 6050 | 208 | 0 | 0 | 6258 | 0 | 2 | 2 | 2 | 1021 | 3 |
| Humidity | -O2 | 5800 | 0 | 0 | 0 | 5800 | 0 | 2 | 2 | 1 | 256 | 3 |
| CO2 | -O2 | 3890 | 208 | 0 | 0 | 4098 | 0 | 2 | 2 | 4 | 822 | 3 |
| RX window | -O2 | 8536 | 208 | 0 | 0 | 8744 | 0 | 4 | 4 | 4 | 3046 | 4 |
| Battery | -O2 | 4252 | 208 | 0 | 0 | 4460 | 0 | 2 | 2 | 1 | 256 | 2 |
| ConnectDuration | -O2 | 2786 | 208 | 0 | 0 | 2994 | 0 | 4 | 4 | 1 | 256 | 2 |
| Thermometer | -O2 | 14868 | 208 | 0 | 0 | 15076 | 0 | - | - | - | - | - |
| All seven | -O2 | 19838 | 208 | 0 | 0 | 20046 | 0 | - | - | - | - | - |
<!-- END GENERATED FOOTPRINT -->

Thermometer = Temperature + Humidity + CO2 + Battery in one TU.
All seven = RSSI + Temperature + Humidity + CO2 + RX + Battery + ConnectDuration.

---

## CyclicCounter ESP32-C6 footprint

<!-- BEGIN GENERATED CYCLIC FOOTPRINT -->
### ESP32-C6 `-Os`

| Configuration | Wire B | Runtime B | .text | .rodata | .data | .bss | Flash | RAM | Half-range |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `uint8_t` -> `uint16_t` | 1 | 2 | 224 | 0 | 0 | 0 | 224 | 0 | 127 |
| `uint8_t` -> `uint32_t` | 1 | 4 | 182 | 0 | 0 | 0 | 182 | 0 | 127 |
| `uint16_t` -> `uint32_t` | 2 | 4 | 136 | 0 | 0 | 0 | 136 | 0 | 32767 |

### ESP32-C6 `-O2`

| Configuration | Wire B | Runtime B | .text | .rodata | .data | .bss | Flash | RAM | Half-range |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `uint8_t` -> `uint16_t` | 1 | 2 | 244 | 0 | 0 | 0 | 244 | 0 | 127 |
| `uint8_t` -> `uint32_t` | 1 | 4 | 188 | 0 | 0 | 0 | 188 | 0 | 127 |
| `uint16_t` -> `uint32_t` | 2 | 4 | 136 | 0 | 0 | 0 | 136 | 0 | 32767 |

All three configurations: `.rodata = 0`, `.data = 0`, `.bss = 0`, heap = 0, tables = 0, 64-bit arithmetic helper undefs = 0.
<!-- END GENERATED CYCLIC FOOTPRINT -->

---

## Code sharing

Standalone footprints sum larger than a combined TU because the linker keeps one copy of shared helpers:

* `Log2` / `Exp2` and their FixedPoint tables
* FixedPoint division / scale conversion
* common curve helpers (`GeomApprox`, ramp, …)
* `TieredInt` wire paths
* shared wire API glue

<!-- BEGIN GENERATED SHARING -->
### ESP32-C6 `-Os`

| Bundle | Standalone .text sum | Combined .text | Saved |
|---|---:|---:|---:|
| Temperature + CO2 (standalone sum) | 8784 | (linked separately) | - |
| Thermometer (T+H+CO2+Bat) | 16144 | 9124 | 7020 |
| All seven | 26772 | 12080 | 14692 |

### ESP32-C6 `-O2`

| Bundle | Standalone .text sum | Combined .text | Saved |
|---|---:|---:|---:|
| Temperature + CO2 (standalone sum) | 9940 | (linked separately) | - |
| Thermometer (T+H+CO2+Bat) | 19992 | 14868 | 5124 |
| All seven | 32450 | 19838 | 12612 |
<!-- END GENERATED SHARING -->

---

## Constant data

Shared tables used by logarithmic / exponential / geometric / ramp math (ESP32-C6, ILP32):

| Symbol | Element type | Count | Bytes | Purpose |
|---|---|---:|---:|---|
| `InvPow2TableHolder<FixedPoint<long,32>,26>::kTable` | `FixedPoint<long,32>` (Rep <= 32-bit) | 26 | 104 | `Log2` / fractional `Exp2` bit weights |
| `Exp2MantTableHolder<FixedPoint<unsigned long,4>,26>::kTable` | `FixedPoint<unsigned long,4>` (Rep <= 32-bit) | 26 | 104 | `Exp2` mantissa factors |

Together these account for the observed **208** bytes of `.rodata` on formats that pull in `Log2`/`Exp2`.

* Elements are our `FixedPoint` values; Rep is at most 32 bits.
* There are no homemade raw Q30/Q31 tables.
* Tables are **common** across formats; linking several formats in one TU does **not** duplicate them (still 208 B `.rodata`).
* Purely linear formats (e.g. RSSI, Humidity in this suite) can show `.rodata = 0`.
* `CyclicCounter` has `.rodata = 0` (no tables, no 64-bit helper undefs).

---

## Runtime object size

```text
sizeof(SegmentedNumber) == sizeof(runtime_type)
sizeof(CyclicCounter)   == sizeof(ValueType)
```

No instance stores segment descriptors, lookup tables, heap pointers, or decoder state.

| Type | sizeof object | Wire size | Heap | Additional runtime state |
|---|---:|---:|---|---|
| RSSI `SegmentedNumber` | 1 | 1 | 0 | none |
| Temperature | 2 | <=2 | 0 | none |
| Humidity | 2 | 1 | 0 | none |
| CO2 | 2 | <=4 | 0 | none |
| RX window | 4 | <=4 | 0 | none |
| Battery | 2 | 1 | 0 | none |
| ConnectDuration | 4 | 1 | 0 | none |
| `CyclicCounter<uint8_t,uint16_t>` | 2 | 1 | 0 | none |
| `CyclicCounter<uint8_t,uint32_t>` | 4 | 1 | 0 | none |
| `CyclicCounter<uint16_t,uint32_t>` | 4 | 2 | 0 | none |

---

## Stack usage

Measured with GCC `-fstack-usage` on the same ESP32-C6 `-Os` object builds used for footprint (per-function static frame size from `.su` files). These are **compiler-reported frames**, not hand estimates. Call-graph worst case for deep helpers (e.g. RX `LinearRampApproxRuntime`) is listed in the Method column.

<!-- BEGIN GENERATED STACK -->
| Type | Encode | Decode | Serialize | Deserialize | Worst case | Method |
|---|---:|---:|---:|---:|---:|---|
| Temperature | 32 | 32 | 32 | 32 | 48 | GCC `-fstack-usage` ESP32-C6 `-Os` (.su) |
| CO2 | 32 | 32 | 32 | 32 | 48 | GCC `-fstack-usage` ESP32-C6 `-Os` (.su) |
| RX | 32 | 48 | 32 | 48 | 112 | GCC `-fstack-usage` ESP32-C6 `-Os` (.su) |
| Battery | 32 | 32 | 32 | 32 | 48 | GCC `-fstack-usage` ESP32-C6 `-Os` (.su) |
| CyclicCounter u8->u32 | 32 | 16 | 32 | 32 | 32 | GCC `-fstack-usage` ESP32-C6 `-Os` (.su) |
<!-- END GENERATED STACK -->

Deserialize for `CyclicCounter` is contextual (`TryDeserializeAnd*`); the Advance probe covers that path’s stack class.

---

## Desktop comparison (optional)

Desktop GCC / Clang / MSVC host objects are useful for relative trends only. Do **not** mix them into the ESP32-C6 tables above, and do not convert desktop nanoseconds into ESP cycles. Prefer regenerating host sizes with the same `minimal_*.cpp` sources if a comparison table is needed for a PR.
