# Æthernet Numeric Types

Specialized numeric formats for constrained systems without an FPU or with limited bandwidth and memory.
They are used across the Æthernet C++ client to represent durations, counters, sizes, fixed-point values, and compact wire codes.

---

## Table of Contents

1. [Overview](#overview)
2. [TieredInt](#tieredint)
3. [FixedPoint](#fixedpoint)
4. [Exponential](#exponential)
5. [Text IO](#text-io)
6. [Ostream IO](#ostream-io)
7. [Wire IO](#wire-io)
8. [Combined Types](#combined-types)
9. [SegmentedNumber](#segmentednumber)
10. [CyclicCounter](#cycliccounter)
11. [Integration Notes](#integration-notes)
12. [Running Tests](#running-tests)

---

## Overview

The Æthernet C++ client targets devices where every byte and every CPU cycle matters.
The numeric types in this repository are header-only, compile-time configured, and deterministic.
They separate two concerns:

* **logical value** — what the application means: milliseconds, bytes, ratios, sensor values;
* **wire/storage representation** — how many bytes are needed to store or transmit that value.

The core types are:

* `TieredInt` — compact integer serialization with compile-time tier boundaries;
* `FixedPoint` — binary-scaled fixed point over an integral or packed integral representation;
* `Exponential` — logarithmic code mapping for values that span several orders of magnitude;
* `CyclicCounter` — full local counter with truncated modular wire bits.

---

## TieredInt

`TieredInt` stores small integer values in fewer bytes while still supporting much larger values.
Compression and decompression happen only during serialization/deserialization; normal arithmetic uses the logical integer value.

```cpp
using SmallCounter = ae::TieredInt<std::uint8_t, 254>;
using SizeBytes = ae::TieredInt<std::uint8_t, 250, 1500>;
using SignedSmall = ae::TieredInt<std::int8_t, 10, 20>;
```

The first template parameter is an integral **cell type**. It defines the minimum serialized chunk size, not merely a C++ value type.
For `std::uint8_t`, the first serialized chunk is 1 byte; for `std::uint16_t`, it is 2 bytes; for `std::uint32_t`, it is 4 bytes.

Each next tier appends a chunk whose size is the sum of the previous chunks. Therefore total serialized sizes grow as:

* `std::uint8_t` cell: 1, 2, 4, then 8 bytes;
* `std::uint16_t` cell: 2, 4, then 8 bytes;
* `std::uint32_t` cell: 4, then 8 bytes.

The remaining template parameters are maximum logical values for each compact tier:

```cpp
using SizeBytes = ae::TieredInt<std::uint8_t, 250, 1500>;
```

For this type:

* values `0..250` serialize to 1 byte;
* values `251..1500` serialize to 2 bytes;
* values `1501..1967580` serialize to 4 bytes.

This is useful for packet or payload sizes: small control payloads use one byte, normal MTU-sized values up to 1500 bytes use two bytes, and rare large values still fit up to 1,967,580 bytes, about 1.88 MiB.

For signed `TieredInt`, tier boundaries are validated in wire space after ZigZag encoding.

`TieredInt` needs extension header space. For a `std::uint8_t` cell, the first tier max must be below `255`, so `254` is valid and `255` is not:

```cpp
using Good = ae::TieredInt<std::uint8_t, 254>;

// Invalid: no extension header left.
using Bad = ae::TieredInt<std::uint8_t, 255>;
```

---

## FixedPoint

`FixedPoint<Rep, Max>` represents a logical value as:

```text
logical_value = raw_value * 2^kScaleExp
```

`Rep` is the raw storage type: a built-in integral type or an integer-like packed type such as `TieredInt`.
`Max` is the required logical range bound. The implementation chooses the most precise binary scale that still covers `Max`.

The binary point is not constrained to sit inside the bit width of `Rep`. It can be far to the right or far to the left of the stored integer.

Examples with `std::uint8_t` storage:

```cpp
using Micro = ae::FixedPoint<std::uint8_t, 0.001>;
// kScaleExp = -17
// step = 2^-17 ~= 0.000007629
// raw 131 ~= 0.000999
// max representable ~= 0.001945

using Huge = ae::FixedPoint<std::uint8_t, 1000000.0>;
// kScaleExp = 12
// step = 4096
// raw 244 ~= 999424
// max representable = 1044480
```

So the point may be many bits beyond an 8-bit value in either direction.
For tiny ranges, the raw byte becomes a fine fractional value. For huge ranges, the raw byte becomes a coarse bucket index.

Signed storage uses a symmetric raw range around zero. For example, `std::int8_t` uses `-127..+127`, not the full `INT8_MIN..INT8_MAX` asymmetry.

Runtime arithmetic is integer-only: shifts, rounding, saturated raw add/sub, and scale conversion. Floating-point appears only at compile-time or parse/debug boundaries.

---

## Exponential

`Exponential` is an approximate logical codec for values that span orders of magnitude.
It stores only a compact integer code (`WireT`). The decoded runtime value lives in `RuntimeT`.

```cpp
using Runtime = ae::FixedPoint<std::uint32_t, 1.0>;
using E = ae::Exponential<Runtime, std::uint8_t, 0.001, 1.0>;

constexpr auto encoded = E::FromDouble(0.1);
constexpr auto decoded = encoded.ToRuntime();
```

API semantics:

* `Code()` / `FromCode()` construct from an exact wire code without magnitude approximation.
* `Value()` / `ToRuntime()` decode a code to a runtime value.
* `FromRuntime()` / `FromDouble()` encode a runtime value to a code.
* `Abs`, `Min`, `Max`, and `Clamp` operate on wire codes directly.

Code `0` is zero. For unsigned runtime types, code `1` is `MinMagnitude`, and `BoundaryCode` is `BoundaryMagnitude`.
For signed runtime types, even codes are positive magnitudes and odd codes are negative magnitudes: code `2` is `+MinMagnitude`, code `1` is `-MinMagnitude`.

When `BoundaryCode` is omitted, `Exponential` uses the maximum logical code representable by `WireT` (`numeric_traits<WireT>::kMaxBoundaryCode`).
For built-in wire types such as `std::uint8_t`, the default boundary code is `255`.
For `TieredInt<std::uint8_t, 249, 1529>`, the default boundary code is `1529`.
Pass the fifth template parameter only when you intentionally want to use a smaller code range than the full wire range.

Use `FixedPoint` or an integral runtime type for embedded builds. For desktop, server, tests, or reference code, include the optional floating runtime header before using `float` or `double` as `RuntimeT`:

```cpp
#include <ae-numeric/exponential_floating_runtime.h>

using EFloat = ae::Exponential<float, std::uint8_t, 0.001f, 1.0f>;
```

`float` and `double` are runtime value types only. They are not valid storage types for `TieredInt` or `FixedPoint`.

---

## Text IO

`ae-numeric/text_io.h` provides integer-only decimal conversion for `TieredInt`, `FixedPoint`, and `Exponential`.
`FixedPoint` formatting derives the logical value from `raw * 2^kScaleExp` using shifts and exact rational fraction expansion.

```cpp
#include <ae-numeric/text_io.h>

using F = ae::FixedPoint<std::uint8_t, 100.0>;

auto s = ae::ToString(F::FromRaw(1));  // "0.5"

F parsed;
ae::FromString("10.5", parsed);
```

Core APIs:

* `ae::ToString(value)` — allocates a `std::string`;
* `ae::FromString(text, value)` — returns `false` on malformed or out-of-range input;
* `ae::ToChars(first, last, value)` — non-allocating buffer write;
* `ae::FromChars(first, last, value)` — non-allocating parse.

---

## Ostream IO

`ae-numeric/ostream_io.h` provides optional `operator<<` for `TieredInt` and `FixedPoint`.
Include it only where stream output is actually needed; core numeric headers do not pull in `<ostream>`.

```cpp
#include <ae-numeric/ostream_io.h>

using F = ae::FixedPoint<std::uint8_t, 100.0>;

std::cout << ae::TieredInt<std::uint8_t, 254>{123};  // 123
std::cout << F::FromRaw(1);                          // 0.5
```

---

## Wire IO

`ae-numeric/wire_io.h` provides a uniform serialization API for built-in integers, `TieredInt`, and `FixedPoint` through `wire_traits<T>` and convenience functions.
`ae-numeric/exponential_wire_io.h` adds wire traits for `Exponential`.

```cpp
#include <ae-numeric/wire_io.h>

using F = ae::FixedPoint<ae::TieredInt<std::uint8_t, 254>, 60.0>;

std::uint8_t buf[ae::MaxWireBytes<F>()];
auto n = ae::Serialize(F::FromInteger(30), buf);
auto restored = ae::Deserialize<F>(buf, n).value;
```

Serialization rules:

* built-in integers — fixed-width little-endian (`sizeof(T)` bytes);
* `TieredInt` — compact variable-length encoding;
* `FixedPoint` — serializes only the raw `Rep` storage;
* `Exponential` — serializes only the `WireT` code.

Short or invalid buffers are handled by the corresponding deserializer result or debug assertions; the numeric core does not use exceptions.

---

## Combined Types

You can combine `TieredInt`, `FixedPoint`, and `Exponential` to get compact wire encoding and useful runtime semantics.

### Exponential duration over packed integer codes

This maps durations from 1 ms to 60 s onto exponential codes stored in a packed integer:

```cpp
#include <ae-numeric/exponential_wire_io.h>

using Runtime = ae::FixedPoint<std::uint32_t, 60.0>;
using Wire = ae::TieredInt<std::uint8_t, 249, 1529>;
using Duration = ae::Exponential<Runtime, Wire, 0.001, 60.0>;

constexpr auto one_second = Duration::FromDouble(1.0);

std::uint8_t buf[ae::MaxWireBytes<Duration>()];
auto n = ae::Serialize(one_second, buf);
```

Here `Wire` stores codes `0..249` in one byte and codes `250..1529` in two bytes.
Because `Duration` stores only the code, small durations use one byte. With this particular logarithmic mapping, the one-byte region covers approximately 1 ms through 6 ms; the full range up to 60 s uses two bytes.

### Latency histogram: 1 ms to about 43 s

For latency and round-trip measurements, it is often useful to keep the common fast path in one byte while still allowing large outliers.

```cpp
using Runtime = ae::FixedPoint<std::uint32_t, 43.0>;
using Wire = ae::TieredInt<std::uint8_t, 254>;
using Latency = ae::Exponential<Runtime, Wire, 0.001, 43.0, 510>;
```

For `TieredInt<std::uint8_t, 254>`, wire codes `0..254` serialize to one byte and codes `255..510` serialize to two bytes.
With the exponential mapping from 1 ms to 43 s:

* code `1` is 1 ms;
* code `254` is about 200.9 ms;
* code `255` is about 205.2 ms;
* code `510` is 43 s.

So typical 1 ms to roughly 200 ms latencies stay in one byte, while slower values up to 43 s still fit in two bytes.

### Linear ping values over a packed integer

For telemetry where linear precision is preferred over relative precision, use `FixedPoint` over `TieredInt`.
This example stores ping time in milliseconds with 1 ms resolution:

```cpp
using PingRaw = ae::TieredInt<std::uint8_t, 250, 1500>;
using PingMs = ae::FixedPoint<PingRaw, 1000000.0>;

constexpr auto fast = PingMs::FromInteger(42);
constexpr auto mtu_like_timeout = PingMs::FromInteger(1500);
```

For this type:

* `0..250 ms` serialize to one byte;
* `251..1500 ms` serialize to two bytes;
* rare large values can use the four-byte tier;
* the physical representable maximum is about 1,967,580 ms, while the declared safe range is 1,000,000 ms.

This is a linear scale: every raw step is one millisecond. Use `Exponential` instead when relative precision is more important than absolute millisecond spacing.

---

## SegmentedNumber

`SegmentedNumber` is a header-only piecewise quantized number. The object stores only the physical runtime value. A dense packed rank is computed when encoding and is serialized through a compiled `uint8_t` or `TieredInt` wire type.

The description splits four layers:

1. runtime representation (`runtime::Fixed<Rep>` or opt-in `runtime::Floating<float|double>`);
2. mathematical curves of representable values;
3. assignment of those codes to wire lengths 1/2/4/8 bytes;
4. the serializable packed rank.

Bounds and steps are written with `ae::Decimal` / `ae::Ratio`, not `double`.

```cpp
#include <ae-numeric/segmented_number.h>
#include <ae-numeric/segmented_number_wire_io.h>

template <std::int64_t M, int E = 0>
using D = ae::Decimal<M, E>;

using Spec = ae::seg::Format<
    ae::seg::runtime::Fixed<std::int16_t>,
    ae::seg::wire::AutoTiered<std::uint8_t, ae::seg::wire::MaxBytes<2>>,
    ae::seg::compute::Formula,
    ae::seg::Layout<
        ae::seg::GeometricStep<
            ae::seg::Range<D<-40>, D<10>>,
            ae::seg::Intervals<349>,
            ae::seg::StepAtUpper<D<1, -1>>,
            ae::seg::Place<ae::seg::Bytes<2>>>,
        ae::seg::UniformStep<
            ae::seg::Range<D<10>, D<352, -1>>,
            ae::seg::Step<D<1, -1>>,
            ae::seg::Place<ae::seg::Bytes<1>>>,
        ae::seg::GeometricStep<
            ae::seg::Range<D<352, -1>, D<125>>,
            ae::seg::Intervals<419>,
            ae::seg::StepAtLower<D<1, -1>>,
            ae::seg::Place<ae::seg::Bytes<2>>>>>;

using Temperature = ae::seg::Compile<Spec>;

static_assert(sizeof(Temperature) == sizeof(Temperature::runtime_type));
static_assert(Temperature::kCodeCount == 1021);
static_assert(Temperature::kMaxWireBytes == 2);
```

`Compile<Spec>` is `SegmentedNumber<Spec>`. The object does not store the wire rank. Encode with `TryEncode` / `TryFromRuntime`; out-of-range input is rejected unless `Saturating` is used. Comparisons use the runtime value, never the packed rank: rank order need not follow physical order (the temperature window uses 1-byte codes in the middle and 2-byte codes on both tails).

Curve primitives: `UniformStep`, `UniformValues`, `ExponentialValues`, `GeometricStep`, `LinearStepRamp`. Allocation helpers: `Intervals<N>`, `FillTier`, `MinimumIntervals`, `AutoSplit`, and `ContinuousExponential` with `WireCuts` / `OptimizeCuts`.

Two compute backends:

* `compute::Formula` (default) — small per-segment coefficients, no per-code table, no runtime `float`/`double` for `Fixed` runtime. Segment selection is O(S). Uniform and linear-ramp paths are O(1) (integer square root for the ramp). Exponential and geometric decode use integer exponentiation-by-squaring of a compiled ratio (O(log n) multiplies). Encode of those curves binary-searches the selected segment and checks neighboring codes. Serialization is O(1), at most 8 bytes, heap = 0;
* `compute::Lookup` — consteval decoded-raw table, O(1) decode and O(log N) encode, used as a Formula oracle. Flash/data is O(N).

Shared physical endpoints are encoded once. The segment with the smaller wire size owns the joint; if the sizes match, the previous physical segment owns it. Unused packed ranks deserialize with `bytes_read == 0`.

Floating runtime is opt-in and does not change the wire ABI:

```cpp
#include <ae-numeric/segmented_number_floating_runtime.h>

using FloatSpec = ae::seg::Format<
    ae::seg::runtime::Floating<double>,
    ae::seg::wire::AutoTiered<std::uint8_t, ae::seg::wire::MaxBytes<2>>,
    ae::seg::compute::Formula,
    typename Spec::layout_type>;
```

Release footprint binaries (section GC, volatile sinks) are the `footprint-*` / `segmented-footprint` targets in `tests/`.

---

## CyclicCounter

`CyclicCounter<WireType, ValueType>` keeps a full unsigned counter locally and sends only its low bits on the wire:

```text
wire = value mod (max(WireType) + 1)
```

Example with `CyclicCounter<std::uint8_t, std::uint32_t>`:

```cpp
Counter counter{1001};
std::uint8_t wire = counter.WireValue();  // 233
// ... later, after gaps, peer sends wire 237 ...
auto restored = counter.TryRestore(237);  // 1005; counter still 1001
counter.TryAdvance(237);                  // counter becomes 1005
```

Wire wrap uses the same rule: `1023`/`255` then wire `0` restores `1024`.

**CyclicCounter cannot be deserialized statelessly** because the truncated wire value does not contain the epoch/high bits. There is no `wire_traits<CyclicCounter>` and `Deserialize<CyclicCounter>(…)` does not compile. Deserialize the `WireType` first (or use `TryDeserializeAndRestore` / `TryDeserializeAndAdvance` on an existing counter) and restore relative to a live full value.

Restoration is unambiguous only for absolute distances strictly less than half the wire space (`127` for `uint8_t`, `32767` for `uint16_t`). Distance exactly half (`128` / `32768`) is ambiguous. `TryAdvance` updates the local base only when the restored value is strictly newer. `sizeof(CyclicCounter) == sizeof(ValueType)`.

---

## Integration Notes

* Header-only numeric types.
* C++20.
* Deterministic integer runtime paths for embedded use.
* Optional floating runtime support for `Exponential` is isolated in `ae-numeric/exponential_floating_runtime.h`.
* Optional floating runtime support for `SegmentedNumber` is isolated in `ae-numeric/segmented_number_floating_runtime.h`.
* Designed for low-overhead serialization on MCUs and constrained networks.

---

## Running Tests

Build and run the unit tests with CMake:

```bash
git clone https://github.com/aethernetio/aethernet-numeric.git
cd aethernet-numeric
git submodule update --init --recursive

cmake -S . -B build-dev \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_STANDARD=20 \
  -DAE_BUILD_TESTS=ON

cmake --build build-dev
ctest --test-dir build-dev --output-on-failure
```

Select a specific compiler through normal CMake command-line options when needed.

---

© Æthernet Inc. — Part of the Æthernet Core Client Library
[https://aethernet.io](https://aethernet.io)
