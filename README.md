# Æthernet Numeric Types

Specialized numeric formats for constrained systems without FPU or with limited bandwidth and memory.
Used across the Æthernet C++ client to efficiently represent durations, fixed-point values, and tiered integers.

---

## Table of Contents

1. [Overview](#overview)
2. [TieredInt](#tieredint)
3. [Fixed](#fixed)
4. [Exponent](#exponent)
5. [Text IO](#text-io)
6. [Wire IO](#wire-io)
7. [Combined Types](#combined-types)
8. [Integration Notes](#integration-notes)
9. [Running Tests](#running-tests)

---

## Overview

The **Æthernet C++ client** is designed for restricted devices with limited memory, bandwidth, and no floating point unit.
To address these constraints, we've implemented specialized numeric formats that maintain precision, range, and compactness without sacrificing runtime performance.

All formats are **header-only**, compile-time defined, and interoperable.

---

## TieredInt

This integer format efficiently compresses frequently used numbers into fewer bytes.
Numbers below **254** (adjustable via template parameters) can occupy a single byte; larger values use additional tiers.

Compression and decompression occur only during serialization and deserialization, keeping runtime overhead minimal.

Although other variable-length encoding methods exist (e.g., Google-optimized varint or UTF-8–style encodings), Æthernet’s `TieredInt` offers **compile-time control** over tier boundaries, improving both **packing ratio** and **predictability**.

**Examples**

```cpp
using T1 = ae::TieredInt<std::uint8_t, 249>;
using T2 = ae::TieredInt<std::uint16_t, 1000>;
using T4 = ae::TieredInt<std::uint32_t, 10>;
using U8 = ae::TieredInt<std::uint8_t, 254>;
using U16 = ae::TieredInt<std::uint8_t, 249, 1529>;
using S8 = ae::TieredInt<std::int8_t, 10, 20>;
```

The first template parameter is the wire cell type (`std::uint8_t`, `std::uint16_t`, or `std::uint32_t`). Subsequent parameters are per-tier maximum **logical** values:

* `249` — maximum value encoded in 1 byte
* `1529` — maximum value encoded in 2 bytes (must fit in the 2-byte format created by the first boundary)
* `16777215` — maximum value encoded in 4 bytes (must fit in the 4-byte format created by earlier boundaries)

For signed `TieredInt`, tier boundaries are validated in wire space after ZigZag encoding.

`TieredInt` always requires extension header space: the first tier max must be **strictly below** the wire cell's header maximum (e.g. `254`, not `255`, for `std::uint8_t`). If you need a full fixed-width integer, use the built-in type directly (`std::uint8_t`, etc.).

Invalid examples:

```cpp
// Invalid: no extension header left.
using Bad1 = ae::TieredInt<std::uint8_t, 255>;

// Invalid: second boundary does not fit in the two-byte format
// created by first boundary 249.
using Bad2 = ae::TieredInt<std::uint8_t, 249, 1000000>;
```

All standard C++ numeric limits and type traits are supported.

---

## Fixed

Microcontrollers often lack an **FPU** or have limited floating-point performance.
Æthernet provides a **binary-scaled inferred-point** fixed type. `Max` is a required logical bound — the storage raw range is **not** mapped exactly to `Max`. The implementation chooses a binary point so the type can represent at least `Max` with the highest precision that fits in `Rep`. The actual representable maximum may be larger than `Max`.

```cpp
using Q53 = ae::FixedPoint<std::uint8_t, 30.0>;   // Q5.3, step 1/8, max ~31.875
using Q71 = ae::FixedPoint<std::uint8_t, 100.0>;  // Q7.1, step 1/2, max ~127.5
using Q80 = ae::FixedPoint<std::uint8_t, 130.0>;  // Q8.0, step 1, max 255
```

Canonical syntax is `FixedPoint<Rep, Max>` where `Rep` is a built-in integral or `TieredInt` storage type and `Max` is a positive C++23 NTTP (integral or floating-point). Logical value is `raw * 2^kScaleExp`. Signed storage uses a **symmetric** raw range around zero (e.g. `int8_t` uses `-127..+127`, not `INT_MIN`).

Runtime arithmetic is **integer-only** (shifts and saturated raw add/sub); floating-point is used only at compile/parse boundaries in tests and consteval helpers. Mixed addition infers `FixedPoint<Rep, MaxA + MaxB>`. Scale conversion uses `Cast<To>()`. Explicit division uses `DivTo<Target>(lhs, rhs)` (currently a slow boundary helper, not an MCU hot-path API).

Configure the compiler explicitly when building (C++23 with floating-point NTTP), for example MacPorts Clang 20:

```bash
cmake -S . -B build-dev \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_C_COMPILER=/opt/local/bin/clang-mp-20 \
  -DCMAKE_CXX_COMPILER=/opt/local/bin/clang++-mp-20 \
  -DCMAKE_CXX_STANDARD=23 \
  -DAE_BUILD_TESTS=ON
```

---

## Exponent

When values span several orders of magnitude, **relative precision** is often more meaningful than absolute precision (e.g., function durations from microseconds to seconds).

`Exponential` is an **approximate logical codec**: it stores only a compact **wire code** (`WireT`); the decoded runtime value lives in `RuntimeT` (typically `FixedPoint`). Magnitude mapping uses compile-time fixed-point `log2` / `exp2` (`numeric/fixed_math.h`), so encode/decode through runtime values is approximate.

**API semantics:**

* **Exact raw-code APIs** — `Code()` / `FromCode()` construct from a wire code without magnitude approximation.
* **Approximate arithmetic APIs** — `Value()` decodes a code to runtime; `FromRuntime()` / `FromDouble()` encode a runtime value to a code. Round-trip tolerance depends on code tier (typically ±1 for single-byte codes, ±3 near the boundary).
* **Comparisons** — `Abs`, `Min`, `Max`, and `Clamp` operate on wire codes directly without decoding. Arithmetic operators are intentionally not implemented yet.

Code `0` is zero. For unsigned runtime types, code `1` is `MinMagnitude` and code `BoundaryCode` is `BoundaryMagnitude`. For signed runtime types, even codes are positive magnitudes and odd codes are negative magnitudes (`code 2` → `+MinMagnitude`, `code 1` → `-MinMagnitude`).

`MinMagnitude` must be representable in `RuntimeT` (a compile-time check rejects values that round to zero).

**Example — store values from `0.001` to `60.0` with a two-byte wire tier:**

```cpp
using Runtime = ae::FixedPoint<std::uint32_t, 60.0>;
using Wire = ae::TieredInt<std::uint8_t, 249, 1529>;

using E = ae::Exponential<Runtime, Wire, 0.001, 60.0, 1529>;

constexpr auto encoded = E::FromDouble(1.0);
constexpr auto decoded = encoded.ToRuntime();
```

Here:

* `Runtime` — decoded runtime type (`FixedPoint` semantics, precision, signedness)
* `Wire` — compact serialized code type (1 byte for codes ≤249; larger codes add extension bytes)
* `0.001` — smallest non-zero magnitude
* `60.0` — magnitude at the boundary code
* `1529` — highest code used by the magnitude mapping

`BoundaryCode` is the **used code range**, not necessarily `numeric_traits<Wire>::kRawMax`. When omitted, the default is `min(255, kRawMax)` (`DefaultBoundaryCode<Wire>()`). Pass an explicit fifth template argument when you need a smaller boundary, e.g. `Exponential<Runtime, Wire, 0.001, 60.0, 200>`.

Ideal for durations between **1 ms** and **60 s**.

### Embedded runtime (default)

Use `FixedPoint` or integral runtime types. No floating-point runtime is pulled in automatically:

```cpp
using Wire = ae::TieredInt<std::uint8_t, 249, 1529>;
using Runtime = ae::FixedPoint<std::uint32_t, 60.0>;

using E = ae::Exponential<Runtime, Wire, 0.001, 60.0, 1529>;
```

### Optional floating runtime

For desktop, server, debug, or reference use, include the optional header before instantiating `Exponential` with `float` or `double` runtime:

```cpp
#include "numeric/exponential_floating_runtime.h"

using Wire = ae::TieredInt<std::uint8_t, 249, 1529>;

using EFloat = ae::Exponential<float, Wire, 0.001f, 60.0f, 1529>;
using EDouble = ae::Exponential<double, Wire, 0.001, 60.0, 1529>;
```

`float` and `double` are **not** valid storage types for `TieredInt` or `FixedPoint`. Wire representation is unchanged: `Exponential` still stores only the `WireT` code.

---

## Text IO

`numeric/text_io.h` provides integer-only decimal conversion for `TieredInt`, `FixedPoint`, and `Exponential`. No runtime floating-point is used; `FixedPoint` formatting derives the logical value from `raw * 2^kScaleExp` using shifts and exact rational fraction expansion.

```cpp
#include "numeric/text_io.h"

using F = ae::FixedPoint<std::uint8_t, 100.0>; // Q7.1
auto s = ae::ToString(F::FromRaw(1));          // "0.5"

F parsed;
ae::FromString("10.5", parsed);
```

Core APIs:

* `ae::ToString(value)` — allocates a `std::string`
* `ae::FromString(text, value)` — returns `false` on malformed or out-of-range input
* `ae::ToChars(first, last, value)` — non-allocating buffer write via `<charconv>`
* `ae::FromChars(first, last, value)` — non-allocating parse

`Exponential` text IO encodes/decodes through `ToRuntime()` / `FromRuntime()`. JSON or logging layers can call `ToString` / `FromString` without pulling a JSON dependency into the numeric core.

---

## Ostream IO

`numeric/ostream_io.h` provides optional `operator<<` for `TieredInt` and `FixedPoint`. Include it only when you need stream output; core numeric headers do not pull in `<ostream>`.

Formatting uses the same integer-only text IO backend (`ToChars` / stack buffer, no runtime floating-point):

```cpp
#include "numeric/ostream_io.h"

using F = ae::FixedPoint<std::uint8_t, 100.0>;

std::cout << ae::TieredInt<std::uint8_t, 254>{123}; // 123
std::cout << F::FromRaw(1);                         // 0.5
```

---

## Wire IO

`numeric/wire_io.h` provides a uniform wire serialization API for built-in integers, `TieredInt`, and `FixedPoint` via `wire_traits<T>` and convenience functions `MaxWireBytes`, `Serialize`, and `Deserialize`.

`numeric/exponential_wire_io.h` adds `wire_traits` for `Exponential` (include it when serializing exponential values).

* **Built-in integers** — fixed-width little-endian (`sizeof(T)` bytes); signed types use two's-complement (not ZigZag).
* **TieredInt** — existing compact variable-length encoding (delegates to `TieredInt::Serialize` / `Deserialize`).
* **FixedPoint** — serializes only the raw `Rep` storage.
* **Exponential** — serializes only the `WireT` code (no runtime decode).

Short buffers throw `std::out_of_range` on deserialize.

```cpp
#include "numeric/wire_io.h"

using F = ae::FixedPoint<ae::TieredInt<std::uint8_t, 254>, 60.0>;

std::uint8_t buf[ae::MaxWireBytes<F>()];
auto n = ae::Serialize(F::FromInteger(30), buf);
auto restored = ae::Deserialize<F>(buf, n).value;
```

For exponential wire IO:

```cpp
#include "numeric/exponential_wire_io.h"

using Runtime = ae::FixedPoint<std::uint32_t, 60.0>;
using Wire = ae::TieredInt<std::uint8_t, 249, 1529>;
using E = ae::Exponential<Runtime, Wire, 0.001, 60.0, 1529>;

std::uint8_t buf[ae::MaxWireBytes<E>()];
auto n = ae::Serialize(E::FromDouble(1.0), buf);
auto restored = ae::Deserialize<E>(buf, n).value;
```

---

## Combined Types

You can combine `TieredInt`, `FixedPoint`, and `Exponential` for compact storage with rich runtime semantics.

**Example — duration from 1 ms to 60 s in one byte:**

```cpp
using Runtime = ae::FixedPoint<std::uint32_t, 60.0>;
using Wire = ae::TieredInt<std::uint8_t, 249, 1529>;
using Duration = ae::Exponential<Runtime, Wire, 0.001, 60.0, 1529>;

constexpr auto one_second = Duration::FromDouble(1.0);
auto runtime = one_second.ToRuntime();

std::uint8_t buf[ae::MaxWireBytes<Duration>()];
auto n = ae::Serialize(one_second, buf);
```

---

## Integration Notes

* **Header-only**, minimal external deps (`gcem` is used only by compile-time `fixed_math` helpers)
* C++23
* Interoperable with Æthernet message packing
* Designed for deterministic, low-overhead serialization on MCUs and embedded systems
* Typical code size per instantiated type is minimal
* Select the compiler explicitly via CMake command-line arguments (no automatic compiler selection in the build)

---

## Running Tests

Build and run the unit tests with CMake. Select the compiler explicitly on the command line:

```bash
# 1) Clone the repository
git clone https://github.com/aethernetio/aethernet-numeric.git

# 2) Enter the repo and update submodules
cd aethernet-numeric
git submodule update --init --recursive

# 3) Configure with tests enabled (example: MacPorts Clang 20)
cmake -S . -B build-dev \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_C_COMPILER=/opt/local/bin/clang-mp-20 \
  -DCMAKE_CXX_COMPILER=/opt/local/bin/clang++-mp-20 \
  -DCMAKE_CXX_STANDARD=23 \
  -DAE_BUILD_TESTS=ON

# 4) Build and run tests
cmake --build build-dev
ctest --test-dir build-dev --output-on-failure
```

---

© Æthernet Inc. — Part of the Æthernet Core Client Library
[https://aethernet.io](https://aethernet.io)
