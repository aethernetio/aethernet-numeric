/*
 * Copyright 2026 Aethernet Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef AE_NUMERIC_DETAILS_SEGMENTED_FORMAT_H_
#define AE_NUMERIC_DETAILS_SEGMENTED_FORMAT_H_

// Purpose: Define the declarative compile-time schema language consumed
// by SegmentedNumber.
//
// Motivation: ranges, precision policies, curve types, and wire tiers
// must be visible to the compiler so invalid formats fail at build time
// and runtime code can be specialized without dynamic configuration.
//
// Analogous concept: a type-level schema or embedded compile-time DSL.
//
// Usage: users normally include segmented_number.h and compose Format,
// Layout, Range, Intervals, Place, WireCuts, and curve declarations.
// This details header is not a standalone runtime API.
//
// How it works: the declarations contain no runtime state. The compiler
// parses their types, validates the schema, derives coefficients and
// wire placement, and produces a SegmentedNumber specialization.


#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "ae-numeric/decimal.h"

namespace ae::seg {

template <typename T>
struct RuntimePolicyTag {
  using rep = T;
};

namespace runtime {
template <typename Rep>
struct Fixed {
  using rep = Rep;
  static constexpr bool kIsFloating = false;
};

template <typename T>
struct Floating {
  using rep = T;
  static constexpr bool kIsFloating = true;
};
}  // namespace runtime

template <typename>
inline constexpr bool kIsFloatingRuntimePolicy = false;
template <typename T>
inline constexpr bool kIsFloatingRuntimePolicy<runtime::Floating<T>> = true;

namespace wire {
template <std::size_t N>
struct MaxBytes {
  static constexpr std::size_t value = N;
};

template <typename Cell, typename... Opts>
struct AutoTiered {
  using cell_type = Cell;
};
}  // namespace wire

namespace compute {
struct Formula {};
}  // namespace compute

template <typename Begin, typename End>
struct Range {};

template <typename V>
struct Step {};

template <std::size_t N>
struct Intervals {
  static constexpr std::size_t value = N;
};

template <std::size_t N>
struct TotalValues {
  static constexpr std::size_t value = N;
};

template <std::size_t N>
struct Bytes {
  static constexpr std::size_t value = N;
};

template <typename B>
struct Place {};

struct InheritStep {};
struct FillTier {};
struct MinimumIntervals {};
struct ExactEndpoints {};
struct OptimizeCuts {};
struct ContinuousAbsoluteStep {};
struct MinimaxRelativeError {};

template <typename Policy>
struct EndpointPolicy {};

template <typename Policy>
struct Allocate {};

template <typename V>
struct StepAtLower {};

template <typename V>
struct StepAtUpper {};

template <typename V>
struct MaxAbsErrorAtLower {};

template <typename V>
struct MaxAbsErrorAtUpper {};

template <typename... Items>
struct Objective {};

template <typename V, typename B>
struct ApproximateCut {};

template <typename B>
struct Rest {};

template <typename... Cuts>
struct WireCuts {};

template <typename... Items>
struct Layout {};

template <typename... Opts>
struct UniformStep {};

template <typename... Opts>
struct UniformValues {};

template <typename... Opts>
struct ExponentialValues {};

template <typename... Opts>
struct GeometricStep {};

template <typename... Opts>
struct LinearStepRamp {};

template <typename... Opts>
struct AutoSplit {};

template <typename... Opts>
struct ContinuousExponential {};

template <typename RuntimePolicy, typename WirePolicy, typename ComputePolicy,
          typename LayoutT>
struct Format {
  using runtime_policy = RuntimePolicy;
  using wire_policy = WirePolicy;
  using compute_policy = ComputePolicy;
  using layout_type = LayoutT;
};

enum class CurveKind : std::uint8_t {
  kUniformStep = 1,
  kUniformValues = 2,
  kExponentialValues = 3,
  kGeometricStep = 4,
  kLinearStepRamp = 5,
};

}  // namespace ae::seg

#endif  // AE_NUMERIC_DETAILS_SEGMENTED_FORMAT_H_
