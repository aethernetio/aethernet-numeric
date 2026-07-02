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

#ifndef AE_NUMERIC_EXPONENTIAL_H_
#define AE_NUMERIC_EXPONENTIAL_H_

#include <bit>
#include <concepts>
#include <cstdint>
#include <optional>
#include <type_traits>
#include <utility>

#include "ae-numeric/exponential_math_policy.h"
#include "ae-numeric/fixed_math.h"
#include "ae-numeric/fixed_point.h"
#include "ae-numeric/numeric_traits.h"
#include "ae-numeric/runtime_numeric_traits.h"

namespace ae {
namespace exponential_internal {

// Intentionally non-constexpr: naming this on a branch reached during constant
// evaluation makes the enclosing consteval construction ill-formed, turning an
// out-of-range logical constant into a compile error instead of a silent clamp.
inline void ExponentialConstantExceedsRange() {}

template <typename WireT>
constexpr auto DefaultBoundaryCode() {
  return numeric_traits<WireT>::kMaxBoundaryCode;
}

template <typename WireT>
constexpr auto WireRawValue(WireT const& wire) {
  if constexpr (requires { wire.value_; }) {
    return wire.value_;
  } else {
    return wire;
  }
}

template <typename WireT, typename WireValue>
constexpr WireT WireFromValue(WireValue value) {
  if constexpr (std::same_as<WireT, WireValue>) {
    return value;
  } else {
    return WireT{value};
  }
}

template <typename WireValue>
constexpr WireValue ClampCode(WireValue code, WireValue boundary_code) {
  if (code < WireValue{0}) {
    return WireValue{0};
  }
  if (code > boundary_code) {
    return boundary_code;
  }
  return code;
}

template <typename WireValue>
constexpr WireValue ClampIndex(WireValue index, WireValue max_index) {
  if (index < WireValue{1}) {
    return WireValue{1};
  }
  if (index > max_index) {
    return max_index;
  }
  return index;
}

template <typename RuntimeT>
constexpr RuntimeT RuntimeSub(RuntimeT const& lhs, RuntimeT const& rhs) {
  if constexpr (std::is_floating_point_v<RuntimeT>) {
    return lhs - rhs;
  } else if constexpr (numeric_traits<RuntimeT>::kIsFixedPoint) {
    return SubTo<RuntimeT>(lhs, rhs);
  } else {
    return lhs - rhs;
  }
}

template <typename RuntimeT, bool kIsSigned>
constexpr RuntimeT AbsRuntime(RuntimeT value) {
  using RuntimeTraits = runtime_numeric_traits<RuntimeT>;

  if constexpr (kIsSigned) {
    const RuntimeT zero = RuntimeTraits::FromInteger(0);
    if (value < zero) {
      return RuntimeSub(zero, value);
    }
  }
  return value;
}

template <typename LogT, typename WireValue>
constexpr LogT LogAtIndexRaw(WireValue index, LogT log_min,
                             std::int64_t log_span_raw, WireValue max_index) {
  if (index <= WireValue{1}) {
    return log_min;
  }
  const std::int64_t num =
      log_span_raw * (static_cast<std::int64_t>(index) - 1);
  const std::int64_t den = static_cast<std::int64_t>(max_index) - 1;
  const std::int64_t offset_raw =
      fixed_point_internal::RoundDivNearest(num, den);
  const std::int64_t raw =
      static_cast<std::int64_t>(log_min.RawValue()) + offset_raw;
  return LogT::FromRaw(
      fixed_point_internal::RepFromRawValue<typename LogT::rep_type>(
          static_cast<typename LogT::rep_value_type>(raw)));
}

template <typename LogT, typename WireValue>
constexpr WireValue EncodeMagnitudeIndexRaw(LogT log_min,
                                            std::int64_t log_span_raw,
                                            WireValue max_index,
                                            LogT log_value) {
  if (max_index <= WireValue{1}) {
    return WireValue{1};
  }
  if (log_span_raw == 0) {
    return WireValue{1};
  }

  const std::int64_t pos_raw = static_cast<std::int64_t>(log_value.RawValue()) -
                               static_cast<std::int64_t>(log_min.RawValue());
  const std::int64_t den = static_cast<std::int64_t>(max_index) - 1;
  const std::int64_t index_offset =
      fixed_point_internal::RoundDivNearest(pos_raw * den, log_span_raw);
  return ClampIndex(static_cast<WireValue>(index_offset + 1), max_index);
}

template <typename FloatingT>
struct FloatingWorkCodec;

inline constexpr std::int32_t kMaxMantissaShift = 62;

template <typename WorkT>
constexpr WorkT WorkFromMantissa(std::uint64_t mantissa, std::int32_t exponent,
                                 std::int32_t mantissa_bits) {
  std::int32_t const scale_bits = WorkT::kScaleExp < 0 ? -WorkT::kScaleExp : 0;
  std::int32_t const shift = exponent + scale_bits - mantissa_bits;

  if (shift >= 0) {
    for (std::int32_t i = 0; i < shift && i < kMaxMantissaShift; ++i) {
      mantissa <<= 1U;
    }
  } else {
    for (std::int32_t i = 0; i < -shift && i < kMaxMantissaShift; ++i) {
      mantissa =
          static_cast<std::uint64_t>(fixed_point_internal::RoundDivNearest(
              static_cast<std::int64_t>(mantissa), std::int64_t{2}));
    }
  }

  using RepValue = typename WorkT::rep_value_type;
  RepValue const raw = mantissa > static_cast<std::uint64_t>(WorkT::kRawMax)
                           ? WorkT::kRawMax
                           : static_cast<RepValue>(mantissa);
  return WorkT::FromRaw(
      fixed_point_internal::RepFromRawValue<typename WorkT::rep_type>(
          WorkT::ClampRaw(raw)));
}

constexpr std::int32_t HighestSetBit(std::uint64_t value) noexcept {
  if (value == 0U) {
    return -1;
  }
  return static_cast<std::int32_t>(63U - std::countl_zero(value));
}

template <typename WorkT, typename FloatingT>
constexpr FloatingT MantissaToFloating(std::uint64_t mantissa,
                                       std::int32_t exponent_bias,
                                       std::int32_t mantissa_bits) {
  if (mantissa == 0U) {
    return FloatingT{0};
  }

  std::int32_t const msb = HighestSetBit(mantissa);
  std::int32_t const scale_bits = WorkT::kScaleExp < 0 ? -WorkT::kScaleExp : 0;
  std::int32_t const exponent = msb - scale_bits + exponent_bias;
  std::int32_t const shift = msb - mantissa_bits;
  if (shift >= 0) {
    mantissa >>= static_cast<unsigned>(shift);
  } else {
    mantissa <<= static_cast<unsigned>(-shift);
  }

  if constexpr (std::same_as<FloatingT, float>) {
    std::uint32_t const bits =
        (static_cast<std::uint32_t>(exponent) << 23U) |
        (static_cast<std::uint32_t>(mantissa) & 0x7FFFFFU);
    return std::bit_cast<float>(bits);
  } else {
    std::uint64_t const bits = (static_cast<std::uint64_t>(exponent) << 52U) |
                               (mantissa & 0xFFFFFFFFFFFFFULL);
    return std::bit_cast<double>(bits);
  }
}

template <>
struct FloatingWorkCodec<float> {
  template <typename WorkT>
  static constexpr WorkT ToWork(float value) {
    std::uint32_t const bits = std::bit_cast<std::uint32_t>(value);
    if (bits == 0U) {
      return WorkT::FromInteger(0);
    }

    std::int32_t const exponent =
        static_cast<std::int32_t>((bits >> 23U) & 0xFFU) - 127;
    std::uint64_t const mantissa =
        (static_cast<std::uint64_t>(bits & 0x7FFFFFU) |
         std::uint64_t{0x800000U});
    return WorkFromMantissa<WorkT>(mantissa, exponent, 23);
  }

  template <typename WorkT>
  static constexpr float FromWork(WorkT value) {
    return MantissaToFloating<WorkT, float>(
        static_cast<std::uint64_t>(value.RawValue()), 127, 23);
  }
};

template <>
struct FloatingWorkCodec<double> {
  template <typename WorkT>
  static constexpr WorkT ToWork(double value) {
    std::uint64_t const bits = std::bit_cast<std::uint64_t>(value);
    if (bits == 0ULL) {
      return WorkT::FromInteger(0);
    }

    std::int32_t const exponent =
        static_cast<std::int32_t>((bits >> 52U) & 0x7FFU) - 1023;
    std::uint64_t const mantissa =
        (bits & 0xFFFFFFFFFFFFFULL) | (std::uint64_t{1} << 52U);
    return WorkFromMantissa<WorkT>(mantissa, exponent, 52);
  }

  template <typename WorkT>
  static constexpr double FromWork(WorkT value) {
    return MantissaToFloating<WorkT, double>(
        static_cast<std::uint64_t>(value.RawValue()), 1023, 52);
  }
};

template <typename WorkT, typename FloatingT>
  requires std::is_floating_point_v<FloatingT>
constexpr WorkT FloatingToWork(FloatingT value) {
  return FloatingWorkCodec<FloatingT>::template ToWork<WorkT>(value);
}

template <typename FloatingT, typename WorkT>
  requires std::is_floating_point_v<FloatingT>
constexpr FloatingT WorkToFloating(WorkT value) {
  return FloatingWorkCodec<FloatingT>::template FromWork<WorkT>(value);
}

template <typename WorkT, typename RuntimeT>
constexpr WorkT ToWorkRuntime(RuntimeT value) {
  if constexpr (std::is_floating_point_v<RuntimeT>) {
    return FloatingToWork<WorkT>(value);
  } else if constexpr (numeric_traits<RuntimeT>::kIsFixedPoint) {
    return fixed_math::internal::cast_fixed<WorkT>(value);
  } else {
    return WorkT::FromInteger(static_cast<std::int64_t>(value));
  }
}

template <typename RuntimeT, typename WorkT>
constexpr RuntimeT FromWorkRuntime(WorkT value) {
  if constexpr (std::is_floating_point_v<RuntimeT>) {
    return WorkToFloating<RuntimeT>(value);
  } else if constexpr (numeric_traits<RuntimeT>::kIsFixedPoint) {
    return fixed_math::internal::cast_fixed<RuntimeT>(value);
  } else {
    return runtime_numeric_traits<RuntimeT>::FromInteger(
        static_cast<std::int64_t>(value.RawValue()));
  }
}

template <typename RuntimeT, typename LogT, typename WorkT, typename MathPolicy>
constexpr RuntimeT LogExp2(LogT log_value) {
  const WorkT work = fixed_math::Exp2To<WorkT, LogT, MathPolicy>(log_value);
  return FromWorkRuntime<RuntimeT, WorkT>(work);
}

}  // namespace exponential_internal

// Exponential is an approximate logical codec: wire codes are exact, but
// magnitude mapping uses fixed-point log2/exp2 and may not round-trip through
// runtime arithmetic exactly.
//
// Exact raw-code APIs (no decode/encode approximation):
//   Code() / FromCode() — construct from a wire code as-is.
//
// Approximate arithmetic APIs (encode/decode through RuntimeT):
//   Value() — decode code to runtime magnitude (approximate).
//   FromRuntime() / FromDouble() — encode runtime to code.
//
// BoundaryCode is the highest code used by the magnitude mapping. When omitted,
// the default is numeric_traits<WireT>::kMaxBoundaryCode via
// DefaultBoundaryCode<WireT>().
// Pass an explicit fifth template argument to use a smaller code range.
template <typename RuntimeT, typename WireT, auto MinMagnitude,
          auto BoundaryMagnitude,
          auto BoundaryCode =
              exponential_internal::DefaultBoundaryCode<WireT>(),
          typename MathPolicy = exponential_internal::ExponentialMathPolicy<
              RuntimeT, MinMagnitude, BoundaryMagnitude, BoundaryCode,
              typename numeric_traits<WireT>::rep_value_type,
              runtime_numeric_traits<RuntimeT>::kIsSigned>>
class Exponential {
 public:
  using runtime_type = RuntimeT;
  using wire_type = WireT;
  using math_policy = MathPolicy;
  using log_type = typename MathPolicy::log_type;
  using LogT = log_type;
  using WorkT = typename MathPolicy::work_type;
  using mant_type = typename MathPolicy::mant_type;
  using wire_value_type = typename numeric_traits<WireT>::rep_value_type;
  using RuntimeTraits = runtime_numeric_traits<RuntimeT>;

  static constexpr bool kIsSigned = RuntimeTraits::kIsSigned;
  static constexpr auto kMinMagnitude = MinMagnitude;
  static constexpr auto kBoundaryMagnitude = BoundaryMagnitude;
  static constexpr wire_value_type kBoundaryCode = BoundaryCode;

  static_assert(RuntimeTraits::kIsSupported,
                "Exponential RuntimeT is not supported. "
                "Use FixedPoint/integer runtime, or include "
                "ae-numeric/exponential_floating_runtime.h for float/double.");
  static_assert(numeric_traits<WireT>::kIsIntegerLike,
                "Exponential WireT must be integer-like");
  static_assert(!numeric_traits<WireT>::kIsSigned,
                "Exponential WireT must be unsigned for now");
  static_assert(BoundaryCode > 0, "Exponential BoundaryCode must be positive");
  static_assert(BoundaryCode <= numeric_traits<WireT>::kMaxBoundaryCode,
                "Exponential BoundaryCode must fit in WireT range");
  static_assert(RuntimeTraits::FromDouble(MinMagnitude) >
                    RuntimeTraits::FromInteger(0),
                "Exponential MinMagnitude is below RuntimeT precision");

  static constexpr wire_value_type kPositiveBoundaryCode =
      kBoundaryCode - (kBoundaryCode % wire_value_type{2});
  static constexpr wire_value_type kNegativeBoundaryCode =
      (kBoundaryCode % static_cast<wire_value_type>(2) ==
       static_cast<wire_value_type>(0))
          ? static_cast<wire_value_type>(kBoundaryCode - wire_value_type{1})
          : kBoundaryCode;
  static constexpr wire_value_type kMaxMagnitudeIndex =
      kIsSigned ? static_cast<wire_value_type>(kPositiveBoundaryCode /
                                               wire_value_type{2})
                : kBoundaryCode;

  static constexpr LogT kLogMin = fixed_math::Log2To<LogT, WorkT, MathPolicy>(
      WorkT::FromDouble(static_cast<double>(MinMagnitude)));
  static constexpr LogT kLogBoundary =
      fixed_math::Log2To<LogT, WorkT, MathPolicy>(
          WorkT::FromDouble(static_cast<double>(BoundaryMagnitude)));
  static constexpr std::int64_t kLogSpanRaw =
      static_cast<std::int64_t>(kLogBoundary.RawValue()) -
      static_cast<std::int64_t>(kLogMin.RawValue());

  static constexpr RuntimeT MagnitudeAt(wire_value_type MagnitudeIndex) {
    if (MagnitudeIndex == wire_value_type{0}) {
      return RuntimeTraits::FromInteger(0);
    }
    const LogT log_value = exponential_internal::LogAtIndexRaw(
        MagnitudeIndex, kLogMin, kLogSpanRaw, kMaxMagnitudeIndex);
    return exponential_internal::LogExp2<RuntimeT, LogT, WorkT, MathPolicy>(
        log_value);
  }

  static constexpr wire_value_type MagnitudeIndexFor(RuntimeT abs_value) {
    const RuntimeT min_mag = RuntimeTraits::FromDouble(MinMagnitude);
    const RuntimeT bound_mag = RuntimeTraits::FromDouble(BoundaryMagnitude);

    if (abs_value <= min_mag) {
      return wire_value_type{1};
    }
    if (abs_value >= bound_mag) {
      return kMaxMagnitudeIndex;
    }

    const LogT log_value = fixed_math::Log2To<LogT, WorkT, MathPolicy>(
        exponential_internal::ToWorkRuntime<WorkT>(abs_value));
    return exponential_internal::EncodeMagnitudeIndexRaw(
        kLogMin, kLogSpanRaw, kMaxMagnitudeIndex, log_value);
  }

  static constexpr Exponential FromCode(WireT code) {
    return Exponential(exponential_internal::WireFromValue<WireT>(
        exponential_internal::ClampCode(
            exponential_internal::WireRawValue(code), kBoundaryCode)));
  }

  static constexpr Exponential Code(WireT code) {
    return FromCode(code);
  }

  static constexpr bool LogicalIntegerInRange(std::int64_t value) {
    if constexpr (!kIsSigned) {
      if (value < 0) {
        return false;
      }
    }
    const double magnitude =
        value < 0 ? -static_cast<double>(value) : static_cast<double>(value);
    return magnitude <= static_cast<double>(BoundaryMagnitude);
  }

  static constexpr Exponential FromRuntimeInteger(std::int64_t value) {
    return FromRuntime(RuntimeTraits::FromInteger(value));
  }

  static constexpr Exponential Saturating(std::int64_t value) {
    return FromRuntime(RuntimeTraits::FromInteger(value));
  }

  static constexpr std::optional<Exponential> TryFromRuntimeInteger(
      std::int64_t value) {
    if (!LogicalIntegerInRange(value)) {
      return std::nullopt;
    }
    return FromRuntime(RuntimeTraits::FromInteger(value));
  }

  static constexpr std::optional<Exponential> TryFromRuntime(RuntimeT value) {
    const RuntimeT zero = RuntimeTraits::FromInteger(0);
    const RuntimeT bound = RuntimeTraits::FromDouble(BoundaryMagnitude);
    const RuntimeT abs_value =
        exponential_internal::AbsRuntime<RuntimeT, kIsSigned>(value);
    if (abs_value > bound) {
      return std::nullopt;
    }
    return FromRuntime(value);
  }

  static constexpr Exponential FromRuntime(RuntimeT value) {
    const RuntimeT zero = RuntimeTraits::FromInteger(0);
    if (value == zero) {
      return FromCode(
          exponential_internal::WireFromValue<WireT>(wire_value_type{0}));
    }

    const RuntimeT abs_value =
        exponential_internal::AbsRuntime<RuntimeT, kIsSigned>(value);
    const RuntimeT boundary = MagnitudeAt(kMaxMagnitudeIndex);

    if (abs_value >= boundary) {
      if constexpr (kIsSigned) {
        if (value < zero) {
          return FromCode(exponential_internal::WireFromValue<WireT>(
              kNegativeBoundaryCode));
        }
        return FromCode(
            exponential_internal::WireFromValue<WireT>(kPositiveBoundaryCode));
      }
      return FromCode(
          exponential_internal::WireFromValue<WireT>(kBoundaryCode));
    }

    const auto index = MagnitudeIndexFor(abs_value);

    if constexpr (kIsSigned) {
      if (value < zero) {
        const wire_value_type code = static_cast<wire_value_type>(
            index * wire_value_type{2} - wire_value_type{1});
        return FromCode(exponential_internal::WireFromValue<WireT>(code));
      }
      const wire_value_type code =
          static_cast<wire_value_type>(index * wire_value_type{2});
      return FromCode(exponential_internal::WireFromValue<WireT>(code));
    }

    return FromCode(exponential_internal::WireFromValue<WireT>(index));
  }

  static consteval Exponential PositiveBoundaryCode() {
    return FromCode(
        exponential_internal::WireFromValue<WireT>(kPositiveBoundaryCode));
  }

  static consteval Exponential NegativeBoundaryCode() {
    return FromCode(
        exponential_internal::WireFromValue<WireT>(kNegativeBoundaryCode));
  }

  static consteval Exponential MinPositiveCode() {
    if constexpr (kIsSigned) {
      return FromCode(
          exponential_internal::WireFromValue<WireT>(wire_value_type{2}));
    }
    return FromCode(
        exponential_internal::WireFromValue<WireT>(wire_value_type{1}));
  }

  static consteval Exponential FromDouble(double value) {
    if (value == 0.0) {
      return FromCode(
          exponential_internal::WireFromValue<WireT>(wire_value_type{0}));
    }

    const double min_mag = static_cast<double>(MinMagnitude);
    const double bound_mag = static_cast<double>(BoundaryMagnitude);

    if constexpr (kIsSigned) {
      if (value > 0.0) {
        if (value >= bound_mag) {
          return PositiveBoundaryCode();
        }
        if (value < min_mag) {
          return MinPositiveCode();
        }
      } else {
        const double abs_value = -value;
        if (abs_value >= bound_mag) {
          return NegativeBoundaryCode();
        }
        if (abs_value < min_mag) {
          return FromCode(
              exponential_internal::WireFromValue<WireT>(wire_value_type{1}));
        }
      }
    } else {
      if (value >= bound_mag) {
        return FromCode(
            exponential_internal::WireFromValue<WireT>(kBoundaryCode));
      }
      if (value < min_mag) {
        return MinPositiveCode();
      }
    }

    return FromRuntime(RuntimeTraits::FromDouble(value));
  }

  static consteval Exponential EncodeCheckedInteger(std::int64_t value) {
    if (!LogicalIntegerInRange(value)) {
      exponential_internal::ExponentialConstantExceedsRange();
    }
    return FromRuntime(RuntimeTraits::FromInteger(value));
  }

  constexpr Exponential() = default;

  template <std::integral T>
  consteval Exponential(T logical_value)
      : Exponential(
            EncodeCheckedInteger(static_cast<std::int64_t>(logical_value))) {}

  consteval Exponential(double logical_value)
      : Exponential(FromDouble(logical_value)) {}

  consteval Exponential(float logical_value)
      : Exponential(FromDouble(static_cast<double>(logical_value))) {}

  constexpr WireT WireCode() const {
    return code_;
  }

  constexpr RuntimeT Value() const {
    return ToRuntime();
  }

  constexpr wire_value_type CodeValue() const {
    return exponential_internal::WireRawValue(code_);
  }

  constexpr bool IsZero() const {
    return CodeValue() == wire_value_type{0};
  }

  constexpr bool IsPositive() const {
    if (IsZero()) {
      return false;
    }
    if constexpr (kIsSigned) {
      return CodeValue() % wire_value_type{2} == wire_value_type{0};
    }
    return true;
  }

  constexpr bool IsNegative() const {
    if constexpr (!kIsSigned) {
      return false;
    }
    return !IsZero() && CodeValue() % wire_value_type{2} == wire_value_type{1};
  }

  constexpr wire_value_type MagnitudeIndex() const {
    const auto code = CodeValue();
    if (code == wire_value_type{0}) {
      return wire_value_type{0};
    }
    if constexpr (kIsSigned) {
      if (code % wire_value_type{2} == wire_value_type{0}) {
        return static_cast<wire_value_type>(code / wire_value_type{2});
      }
      return static_cast<wire_value_type>((code + wire_value_type{1}) /
                                          wire_value_type{2});
    }
    return code;
  }

  constexpr RuntimeT ToRuntime() const {
    const auto index = MagnitudeIndex();
    if (index == wire_value_type{0}) {
      return RuntimeTraits::FromInteger(0);
    }
    const RuntimeT magnitude = MagnitudeAt(index);
    if (IsNegative()) {
      return exponential_internal::RuntimeSub(RuntimeTraits::FromInteger(0),
                                              magnitude);
    }
    return magnitude;
  }

  constexpr Exponential Abs() const {
    if (IsZero() || IsPositive()) {
      return *this;
    }
    const wire_value_type code =
        static_cast<wire_value_type>(MagnitudeIndex() * wire_value_type{2});
    return FromCode(exponential_internal::WireFromValue<WireT>(code));
  }

  constexpr Exponential operator-() const requires(kIsSigned) {
    if (IsZero()) {
      return *this;
    }
    if (IsPositive()) {
      const wire_value_type code =
          static_cast<wire_value_type>(CodeValue() - wire_value_type{1});
      return FromCode(exponential_internal::WireFromValue<WireT>(code));
    }
    const wire_value_type code =
        static_cast<wire_value_type>(CodeValue() + wire_value_type{1});
    return FromCode(exponential_internal::WireFromValue<WireT>(code));
  }

  constexpr bool operator==(const Exponential& other) const {
    return CodeValue() == other.CodeValue();
  }

  constexpr bool operator!=(const Exponential& other) const {
    return !(*this == other);
  }

  constexpr bool operator<(const Exponential& other) const {
    if (CodeValue() == other.CodeValue()) {
      return false;
    }
    if constexpr (kIsSigned) {
      if (IsZero() || other.IsZero()) {
        if (IsZero()) {
          return other.IsPositive();
        }
        return IsNegative();
      }
      if (IsNegative() != other.IsNegative()) {
        return IsNegative();
      }
      if (IsNegative()) {
        return CodeValue() > other.CodeValue();
      }
      return CodeValue() < other.CodeValue();
    }
    return CodeValue() < other.CodeValue();
  }

  constexpr bool operator>(const Exponential& other) const {
    return other < *this;
  }

  constexpr bool operator<=(const Exponential& other) const {
    return !(other < *this);
  }

  constexpr bool operator>=(const Exponential& other) const {
    return !(*this < other);
  }

 private:
  constexpr explicit Exponential(WireT code) : code_(code) {}

  WireT code_{};
};

template <typename RuntimeT, typename WireT, auto MinMagnitude,
          auto BoundaryMagnitude, auto BoundaryCode, typename MathPolicy>
constexpr Exponential<RuntimeT, WireT, MinMagnitude, BoundaryMagnitude,
                      BoundaryCode, MathPolicy>
Min(Exponential<RuntimeT, WireT, MinMagnitude, BoundaryMagnitude, BoundaryCode,
                MathPolicy>
        a,
    Exponential<RuntimeT, WireT, MinMagnitude, BoundaryMagnitude, BoundaryCode,
                MathPolicy>
        b) {
  return a < b ? a : b;
}

template <typename RuntimeT, typename WireT, auto MinMagnitude,
          auto BoundaryMagnitude, auto BoundaryCode, typename MathPolicy>
constexpr Exponential<RuntimeT, WireT, MinMagnitude, BoundaryMagnitude,
                      BoundaryCode, MathPolicy>
Max(Exponential<RuntimeT, WireT, MinMagnitude, BoundaryMagnitude, BoundaryCode,
                MathPolicy>
        a,
    Exponential<RuntimeT, WireT, MinMagnitude, BoundaryMagnitude, BoundaryCode,
                MathPolicy>
        b) {
  return a < b ? b : a;
}

template <typename RuntimeT, typename WireT, auto MinMagnitude,
          auto BoundaryMagnitude, auto BoundaryCode, typename MathPolicy>
constexpr Exponential<RuntimeT, WireT, MinMagnitude, BoundaryMagnitude,
                      BoundaryCode, MathPolicy>
Clamp(Exponential<RuntimeT, WireT, MinMagnitude, BoundaryMagnitude,
                  BoundaryCode, MathPolicy>
          value,
      Exponential<RuntimeT, WireT, MinMagnitude, BoundaryMagnitude,
                  BoundaryCode, MathPolicy>
          low,
      Exponential<RuntimeT, WireT, MinMagnitude, BoundaryMagnitude,
                  BoundaryCode, MathPolicy>
          high) {
  return Min(Max(value, low), high);
}

template <typename RuntimeT, typename WireT, auto MinMagnitude,
          auto BoundaryMagnitude, auto BoundaryCode, typename MathPolicy>
struct numeric_traits<
    Exponential<RuntimeT, WireT, MinMagnitude, BoundaryMagnitude, BoundaryCode,
                MathPolicy>> {
  using rep_type = WireT;
  using rep_value_type = typename numeric_traits<WireT>::rep_value_type;
  using value_type = Exponential<RuntimeT, WireT, MinMagnitude,
                                 BoundaryMagnitude, BoundaryCode, MathPolicy>;

  static constexpr bool kIsIntegerLike = false;
  static constexpr bool kIsFixedPoint = false;
  static constexpr bool kIsExponential = true;
  static constexpr bool kIsSigned = value_type::kIsSigned;
};

}  // namespace ae

#endif  // AE_NUMERIC_EXPONENTIAL_H_
