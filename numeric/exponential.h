/*
 * Copyright 2025 Aethernet Inc.
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

#ifndef NUMERIC_EXPONENTIAL_H_
#define NUMERIC_EXPONENTIAL_H_

#include <bit>
#include <cstdint>
#include <optional>
#include <type_traits>
#include <utility>

#include "numeric/exponential_math_policy.h"
#include "numeric/fixed_math.h"
#include "numeric/fixed_point.h"
#include "numeric/numeric_traits.h"
#include "numeric/runtime_numeric_traits.h"

namespace ae {
namespace exponential_internal {

// Intentionally non-constexpr: naming this on a branch reached during constant
// evaluation makes the enclosing consteval construction ill-formed, turning an
// out-of-range logical constant into a compile error instead of a silent clamp.
inline void ExponentialConstantExceedsRange() {}

template <typename WireT>
constexpr auto DefaultBoundaryCode() {
  using WireValue = typename numeric_traits<WireT>::rep_value_type;
  constexpr WireValue kRawMax = numeric_traits<WireT>::kRawMax;
  if constexpr (kRawMax > WireValue{255}) {
    return WireValue{255};
  }
  return kRawMax;
}

template <typename WireT>
constexpr auto WireRawValue(const WireT& wire) {
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
constexpr RuntimeT RuntimeSub(const RuntimeT& lhs, const RuntimeT& rhs) {
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

template <typename WorkT>
constexpr WorkT FloatToWork(float value) {
  const auto bits = std::bit_cast<std::uint32_t>(value);
  if (bits == 0U) {
    return WorkT::FromInteger(0);
  }

  const int exponent = static_cast<int>((bits >> 23U) & 0xFFU) - 127;
  std::uint64_t mantissa =
      (static_cast<std::uint64_t>(bits & 0x7FFFFFU) | std::uint64_t{0x800000U});
  const int shift =
      exponent + (WorkT::kScaleExp < 0 ? -WorkT::kScaleExp : 0) - 23;

  if (shift >= 0) {
    for (int i = 0; i < shift && i < 62; ++i) {
      mantissa <<= 1U;
    }
  } else {
    for (int i = 0; i < -shift && i < 62; ++i) {
      mantissa =
          static_cast<std::uint64_t>(fixed_point_internal::RoundDivNearest(
              static_cast<std::int64_t>(mantissa), std::int64_t{2}));
    }
  }

  return WorkT::FromRaw(
      fixed_point_internal::RepFromRawValue<typename WorkT::rep_type>(
          WorkT::ClampRaw(
              static_cast<typename WorkT::rep_value_type>(mantissa))));
}

template <typename WorkT>
constexpr WorkT DoubleToWork(double value) {
  const auto bits = std::bit_cast<std::uint64_t>(value);
  if (bits == 0ULL) {
    return WorkT::FromInteger(0);
  }
  const int exponent = static_cast<int>((bits >> 52U) & 0x7FFU) - 1023;
  std::uint64_t mantissa =
      (bits & 0xFFFFFFFFFFFFFULL) | (std::uint64_t{1} << 52U);
  const int shift =
      exponent + (WorkT::kScaleExp < 0 ? -WorkT::kScaleExp : 0) - 52;
  if (shift >= 0) {
    for (int i = 0; i < shift && i < 62; ++i) {
      mantissa <<= 1U;
    }
  } else {
    for (int i = 0; i < -shift && i < 62; ++i) {
      mantissa =
          static_cast<std::uint64_t>(fixed_point_internal::RoundDivNearest(
              static_cast<std::int64_t>(mantissa), std::int64_t{2}));
    }
  }
  return WorkT::FromRaw(
      fixed_point_internal::RepFromRawValue<typename WorkT::rep_type>(
          WorkT::ClampRaw(
              static_cast<typename WorkT::rep_value_type>(mantissa))));
}

constexpr int HighestSetBit(std::uint64_t value) {
  int bit = -1;
  for (int i = 63; i >= 0; --i) {
    if ((value >> static_cast<unsigned>(i)) & 1U) {
      bit = i;
      break;
    }
  }
  return bit;
}

template <typename WorkT>
constexpr float WorkToFloat(WorkT value) {
  if (value.RawValue() == typename WorkT::rep_value_type{0}) {
    return 0.0f;
  }

  std::uint64_t mantissa = value.RawValue();
  const int msb = HighestSetBit(mantissa);
  const int scale_bits = WorkT::kScaleExp < 0 ? -WorkT::kScaleExp : 0;
  const int exponent = msb - scale_bits + 127;
  const int shift = msb - 23;
  if (shift >= 0) {
    mantissa >>= static_cast<unsigned>(shift);
  } else {
    mantissa <<= static_cast<unsigned>(-shift);
  }
  const std::uint32_t bits = (static_cast<std::uint32_t>(exponent) << 23U) |
                             (static_cast<std::uint32_t>(mantissa) & 0x7FFFFFU);
  return std::bit_cast<float>(bits);
}

template <typename WorkT>
constexpr double WorkToDouble(WorkT value) {
  if (value.RawValue() == typename WorkT::rep_value_type{0}) {
    return 0.0;
  }

  std::uint64_t mantissa = value.RawValue();
  const int msb = HighestSetBit(mantissa);
  const int scale_bits = WorkT::kScaleExp < 0 ? -WorkT::kScaleExp : 0;
  const int exponent = msb - scale_bits + 1023;
  const int shift = msb - 52;
  if (shift >= 0) {
    mantissa >>= static_cast<unsigned>(shift);
  } else {
    mantissa <<= static_cast<unsigned>(-shift);
  }
  const std::uint64_t bits = (static_cast<std::uint64_t>(exponent) << 52U) |
                             (mantissa & 0xFFFFFFFFFFFFFULL);
  return std::bit_cast<double>(bits);
}

template <typename WorkT, typename FloatT>
  requires std::is_floating_point_v<FloatT>
constexpr WorkT PositiveFloatToWork(FloatT value) {
  if constexpr (std::same_as<FloatT, double>) {
    return DoubleToWork<WorkT>(value);
  } else {
    return FloatToWork<WorkT>(static_cast<float>(value));
  }
}

template <typename WorkT, typename FloatT>
  requires std::is_floating_point_v<FloatT>
constexpr FloatT WorkToPositiveFloat(WorkT value) {
  if constexpr (std::same_as<FloatT, double>) {
    return WorkToDouble(value);
  } else {
    return WorkToFloat(value);
  }
}

template <typename WorkT, typename RuntimeT>
constexpr WorkT ToWorkRuntime(RuntimeT value) {
  if constexpr (std::is_floating_point_v<RuntimeT>) {
    return PositiveFloatToWork<WorkT>(value);
  } else if constexpr (numeric_traits<RuntimeT>::kIsFixedPoint) {
    return fixed_math::internal::cast_fixed<WorkT>(value);
  } else {
    return WorkT::FromInteger(static_cast<std::int64_t>(value));
  }
}

template <typename RuntimeT, typename WorkT>
constexpr RuntimeT FromWorkRuntime(WorkT value) {
  if constexpr (std::is_floating_point_v<RuntimeT>) {
    return WorkToPositiveFloat<WorkT, RuntimeT>(value);
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
// BoundaryCode is the highest code used by the magnitude mapping; it may be
// less than numeric_traits<WireT>::kRawMax. When omitted, the default is
// min(255, kRawMax) via DefaultBoundaryCode<WireT>().
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
                "numeric/exponential_floating_runtime.h for float/double.");
  static_assert(numeric_traits<WireT>::kIsIntegerLike,
                "Exponential WireT must be integer-like");
  static_assert(!numeric_traits<WireT>::kIsSigned,
                "Exponential WireT must be unsigned for now");
  static_assert(BoundaryCode > 0, "Exponential BoundaryCode must be positive");
  static_assert(BoundaryCode <= numeric_traits<WireT>::kRawMax,
                "Exponential BoundaryCode must fit in WireT range");
  static_assert(RuntimeTraits::FromDouble(MinMagnitude) >
                    RuntimeTraits::FromInteger(0),
                "Exponential MinMagnitude is below RuntimeT precision");

  static constexpr wire_value_type kPositiveBoundaryCode =
      kBoundaryCode - (kBoundaryCode % wire_value_type{2});
  static constexpr wire_value_type kNegativeBoundaryCode =
      kBoundaryCode % wire_value_type {2} == wire_value_type {0}
          ? static_cast<wire_value_type>(kBoundaryCode - wire_value_type {1})
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

  constexpr Exponential operator-() const requires kIsSigned {
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

#endif  // NUMERIC_EXPONENTIAL_H_
