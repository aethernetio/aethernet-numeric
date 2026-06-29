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
    return sub_to<RuntimeT>(lhs, rhs);
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
constexpr LogT LogAtIndexRaw(WireValue index, LogT log_min, std::int64_t log_span_raw,
                               WireValue max_index) {
  if (index <= WireValue{1}) {
    return log_min;
  }
  const std::int64_t num =
      log_span_raw * (static_cast<std::int64_t>(index) - 1);
  const std::int64_t den = static_cast<std::int64_t>(max_index) - 1;
  const std::int64_t offset_raw =
      fixed_point_internal::RoundDivNearest(num, den);
  const std::int64_t raw =
      static_cast<std::int64_t>(log_min.raw_value()) + offset_raw;
  return LogT::FromRaw(fixed_point_internal::RepFromRawValue<
                       typename LogT::rep_type>(
      static_cast<typename LogT::rep_value_type>(raw)));
}

template <typename LogT, typename WireValue>
constexpr WireValue EncodeMagnitudeIndexRaw(LogT log_min, std::int64_t log_span_raw,
                                              WireValue max_index, LogT log_value) {
  if (max_index <= WireValue{1}) {
    return WireValue{1};
  }
  if (log_span_raw == 0) {
    return WireValue{1};
  }

  const std::int64_t pos_raw =
      static_cast<std::int64_t>(log_value.raw_value()) -
      static_cast<std::int64_t>(log_min.raw_value());
  const std::int64_t den = static_cast<std::int64_t>(max_index) - 1;
  const std::int64_t adjusted =
      pos_raw * den + log_span_raw / std::int64_t{2};
  const std::int64_t index_offset =
      fixed_point_internal::RoundDivNearest(adjusted, log_span_raw);
  return ClampIndex(static_cast<WireValue>(index_offset + 1), max_index);
}

using WorkFixed = FixedPoint<std::uint32_t, 60.0>;

constexpr WorkFixed FloatToWork(float value) {
  const auto bits = std::bit_cast<std::uint32_t>(value);
  if (bits == 0U) {
    return WorkFixed::FromInteger(0);
  }

  const int exponent =
      static_cast<int>((bits >> 23U) & 0xFFU) - 127;
  std::uint64_t mantissa = (static_cast<std::uint64_t>(bits & 0x7FFFFFU) |
                            std::uint64_t{0x800000U});
  const int shift =
      exponent + (WorkFixed::kScaleExp < 0 ? -WorkFixed::kScaleExp : 0) - 23;

  if (shift >= 0) {
    for (int i = 0; i < shift && i < 62; ++i) {
      mantissa <<= 1U;
    }
  } else {
    for (int i = 0; i < -shift && i < 62; ++i) {
      mantissa = static_cast<std::uint64_t>(fixed_point_internal::RoundDivNearest(
          static_cast<std::int64_t>(mantissa), std::int64_t{2}));
    }
  }

  return WorkFixed::FromRaw(fixed_point_internal::RepFromRawValue<
                            typename WorkFixed::rep_type>(
      WorkFixed::ClampRaw(static_cast<typename WorkFixed::rep_value_type>(
          mantissa))));
}

constexpr WorkFixed DoubleToWork(double value) {
  const auto bits = std::bit_cast<std::uint64_t>(value);
  if (bits == 0ULL) {
    return WorkFixed::FromInteger(0);
  }
  const int exponent =
      static_cast<int>((bits >> 52U) & 0x7FFU) - 1023;
  std::uint64_t mantissa =
      (bits & 0xFFFFFFFFFFFFFULL) | (std::uint64_t{1} << 52U);
  const int shift =
      exponent + (WorkFixed::kScaleExp < 0 ? -WorkFixed::kScaleExp : 0) - 52;
  if (shift >= 0) {
    for (int i = 0; i < shift && i < 62; ++i) {
      mantissa <<= 1U;
    }
  } else {
    for (int i = 0; i < -shift && i < 62; ++i) {
      mantissa = static_cast<std::uint64_t>(fixed_point_internal::RoundDivNearest(
          static_cast<std::int64_t>(mantissa), std::int64_t{2}));
    }
  }
  return WorkFixed::FromRaw(fixed_point_internal::RepFromRawValue<
                            typename WorkFixed::rep_type>(
      WorkFixed::ClampRaw(static_cast<typename WorkFixed::rep_value_type>(
          mantissa))));
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

constexpr float WorkToFloat(WorkFixed value) {
  if (value.raw_value() == typename WorkFixed::rep_value_type{0}) {
    return 0.0f;
  }

  std::uint64_t mantissa = value.raw_value();
  const int msb = HighestSetBit(mantissa);
  const int scale_bits =
      WorkFixed::kScaleExp < 0 ? -WorkFixed::kScaleExp : 0;
  const int exponent = msb - scale_bits + 127;
  const int shift = msb - 23;
  if (shift >= 0) {
    mantissa >>= static_cast<unsigned>(shift);
  } else {
    mantissa <<= static_cast<unsigned>(-shift);
  }
  const std::uint32_t bits =
      (static_cast<std::uint32_t>(exponent) << 23U) |
      (static_cast<std::uint32_t>(mantissa) & 0x7FFFFFU);
  return std::bit_cast<float>(bits);
}

constexpr double WorkToDouble(WorkFixed value) {
  if (value.raw_value() == typename WorkFixed::rep_value_type{0}) {
    return 0.0;
  }

  std::uint64_t mantissa = value.raw_value();
  const int msb = HighestSetBit(mantissa);
  const int scale_bits =
      WorkFixed::kScaleExp < 0 ? -WorkFixed::kScaleExp : 0;
  const int exponent = msb - scale_bits + 1023;
  const int shift = msb - 52;
  if (shift >= 0) {
    mantissa >>= static_cast<unsigned>(shift);
  } else {
    mantissa <<= static_cast<unsigned>(-shift);
  }
  const std::uint64_t bits =
      (static_cast<std::uint64_t>(exponent) << 52U) |
      (mantissa & 0xFFFFFFFFFFFFFULL);
  return std::bit_cast<double>(bits);
}

template <typename FloatT>
  requires std::is_floating_point_v<FloatT>
constexpr WorkFixed PositiveFloatToWork(FloatT value) {
  if constexpr (std::same_as<FloatT, double>) {
    return DoubleToWork(value);
  } else {
    return FloatToWork(static_cast<float>(value));
  }
}

template <typename FloatT>
  requires std::is_floating_point_v<FloatT>
constexpr FloatT WorkToPositiveFloat(WorkFixed value) {
  if constexpr (std::same_as<FloatT, double>) {
    return WorkToDouble(value);
  } else {
    return WorkToFloat(value);
  }
}

template <typename RuntimeT>
constexpr auto ToWorkRuntime(RuntimeT value) {
  if constexpr (std::is_floating_point_v<RuntimeT>) {
    return PositiveFloatToWork(value);
  } else if constexpr (numeric_traits<RuntimeT>::kIsFixedPoint) {
    return fixed_math::internal::cast_fixed<WorkFixed>(value);
  } else {
    return WorkFixed::FromInteger(static_cast<std::int64_t>(value));
  }
}

template <typename RuntimeT>
constexpr RuntimeT FromWorkRuntime(WorkFixed value) {
  if constexpr (std::is_floating_point_v<RuntimeT>) {
    return WorkToPositiveFloat<RuntimeT>(value);
  } else if constexpr (numeric_traits<RuntimeT>::kIsFixedPoint) {
    return fixed_math::internal::cast_fixed<RuntimeT>(value);
  } else {
    return runtime_numeric_traits<RuntimeT>::FromInteger(
        static_cast<std::int64_t>(value.raw_value()));
  }
}

template <typename RuntimeT, typename LogT>
constexpr RuntimeT LogExp2(LogT log_value) {
  const WorkFixed work = fixed_math::exp2_to<WorkFixed>(log_value);
  return FromWorkRuntime<RuntimeT>(work);
}

}  // namespace exponential_internal

template <typename RuntimeT, typename WireT, auto MinMagnitude,
          auto BoundaryMagnitude,
          auto BoundaryCode =
              exponential_internal::DefaultBoundaryCode<WireT>(),
          typename LogT = FixedPoint<std::int16_t, 16.0>>
class Exponential {
 public:
  using runtime_type = RuntimeT;
  using wire_type = WireT;
  using log_type = LogT;
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
  static_assert(BoundaryCode > 0,
                "Exponential BoundaryCode must be positive");
  static_assert(BoundaryCode <= numeric_traits<WireT>::kRawMax,
                "Exponential BoundaryCode must fit in WireT range");
  static_assert(
      RuntimeTraits::FromDouble(MinMagnitude) > RuntimeTraits::FromInteger(0),
      "Exponential MinMagnitude is below RuntimeT precision");

  static constexpr wire_value_type kPositiveBoundaryCode =
      kBoundaryCode - (kBoundaryCode % wire_value_type{2});
  static constexpr wire_value_type kNegativeBoundaryCode =
      kBoundaryCode % wire_value_type{2} == wire_value_type{0}
          ? static_cast<wire_value_type>(kBoundaryCode - wire_value_type{1})
          : kBoundaryCode;
  static constexpr wire_value_type kMaxMagnitudeIndex =
      kIsSigned ? static_cast<wire_value_type>(kPositiveBoundaryCode /
                                               wire_value_type{2})
                : kBoundaryCode;

  static constexpr LogT kLogMin =
      fixed_math::log2_to<LogT>(exponential_internal::WorkFixed::FromDouble(
          static_cast<double>(MinMagnitude)));
  static constexpr LogT kLogBoundary =
      fixed_math::log2_to<LogT>(exponential_internal::WorkFixed::FromDouble(
          static_cast<double>(BoundaryMagnitude)));
  static constexpr std::int64_t kLogSpanRaw =
      static_cast<std::int64_t>(kLogBoundary.raw_value()) -
      static_cast<std::int64_t>(kLogMin.raw_value());

  static constexpr RuntimeT MagnitudeAt(wire_value_type magnitude_index) {
    if (magnitude_index == wire_value_type{0}) {
      return RuntimeTraits::FromInteger(0);
    }
    const LogT log_value = exponential_internal::LogAtIndexRaw(
        magnitude_index, kLogMin, kLogSpanRaw, kMaxMagnitudeIndex);
    return exponential_internal::LogExp2<RuntimeT, LogT>(log_value);
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

    const LogT log_value = fixed_math::log2_to<LogT>(
        exponential_internal::ToWorkRuntime(abs_value));
    return exponential_internal::EncodeMagnitudeIndexRaw(
        kLogMin, kLogSpanRaw, kMaxMagnitudeIndex, log_value);
  }

  static constexpr Exponential from_code(WireT code) {
    return Exponential(exponential_internal::WireFromValue<WireT>(
        exponential_internal::ClampCode(
            exponential_internal::WireRawValue(code), kBoundaryCode)));
  }

  static constexpr Exponential FromCode(WireT code) { return from_code(code); }
  static constexpr Exponential Code(WireT code) { return from_code(code); }

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
    return from_runtime(RuntimeTraits::FromInteger(value));
  }

  static constexpr Exponential Saturating(std::int64_t value) {
    return from_runtime(RuntimeTraits::FromInteger(value));
  }

  static constexpr std::optional<Exponential> TryFromRuntimeInteger(
      std::int64_t value) {
    if (!LogicalIntegerInRange(value)) {
      return std::nullopt;
    }
    return from_runtime(RuntimeTraits::FromInteger(value));
  }

  static constexpr Exponential FromRuntime(RuntimeT value) {
    return from_runtime(value);
  }

  static constexpr std::optional<Exponential> TryFromRuntime(RuntimeT value) {
    const RuntimeT zero = RuntimeTraits::FromInteger(0);
    const RuntimeT bound = RuntimeTraits::FromDouble(BoundaryMagnitude);
    const RuntimeT abs_value =
        exponential_internal::AbsRuntime<RuntimeT, kIsSigned>(value);
    if (abs_value > bound) {
      return std::nullopt;
    }
    return from_runtime(value);
  }

  static constexpr Exponential from_runtime(RuntimeT value) {
    const RuntimeT zero = RuntimeTraits::FromInteger(0);
    if (value == zero) {
      return from_code(exponential_internal::WireFromValue<WireT>(
          wire_value_type{0}));
    }

    const RuntimeT abs_value =
        exponential_internal::AbsRuntime<RuntimeT, kIsSigned>(value);
    const RuntimeT boundary = MagnitudeAt(kMaxMagnitudeIndex);

    if (abs_value >= boundary) {
      if constexpr (kIsSigned) {
        if (value < zero) {
          return from_code(exponential_internal::WireFromValue<WireT>(
              kNegativeBoundaryCode));
        }
        return from_code(exponential_internal::WireFromValue<WireT>(
            kPositiveBoundaryCode));
      }
      return from_code(
          exponential_internal::WireFromValue<WireT>(kBoundaryCode));
    }

    const auto index = MagnitudeIndexFor(abs_value);

    if constexpr (kIsSigned) {
      if (value < zero) {
        const wire_value_type code =
            static_cast<wire_value_type>(index * wire_value_type{2} -
                                         wire_value_type{1});
        return from_code(exponential_internal::WireFromValue<WireT>(code));
      }
      const wire_value_type code =
          static_cast<wire_value_type>(index * wire_value_type{2});
      return from_code(exponential_internal::WireFromValue<WireT>(code));
    }

    return from_code(
        exponential_internal::WireFromValue<WireT>(index));
  }

  static consteval Exponential PositiveBoundaryCode() {
    return from_code(
        exponential_internal::WireFromValue<WireT>(kPositiveBoundaryCode));
  }

  static consteval Exponential NegativeBoundaryCode() {
    return from_code(
        exponential_internal::WireFromValue<WireT>(kNegativeBoundaryCode));
  }

  static consteval Exponential MinPositiveCode() {
    if constexpr (kIsSigned) {
      return from_code(exponential_internal::WireFromValue<WireT>(
          wire_value_type{2}));
    }
    return from_code(exponential_internal::WireFromValue<WireT>(
        wire_value_type{1}));
  }

  static consteval Exponential from_double(double value) {
    if (value == 0.0) {
      return from_code(exponential_internal::WireFromValue<WireT>(
          wire_value_type{0}));
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
          return from_code(exponential_internal::WireFromValue<WireT>(
              wire_value_type{1}));
        }
      }
    } else {
      if (value >= bound_mag) {
        return from_code(
            exponential_internal::WireFromValue<WireT>(kBoundaryCode));
      }
      if (value < min_mag) {
        return MinPositiveCode();
      }
    }

    return from_runtime(RuntimeTraits::FromDouble(value));
  }

  static consteval Exponential EncodeCheckedInteger(std::int64_t value) {
    if (!LogicalIntegerInRange(value)) {
      exponential_internal::ExponentialConstantExceedsRange();
    }
    return from_runtime(RuntimeTraits::FromInteger(value));
  }

  constexpr Exponential() = default;

  template <std::integral T>
  consteval Exponential(T logical_value)
      : Exponential(EncodeCheckedInteger(static_cast<std::int64_t>(
            logical_value))) {}

  consteval Exponential(double logical_value)
      : Exponential(from_double(logical_value)) {}

  consteval Exponential(float logical_value)
      : Exponential(from_double(static_cast<double>(logical_value))) {}

  constexpr WireT code() const { return code_; }

  constexpr RuntimeT value() const { return to_runtime(); }

  constexpr wire_value_type code_value() const {
    return exponential_internal::WireRawValue(code_);
  }

  constexpr bool is_zero() const { return code_value() == wire_value_type{0}; }

  constexpr bool is_positive() const {
    if (is_zero()) {
      return false;
    }
    if constexpr (kIsSigned) {
      return code_value() % wire_value_type{2} == wire_value_type{0};
    }
    return true;
  }

  constexpr bool is_negative() const {
    if constexpr (!kIsSigned) {
      return false;
    }
    return !is_zero() && code_value() % wire_value_type{2} == wire_value_type{1};
  }

  constexpr wire_value_type magnitude_index() const {
    const auto code = code_value();
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

  constexpr RuntimeT to_runtime() const {
    const auto index = magnitude_index();
    if (index == wire_value_type{0}) {
      return RuntimeTraits::FromInteger(0);
    }
    const RuntimeT magnitude = MagnitudeAt(index);
    if (is_negative()) {
      return exponential_internal::RuntimeSub(RuntimeTraits::FromInteger(0),
                                            magnitude);
    }
    return magnitude;
  }

  constexpr Exponential abs() const {
    if (is_zero() || is_positive()) {
      return *this;
    }
    const wire_value_type code =
        static_cast<wire_value_type>(magnitude_index() * wire_value_type{2});
    return from_code(exponential_internal::WireFromValue<WireT>(code));
  }

  constexpr Exponential operator-() const
      requires kIsSigned
  {
    if (is_zero()) {
      return *this;
    }
    if (is_positive()) {
      const wire_value_type code =
          static_cast<wire_value_type>(code_value() - wire_value_type{1});
      return from_code(exponential_internal::WireFromValue<WireT>(code));
    }
    const wire_value_type code =
        static_cast<wire_value_type>(code_value() + wire_value_type{1});
    return from_code(exponential_internal::WireFromValue<WireT>(code));
  }

  constexpr bool operator==(const Exponential& other) const {
    return code_value() == other.code_value();
  }

  constexpr bool operator!=(const Exponential& other) const {
    return !(*this == other);
  }

  constexpr bool operator<(const Exponential& other) const {
    if (code_value() == other.code_value()) {
      return false;
    }
    if constexpr (kIsSigned) {
      if (is_zero() || other.is_zero()) {
        if (is_zero()) {
          return other.is_positive();
        }
        return is_negative();
      }
      if (is_negative() != other.is_negative()) {
        return is_negative();
      }
      if (is_negative()) {
        return code_value() > other.code_value();
      }
      return code_value() < other.code_value();
    }
    return code_value() < other.code_value();
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
          auto BoundaryMagnitude, auto BoundaryCode, typename LogT>
constexpr Exponential<RuntimeT, WireT, MinMagnitude, BoundaryMagnitude,
                      BoundaryCode, LogT>
min(Exponential<RuntimeT, WireT, MinMagnitude, BoundaryMagnitude,
                BoundaryCode, LogT>
        a,
    Exponential<RuntimeT, WireT, MinMagnitude, BoundaryMagnitude,
                BoundaryCode, LogT>
        b) {
  return a < b ? a : b;
}

template <typename RuntimeT, typename WireT, auto MinMagnitude,
          auto BoundaryMagnitude, auto BoundaryCode, typename LogT>
constexpr Exponential<RuntimeT, WireT, MinMagnitude, BoundaryMagnitude,
                      BoundaryCode, LogT>
max(Exponential<RuntimeT, WireT, MinMagnitude, BoundaryMagnitude,
                BoundaryCode, LogT>
        a,
    Exponential<RuntimeT, WireT, MinMagnitude, BoundaryMagnitude,
                BoundaryCode, LogT>
        b) {
  return a < b ? b : a;
}

template <typename RuntimeT, typename WireT, auto MinMagnitude,
          auto BoundaryMagnitude, auto BoundaryCode, typename LogT>
constexpr Exponential<RuntimeT, WireT, MinMagnitude, BoundaryMagnitude,
                      BoundaryCode, LogT>
clamp(Exponential<RuntimeT, WireT, MinMagnitude, BoundaryMagnitude,
                  BoundaryCode, LogT>
          value,
      Exponential<RuntimeT, WireT, MinMagnitude, BoundaryMagnitude,
                    BoundaryCode, LogT>
          low,
      Exponential<RuntimeT, WireT, MinMagnitude, BoundaryMagnitude,
                    BoundaryCode, LogT>
          high) {
  return min(max(value, low), high);
}

template <typename RuntimeT, typename WireT, auto MinMagnitude,
          auto BoundaryMagnitude, auto BoundaryCode, typename LogT>
struct numeric_traits<
    Exponential<RuntimeT, WireT, MinMagnitude, BoundaryMagnitude, BoundaryCode,
                LogT>> {
  using rep_type = WireT;
  using rep_value_type = typename numeric_traits<WireT>::rep_value_type;
  using value_type =
      Exponential<RuntimeT, WireT, MinMagnitude, BoundaryMagnitude,
                  BoundaryCode, LogT>;

  static constexpr bool kIsIntegerLike = false;
  static constexpr bool kIsFixedPoint = false;
  static constexpr bool kIsExponential = true;
  static constexpr bool kIsSigned = value_type::kIsSigned;
};

}  // namespace ae

#endif  // NUMERIC_EXPONENTIAL_H_
