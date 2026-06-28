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

#include <array>
#include <cstdint>
#include <optional>
#include <type_traits>
#include <utility>

#include <gcem.hpp>

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

template <typename RuntimeT, auto MinMagnitude, auto BoundaryMagnitude,
          std::size_t Index, std::size_t MaxIndex>
consteval RuntimeT MagnitudeAtIndex() {
  using RuntimeTraits = runtime_numeric_traits<RuntimeT>;

  if constexpr (Index == 0) {
    return RuntimeTraits::FromInteger(0);
  } else if constexpr (MaxIndex <= 1) {
    return RuntimeTraits::FromDouble(static_cast<double>(MinMagnitude));
  } else {
    const double min_mag = static_cast<double>(MinMagnitude);
    const double bound_mag = static_cast<double>(BoundaryMagnitude);
    const double ratio = gcem::pow(
        bound_mag / min_mag, 1.0 / static_cast<double>(MaxIndex - 1));
    return RuntimeTraits::FromDouble(
        min_mag * gcem::pow(ratio, static_cast<double>(Index - 1)));
  }
}

template <typename RuntimeT, auto MinMagnitude, auto BoundaryMagnitude,
          std::size_t MaxIndex, std::size_t... Is>
consteval std::array<RuntimeT, MaxIndex + 1> BuildMagnitudeTableFromSequence(
    std::index_sequence<Is...>) {
  return {MagnitudeAtIndex<RuntimeT, MinMagnitude, BoundaryMagnitude, Is,
                           MaxIndex>()...};
}

template <typename RuntimeT, auto MinMagnitude, auto BoundaryMagnitude,
          typename WireValue, WireValue MaxIndex>
consteval auto BuildMagnitudeTable() {
  constexpr std::size_t kMaxIndex = static_cast<std::size_t>(MaxIndex);
  return BuildMagnitudeTableFromSequence<RuntimeT, MinMagnitude,
                                         BoundaryMagnitude, kMaxIndex>(
      std::make_index_sequence<kMaxIndex + 1>{});
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

template <typename RuntimeT, typename WireValue, WireValue MaxIndex,
          const std::array<RuntimeT, static_cast<std::size_t>(MaxIndex) + 1>&
              Table>
constexpr WireValue NearestMagnitudeIndex(RuntimeT abs_value) {
  using RuntimeTraits = runtime_numeric_traits<RuntimeT>;

  if (abs_value == RuntimeTraits::FromInteger(0)) {
    return WireValue{0};
  }

  if (abs_value <= Table[1]) {
    return WireValue{1};
  }

  if (abs_value >= Table[static_cast<std::size_t>(MaxIndex)]) {
    return MaxIndex;
  }

  WireValue lo = WireValue{1};
  WireValue hi = MaxIndex;
  while (lo < hi) {
    const WireValue mid = static_cast<WireValue>(lo + (hi - lo + WireValue{1}) /
                                                           WireValue{2});
    if (Table[static_cast<std::size_t>(mid)] <= abs_value) {
      lo = mid;
    } else {
      hi = static_cast<WireValue>(mid - WireValue{1});
    }
  }

  if (lo >= MaxIndex) {
    return MaxIndex;
  }

  const RuntimeT diff_lo =
      RuntimeSub(abs_value, Table[static_cast<std::size_t>(lo)]);
  const RuntimeT diff_hi = RuntimeSub(
      Table[static_cast<std::size_t>(lo + WireValue{1})], abs_value);
  if (diff_hi < diff_lo) {
    return static_cast<WireValue>(lo + WireValue{1});
  }
  return lo;
}

}  // namespace exponential_internal

template <typename RuntimeT, typename WireT, auto MinMagnitude,
          auto BoundaryMagnitude,
          auto BoundaryCode = numeric_traits<WireT>::kRawMax>
class Exponential {
 public:
  using runtime_type = RuntimeT;
  using wire_type = WireT;
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

  static constexpr auto kMagnitudeTable =
      exponential_internal::BuildMagnitudeTable<RuntimeT, MinMagnitude,
                                                BoundaryMagnitude,
                                                wire_value_type,
                                                kMaxMagnitudeIndex>();

  static constexpr RuntimeT MagnitudeAt(wire_value_type magnitude_index) {
    return kMagnitudeTable[static_cast<std::size_t>(magnitude_index)];
  }

  static constexpr Exponential from_code(WireT code) {
    return Exponential(exponential_internal::WireFromValue<WireT>(
        exponential_internal::ClampCode(
            exponential_internal::WireRawValue(code), kBoundaryCode)));
  }

  // Explicit raw-code construction.
  static constexpr Exponential FromCode(WireT code) { return from_code(code); }
  static constexpr Exponential Code(WireT code) { return from_code(code); }

  // Whether an integer logical value fits within the declared magnitude range.
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

  // Explicit runtime conversion: encodes a logical integer, clamping to range.
  static constexpr Exponential FromRuntimeInteger(std::int64_t value) {
    return from_runtime(RuntimeTraits::FromInteger(value));
  }

  // Explicit saturating runtime conversion: clamps to the declared range.
  static constexpr Exponential Saturating(std::int64_t value) {
    return from_runtime(RuntimeTraits::FromInteger(value));
  }

  // Checked runtime conversion: nullopt when value exceeds the declared range.
  static constexpr std::optional<Exponential> TryFromRuntimeInteger(
      std::int64_t value) {
    if (!LogicalIntegerInRange(value)) {
      return std::nullopt;
    }
    return from_runtime(RuntimeTraits::FromInteger(value));
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

    if constexpr (kIsSigned) {
      if (value < zero) {
        const auto index =
            exponential_internal::NearestMagnitudeIndex<
                RuntimeT, wire_value_type, kMaxMagnitudeIndex, kMagnitudeTable>(
                abs_value);
        const wire_value_type code =
            static_cast<wire_value_type>(index * wire_value_type{2} -
                                         wire_value_type{1});
        return from_code(exponential_internal::WireFromValue<WireT>(code));
      }
    }

    const auto index =
        exponential_internal::NearestMagnitudeIndex<
            RuntimeT, wire_value_type, kMaxMagnitudeIndex, kMagnitudeTable>(
            abs_value);

    if constexpr (kIsSigned) {
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

  // Encodes a logical integer with compile-time range checking.
  static consteval Exponential EncodeCheckedInteger(std::int64_t value) {
    if (!LogicalIntegerInRange(value)) {
      exponential_internal::ExponentialConstantExceedsRange();
    }
    return from_runtime(RuntimeTraits::FromInteger(value));
  }

  constexpr Exponential() = default;

  // Ordinary numeric construction uses logical values and is consteval:
  // integer constants are range-checked at compile time and runtime values are
  // rejected, so there is no silent clamp through the normal constructor.
  template <std::integral T>
  consteval Exponential(T logical_value)
      : Exponential(EncodeCheckedInteger(static_cast<std::int64_t>(
            logical_value))) {}

  // Floating logical construction is consteval-only (no runtime float/double).
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
          auto BoundaryMagnitude, auto BoundaryCode>
constexpr Exponential<RuntimeT, WireT, MinMagnitude, BoundaryMagnitude,
                      BoundaryCode>
min(Exponential<RuntimeT, WireT, MinMagnitude, BoundaryMagnitude,
                BoundaryCode>
        a,
    Exponential<RuntimeT, WireT, MinMagnitude, BoundaryMagnitude,
                BoundaryCode>
        b) {
  return a < b ? a : b;
}

template <typename RuntimeT, typename WireT, auto MinMagnitude,
          auto BoundaryMagnitude, auto BoundaryCode>
constexpr Exponential<RuntimeT, WireT, MinMagnitude, BoundaryMagnitude,
                      BoundaryCode>
max(Exponential<RuntimeT, WireT, MinMagnitude, BoundaryMagnitude,
                BoundaryCode>
        a,
    Exponential<RuntimeT, WireT, MinMagnitude, BoundaryMagnitude,
                BoundaryCode>
        b) {
  return a < b ? b : a;
}

template <typename RuntimeT, typename WireT, auto MinMagnitude,
          auto BoundaryMagnitude, auto BoundaryCode>
constexpr Exponential<RuntimeT, WireT, MinMagnitude, BoundaryMagnitude,
                      BoundaryCode>
clamp(Exponential<RuntimeT, WireT, MinMagnitude, BoundaryMagnitude,
                  BoundaryCode>
          value,
      Exponential<RuntimeT, WireT, MinMagnitude, BoundaryMagnitude,
                    BoundaryCode>
          low,
      Exponential<RuntimeT, WireT, MinMagnitude, BoundaryMagnitude,
                    BoundaryCode>
          high) {
  return min(max(value, low), high);
}

template <typename RuntimeT, typename WireT, auto MinMagnitude,
          auto BoundaryMagnitude, auto BoundaryCode>
struct numeric_traits<
    Exponential<RuntimeT, WireT, MinMagnitude, BoundaryMagnitude,
                BoundaryCode>> {
  using rep_type = WireT;
  using rep_value_type = typename numeric_traits<WireT>::rep_value_type;
  using value_type =
      Exponential<RuntimeT, WireT, MinMagnitude, BoundaryMagnitude,
                  BoundaryCode>;

  static constexpr bool kIsIntegerLike = false;
  static constexpr bool kIsFixedPoint = false;
  static constexpr bool kIsExponential = true;
  static constexpr bool kIsSigned = value_type::kIsSigned;
};

}  // namespace ae

#endif  // NUMERIC_EXPONENTIAL_H_
