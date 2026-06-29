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

#ifndef NUMERIC_FIXED_POINT_H_
#define NUMERIC_FIXED_POINT_H_

#include <cstdint>
#include <limits>
#include <optional>
#include <type_traits>
#include <utility>

#include "numeric/decimal.h"
#include "numeric/numeric_traits.h"
#include "numeric/runtime_numeric_traits.h"

namespace ae {

template <typename RepA, typename RepB>
struct PromoteRep;

namespace fixed_point_internal {

struct Rational {
  std::int64_t num = 0;
  std::int64_t den = 1;
};

struct raw_storage_t {};
struct logical_storage_t {};

template <typename>
constexpr bool DependentlyFalse = false;

constexpr Rational Normalize(Rational r) {
  if (r.den < 0) {
    r.num = -r.num;
    r.den = -r.den;
  }
  if (r.num == 0) {
    return {0, 1};
  }
  const auto g = Gcd64(r.num, r.den);
  return {r.num / g, r.den / g};
}

consteval Rational DoubleToRational(double value) {
  if (value == 0.0) {
    return {0, 1};
  }

  const bool negative = value < 0.0;
  const double abs_value = negative ? -value : value;

  for (unsigned scale = 0; scale <= 18; ++scale) {
    const std::uint64_t pow10 = Pow10u(scale);
    const double scaled = abs_value * static_cast<double>(pow10);
    const auto rounded = static_cast<std::int64_t>(scaled + 0.5);

    if (rounded != 0 || abs_value == 0.0) {
      const double reconstructed =
          static_cast<double>(rounded) / static_cast<double>(pow10);
      const double abs_error = reconstructed > abs_value
                                   ? reconstructed - abs_value
                                   : abs_value - reconstructed;
      if (abs_error < 0.000000001) {
        Rational r{negative ? -rounded : rounded,
                   static_cast<std::int64_t>(pow10)};
        return Normalize(r);
      }
    }
  }

  return {negative ? -1 : 1, 1};
}

template <auto V>
consteval Rational BoundValueAsRational() {
  using ValueType = decltype(V);
  if constexpr (std::is_integral_v<ValueType>) {
    return {static_cast<std::int64_t>(V), 1};
  } else {
    return DoubleToRational(V);
  }
}

template <auto V>
struct BoundRatio {
  static constexpr auto kValue = V;
  static constexpr Rational kRational = BoundValueAsRational<V>();
  static constexpr std::int64_t num = kRational.num;
  static constexpr std::int64_t den = kRational.den;
  static constexpr bool kIsPositive = (num > 0) && (den > 0);
};

constexpr bool PositiveRationalLE(std::int64_t an, std::int64_t ad,
                                  std::int64_t bn, std::int64_t bd) {
  return an * bd <= bn * ad;
}

template <auto Max, bool kIsSigned>
constexpr bool LogicalWithinDeclaredMax(std::int64_t num, std::int64_t den) {
  if (den <= 0) {
    return false;
  }
  constexpr std::int64_t max_num = BoundRatio<Max>::num;
  constexpr std::int64_t max_den = BoundRatio<Max>::den;

  if constexpr (kIsSigned) {
    if (num < 0) {
      if (num == std::numeric_limits<std::int64_t>::min()) {
        return false;
      }
      num = -num;
    }
    return PositiveRationalLE(num, den, max_num, max_den);
  } else {
    if (num < 0) {
      return false;
    }
    return PositiveRationalLE(num, den, max_num, max_den);
  }
}

// Intentionally non-constexpr: naming this function on a branch that is
// reached during constant evaluation makes the enclosing consteval
// construction ill-formed, turning an out-of-range constant into a compile
// error instead of a silent clamp.
inline void FixedPointConstantExceedsDeclaredMax() {}

template <auto Max, bool kIsSigned>
constexpr std::pair<std::int64_t, std::int64_t> ClampLogicalRational(
    std::int64_t num, std::int64_t den) {
  if (den <= 0) {
    den = 1;
  }
  if (LogicalWithinDeclaredMax<Max, kIsSigned>(num, den)) {
    return {num, den};
  }
  if constexpr (kIsSigned) {
    if (num < 0) {
      return {-BoundRatio<Max>::num, BoundRatio<Max>::den};
    }
  }
  return {BoundRatio<Max>::num, BoundRatio<Max>::den};
}

consteval bool RawScaledCoversMax(std::int64_t raw_max, int scale_exp,
                                  std::int64_t max_num, std::int64_t max_den) {
  if (scale_exp >= 0) {
    std::int64_t factor = 1;
    for (int i = 0; i < scale_exp; ++i) {
      if (factor > std::numeric_limits<std::int64_t>::max() / 2) {
        return true;
      }
      factor *= 2;
    }
    if (raw_max > std::numeric_limits<std::int64_t>::max() / factor) {
      return true;
    }
    const std::int64_t lhs = raw_max * factor * max_den;
    return lhs >= max_num;
  }

  const int shift = -scale_exp;
  std::int64_t rhs = max_num;
  for (int i = 0; i < shift; ++i) {
    if (rhs > std::numeric_limits<std::int64_t>::max() / 2) {
      return raw_max * max_den >= rhs;
    }
    rhs *= 2;
  }
  return raw_max * max_den >= rhs;
}

consteval int ComputeScaleExp(std::int64_t raw_max, std::int64_t max_num,
                              std::int64_t max_den) {
  for (int k = -60; k <= 60; ++k) {
    if (RawScaledCoversMax(raw_max, k, max_num, max_den)) {
      return k;
    }
  }
  return 0;
}

consteval Rational ScaleRationalByPow2(Rational value, int scale_exp) {
  if (scale_exp == 0 || value.num == 0) {
    return value;
  }
  if (scale_exp > 0) {
    for (int i = 0; i < scale_exp; ++i) {
      value.num *= 2;
    }
    return Normalize(value);
  }
  for (int i = 0; i < -scale_exp; ++i) {
    value.den *= 2;
  }
  return Normalize(value);
}

template <typename RepValue>
constexpr RepValue ClampRaw(RepValue raw, RepValue raw_min, RepValue raw_max) {
  if (raw < raw_min) {
    return raw_min;
  }
  if (raw > raw_max) {
    return raw_max;
  }
  return raw;
}

template <typename Rep>
constexpr auto RepRawValue(const Rep& rep) {
  if constexpr (requires { rep.value_; }) {
    return rep.value_;
  } else {
    return rep;
  }
}

template <typename Rep, typename RepValue>
constexpr Rep RepFromRawValue(RepValue raw) {
  if constexpr (std::same_as<Rep, RepValue>) {
    return raw;
  } else if constexpr (requires { Rep{raw}; }) {
    return Rep{raw};
  } else {
    return Rep{static_cast<typename Rep::ValueType>(raw)};
  }
}

template <typename RepValue>
constexpr RepValue SaturatedAddRaw(RepValue lhs, RepValue rhs, RepValue raw_min,
                                   RepValue raw_max) {
  if constexpr (std::is_signed_v<RepValue>) {
    if (rhs > 0 && lhs > raw_max - rhs) {
      return raw_max;
    }
    if (rhs < 0 && lhs < raw_min - rhs) {
      return raw_min;
    }
    return static_cast<RepValue>(lhs + rhs);
  } else {
    if (lhs > raw_max - rhs) {
      return raw_max;
    }
    return static_cast<RepValue>(lhs + rhs);
  }
}

template <typename RepValue>
constexpr RepValue SaturatedSubRaw(RepValue lhs, RepValue rhs, RepValue raw_min,
                                   RepValue raw_max) {
  if constexpr (std::is_signed_v<RepValue>) {
    if (rhs > 0 && lhs < raw_min + rhs) {
      return raw_min;
    }
    if (rhs < 0 && lhs > raw_max + rhs) {
      return raw_max;
    }
    return static_cast<RepValue>(lhs - rhs);
  } else {
    if (rhs > lhs) {
      return raw_min;
    }
    return static_cast<RepValue>(lhs - rhs);
  }
}

template <typename RepValue>
constexpr RepValue SaturatedLeftShift(RepValue raw, unsigned bits,
                                      RepValue raw_min, RepValue raw_max) {
  if (bits == 0) {
    return raw;
  }
  if constexpr (std::is_signed_v<RepValue>) {
    for (unsigned i = 0; i < bits; ++i) {
      if (raw > 0 && raw > raw_max / RepValue{2}) {
        return raw_max;
      }
      if (raw < 0 && raw < raw_min / RepValue{2}) {
        return raw_min;
      }
      raw = static_cast<RepValue>(raw * RepValue{2});
    }
    return ClampRaw(raw, raw_min, raw_max);
  } else {
    for (unsigned i = 0; i < bits; ++i) {
      if (raw > raw_max / RepValue{2}) {
        return raw_max;
      }
      raw = static_cast<RepValue>(raw * RepValue{2});
    }
    return raw;
  }
}

template <typename RepValue>
constexpr RepValue RoundShiftRight(RepValue raw, unsigned bits) {
  if (bits == 0) {
    return raw;
  }

  using Unsigned = std::make_unsigned_t<RepValue>;
  constexpr unsigned kWidth = sizeof(RepValue) * 8;

  if (bits >= kWidth) {
    if (raw == RepValue{0}) {
      return RepValue{0};
    }
    if (bits > kWidth) {
      return RepValue{0};
    }
    const Unsigned threshold = Unsigned{1} << (kWidth - 1);
    if constexpr (std::is_signed_v<RepValue>) {
      const Unsigned magnitude =
          raw >= RepValue{0}
              ? static_cast<Unsigned>(raw)
              : static_cast<Unsigned>(static_cast<RepValue>(-raw));
      if (magnitude >= threshold) {
        return raw >= RepValue{0} ? RepValue{1} : RepValue{-1};
      }
      return RepValue{0};
    }
    const Unsigned magnitude = static_cast<Unsigned>(raw);
    return magnitude >= threshold ? RepValue{1} : RepValue{0};
  }

  const Unsigned divisor = Unsigned{1} << bits;
  const Unsigned bias = divisor / Unsigned{2};
  if constexpr (std::is_signed_v<RepValue>) {
    if (raw >= RepValue{0}) {
      return static_cast<RepValue>((static_cast<Unsigned>(raw) + bias) /
                                   divisor);
    }
    const Unsigned magnitude =
        static_cast<Unsigned>(static_cast<RepValue>(-raw));
    return static_cast<RepValue>(
        -static_cast<RepValue>((magnitude + bias) / divisor));
  }
  return static_cast<RepValue>((static_cast<Unsigned>(raw) + bias) / divisor);
}

template <typename RepValue>
constexpr RepValue ConvertRawScale(RepValue raw, int source_scale_exp,
                                   int target_scale_exp, RepValue raw_min,
                                   RepValue raw_max) {
  const int delta = source_scale_exp - target_scale_exp;
  if (delta > 0) {
    return SaturatedLeftShift(raw, static_cast<unsigned>(delta), raw_min,
                              raw_max);
  }
  if (delta < 0) {
    return ClampRaw(RoundShiftRight(raw, static_cast<unsigned>(-delta)),
                    raw_min, raw_max);
  }
  return ClampRaw(raw, raw_min, raw_max);
}

template <typename RepValue>
constexpr RepValue RoundDivNearest(RepValue num, RepValue den) {
  if (den == RepValue{0}) {
    return num;
  }
  if constexpr (std::is_signed_v<RepValue>) {
    if (num >= 0 && den > 0) {
      return static_cast<RepValue>((num + den / RepValue{2}) / den);
    }
    if (num < 0 && den > 0) {
      return static_cast<RepValue>((num - den / RepValue{2}) / den);
    }
    if (num >= 0 && den < 0) {
      return static_cast<RepValue>((num - den / RepValue{2}) / den);
    }
    return static_cast<RepValue>((num + den / RepValue{2}) / den);
  } else {
    return static_cast<RepValue>((num + den / RepValue{2}) / den);
  }
}

template <typename RepValue>
constexpr RepValue RawFromRatioAtScale(std::int64_t num, std::int64_t den,
                                       int scale_exp, RepValue raw_min,
                                       RepValue raw_max) {
  if (den == 0) {
    return raw_min;
  }

  const bool negative = (num < 0) ^ (den < 0);
  num = num < 0 ? -num : num;
  den = den < 0 ? -den : den;

  if (scale_exp < 0) {
    const unsigned shift = static_cast<unsigned>(-scale_exp);
    std::int64_t factor = 1;
    for (unsigned i = 0; i < shift; ++i) {
      factor *= 2;
    }
    num *= factor;
  } else if (scale_exp > 0) {
    const unsigned shift = static_cast<unsigned>(scale_exp);
    std::int64_t factor = 1;
    for (unsigned i = 0; i < shift; ++i) {
      factor *= 2;
    }
    den *= factor;
  }

  const std::int64_t rounded = RoundDivNearest(num, den);
  const std::int64_t clamped =
      negative
          ? std::max<std::int64_t>(static_cast<std::int64_t>(raw_min), -rounded)
          : std::min<std::int64_t>(static_cast<std::int64_t>(raw_max), rounded);
  return static_cast<RepValue>(clamped);
}

template <typename Rep, auto Max, bool kIsSigned>
constexpr typename numeric_traits<Rep>::rep_value_type MakeRawFromLogical(
    std::int64_t num, std::int64_t den) {
  using RepValue = typename numeric_traits<Rep>::rep_value_type;
  constexpr RepValue kRawMax = numeric_traits<Rep>::kRawMax;
  constexpr RepValue kRawMin =
      kIsSigned ? static_cast<RepValue>(-static_cast<std::int64_t>(kRawMax))
                : RepValue{0};
  constexpr int kScaleExp =
      ComputeScaleExp(static_cast<std::int64_t>(kRawMax), BoundRatio<Max>::num,
                      BoundRatio<Max>::den);
  const auto clamped = ClampLogicalRational<Max, kIsSigned>(num, den);
  return RawFromRatioAtScale(clamped.first, clamped.second, kScaleExp, kRawMin,
                             kRawMax);
}

template <auto Max, bool kIsSigned, std::int64_t Num, std::int64_t Den = 1>
struct LogicalBoundChecker {
  static_assert(LogicalWithinDeclaredMax<Max, kIsSigned>(Num, Den),
                "logical value exceeds declared FixedPoint Max");
  static constexpr bool kOk = true;
};

template <typename Rep, auto Max, bool kIsSigned, std::int64_t Num,
          std::int64_t Den = 1>
constexpr typename numeric_traits<Rep>::rep_value_type
MakeRawFromLogicalConstexpr(std::integral_constant<std::int64_t, Num>,
                            std::integral_constant<std::int64_t, Den> = {}) {
  (void)LogicalBoundChecker<Max, kIsSigned, Num, Den>{};
  return MakeRawFromLogical<Rep, Max, kIsSigned>(Num, Den);
}

}  // namespace fixed_point_internal

template <auto V>
using BoundRatio = fixed_point_internal::BoundRatio<V>;

template <typename RepA, typename RepB>
struct PromoteRep {
  static_assert(fixed_point_internal::DependentlyFalse<RepA>,
                "PromoteRep: unsupported FixedPoint Rep combination");
};

template <typename Rep>
struct PromoteRep<Rep, Rep> {
  using type = Rep;
};

template <>
struct PromoteRep<std::uint8_t, std::uint8_t> {
  using type = std::uint8_t;
};
template <>
struct PromoteRep<std::uint8_t, std::uint16_t> {
  using type = std::uint16_t;
};
template <>
struct PromoteRep<std::uint16_t, std::uint8_t> {
  using type = std::uint16_t;
};
template <>
struct PromoteRep<std::uint16_t, std::uint16_t> {
  using type = std::uint16_t;
};
template <>
struct PromoteRep<std::uint16_t, std::uint32_t> {
  using type = std::uint32_t;
};
template <>
struct PromoteRep<std::uint32_t, std::uint16_t> {
  using type = std::uint32_t;
};
template <>
struct PromoteRep<std::uint32_t, std::uint32_t> {
  using type = std::uint32_t;
};
template <>
struct PromoteRep<std::int8_t, std::int8_t> {
  using type = std::int8_t;
};
template <>
struct PromoteRep<std::int8_t, std::int16_t> {
  using type = std::int16_t;
};
template <>
struct PromoteRep<std::int16_t, std::int8_t> {
  using type = std::int16_t;
};
template <>
struct PromoteRep<std::int16_t, std::int16_t> {
  using type = std::int16_t;
};
template <>
struct PromoteRep<std::int16_t, std::int32_t> {
  using type = std::int32_t;
};
template <>
struct PromoteRep<std::int32_t, std::int16_t> {
  using type = std::int32_t;
};
template <>
struct PromoteRep<std::int32_t, std::int32_t> {
  using type = std::int32_t;
};
template <>
struct PromoteRep<std::uint8_t, std::int8_t> {
  using type = std::int16_t;
};
template <>
struct PromoteRep<std::int8_t, std::uint8_t> {
  using type = std::int16_t;
};
template <>
struct PromoteRep<std::uint16_t, std::int16_t> {
  using type = std::int32_t;
};
template <>
struct PromoteRep<std::int16_t, std::uint16_t> {
  using type = std::int32_t;
};
template <>
struct PromoteRep<std::uint32_t, std::int32_t> {
  using type = std::int64_t;
};
template <>
struct PromoteRep<std::int32_t, std::uint32_t> {
  using type = std::int64_t;
};

template <typename Rep, auto Max>
  requires IntegralStorage<Rep>
class FixedPoint {
 public:
  using rep_type = Rep;
  using rep_value_type = typename numeric_traits<Rep>::rep_value_type;

  static constexpr auto kDeclaredMax = Max;
  static constexpr auto kRequiredMax = Max;
  static constexpr bool kIsSigned = numeric_traits<Rep>::kIsSigned;
  static constexpr rep_value_type kStorageRawMax = numeric_traits<Rep>::kRawMax;
  static constexpr rep_value_type kRawMax = kStorageRawMax;
  static constexpr rep_value_type kRawMin = [] {
    if constexpr (kIsSigned) {
      return static_cast<rep_value_type>(
          -static_cast<std::int64_t>(kStorageRawMax));
    } else {
      return rep_value_type{0};
    }
  }();

  static_assert(BoundRatio<Max>::kIsPositive,
                "FixedPoint Max must be positive");

  static constexpr std::int64_t kMaxNum = BoundRatio<Max>::num;
  static constexpr std::int64_t kMaxDen = BoundRatio<Max>::den;

  static constexpr int kScaleExp = fixed_point_internal::ComputeScaleExp(
      static_cast<std::int64_t>(kStorageRawMax), kMaxNum, kMaxDen);

  static constexpr int kFractionBits = kScaleExp < 0 ? -kScaleExp : 0;
  static constexpr int kLeftShift = kScaleExp > 0 ? kScaleExp : 0;

  static constexpr fixed_point_internal::Rational kRepresentableMaxRational =
      fixed_point_internal::ScaleRationalByPow2(
          {static_cast<std::int64_t>(kStorageRawMax), 1}, kScaleExp);

  static constexpr rep_value_type ClampRaw(rep_value_type raw) {
    return fixed_point_internal::ClampRaw(raw, kRawMin, kRawMax);
  }

  static constexpr rep_value_type AlignRawFromScale(rep_value_type raw,
                                                    int source_scale_exp) {
    return fixed_point_internal::ConvertRawScale(raw, source_scale_exp,
                                                 kScaleExp, kRawMin, kRawMax);
  }

  template <typename Other>
  static constexpr rep_value_type AlignRawFrom(const Other& other) {
    return AlignRawFromScale(other.RawValue(), Other::kScaleExp);
  }

  static constexpr FixedPoint FromRaw(Rep raw) {
    return FixedPoint(fixed_point_internal::raw_storage_t{}, raw);
  }

  static constexpr FixedPoint Raw(Rep raw) {
    return FromRaw(raw);
  }

  static constexpr FixedPoint FromRatio(std::int64_t num, std::int64_t den) {
    return FixedPoint(fixed_point_internal::logical_storage_t{}, num, den);
  }

  static constexpr FixedPoint FromInteger(std::int64_t value) {
    return FixedPoint(fixed_point_internal::logical_storage_t{}, value, 1);
  }

  // Explicit runtime conversion: clamps to the declared logical range.
  static constexpr FixedPoint FromRuntimeInteger(std::int64_t value) {
    return FixedPoint(fixed_point_internal::logical_storage_t{}, value, 1);
  }

  // Explicit saturating runtime conversion: clamps to the declared range.
  static constexpr FixedPoint Saturating(std::int64_t value) {
    return FixedPoint(fixed_point_internal::logical_storage_t{}, value, 1);
  }

  // Checked runtime conversion: nullopt when value exceeds the declared range.
  static constexpr std::optional<FixedPoint> TryFromRuntimeInteger(
      std::int64_t value) {
    if (!fixed_point_internal::LogicalWithinDeclaredMax<Max, kIsSigned>(value,
                                                                        1)) {
      return std::nullopt;
    }
    return FixedPoint(fixed_point_internal::logical_storage_t{}, value, 1);
  }

  static consteval FixedPoint FromDouble(double value) {
    const auto rational = fixed_point_internal::DoubleToRational(value);
    return FixedPoint(fixed_point_internal::logical_storage_t{}, rational.num,
                      rational.den);
  }

  // Ordinary numeric construction is consteval: constants are bound-checked at
  // compile time and runtime values are rejected, so there is no silent clamp.
  template <std::integral T>
  consteval FixedPoint(T logical)
      : FixedPoint(fixed_point_internal::logical_storage_t{},
                   static_cast<std::int64_t>(logical), 1) {
    if (!fixed_point_internal::LogicalWithinDeclaredMax<Max, kIsSigned>(
            static_cast<std::int64_t>(logical), 1)) {
      fixed_point_internal::FixedPointConstantExceedsDeclaredMax();
    }
  }

  static consteval FixedPoint MakeFromDouble(double logical) {
    const auto rational = fixed_point_internal::DoubleToRational(logical);
    if (!fixed_point_internal::LogicalWithinDeclaredMax<Max, kIsSigned>(
            rational.num, rational.den)) {
      fixed_point_internal::FixedPointConstantExceedsDeclaredMax();
    }
    return FixedPoint(fixed_point_internal::logical_storage_t{}, rational.num,
                      rational.den);
  }

  consteval FixedPoint(double logical) : FixedPoint(MakeFromDouble(logical)) {}

  consteval FixedPoint(float logical)
      : FixedPoint(static_cast<double>(logical)) {}

  template <typename To>
    requires IntegralStorage<typename To::rep_type>
  static constexpr To Cast(const FixedPoint& value) {
    return To::FromRaw(
        fixed_point_internal::RepFromRawValue<typename To::rep_type>(
            To::AlignRawFromScale(value.RawValue(), kScaleExp)));
  }

  constexpr Rep Raw() const {
    return raw_;
  }

  constexpr rep_value_type RawValue() const {
    return fixed_point_internal::RepRawValue(raw_);
  }

  constexpr bool operator==(const FixedPoint& other) const {
    return RawValue() == other.RawValue();
  }

  constexpr bool operator!=(const FixedPoint& other) const {
    return !(*this == other);
  }

  constexpr bool operator<(const FixedPoint& other) const {
    return RawValue() < other.RawValue();
  }

  constexpr bool operator>(const FixedPoint& other) const {
    return other < *this;
  }

  constexpr bool operator<=(const FixedPoint& other) const {
    return !(other < *this);
  }

  constexpr bool operator>=(const FixedPoint& other) const {
    return !(*this < other);
  }

 private:
  constexpr FixedPoint(fixed_point_internal::raw_storage_t, Rep raw)
      : raw_(fixed_point_internal::RepFromRawValue<Rep>(
            ClampRaw(fixed_point_internal::RepRawValue(raw)))) {}

  constexpr FixedPoint(fixed_point_internal::logical_storage_t,
                       std::int64_t num, std::int64_t den)
      : raw_(fixed_point_internal::RepFromRawValue<Rep>(
            fixed_point_internal::MakeRawFromLogical<Rep, Max, kIsSigned>(
                num, den))) {}

  Rep raw_{};
};

namespace fixed_point_internal {

template <typename ResultRep, auto ResultMax, typename L, typename R>
constexpr FixedPoint<ResultRep, ResultMax> AddFixedPoint(L lhs, R rhs) {
  using Result = FixedPoint<ResultRep, ResultMax>;
  const auto lhs_raw = Result::AlignRawFromScale(lhs.RawValue(), L::kScaleExp);
  const auto rhs_raw = Result::AlignRawFromScale(rhs.RawValue(), R::kScaleExp);
  const std::int64_t sum =
      static_cast<std::int64_t>(lhs_raw) + static_cast<std::int64_t>(rhs_raw);
  return Result::FromRaw(RepFromRawValue<ResultRep>(
      Result::ClampRaw(static_cast<typename Result::rep_value_type>(sum))));
}

template <typename ResultRep, auto ResultMax, typename L, typename R>
constexpr FixedPoint<ResultRep, ResultMax> SubFixedPoint(L lhs, R rhs) {
  using Result = FixedPoint<ResultRep, ResultMax>;
  const auto lhs_raw = Result::AlignRawFromScale(lhs.RawValue(), L::kScaleExp);
  const auto rhs_raw = Result::AlignRawFromScale(rhs.RawValue(), R::kScaleExp);
  const std::int64_t diff =
      static_cast<std::int64_t>(lhs_raw) - static_cast<std::int64_t>(rhs_raw);
  return Result::FromRaw(RepFromRawValue<ResultRep>(
      Result::ClampRaw(static_cast<typename Result::rep_value_type>(diff))));
}

template <typename ResultRep, auto ResultMax, typename L, typename R>
constexpr FixedPoint<ResultRep, ResultMax> MulFixedPoint(L lhs, R rhs) {
  using Result = FixedPoint<ResultRep, ResultMax>;
  const int scale_adjust = L::kScaleExp + R::kScaleExp - Result::kScaleExp;

  std::int64_t num = static_cast<std::int64_t>(lhs.RawValue()) *
                     static_cast<std::int64_t>(rhs.RawValue());

  if constexpr (scale_adjust > 0) {
    for (int i = 0; i < scale_adjust; ++i) {
      num *= 2;
    }
  } else if constexpr (scale_adjust < 0) {
    for (int i = 0; i < -scale_adjust; ++i) {
      num = RoundDivNearest(num, std::int64_t{2});
    }
  }

  return Result::FromRaw(RepFromRawValue<ResultRep>(
      Result::ClampRaw(static_cast<typename Result::rep_value_type>(num))));
}

}  // namespace fixed_point_internal

template <typename RepA, auto MaxA, typename RepB, auto MaxB>
constexpr FixedPoint<typename PromoteRep<RepA, RepB>::type, MaxA + MaxB>
operator+(FixedPoint<RepA, MaxA> lhs, FixedPoint<RepB, MaxB> rhs) {
  using ResultRep = typename PromoteRep<RepA, RepB>::type;
  return fixed_point_internal::AddFixedPoint<ResultRep, MaxA + MaxB>(lhs, rhs);
}

template <typename RepA, auto MaxA, typename RepB, auto MaxB>
constexpr FixedPoint<typename PromoteRep<RepA, RepB>::type, MaxA + MaxB>
operator-(FixedPoint<RepA, MaxA> lhs, FixedPoint<RepB, MaxB> rhs) {
  using ResultRep = typename PromoteRep<RepA, RepB>::type;
  return fixed_point_internal::SubFixedPoint<ResultRep, MaxA + MaxB>(lhs, rhs);
}

template <typename RepA, auto MaxA, typename RepB, auto MaxB>
constexpr FixedPoint<typename PromoteRep<RepA, RepB>::type, MaxA * MaxB>
operator*(FixedPoint<RepA, MaxA> lhs, FixedPoint<RepB, MaxB> rhs) {
  using ResultRep = typename PromoteRep<RepA, RepB>::type;
  return fixed_point_internal::MulFixedPoint<ResultRep, MaxA * MaxB>(lhs, rhs);
}

template <typename Rep, auto Max, std::integral T>
consteval FixedPoint<Rep, Max + Max> operator+(FixedPoint<Rep, Max> lhs,
                                               T rhs) {
  return lhs + FixedPoint<Rep, Max>{rhs};
}

template <typename Rep, auto Max, std::integral T>
consteval FixedPoint<Rep, Max + Max> operator+(T lhs,
                                               FixedPoint<Rep, Max> rhs) {
  return FixedPoint<Rep, Max>{lhs} + rhs;
}

template <typename Target, typename L, typename R>
constexpr Target AddTo(L lhs, R rhs) {
  using Sum = decltype(lhs + rhs);
  const Sum sum = lhs + rhs;
  return Sum::template Cast<Target>(sum);
}

template <typename Target, typename L, typename R>
constexpr Target SubTo(L lhs, R rhs) {
  using Diff = decltype(lhs - rhs);
  const Diff diff = lhs - rhs;
  return Diff::template Cast<Target>(diff);
}

template <typename Target, typename L, typename R>
constexpr Target MulTo(L lhs, R rhs) {
  using Prod = decltype(lhs * rhs);
  const Prod prod = lhs * rhs;
  return Prod::template Cast<Target>(prod);
}

template <typename Target, typename L, typename R>
constexpr Target DivTo(L lhs, R rhs) {
  const auto lhs_raw = lhs.RawValue();
  const auto rhs_raw = rhs.RawValue();

  if (rhs_raw == typename Target::rep_value_type {0}) {
    if constexpr (Target::kIsSigned) {
      return lhs_raw >= typename Target::rep_value_type {0}
                 ? Target::FromRaw(Target::kRawMax)
                 : Target::FromRaw(Target::kRawMin);
    }
    return Target::FromRaw(Target::kRawMax);
  }

  const int scale_adjust = L::kScaleExp - R::kScaleExp - Target::kScaleExp;

  std::int64_t num = static_cast<std::int64_t>(lhs_raw);
  std::int64_t den = static_cast<std::int64_t>(rhs_raw);

  if constexpr (scale_adjust > 0) {
    for (int i = 0; i < scale_adjust; ++i) {
      num *= 2;
    }
  } else if constexpr (scale_adjust < 0) {
    for (int i = 0; i < -scale_adjust; ++i) {
      den *= 2;
    }
  }

  const auto quotient = fixed_point_internal::RoundDivNearest(num, den);
  return Target::FromRaw(
      fixed_point_internal::RepFromRawValue<typename Target::rep_type>(
          Target::ClampRaw(
              static_cast<typename Target::rep_value_type>(quotient))));
}

template <typename Rep, auto Max>
struct runtime_numeric_traits<FixedPoint<Rep, Max>> {
  using value_type = FixedPoint<Rep, Max>;

  static constexpr bool kIsSupported = true;
  static constexpr bool kIsSigned = FixedPoint<Rep, Max>::kIsSigned;

  static constexpr value_type FromInteger(std::int64_t value) {
    return value_type::FromInteger(value);
  }

  static consteval value_type FromDouble(double value) {
    return value_type::FromDouble(value);
  }
};

template <typename Rep, auto Max>
struct numeric_traits<FixedPoint<Rep, Max>> {
  using rep_type = Rep;
  using rep_value_type = typename FixedPoint<Rep, Max>::rep_value_type;
  using value_type = FixedPoint<Rep, Max>;

  static constexpr bool kIsIntegerLike = false;
  static constexpr bool kIsFixedPoint = true;
  static constexpr bool kIsExponential = false;
  static constexpr bool kIsSigned = FixedPoint<Rep, Max>::kIsSigned;
};

}  // namespace ae

#endif  // NUMERIC_FIXED_POINT_H_
