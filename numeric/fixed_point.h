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
#include <type_traits>
#include <utility>

#include "numeric/decimal.h"
#include "numeric/numeric_traits.h"

namespace ae {
namespace fixed_point_internal {

struct Rational {
  std::int64_t num = 0;
  std::int64_t den = 1;
};

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
constexpr RepValue ClampRaw(RepValue raw, RepValue raw_min,
                            RepValue raw_max) {
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
constexpr RepValue SaturatedAddRaw(RepValue lhs, RepValue rhs,
                                   RepValue raw_min, RepValue raw_max) {
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
constexpr RepValue SaturatedSubRaw(RepValue lhs, RepValue rhs,
                                   RepValue raw_min, RepValue raw_max) {
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
  const RepValue divisor = static_cast<RepValue>(RepValue{1} << bits);
  const RepValue bias = static_cast<RepValue>(divisor / RepValue{2});
  if constexpr (std::is_signed_v<RepValue>) {
    if (raw >= 0) {
      return static_cast<RepValue>((raw + bias) / divisor);
    }
    return static_cast<RepValue>((raw - bias) / divisor);
  } else {
    return static_cast<RepValue>((raw + bias) / divisor);
  }
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

  const auto rounded =
      RoundDivNearest(static_cast<std::int64_t>(num),
                      static_cast<std::int64_t>(den));
  if (negative) {
    return ClampRaw(static_cast<RepValue>(-rounded), raw_min, raw_max);
  }
  return ClampRaw(static_cast<RepValue>(rounded), raw_min, raw_max);
}

}  // namespace fixed_point_internal

template <auto V>
struct BoundRatio {
  static constexpr auto kValue = V;
  static constexpr fixed_point_internal::Rational kRational =
      fixed_point_internal::BoundValueAsRational<V>();
  static constexpr std::int64_t num = kRational.num;
  static constexpr std::int64_t den = kRational.den;
  static constexpr bool kIsPositive = (num > 0) && (den > 0);
};

template <typename Rep, auto Max>
  requires IntegralStorage<Rep>
class FixedPoint {
 public:
  using rep_type = Rep;
  using rep_value_type = typename numeric_traits<Rep>::rep_value_type;

  static constexpr auto kRequiredMax = Max;
  static constexpr bool kIsSigned = numeric_traits<Rep>::kIsSigned;
  static constexpr rep_value_type kStorageRawMax = numeric_traits<Rep>::kRawMax;
  static constexpr rep_value_type kRawMax = kStorageRawMax;
  static constexpr rep_value_type kRawMin =
      kIsSigned ? static_cast<rep_value_type>(-kStorageRawMax)
                : rep_value_type{0};

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
    return AlignRawFromScale(other.raw_value(), Other::kScaleExp);
  }

  static constexpr FixedPoint FromAlignedRaw(rep_value_type raw) {
    return FromRaw(fixed_point_internal::RepFromRawValue<Rep>(
        ClampRaw(raw)));
  }

  static constexpr FixedPoint FromRaw(Rep raw) {
    return FixedPoint(raw);
  }

  static constexpr FixedPoint FromRatio(std::int64_t num, std::int64_t den) {
    return FromRaw(fixed_point_internal::RepFromRawValue<Rep>(
        fixed_point_internal::RawFromRatioAtScale(
            num, den, kScaleExp, kRawMin, kRawMax)));
  }

  static constexpr FixedPoint FromInteger(std::int64_t value) {
    return FromRatio(value, 1);
  }

  static consteval FixedPoint FromDouble(double value) {
    const auto rational = fixed_point_internal::DoubleToRational(value);
    return FromRatio(rational.num, rational.den);
  }

  template <typename To>
    requires IntegralStorage<typename To::rep_type>
  static constexpr To Cast(const FixedPoint& value) {
    return To::FromRaw(fixed_point_internal::RepFromRawValue<
                       typename To::rep_type>(To::AlignRawFromScale(
        value.raw_value(), kScaleExp)));
  }

  constexpr Rep raw() const { return raw_; }

  constexpr rep_value_type raw_value() const {
    return fixed_point_internal::RepRawValue(raw_);
  }

  constexpr bool operator==(const FixedPoint& other) const {
    return raw_value() == other.raw_value();
  }

  constexpr bool operator!=(const FixedPoint& other) const {
    return !(*this == other);
  }

  constexpr FixedPoint operator+(const FixedPoint& other) const {
    return FromRaw(fixed_point_internal::RepFromRawValue<Rep>(
        fixed_point_internal::SaturatedAddRaw(raw_value(), other.raw_value(),
                                              kRawMin, kRawMax)));
  }

  constexpr FixedPoint operator-(const FixedPoint& other) const {
    return FromRaw(fixed_point_internal::RepFromRawValue<Rep>(
        fixed_point_internal::SaturatedSubRaw(raw_value(), other.raw_value(),
                                              kRawMin, kRawMax)));
  }

  constexpr FixedPoint& operator+=(const FixedPoint& rhs) {
    *this = *this + rhs;
    return *this;
  }

  constexpr FixedPoint& operator-=(const FixedPoint& rhs) {
    *this = *this - rhs;
    return *this;
  }

 private:
  constexpr explicit FixedPoint(Rep raw)
      : raw_(fixed_point_internal::RepFromRawValue<Rep>(ClampRaw(
            fixed_point_internal::RepRawValue(raw)))) {}

  Rep raw_{};
};

template <typename Rep, auto MaxA, auto MaxB>
  requires(IntegralStorage<Rep>)
constexpr FixedPoint<Rep, MaxA + MaxB> operator+(FixedPoint<Rep, MaxA> lhs,
                                                 FixedPoint<Rep, MaxB> rhs) {
  using Result = FixedPoint<Rep, MaxA + MaxB>;
  const auto lhs_raw = Result::AlignRawFrom(lhs);
  const auto rhs_raw = Result::AlignRawFrom(rhs);
  return Result::FromAlignedRaw(fixed_point_internal::SaturatedAddRaw(
      lhs_raw, rhs_raw, Result::kRawMin, Result::kRawMax));
}

template <typename Target, typename L, typename R>
constexpr Target div_to(L lhs, R rhs) {
  const auto lhs_raw = lhs.raw_value();
  const auto rhs_raw = rhs.raw_value();

  if (rhs_raw == typename Target::rep_value_type{0}) {
    if constexpr (Target::kIsSigned) {
      return lhs_raw >= typename Target::rep_value_type{0}
                 ? Target::FromRaw(Target::kRawMax)
                 : Target::FromRaw(Target::kRawMin);
    }
    return Target::FromRaw(Target::kRawMax);
  }

  const int scale_adjust =
      L::kScaleExp - R::kScaleExp - Target::kScaleExp;

  std::int64_t num = static_cast<std::int64_t>(lhs_raw);
  std::int64_t den = static_cast<std::int64_t>(rhs_raw);

  if (scale_adjust > 0) {
    for (int i = 0; i < scale_adjust; ++i) {
      num *= 2;
    }
  } else if (scale_adjust < 0) {
    for (int i = 0; i < -scale_adjust; ++i) {
      den *= 2;
    }
  }

  const auto quotient = fixed_point_internal::RoundDivNearest(num, den);
  return Target::FromRaw(fixed_point_internal::RepFromRawValue<
                         typename Target::rep_type>(
      Target::ClampRaw(static_cast<typename Target::rep_value_type>(quotient))));
}

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
