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

#ifdef __SIZEOF_INT128__
using Wide = __int128_t;
#else
using Wide = std::int64_t;
#endif

constexpr Wide ToWide(std::int64_t value) { return static_cast<Wide>(value); }

constexpr Wide RoundDiv(Wide num, Wide den) {
  if (den == 0) {
    return 0;
  }
  if (den < 0) {
    num = -num;
    den = -den;
  }
  if (num >= 0) {
    return (num + den / 2) / den;
  }
  return (num - den / 2) / den;
}

template <typename RepValue>
constexpr RepValue SaturateRaw(std::int64_t raw, RepValue raw_min,
                               RepValue raw_max) {
  if (raw < static_cast<std::int64_t>(raw_min)) {
    return raw_min;
  }
  if (raw > static_cast<std::int64_t>(raw_max)) {
    return raw_max;
  }
  return static_cast<RepValue>(raw);
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

template <typename RepValue, bool kIsSigned>
constexpr RepValue SaturatedAdd(RepValue lhs, RepValue rhs, RepValue raw_min,
                                RepValue raw_max) {
#ifdef __SIZEOF_INT128__
  const Wide sum = ToWide(static_cast<std::int64_t>(lhs)) +
                   ToWide(static_cast<std::int64_t>(rhs));
  if constexpr (kIsSigned) {
    if (sum < ToWide(static_cast<std::int64_t>(raw_min))) {
      return raw_min;
    }
    if (sum > ToWide(static_cast<std::int64_t>(raw_max))) {
      return raw_max;
    }
    return static_cast<RepValue>(sum);
  } else {
    if (sum < 0) {
      return raw_min;
    }
    if (sum > ToWide(static_cast<std::int64_t>(raw_max))) {
      return raw_max;
    }
    return static_cast<RepValue>(sum);
  }
#else
  if constexpr (kIsSigned) {
    const Wide sum = ToWide(lhs) + ToWide(rhs);
    return SaturateRaw(static_cast<RepValue>(sum), raw_min, raw_max);
  } else {
    using UnsignedWide = std::make_unsigned_t<RepValue>;
    const auto sum = static_cast<UnsignedWide>(lhs) +
                     static_cast<UnsignedWide>(rhs);
    if (sum > static_cast<UnsignedWide>(raw_max)) {
      return raw_max;
    }
    return static_cast<RepValue>(sum);
  }
#endif
}

template <typename RepValue, bool kIsSigned>
constexpr RepValue SaturatedSub(RepValue lhs, RepValue rhs, RepValue raw_min,
                                RepValue raw_max) {
#ifdef __SIZEOF_INT128__
  const Wide diff = ToWide(static_cast<std::int64_t>(lhs)) -
                    ToWide(static_cast<std::int64_t>(rhs));
  if constexpr (kIsSigned) {
    if (diff < ToWide(static_cast<std::int64_t>(raw_min))) {
      return raw_min;
    }
    if (diff > ToWide(static_cast<std::int64_t>(raw_max))) {
      return raw_max;
    }
    return static_cast<RepValue>(diff);
  } else {
    if (diff < 0) {
      return raw_min;
    }
    if (diff > ToWide(static_cast<std::int64_t>(raw_max))) {
      return raw_max;
    }
    return static_cast<RepValue>(diff);
  }
#else
  if constexpr (kIsSigned) {
    const Wide diff = ToWide(lhs) - ToWide(rhs);
    return SaturateRaw(static_cast<RepValue>(diff), raw_min, raw_max);
  } else {
    if (rhs > lhs) {
      return raw_min;
    }
    return static_cast<RepValue>(lhs - rhs);
  }
#endif
}

template <typename RepValue, bool kIsSigned>
constexpr RepValue SaturatedNegate(RepValue raw, RepValue raw_min,
                                   RepValue raw_max) {
  if constexpr (!kIsSigned) {
    return raw_min;
  }
  if (raw == raw_min) {
    return raw_max;
  }
  return SaturateRaw(static_cast<RepValue>(-static_cast<std::int64_t>(raw)),
                     raw_min, raw_max);
}

template <typename RepValue>
constexpr RepValue SaturatedAbs(RepValue raw, RepValue raw_min,
                                RepValue raw_max) {
  if (raw >= RepValue{0}) {
    return raw;
  }
  if constexpr (std::is_signed_v<RepValue>) {
    if (raw == raw_min) {
      return raw_max;
    }
    return SaturateRaw(static_cast<RepValue>(-static_cast<std::int64_t>(raw)),
                       raw_min, raw_max);
  }
  return raw;
}

constexpr std::int64_t CastRawValue(
    std::int64_t source_raw, std::int64_t source_raw_max,
    std::int64_t source_max_num, std::int64_t source_max_den,
    std::int64_t target_max_num, std::int64_t target_max_den,
    std::int64_t target_raw_min, std::int64_t target_raw_max) {
  const Wide numerator = ToWide(source_raw) * ToWide(source_max_num) *
                         ToWide(target_max_den) * ToWide(target_raw_max);
  const Wide denominator = ToWide(source_raw_max) * ToWide(source_max_den) *
                           ToWide(target_max_num);
  const auto rounded =
      static_cast<std::int64_t>(RoundDiv(numerator, denominator));
  return SaturateRaw<std::int64_t>(rounded, target_raw_min, target_raw_max);
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

  static constexpr auto kMax = Max;
  static constexpr bool kIsSigned = numeric_traits<Rep>::kIsSigned;
  static constexpr rep_value_type kRawMax = numeric_traits<Rep>::kRawMax;
  static constexpr rep_value_type kRawMin = numeric_traits<Rep>::kRawMin;

  static_assert(BoundRatio<Max>::kIsPositive,
                "FixedPoint Max must be positive");

 private:
  static constexpr std::int64_t kMaxNum = BoundRatio<Max>::num;
  static constexpr std::int64_t kMaxDen = BoundRatio<Max>::den;

  static constexpr rep_value_type RawFromRatio(std::int64_t num,
                                               std::int64_t den) {
    if (den == 0) {
      return kRawMin;
    }

    const fixed_point_internal::Wide numerator =
        fixed_point_internal::ToWide(num) *
        fixed_point_internal::ToWide(kMaxDen) *
        fixed_point_internal::ToWide(static_cast<std::int64_t>(kRawMax));
    const fixed_point_internal::Wide denominator =
        fixed_point_internal::ToWide(den) *
        fixed_point_internal::ToWide(kMaxNum);

    const auto rounded = static_cast<std::int64_t>(
        fixed_point_internal::RoundDiv(numerator, denominator));

    return fixed_point_internal::SaturateRaw<rep_value_type>(rounded, kRawMin,
                                                               kRawMax);
  }

  static constexpr rep_value_type MulRaw(rep_value_type a_raw,
                                         rep_value_type b_raw) {
    const fixed_point_internal::Wide numerator =
        fixed_point_internal::ToWide(static_cast<std::int64_t>(a_raw)) *
        fixed_point_internal::ToWide(static_cast<std::int64_t>(b_raw)) *
        fixed_point_internal::ToWide(kMaxNum);
    const fixed_point_internal::Wide denominator =
        fixed_point_internal::ToWide(static_cast<std::int64_t>(kRawMax)) *
        fixed_point_internal::ToWide(kMaxDen);
    const auto rounded = static_cast<std::int64_t>(
        fixed_point_internal::RoundDiv(numerator, denominator));
    return fixed_point_internal::SaturateRaw<rep_value_type>(rounded, kRawMin,
                                                               kRawMax);
  }

  static constexpr rep_value_type DivRaw(rep_value_type a_raw,
                                         rep_value_type b_raw) {
    if (b_raw == rep_value_type{0}) {
      if constexpr (kIsSigned) {
        return a_raw >= rep_value_type{0} ? kRawMax : kRawMin;
      }
      return kRawMax;
    }
    const fixed_point_internal::Wide numerator =
        fixed_point_internal::ToWide(static_cast<std::int64_t>(a_raw)) *
        fixed_point_internal::ToWide(static_cast<std::int64_t>(kRawMax)) *
        fixed_point_internal::ToWide(kMaxDen);
    const fixed_point_internal::Wide denominator =
        fixed_point_internal::ToWide(static_cast<std::int64_t>(b_raw)) *
        fixed_point_internal::ToWide(kMaxNum);
    const auto rounded = static_cast<std::int64_t>(
        fixed_point_internal::RoundDiv(numerator, denominator));
    return fixed_point_internal::SaturateRaw<rep_value_type>(rounded, kRawMin,
                                                               kRawMax);
  }

  template <typename ToRep, auto ToMax>
    requires IntegralStorage<ToRep>
  static constexpr FixedPoint<ToRep, ToMax> CastTo(const FixedPoint& value) {
    using To = FixedPoint<ToRep, ToMax>;
    static_assert(BoundRatio<ToMax>::kIsPositive,
                  "FixedPoint Max must be positive");
    constexpr std::int64_t target_max_num = BoundRatio<ToMax>::num;
    constexpr std::int64_t target_max_den = BoundRatio<ToMax>::den;
    const auto target_raw = fixed_point_internal::CastRawValue(
        static_cast<std::int64_t>(value.raw_value()),
        static_cast<std::int64_t>(kRawMax), kMaxNum, kMaxDen, target_max_num,
        target_max_den, static_cast<std::int64_t>(To::kRawMin),
        static_cast<std::int64_t>(To::kRawMax));
    return To::FromRaw(fixed_point_internal::RepFromRawValue<ToRep>(
        static_cast<typename To::rep_value_type>(target_raw)));
  }

 public:
  constexpr FixedPoint() = default;

  static constexpr FixedPoint FromRaw(Rep raw) { return FixedPoint(raw); }

  static constexpr FixedPoint FromRatio(std::int64_t num, std::int64_t den) {
    return FromRaw(
        fixed_point_internal::RepFromRawValue<Rep>(RawFromRatio(num, den)));
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
    return CastTo<typename To::rep_type, To::kMax>(value);
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

  constexpr bool operator<(const FixedPoint& other) const {
    return raw_value() < other.raw_value();
  }

  constexpr bool operator<=(const FixedPoint& other) const {
    return raw_value() <= other.raw_value();
  }

  constexpr bool operator>(const FixedPoint& other) const {
    return raw_value() > other.raw_value();
  }

  constexpr bool operator>=(const FixedPoint& other) const {
    return raw_value() >= other.raw_value();
  }

  constexpr FixedPoint operator+(const FixedPoint& other) const {
    return FromRaw(fixed_point_internal::RepFromRawValue<Rep>(
        fixed_point_internal::SaturatedAdd<rep_value_type, kIsSigned>(
            raw_value(), other.raw_value(), kRawMin, kRawMax)));
  }

  constexpr FixedPoint operator-(const FixedPoint& other) const {
    return FromRaw(fixed_point_internal::RepFromRawValue<Rep>(
        fixed_point_internal::SaturatedSub<rep_value_type, kIsSigned>(
            raw_value(), other.raw_value(), kRawMin, kRawMax)));
  }

  constexpr FixedPoint operator-() const
      requires kIsSigned
  {
    return FromRaw(fixed_point_internal::RepFromRawValue<Rep>(
        fixed_point_internal::SaturatedNegate<rep_value_type, kIsSigned>(
            raw_value(), kRawMin, kRawMax)));
  }

  friend constexpr FixedPoint operator*(FixedPoint lhs, FixedPoint rhs) {
    return lhs.FromRaw(fixed_point_internal::RepFromRawValue<Rep>(
        MulRaw(lhs.raw_value(), rhs.raw_value())));
  }

  friend constexpr FixedPoint operator/(FixedPoint lhs, FixedPoint rhs) {
    return lhs.FromRaw(fixed_point_internal::RepFromRawValue<Rep>(
        DivRaw(lhs.raw_value(), rhs.raw_value())));
  }

  constexpr FixedPoint& operator+=(FixedPoint rhs) {
    *this = *this + rhs;
    return *this;
  }

  constexpr FixedPoint& operator-=(FixedPoint rhs) {
    *this = *this - rhs;
    return *this;
  }

  constexpr FixedPoint& operator*=(FixedPoint rhs) {
    *this = *this * rhs;
    return *this;
  }

  constexpr FixedPoint& operator/=(FixedPoint rhs) {
    *this = *this / rhs;
    return *this;
  }

  constexpr FixedPoint abs() const {
    return FromRaw(fixed_point_internal::RepFromRawValue<Rep>(
        fixed_point_internal::SaturatedAbs(raw_value(), kRawMin, kRawMax)));
  }

 private:
  constexpr explicit FixedPoint(Rep raw) : raw_(raw) {}

  Rep raw_{};
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

template <typename Rep, auto Max>
constexpr FixedPoint<Rep, Max> min(FixedPoint<Rep, Max> a,
                                   FixedPoint<Rep, Max> b) {
  return a < b ? a : b;
}

template <typename Rep, auto Max>
constexpr FixedPoint<Rep, Max> max(FixedPoint<Rep, Max> a,
                                   FixedPoint<Rep, Max> b) {
  return a > b ? a : b;
}

template <typename Rep, auto Max>
constexpr FixedPoint<Rep, Max> clamp(FixedPoint<Rep, Max> value,
                                     FixedPoint<Rep, Max> low,
                                     FixedPoint<Rep, Max> high) {
  return min(max(value, low), high);
}

}  // namespace ae

#endif  // NUMERIC_FIXED_POINT_H_
