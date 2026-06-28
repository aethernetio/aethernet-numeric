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

#include "numeric/numeric_traits.h"

namespace ae {
namespace fixed_point_internal {

struct Rational {
  std::int64_t num = 0;
  std::int64_t den = 1;
};

constexpr std::int64_t Gcd(std::int64_t a, std::int64_t b) {
  a = a < 0 ? -a : a;
  b = b < 0 ? -b : b;
  while (b != 0) {
    const auto t = a % b;
    a = b;
    b = t;
  }
  return a;
}

constexpr Rational Normalize(Rational r) {
  if (r.den < 0) {
    r.num = -r.num;
    r.den = -r.den;
  }
  if (r.num == 0) {
    return {0, 1};
  }
  const auto g = Gcd(r.num, r.den);
  return {r.num / g, r.den / g};
}

constexpr Rational MakeRational(std::int64_t num, std::int64_t den) {
  return Normalize({num, den});
}

consteval Rational DoubleToRational(double value) {
  if (value == 0.0) {
    return {0, 1};
  }

  bool negative = value < 0.0;
  double abs_value = negative ? -value : value;

  for (unsigned scale = 0; scale <= 18; ++scale) {
    std::uint64_t pow10 = 1;
    for (unsigned i = 0; i < scale; ++i) {
      pow10 *= 10;
    }

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

template <auto Max>
consteval Rational MaxAsRational() {
  using MaxType = decltype(Max);
  if constexpr (std::is_integral_v<MaxType>) {
    return {static_cast<std::int64_t>(Max), 1};
  } else {
    return DoubleToRational(Max);
  }
}

template <auto Max>
consteval bool MaxIsPositive() {
  using MaxType = decltype(Max);
  if constexpr (std::is_integral_v<MaxType>) {
    return Max > MaxType{0};
  } else {
    return Max > MaxType{0};
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
  } else {
    return Rep{raw};
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
  return SaturateRaw(static_cast<RepValue>(-raw), raw_min, raw_max);
}

}  // namespace fixed_point_internal

template <typename Rep, auto Max>
  requires IntegralStorage<Rep>
class FixedPoint {
 public:
  using rep_type = Rep;
  using rep_traits = numeric_traits<Rep>;
  using rep_value_type = typename rep_traits::rep_value_type;

  static constexpr bool kIsSigned = rep_traits::kIsSigned;
  static constexpr rep_value_type kRawMax = rep_traits::kRawMax;
  static constexpr rep_value_type kRawMin = rep_traits::kRawMin;

  static_assert(fixed_point_internal::MaxIsPositive<Max>(),
                "FixedPoint Max must be positive");

 private:
  static constexpr fixed_point_internal::Rational kMaxRational =
      fixed_point_internal::MaxAsRational<Max>();
  static constexpr std::int64_t kMaxNum = kMaxRational.num;
  static constexpr std::int64_t kMaxDen = kMaxRational.den;

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

}  // namespace ae

#endif  // NUMERIC_FIXED_POINT_H_
