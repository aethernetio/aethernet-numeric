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

#ifndef AE_NUMERIC_FIXED_POINT_H_
#define AE_NUMERIC_FIXED_POINT_H_

#include <concepts>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <type_traits>
#include <utility>

#include "ae-numeric/decimal.h"
#include "ae-numeric/integer_math.h"
#include "ae-numeric/numeric_traits.h"
#include "ae-numeric/runtime_numeric_traits.h"

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

template <typename RV>
inline constexpr bool kRepAtMost32 = sizeof(RV) <= 4;

template <typename ToR, typename FromR>
constexpr ToR ConvertRawScaleTo(FromR raw, int source_scale_exp,
                                int target_scale_exp, ToR raw_min,
                                ToR raw_max) {
  if constexpr (std::is_same_v<ToR, FromR>) {
    return ConvertRawScale(raw, source_scale_exp, target_scale_exp, raw_min,
                           raw_max);
  } else if constexpr (kRepAtMost32<ToR> && kRepAtMost32<FromR>) {
    if constexpr (std::is_signed_v<ToR>) {
      std::int32_t src = 0;
      if constexpr (std::is_signed_v<FromR>) {
        src = static_cast<std::int32_t>(raw);
      } else {
        auto const u = static_cast<std::uint32_t>(raw);
        constexpr auto kI32Max = static_cast<std::uint32_t>(
            std::numeric_limits<std::int32_t>::max());
        src = u > kI32Max ? std::numeric_limits<std::int32_t>::max()
                          : static_cast<std::int32_t>(u);
      }
      std::int32_t const aligned = ConvertRawScale(
          src, source_scale_exp, target_scale_exp,
          static_cast<std::int32_t>(raw_min),
          static_cast<std::int32_t>(raw_max));
      return static_cast<ToR>(aligned);
    } else {
      std::uint32_t src = 0;
      if constexpr (std::is_signed_v<FromR>) {
        src = raw < static_cast<FromR>(0) ? 0U
                                          : static_cast<std::uint32_t>(raw);
      } else {
        src = static_cast<std::uint32_t>(raw);
      }
      std::uint32_t const aligned = ConvertRawScale(
          src, source_scale_exp, target_scale_exp,
          static_cast<std::uint32_t>(raw_min),
          static_cast<std::uint32_t>(raw_max));
      if (aligned > static_cast<std::uint32_t>(raw_max)) {
        return raw_max;
      }
      return static_cast<ToR>(aligned);
    }
  } else {
    const std::int64_t aligned = ConvertRawScale(
        static_cast<std::int64_t>(raw), source_scale_exp, target_scale_exp,
        static_cast<std::int64_t>(raw_min), static_cast<std::int64_t>(raw_max));
    return static_cast<ToR>(aligned);
  }
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

// 32-bit-only logical→raw conversion for Rep <= 32 bits. Uses MulU32Wide /
// DivU32Wide / ShlU32WideChecked — no int64_t/uint64_t arithmetic types.
template <typename RepValue>
constexpr RepValue RawFromRatioAtScale32(std::int32_t num, std::int32_t den,
                                         int scale_exp, RepValue raw_min,
                                         RepValue raw_max) {
  if (den == 0) {
    return raw_min;
  }

  bool const negative = (num < 0) != (den < 0);
  std::uint32_t un = integer_math::AbsI32ToU32(num);
  std::uint32_t ud = integer_math::AbsI32ToU32(den);

  std::uint32_t hi = 0;
  std::uint32_t lo = un;
  if (scale_exp < 0) {
    if (!integer_math::ShlU32WideChecked(
            hi, lo, static_cast<unsigned>(-scale_exp))) {
      return negative ? raw_min : raw_max;
    }
  } else if (scale_exp > 0) {
    std::uint32_t dhi = 0;
    std::uint32_t dlo = ud;
    if (!integer_math::ShlU32WideChecked(dhi, dlo,
                                         static_cast<unsigned>(scale_exp))) {
      return RepValue{0};
    }
    if (dhi != 0U) {
      // Divisor >= 2^32 and dividend < 2^32 ⇒ quotient 0 (after rounding: 0).
      return RepValue{0};
    }
    ud = dlo;
    hi = 0;
    lo = un;
  }

  std::uint32_t q = 0;
  std::uint32_t r = 0;
  if (!integer_math::DivU32Wide(hi, lo, ud, q, r)) {
    return negative ? raw_min : raw_max;
  }
  if (r >= ud - r) {
    if (q == std::numeric_limits<std::uint32_t>::max()) {
      return negative ? raw_min : raw_max;
    }
    ++q;
  }

  if (!negative) {
    if (q > static_cast<std::uint32_t>(raw_max)) {
      return raw_max;
    }
    return static_cast<RepValue>(q);
  }
  if constexpr (std::is_signed_v<RepValue>) {
    auto const min_mag =
        integer_math::AbsI32ToU32(static_cast<std::int32_t>(raw_min));
    if (q > min_mag) {
      return raw_min;
    }
    return static_cast<RepValue>(-static_cast<std::int32_t>(q));
  } else {
    return raw_min;
  }
}

template <auto Max, bool kIsSigned>
constexpr bool LogicalWithinDeclaredMax32(std::int32_t num, std::int32_t den) {
  if (den <= 0) {
    return false;
  }
  constexpr std::int64_t max_num64 = BoundRatio<Max>::num;
  constexpr std::int64_t max_den64 = BoundRatio<Max>::den;
  static_assert(max_num64 > 0 && max_den64 > 0);
  static_assert(max_num64 <= std::numeric_limits<std::int32_t>::max());
  static_assert(max_den64 <= std::numeric_limits<std::int32_t>::max());
  constexpr auto max_num = static_cast<std::uint32_t>(max_num64);
  constexpr auto max_den = static_cast<std::uint32_t>(max_den64);
  if constexpr (!kIsSigned) {
    if (num < 0) {
      return false;
    }
  }
  return integer_math::CmpMulU32(integer_math::AbsI32ToU32(num), max_den,
                                 max_num, integer_math::AbsI32ToU32(den)) <= 0;
}

template <auto Max, bool kIsSigned>
constexpr std::pair<std::int32_t, std::int32_t> ClampLogicalRational32(
    std::int32_t num, std::int32_t den) {
  if (den <= 0) {
    den = 1;
  }
  if (LogicalWithinDeclaredMax32<Max, kIsSigned>(num, den)) {
    return {num, den};
  }
  constexpr auto max_num = static_cast<std::int32_t>(BoundRatio<Max>::num);
  constexpr auto max_den = static_cast<std::int32_t>(BoundRatio<Max>::den);
  if constexpr (kIsSigned) {
    if (num < 0) {
      return {-max_num, max_den};
    }
  }
  return {max_num, max_den};
}

template <typename Rep, auto Max, bool kIsSigned>
constexpr typename numeric_traits<Rep>::rep_value_type
MakeRawFromLogicalRuntime32(std::int32_t num, std::int32_t den) {
  using RepValue = typename numeric_traits<Rep>::rep_value_type;
  static_assert(sizeof(RepValue) <= 4,
                "MakeRawFromLogicalRuntime32 requires Rep <= 32 bits");
  constexpr RepValue kRawMax = numeric_traits<Rep>::kRawMax;
  constexpr RepValue kRawMin =
      kIsSigned ? static_cast<RepValue>(-static_cast<std::int32_t>(
                      static_cast<std::uint32_t>(kRawMax)))
                : RepValue{0};
  constexpr int kScaleExp =
      ComputeScaleExp(static_cast<std::int64_t>(kRawMax), BoundRatio<Max>::num,
                      BoundRatio<Max>::den);
  auto const clamped = ClampLogicalRational32<Max, kIsSigned>(num, den);
  return RawFromRatioAtScale32(clamped.first, clamped.second, kScaleExp,
                               kRawMin, kRawMax);
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

template <bool kSigned, int kDigits>
struct StorageForDigits;

template <int kDigits>
struct StorageForDigits<false, kDigits> {
  static_assert(kDigits <= 64,
                "PromoteRep: unsigned result requires more than 64 bits");

  using type = std::conditional_t<
      (kDigits <= 8), std::uint8_t,
      std::conditional_t<
          (kDigits <= 16), std::uint16_t,
          std::conditional_t<(kDigits <= 32), std::uint32_t, std::uint64_t>>>;
};

template <int kDigits>
struct StorageForDigits<true, kDigits> {
  static_assert(kDigits <= 63,
                "PromoteRep: signed result requires more than 64 bits");

  using type = std::conditional_t<
      (kDigits <= 7), std::int8_t,
      std::conditional_t<
          (kDigits <= 15), std::int16_t,
          std::conditional_t<(kDigits <= 31), std::int32_t, std::int64_t>>>;
};

template <typename RepA, typename RepB>
struct PromoteIntegralRep {
  static constexpr bool kIsSigned =
      std::is_signed_v<RepA> || std::is_signed_v<RepB>;
  static constexpr int kDigits =
      std::numeric_limits<RepA>::digits > std::numeric_limits<RepB>::digits
          ? std::numeric_limits<RepA>::digits
          : std::numeric_limits<RepB>::digits;

  using type = typename StorageForDigits<kIsSigned, kDigits>::type;
};

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

template <typename RepA, typename RepB>
  requires(std::is_integral_v<RepA> && std::is_integral_v<RepB> &&
           !std::is_same_v<RepA, bool> && !std::is_same_v<RepB, bool>)
struct PromoteRep<RepA, RepB> {
  using type =
      typename fixed_point_internal::PromoteIntegralRep<RepA, RepB>::type;
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
  static constexpr rep_value_type kRawMin = std::invoke([]() constexpr {
    if constexpr (kIsSigned) {
      return static_cast<rep_value_type>(
          -static_cast<std::int64_t>(kStorageRawMax));
    } else {
      return rep_value_type{0};
    }
  });

  static_assert(BoundRatio<Max>::kIsPositive,
                "FixedPoint Max must be positive");

  static constexpr std::int64_t kMaxNum = BoundRatio<Max>::num;
  static constexpr std::int64_t kMaxDen = BoundRatio<Max>::den;

  static constexpr int kScaleExp = fixed_point_internal::ComputeScaleExp(
      static_cast<std::int64_t>(kStorageRawMax), kMaxNum, kMaxDen);

  static constexpr int kFractionBits = kScaleExp < 0 ? -kScaleExp : 0;
  static constexpr int kLeftShift = kScaleExp > 0 ? kScaleExp : 0;

  constexpr FixedPoint() = default;

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

  // For Rep<=32 always use the 32-bit-only logical→raw path so runtime
  // encode/decode cannot pull in int64 division helpers (__divdi3 etc.).
  // Compile-time values for SegmentedNumber fit int32 (Rat / interval counts).
  // Wider-Rep FixedPoint (e.g. Instant) keeps the legacy int64 converter.
  static constexpr FixedPoint FromRatio(std::int64_t num, std::int64_t den) {
    if constexpr (sizeof(rep_value_type) <= 4) {
      return FixedPoint(
          fixed_point_internal::raw_storage_t{},
          fixed_point_internal::RepFromRawValue<Rep>(
              fixed_point_internal::MakeRawFromLogicalRuntime32<Rep, Max,
                                                               kIsSigned>(
                  static_cast<std::int32_t>(num),
                  static_cast<std::int32_t>(den))));
    } else {
      return FixedPoint(fixed_point_internal::logical_storage_t{}, num, den);
    }
  }

  static constexpr FixedPoint FromInteger(std::int64_t value) {
    return FromRatio(value, static_cast<std::int64_t>(1));
  }

  // Explicit runtime conversion: clamps to the declared logical range.
  static constexpr FixedPoint FromRuntimeInteger(std::int64_t value) {
    return FromRatio(value, static_cast<std::int64_t>(1));
  }

  // Explicit saturating runtime conversion: clamps to the declared range.
  static constexpr FixedPoint Saturating(std::int64_t value) {
    return FromRatio(value, static_cast<std::int64_t>(1));
  }

  // Checked runtime conversion: nullopt when value exceeds the declared range.
  static constexpr std::optional<FixedPoint> TryFromRuntimeInteger(
      std::int64_t value) {
    if (!fixed_point_internal::LogicalWithinDeclaredMax<Max, kIsSigned>(value,
                                                                        1)) {
      return std::nullopt;
    }
    return FromRatio(value, static_cast<std::int64_t>(1));
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
            fixed_point_internal::ConvertRawScaleTo<
                typename To::rep_value_type>(value.RawValue(), kScaleExp,
                                             To::kScaleExp, To::kRawMin,
                                             To::kRawMax)));
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

template <typename RV>
constexpr RV SaturatingAdd32(RV a, RV b, RV raw_min, RV raw_max) {
  if constexpr (std::is_signed_v<RV>) {
    auto const ua = static_cast<std::uint32_t>(static_cast<std::int32_t>(a));
    auto const ub = static_cast<std::uint32_t>(static_cast<std::int32_t>(b));
    auto const us = ua + ub;
    auto const sum = static_cast<std::int32_t>(us);
    bool const a_pos = a > RV{0};
    bool const b_pos = b > RV{0};
    bool const a_neg = a < RV{0};
    bool const b_neg = b < RV{0};
    if (a_pos && b_pos && sum < 0) {
      return raw_max;
    }
    if (a_neg && b_neg && sum >= 0) {
      return raw_min;
    }
    return ClampRaw(static_cast<RV>(sum), raw_min, raw_max);
  } else {
    auto const ua = static_cast<std::uint32_t>(a);
    auto const ub = static_cast<std::uint32_t>(b);
    std::uint32_t s = 0;
    if (!integer_math::AddU32Checked(ua, ub, s)) {
      return raw_max;
    }
    if (s > static_cast<std::uint32_t>(raw_max)) {
      return raw_max;
    }
    return static_cast<RV>(s);
  }
}

template <typename RV>
constexpr RV SaturatingSub32(RV a, RV b, RV raw_min, RV raw_max) {
  if constexpr (std::is_signed_v<RV>) {
    auto const ua = static_cast<std::uint32_t>(static_cast<std::int32_t>(a));
    auto const ub = static_cast<std::uint32_t>(static_cast<std::int32_t>(b));
    auto const ud = ua - ub;
    auto const diff = static_cast<std::int32_t>(ud);
    bool const a_pos = a >= RV{0};
    bool const b_neg = b < RV{0};
    bool const a_neg = a < RV{0};
    bool const b_pos = b > RV{0};
    if (a_pos && b_neg && diff < 0) {
      return raw_max;
    }
    if (a_neg && b_pos && diff >= 0) {
      return raw_min;
    }
    return ClampRaw(static_cast<RV>(diff), raw_min, raw_max);
  } else {
    auto const ua = static_cast<std::uint32_t>(a);
    auto const ub = static_cast<std::uint32_t>(b);
    if (ua < ub) {
      return raw_min;
    }
    auto const d = ua - ub;
    if (d > static_cast<std::uint32_t>(raw_max)) {
      return raw_max;
    }
    return static_cast<RV>(d);
  }
}

template <typename ResultRep, auto ResultMax>
constexpr FixedPoint<ResultRep, ResultMax> FromWideProduct32(
    std::uint32_t hi, std::uint32_t lo, bool negative, int scale_adjust) {
  using Result = FixedPoint<ResultRep, ResultMax>;
  using RV = typename Result::rep_value_type;
  if (scale_adjust > 0) {
    if (!integer_math::ShlU32WideChecked(hi, lo,
                                         static_cast<unsigned>(scale_adjust))) {
      return negative ? Result::FromRaw(Result::kRawMin)
                      : Result::FromRaw(Result::kRawMax);
    }
  } else if (scale_adjust < 0) {
    integer_math::ShrU32Wide(hi, lo, static_cast<unsigned>(-scale_adjust),
                             true);
  }
  if (!negative) {
    if (hi != 0U) {
      return Result::FromRaw(Result::kRawMax);
    }
    if constexpr (std::is_signed_v<RV>) {
      if (lo > static_cast<std::uint32_t>(Result::kRawMax)) {
        return Result::FromRaw(Result::kRawMax);
      }
    } else if (lo > static_cast<std::uint32_t>(Result::kRawMax)) {
      return Result::FromRaw(Result::kRawMax);
    }
    return Result::FromRaw(RepFromRawValue<ResultRep>(static_cast<RV>(lo)));
  }
  // Negative: (hi,lo) is an unsigned magnitude.
  if (hi != 0U) {
    return Result::FromRaw(Result::kRawMin);
  }
  if constexpr (!std::is_signed_v<RV>) {
    return Result::FromRaw(Result::kRawMin);
  } else {
    auto const min_mag = integer_math::AbsI32ToU32(
        static_cast<std::int32_t>(Result::kRawMin));
    if (lo > min_mag) {
      return Result::FromRaw(Result::kRawMin);
    }
    if (lo == min_mag) {
      return Result::FromRaw(Result::kRawMin);
    }
    return Result::FromRaw(RepFromRawValue<ResultRep>(
        static_cast<RV>(-static_cast<std::int32_t>(lo))));
  }
}

template <typename ResultRep, auto ResultMax, typename L, typename R>
constexpr FixedPoint<ResultRep, ResultMax> AddFixedPoint(L lhs, R rhs) {
  using Result = FixedPoint<ResultRep, ResultMax>;
  using RV = typename Result::rep_value_type;
  if constexpr (kRepAtMost32<RV> && kRepAtMost32<typename L::rep_value_type> &&
                kRepAtMost32<typename R::rep_value_type>) {
    RV const lhs_raw = ConvertRawScaleTo<RV>(
        lhs.RawValue(), L::kScaleExp, Result::kScaleExp, Result::kRawMin,
        Result::kRawMax);
    RV const rhs_raw = ConvertRawScaleTo<RV>(
        rhs.RawValue(), R::kScaleExp, Result::kScaleExp, Result::kRawMin,
        Result::kRawMax);
    return Result::FromRaw(RepFromRawValue<ResultRep>(
        SaturatingAdd32(lhs_raw, rhs_raw, Result::kRawMin, Result::kRawMax)));
  } else {
    const auto lhs_raw =
        Result::AlignRawFromScale(lhs.RawValue(), L::kScaleExp);
    const auto rhs_raw =
        Result::AlignRawFromScale(rhs.RawValue(), R::kScaleExp);
    const std::int64_t sum =
        static_cast<std::int64_t>(lhs_raw) + static_cast<std::int64_t>(rhs_raw);
    return Result::FromRaw(RepFromRawValue<ResultRep>(
        Result::ClampRaw(static_cast<RV>(sum))));
  }
}

template <typename ResultRep, auto ResultMax, typename L, typename R>
constexpr FixedPoint<ResultRep, ResultMax> SubFixedPoint(L lhs, R rhs) {
  using Result = FixedPoint<ResultRep, ResultMax>;
  using RV = typename Result::rep_value_type;
  if constexpr (kRepAtMost32<RV> && kRepAtMost32<typename L::rep_value_type> &&
                kRepAtMost32<typename R::rep_value_type>) {
    RV const lhs_raw = ConvertRawScaleTo<RV>(
        lhs.RawValue(), L::kScaleExp, Result::kScaleExp, Result::kRawMin,
        Result::kRawMax);
    RV const rhs_raw = ConvertRawScaleTo<RV>(
        rhs.RawValue(), R::kScaleExp, Result::kScaleExp, Result::kRawMin,
        Result::kRawMax);
    return Result::FromRaw(RepFromRawValue<ResultRep>(
        SaturatingSub32(lhs_raw, rhs_raw, Result::kRawMin, Result::kRawMax)));
  } else {
    const auto lhs_raw =
        Result::AlignRawFromScale(lhs.RawValue(), L::kScaleExp);
    const auto rhs_raw =
        Result::AlignRawFromScale(rhs.RawValue(), R::kScaleExp);
    const std::int64_t diff =
        static_cast<std::int64_t>(lhs_raw) - static_cast<std::int64_t>(rhs_raw);
    return Result::FromRaw(RepFromRawValue<ResultRep>(
        Result::ClampRaw(static_cast<RV>(diff))));
  }
}

template <typename ResultRep, auto ResultMax, typename L, typename R>
constexpr FixedPoint<ResultRep, ResultMax> MulFixedPoint(L lhs, R rhs) {
  using Result = FixedPoint<ResultRep, ResultMax>;
  using RV = typename Result::rep_value_type;
  const int scale_adjust = L::kScaleExp + R::kScaleExp - Result::kScaleExp;
  if constexpr (kRepAtMost32<RV> && kRepAtMost32<typename L::rep_value_type> &&
                kRepAtMost32<typename R::rep_value_type>) {
    bool const lneg = std::is_signed_v<typename L::rep_value_type> &&
                      lhs.RawValue() < static_cast<typename L::rep_value_type>(0);
    bool const rneg = std::is_signed_v<typename R::rep_value_type> &&
                      rhs.RawValue() < static_cast<typename R::rep_value_type>(0);
    std::uint32_t a = 0;
    std::uint32_t b = 0;
    if constexpr (std::is_signed_v<typename L::rep_value_type>) {
      a = integer_math::AbsI32ToU32(static_cast<std::int32_t>(lhs.RawValue()));
    } else {
      a = static_cast<std::uint32_t>(lhs.RawValue());
    }
    if constexpr (std::is_signed_v<typename R::rep_value_type>) {
      b = integer_math::AbsI32ToU32(static_cast<std::int32_t>(rhs.RawValue()));
    } else {
      b = static_cast<std::uint32_t>(rhs.RawValue());
    }
    std::uint32_t hi = 0;
    std::uint32_t lo = 0;
    integer_math::MulU32Wide(a, b, hi, lo);
    return FromWideProduct32<ResultRep, ResultMax>(hi, lo, lneg != rneg,
                                                   scale_adjust);
  } else {
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
        Result::ClampRaw(static_cast<RV>(num))));
  }
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
  return fixed_point_internal::AddFixedPoint<typename Target::rep_type,
                                             Target::kDeclaredMax>(lhs, rhs);
}

template <typename Target, typename L, typename R>
constexpr Target SubTo(L lhs, R rhs) {
  return fixed_point_internal::SubFixedPoint<typename Target::rep_type,
                                             Target::kDeclaredMax>(lhs, rhs);
}

template <typename Target, typename L, typename R>
constexpr Target MulTo(L lhs, R rhs) {
  return fixed_point_internal::MulFixedPoint<typename Target::rep_type,
                                             Target::kDeclaredMax>(lhs, rhs);
}

template <typename Target, typename L, typename R>
constexpr Target DivTo(L lhs, R rhs) {
  using TR = typename Target::rep_value_type;
  using LR = typename L::rep_value_type;
  using RR = typename R::rep_value_type;
  const auto lhs_raw = lhs.RawValue();
  const auto rhs_raw = rhs.RawValue();

  if (rhs_raw == static_cast<RR>(0)) {
    if constexpr (Target::kIsSigned) {
      bool const pos = !std::is_signed_v<LR> ||
                       lhs_raw >= static_cast<LR>(0);
      return pos ? Target::FromRaw(Target::kRawMax)
                 : Target::FromRaw(Target::kRawMin);
    }
    return Target::FromRaw(Target::kRawMax);
  }

  const int scale_adjust = L::kScaleExp - R::kScaleExp - Target::kScaleExp;

  if constexpr (fixed_point_internal::kRepAtMost32<TR> &&
                fixed_point_internal::kRepAtMost32<LR> &&
                fixed_point_internal::kRepAtMost32<RR>) {
    bool const lneg =
        std::is_signed_v<LR> && lhs_raw < static_cast<LR>(0);
    bool const rneg =
        std::is_signed_v<RR> && rhs_raw < static_cast<RR>(0);
    bool const negative = lneg != rneg;
    std::uint32_t a = 0;
    std::uint32_t d = 0;
    if constexpr (std::is_signed_v<LR>) {
      a = integer_math::AbsI32ToU32(static_cast<std::int32_t>(lhs_raw));
    } else {
      a = static_cast<std::uint32_t>(lhs_raw);
    }
    if constexpr (std::is_signed_v<RR>) {
      d = integer_math::AbsI32ToU32(static_cast<std::int32_t>(rhs_raw));
    } else {
      d = static_cast<std::uint32_t>(rhs_raw);
    }
    std::uint32_t hi = 0;
    std::uint32_t lo = a;
    std::uint32_t dhi = 0;
    std::uint32_t dlo = d;
    if (scale_adjust > 0) {
      if (!integer_math::ShlU32WideChecked(
              hi, lo, static_cast<unsigned>(scale_adjust))) {
        return negative ? Target::FromRaw(Target::kRawMin)
                        : Target::FromRaw(Target::kRawMax);
      }
    } else if (scale_adjust < 0) {
      if (!integer_math::ShlU32WideChecked(
              dhi, dlo, static_cast<unsigned>(-scale_adjust))) {
        return Target::FromRaw(static_cast<TR>(0));
      }
    }
    while (dhi != 0U) {
      integer_math::ShrU32Wide(hi, lo, 1U, false);
      integer_math::ShrU32Wide(dhi, dlo, 1U, false);
    }
    if (dlo == 0U) {
      return negative ? Target::FromRaw(Target::kRawMin)
                      : Target::FromRaw(Target::kRawMax);
    }
    std::uint32_t q = 0;
    std::uint32_t rem = 0;
    if (!integer_math::DivU32Wide(hi, lo, dlo, q, rem)) {
      return negative ? Target::FromRaw(Target::kRawMin)
                      : Target::FromRaw(Target::kRawMax);
    }
    if (rem >= dlo - rem) {
      if (q != std::numeric_limits<std::uint32_t>::max()) {
        ++q;
      }
    }
    return fixed_point_internal::FromWideProduct32<
        typename Target::rep_type, Target::kDeclaredMax>(0U, q, negative, 0);
  } else {
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
            Target::ClampRaw(static_cast<TR>(quotient))));
  }
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

#endif  // AE_NUMERIC_FIXED_POINT_H_
