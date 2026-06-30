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

#ifndef NUMERIC_FIXED_MATH_H_
#define NUMERIC_FIXED_MATH_H_

#include <array>
#include <cassert>
#include <cstdint>
#include <type_traits>

#include <gcem.hpp>

#include "numeric/fixed_point.h"
#include "numeric/numeric_traits.h"

namespace ae::fixed_math {

inline constexpr int kDefaultLogIterations = 11;

struct DefaultFixedMathPolicy {
  using log_type = FixedPoint<std::int16_t, 16.0>;
  using mant_type = FixedPoint<std::uint16_t, 4.0>;
  using mul_intermediate_type = std::int64_t;
  static constexpr int kLogIterations = kDefaultLogIterations;
  static constexpr int kExp2FractionBits = kDefaultLogIterations;
};

namespace internal {

template <typename T>
inline constexpr bool is_fixed_point_v = numeric_traits<T>::kIsFixedPoint;

template <typename T>
  requires is_fixed_point_v<T>
constexpr auto raw_value(T x) {
  return x.RawValue();
}

template <typename T>
  requires is_fixed_point_v<T>
constexpr int scale_of() {
  return T::kScaleExp;
}

template <typename Target, typename Source>
  requires is_fixed_point_v<Target> && is_fixed_point_v<Source>
constexpr Target cast_fixed(Source x) {
  const std::int64_t src_raw = static_cast<std::int64_t>(x.RawValue());
  const std::int64_t aligned = fixed_point_internal::ConvertRawScale(
      src_raw, Source::kScaleExp, Target::kScaleExp,
      static_cast<std::int64_t>(Target::kRawMin),
      static_cast<std::int64_t>(Target::kRawMax));
  return Target::FromRaw(
      fixed_point_internal::RepFromRawValue<typename Target::rep_type>(
          static_cast<typename Target::rep_value_type>(aligned)));
}

template <typename T>
  requires is_fixed_point_v<T>
constexpr T one() {
  return T{1};
}

template <typename T>
  requires is_fixed_point_v<T>
constexpr T two() {
  return T{2};
}

template <typename T>
  requires is_fixed_point_v<T>
constexpr T half() {
  return T{0.5};
}

template <typename Log, int I>
consteval Log InvPow2LogEntry() {
  return Log::FromDouble(1.0 / static_cast<double>(std::uint64_t{1} << I));
}

template <typename Log, int Iterations, std::size_t... Is>
consteval std::array<Log, Iterations> MakeInvPow2Table(
    std::index_sequence<Is...>) {
  return {InvPow2LogEntry<Log, static_cast<int>(Is + 1)>()...};
}

template <typename Log, int Iterations>
constexpr Log InvPow2Log(int i) {
  static_assert(Iterations >= 1);
  constexpr auto kTable =
      MakeInvPow2Table<Log, Iterations>(std::make_index_sequence<Iterations>{});
  return kTable[static_cast<std::size_t>(i - 1)];
}

template <typename Mant, int I>
consteval Mant Exp2FactorEntry() {
  return Mant::FromDouble(
      gcem::pow(2.0, 1.0 / static_cast<double>(std::uint64_t{1} << I)));
}

template <typename Mant, int Iterations, std::size_t... Is>
consteval std::array<Mant, Iterations> MakeExp2FactorTable(
    std::index_sequence<Is...>) {
  return {Exp2FactorEntry<Mant, static_cast<int>(Is + 1)>()...};
}

template <typename Mant, int Iterations>
constexpr Mant Exp2Factor(int i) {
  static_assert(Iterations >= 1);
  constexpr auto kTable = MakeExp2FactorTable<Mant, Iterations>(
      std::make_index_sequence<Iterations>{});
  return kTable[static_cast<std::size_t>(i - 1)];
}

template <typename Log>
constexpr std::int64_t FloorLogical(Log y) {
  std::int64_t num = static_cast<std::int64_t>(y.RawValue());
  std::int64_t den = 1;
  const int scale = Log::kScaleExp;
  if constexpr (scale > 0) {
    for (int i = 0; i < scale; ++i) {
      num *= 2;
    }
  } else if constexpr (scale < 0) {
    for (int i = 0; i < -scale; ++i) {
      den *= 2;
    }
  }
  if (num >= 0) {
    return num / den;
  }
  const std::int64_t q = num / den;
  if (num % den == 0) {
    return q;
  }
  return q - 1;
}

template <typename Mant, typename X>
constexpr Mant NormalizeMantissa(X x, int& exponent_out) {
  int e = 0;
  X m = x;
  const X one_x = one<X>();
  const X two_x = two<X>();

  for (int guard = 0; guard < 64 && m >= two_x; ++guard) {
    m = DivTo<X>(m, two_x);
    ++e;
  }
  for (int guard = 0;
       guard < 64 &&
           m.RawValue() != typename X::rep_value_type {0} && m < one_x;
       ++guard) {
    m = MulTo<X>(m, two_x);
    --e;
  }

  exponent_out = e;
  return cast_fixed<Mant>(m);
}

template <typename Target>
  requires is_fixed_point_v<Target>
constexpr std::int64_t raw_one() {
  return static_cast<std::int64_t>(Target::FromRuntimeInteger(1).RawValue());
}

template <typename Target, typename Source>
  requires is_fixed_point_v<Target> && is_fixed_point_v<Source>
constexpr std::int64_t align_raw_to_target(Source x) {
  return fixed_point_internal::ConvertRawScale(
      static_cast<std::int64_t>(x.RawValue()), Source::kScaleExp,
      Target::kScaleExp, static_cast<std::int64_t>(Target::kRawMin),
      static_cast<std::int64_t>(Target::kRawMax));
}

template <typename Intermediate>
constexpr std::int64_t RoundDivNearestWide(Intermediate num, Intermediate den) {
  if (den == Intermediate{0}) {
    return static_cast<std::int64_t>(num);
  }

  if constexpr (std::is_signed_v<Intermediate>) {
    if (num >= 0 && den > 0) {
      return static_cast<std::int64_t>((num + den / 2) / den);
    }
    if (num < 0 && den > 0) {
      return static_cast<std::int64_t>((num - den / 2) / den);
    }
    if (num >= 0 && den < 0) {
      return static_cast<std::int64_t>((num - den / 2) / den);
    }
    return static_cast<std::int64_t>((num + den / 2) / den);
  } else {
    return static_cast<std::int64_t>((num + den / 2) / den);
  }
}

template <typename Target, typename Policy>
  requires is_fixed_point_v<Target>
constexpr std::int64_t mul_raw_fixed(std::int64_t acc, std::int64_t factor) {
  using Intermediate = typename Policy::mul_intermediate_type;
  const Intermediate product =
      static_cast<Intermediate>(acc) * static_cast<Intermediate>(factor);
  const Intermediate one = static_cast<Intermediate>(raw_one<Target>());
  return RoundDivNearestWide(product, one);
}

template <typename Target>
  requires is_fixed_point_v<Target>
constexpr std::int64_t apply_pow2_int_raw(std::int64_t acc,
                                          std::int64_t int_part) {
  if (int_part > 0) {
    for (std::int64_t i = 0; i < int_part && i < 62; ++i) {
      const std::int64_t shifted = acc * 2;
      if (shifted > static_cast<std::int64_t>(Target::kRawMax)) {
        return static_cast<std::int64_t>(Target::kRawMax);
      }
      acc = shifted;
    }
    return acc;
  }
  if (int_part < 0) {
    for (std::int64_t i = 0; i < -int_part && i < 62; ++i) {
      acc = fixed_point_internal::RoundDivNearest(acc, std::int64_t{2});
    }
  }
  return acc;
}

template <typename Target>
  requires is_fixed_point_v<Target>
constexpr Target from_clamped_raw(std::int64_t raw) {
  const std::int64_t min_raw = static_cast<std::int64_t>(Target::kRawMin);
  const std::int64_t max_raw = static_cast<std::int64_t>(Target::kRawMax);

  if (raw < min_raw) {
    raw = min_raw;
  }
  if (raw > max_raw) {
    raw = max_raw;
  }

  return Target::FromRaw(
      fixed_point_internal::RepFromRawValue<typename Target::rep_type>(
          static_cast<typename Target::rep_value_type>(raw)));
}

// Precondition: x > 0.
template <typename Target, typename X, typename Policy>
  requires internal::is_fixed_point_v<X> && internal::is_fixed_point_v<Target>
constexpr Target Log2To(X x) {
  if (!std::is_constant_evaluated()) {
    assert(x.RawValue() != typename X::rep_value_type{0});
  }

  using Mant = typename Policy::mant_type;
  using Log = Target;

  int exponent = 0;
  Mant mantissa = NormalizeMantissa<Mant>(x, exponent);

  Log result = Log::FromRuntimeInteger(exponent);
  const Mant two_m = two<Mant>();

  for (int i = 1; i <= Policy::kLogIterations; ++i) {
    mantissa = MulTo<Mant>(mantissa, mantissa);
    if (mantissa >= two_m) {
      mantissa = DivTo<Mant>(mantissa, two_m);
      result = AddTo<Log>(result, InvPow2Log<Log, Policy::kLogIterations>(i));
    }
  }

  return result;
}

template <typename Target, typename X, typename Policy>
  requires internal::is_fixed_point_v<X> && internal::is_fixed_point_v<Target>
constexpr Target Exp2To(X y) {
  using Log = typename Policy::log_type;
  using Mant = typename Policy::mant_type;

  const Log ly = cast_fixed<Log>(y);
  const std::int64_t int_part = FloorLogical(ly);
  Log frac = SubTo<Log>(ly, Log::FromRuntimeInteger(int_part));

  std::int64_t acc = raw_one<Target>();
  for (int i = 1; i <= Policy::kExp2FractionBits; ++i) {
    const Log bit = InvPow2Log<Log, Policy::kExp2FractionBits>(i);
    if (bit.RawValue() == typename Log::rep_value_type{0}) {
      break;
    }
    if (frac >= bit) {
      const std::int64_t factor = align_raw_to_target<Target>(
          Exp2Factor<Mant, Policy::kExp2FractionBits>(i));
      acc = mul_raw_fixed<Target, Policy>(acc, factor);
      frac = SubTo<Log>(frac, bit);
    }
  }

  acc = apply_pow2_int_raw<Target>(acc, int_part);
  return from_clamped_raw<Target>(acc);
}

}  // namespace internal

template <typename Target, typename X>
  requires internal::is_fixed_point_v<X> && internal::is_fixed_point_v<Target>
constexpr Target Log2To(X x) {
  return internal::Log2To<Target, X, DefaultFixedMathPolicy>(x);
}

template <typename Target, typename X, typename Policy>
  requires internal::is_fixed_point_v<X> && internal::is_fixed_point_v<Target>
constexpr Target Log2To(X x) {
  return internal::Log2To<Target, X, Policy>(x);
}

template <typename Target, typename X>
  requires internal::is_fixed_point_v<X> && internal::is_fixed_point_v<Target>
constexpr Target Exp2To(X y) {
  return internal::Exp2To<Target, X, DefaultFixedMathPolicy>(y);
}

template <typename Target, typename X, typename Policy>
  requires internal::is_fixed_point_v<X> && internal::is_fixed_point_v<Target>
constexpr Target Exp2To(X y) {
  return internal::Exp2To<Target, X, Policy>(y);
}

}  // namespace ae::fixed_math

#endif  // NUMERIC_FIXED_MATH_H_
