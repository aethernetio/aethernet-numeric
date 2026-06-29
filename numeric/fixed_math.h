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
inline constexpr int kExp2FractionBits = 11;

namespace internal {

template <typename T>
inline constexpr bool is_fixed_point_v = numeric_traits<T>::kIsFixedPoint;

template <typename T>
  requires is_fixed_point_v<T>
constexpr auto raw_value(T x) {
  return x.raw_value();
}

template <typename T>
  requires is_fixed_point_v<T>
constexpr int scale_of() {
  return T::kScaleExp;
}

template <typename Target, typename Source>
  requires is_fixed_point_v<Target> && is_fixed_point_v<Source>
constexpr Target cast_fixed(Source x) {
  const std::int64_t src_raw = static_cast<std::int64_t>(x.raw_value());
  const std::int64_t aligned = fixed_point_internal::ConvertRawScale(
      src_raw, Source::kScaleExp, Target::kScaleExp,
      static_cast<std::int64_t>(Target::kRawMin),
      static_cast<std::int64_t>(Target::kRawMax));
  return Target::FromRaw(fixed_point_internal::RepFromRawValue<
                           typename Target::rep_type>(
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

template <typename Log, std::size_t... Is>
consteval std::array<Log, sizeof...(Is)> MakeInvPow2Table(
    std::index_sequence<Is...>) {
  return {InvPow2LogEntry<Log, static_cast<int>(Is + 1)>()...};
}

template <typename Log>
constexpr Log InvPow2Log(int i) {
  constexpr auto kTable =
      MakeInvPow2Table<Log>(std::make_index_sequence<kDefaultLogIterations>{});
  return kTable[static_cast<std::size_t>(i - 1)];
}

template <typename Mant, int I>
consteval Mant Exp2FactorEntry() {
  return Mant::FromDouble(
      gcem::pow(2.0, 1.0 / static_cast<double>(std::uint64_t{1} << I)));
}

template <typename Mant, std::size_t... Is>
consteval std::array<Mant, sizeof...(Is)> MakeExp2FactorTable(
    std::index_sequence<Is...>) {
  return {Exp2FactorEntry<Mant, static_cast<int>(Is + 1)>()...};
}

template <typename Mant>
constexpr Mant Exp2Factor(int i) {
  constexpr auto kTable =
      MakeExp2FactorTable<Mant>(std::make_index_sequence<kExp2FractionBits>{});
  return kTable[static_cast<std::size_t>(i - 1)];
}

template <typename Log>
constexpr std::int64_t FloorLogical(Log y) {
  std::int64_t num = static_cast<std::int64_t>(y.raw_value());
  std::int64_t den = 1;
  const int scale = Log::kScaleExp;
  if (scale > 0) {
    for (int i = 0; i < scale; ++i) {
      num *= 2;
    }
  } else if (scale < 0) {
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
    m = div_to<X>(m, two_x);
    ++e;
  }
  for (int guard = 0; guard < 64 && m.raw_value() != typename X::rep_value_type{0} &&
                      m < one_x;
       ++guard) {
    m = mul_to<X>(m, two_x);
    --e;
  }

  exponent_out = e;
  return cast_fixed<Mant>(m);
}

template <typename Target>
  requires is_fixed_point_v<Target>
constexpr std::int64_t raw_one() {
  return static_cast<std::int64_t>(Target::FromRuntimeInteger(1).raw_value());
}

template <typename Target, typename Source>
  requires is_fixed_point_v<Target> && is_fixed_point_v<Source>
constexpr std::int64_t align_raw_to_target(Source x) {
  return fixed_point_internal::ConvertRawScale(
      static_cast<std::int64_t>(x.raw_value()), Source::kScaleExp,
      Target::kScaleExp, static_cast<std::int64_t>(Target::kRawMin),
      static_cast<std::int64_t>(Target::kRawMax));
}

template <typename Target>
  requires is_fixed_point_v<Target>
constexpr std::int64_t mul_raw_fixed(std::int64_t acc, std::int64_t factor) {
  const std::int64_t one = raw_one<Target>();
  return fixed_point_internal::RoundDivNearest(acc * factor, one);
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
  const auto clamped = fixed_point_internal::ClampRaw(
      static_cast<typename Target::rep_value_type>(raw), Target::kRawMin,
      Target::kRawMax);
  return Target::FromRaw(
      fixed_point_internal::RepFromRawValue<typename Target::rep_type>(
          clamped));
}

}  // namespace internal

// Precondition: x > 0.
template <typename Target, typename X>
  requires internal::is_fixed_point_v<X> && internal::is_fixed_point_v<Target>
constexpr Target log2_to(X x) {
  if (!std::is_constant_evaluated()) {
    assert(x.raw_value() != typename X::rep_value_type{0});
  }

  using Mant = FixedPoint<std::uint16_t, 4.0>;
  using Log = Target;

  int exponent = 0;
  Mant mantissa = internal::NormalizeMantissa<Mant>(x, exponent);

  Log result = Log::FromRuntimeInteger(exponent);
  const Mant two_m = internal::two<Mant>();

  for (int i = 1; i <= kDefaultLogIterations; ++i) {
    mantissa = mul_to<Mant>(mantissa, mantissa);
    if (mantissa >= two_m) {
      mantissa = div_to<Mant>(mantissa, two_m);
      result = add_to<Log>(result, internal::InvPow2Log<Log>(i));
    }
  }

  return result;
}

template <typename Target, typename X>
  requires internal::is_fixed_point_v<X> && internal::is_fixed_point_v<Target>
constexpr Target exp2_to(X y) {
  using Log = FixedPoint<std::int16_t, 16.0>;
  using Mant = FixedPoint<std::uint16_t, 4.0>;

  const Log ly = internal::cast_fixed<Log>(y);
  const std::int64_t int_part = internal::FloorLogical(ly);
  Log frac =
      sub_to<Log>(ly, Log::FromRuntimeInteger(int_part));

  std::int64_t acc = internal::raw_one<Target>();
  for (int i = 1; i <= kExp2FractionBits; ++i) {
    const Log bit = internal::InvPow2Log<Log>(i);
    if (bit.raw_value() == typename Log::rep_value_type{0}) {
      break;
    }
    if (frac >= bit) {
      const std::int64_t factor = internal::align_raw_to_target<Target>(
          internal::Exp2Factor<Mant>(i));
      acc = internal::mul_raw_fixed<Target>(acc, factor);
      frac = sub_to<Log>(frac, bit);
    }
  }

  acc = internal::apply_pow2_int_raw<Target>(acc, int_part);
  return internal::from_clamped_raw<Target>(acc);
}

}  // namespace ae::fixed_math

#endif  // NUMERIC_FIXED_MATH_H_
