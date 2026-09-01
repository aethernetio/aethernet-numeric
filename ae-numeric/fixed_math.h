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

#ifndef AE_NUMERIC_FIXED_MATH_H_
#define AE_NUMERIC_FIXED_MATH_H_

// Purpose: Supply deterministic transcendental and root operations for
// FixedPoint values without requiring an FPU.
//
// Motivation: Exponential and SegmentedNumber need Log2, Exp2, powers,
// square roots, and roots on constrained embedded targets.
//
// Analogous concepts: fixed-point elementary-function kernels and
// digit-by-digit/iterative numerical approximations.
//
// Usage: call the typed Log2To, Exp2To, PowTo, Sqrt, and related helpers,
// optionally selecting a math policy. SegmentedNumber uses the
// Segmented32MathPolicy with Rep widths no greater than 32 bits.
//
// How it works: values are normalized and refined for a fixed number of
// iterations. Small fixed-size FixedPoint tables support Log2/Exp2; they
// are independent of SegmentedNumber rank count and are not per-code
// lookup tables.


#include <array>
#include <cassert>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <utility>

#include "ae-numeric/fixed_point.h"
#include "ae-numeric/integer_math.h"
#include "ae-numeric/numeric_traits.h"

namespace ae::fixed_math {

inline constexpr int kDefaultLogIterations = 11;

struct DefaultFixedMathPolicy {
  using log_type = FixedPoint<std::int16_t, 16>;
  using mant_type = FixedPoint<std::uint32_t, 4>;
  static constexpr int kLogIterations = kDefaultLogIterations;
  static constexpr int kExp2FractionBits = kDefaultLogIterations;
};

// Higher-resolution policy for values near 1 and for SegmentedNumber.
// log_type Max is large enough for log2 of physical quantities (RX window,
// CO2) but too small to hold interval counts; multiply/divide logs by
// integer n with ScaleLogByInt / DivLogByInt.
struct HighPrecisionFixedMathPolicy {
  using log_type = FixedPoint<std::int32_t, 32>;
  using mant_type = FixedPoint<std::uint32_t, 4>;
  static constexpr int kLogIterations = 26;
  static constexpr int kExp2FractionBits = 26;
};

// SegmentedNumber production math: FixedPoint Rep width <= 32 bits, no
// 64-bit runtime arithmetic in Log2/Exp2 instantiations.
struct Segmented32MathPolicy : HighPrecisionFixedMathPolicy {
  static_assert(sizeof(typename log_type::rep_value_type) <= 4);
  static_assert(sizeof(typename mant_type::rep_value_type) <= 4);
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
  using ToR = typename Target::rep_value_type;
  ToR const aligned = fixed_point_internal::ConvertRawScaleTo<ToR>(
      x.RawValue(), Source::kScaleExp, Target::kScaleExp, Target::kRawMin,
      Target::kRawMax);
  return Target::FromRaw(
      fixed_point_internal::RepFromRawValue<typename Target::rep_type>(
          aligned));
}

template <typename Log, int I>
consteval Log InvPow2LogEntry() {
  static_assert(I >= 1 && I < 31);
  return Log::FromRatio(1, static_cast<std::int32_t>(1) << I);
}

template <typename Log, int Iterations, std::size_t... Is>
consteval std::array<Log, Iterations> MakeInvPow2Table(
    std::index_sequence<Is...>) {
  return {InvPow2LogEntry<Log, static_cast<int>(Is + 1)>()...};
}

template <typename Log, int Iterations>
struct InvPow2TableHolder {
  static constexpr auto kTable =
      MakeInvPow2Table<Log, Iterations>(std::make_index_sequence<Iterations>{});
};

template <typename Log, int Iterations>
constexpr Log InvPow2Log(int i) {
  static_assert(Iterations >= 1 && Iterations < 31);
  return InvPow2TableHolder<Log, Iterations>::kTable[static_cast<std::size_t>(
      i - 1)];
}

template <typename T>
  requires is_fixed_point_v<T>
constexpr T SqrtNewton(T x) {
  if (x.RawValue() == static_cast<typename T::rep_value_type>(0)) {
    return x;
  }
  T y = x;
  T const one = T::FromRuntimeInteger(1);
  T const two = T::FromRuntimeInteger(2);
  if (x < one) {
    y = one;
  }
  for (int i = 0; i < 16; ++i) {
    if (y.RawValue() == static_cast<typename T::rep_value_type>(0)) {
      break;
    }
    T const q = DivTo<T>(x, y);
    y = DivTo<T>(AddTo<T>(y, q), two);
  }
  return y;
}

template <typename Mant, int Iterations, std::size_t... Is>
constexpr std::array<Mant, static_cast<std::size_t>(Iterations)>
MakeExp2MantTable(std::index_sequence<Is...>) {
  static_assert(Iterations >= 1 && Iterations < 31);
  std::array<Mant, static_cast<std::size_t>(Iterations)> t{
      ((void)Is, Mant::FromRuntimeInteger(0))...};
  Mant v = Mant::FromRuntimeInteger(2);
  for (int i = 0; i < Iterations; ++i) {
    v = SqrtNewton(v);
    t[static_cast<std::size_t>(i)] = v;
  }
  return t;
}

template <typename Mant, int Iterations>
struct Exp2MantTableHolder {
  static constexpr auto kTable = MakeExp2MantTable<Mant, Iterations>(
      std::make_index_sequence<static_cast<std::size_t>(Iterations)>{});
};

template <typename Mant, int Iterations>
constexpr Mant Exp2Factor(int i) {
  static_assert(Iterations >= 1 && Iterations < 31);
  return Exp2MantTableHolder<Mant, Iterations>::kTable[static_cast<std::size_t>(
      i - 1)];
}

template <typename Log>
constexpr int FloorLogical32(Log y) {
  static_assert(sizeof(typename Log::rep_value_type) <= 4);
  auto const raw = static_cast<std::int32_t>(y.RawValue());
  int const scale = Log::kScaleExp;
  if (scale >= 0) {
    std::int32_t v = raw;
    for (int i = 0; i < scale; ++i) {
      if (v > std::numeric_limits<std::int32_t>::max() / 2) {
        return std::numeric_limits<int>::max() / 4;
      }
      if (v < std::numeric_limits<std::int32_t>::min() / 2) {
        return std::numeric_limits<int>::min() / 4;
      }
      v *= 2;
    }
    return static_cast<int>(v);
  }
  unsigned const bits = static_cast<unsigned>(-scale);
  if (bits >= 31U) {
    return raw < 0 ? -1 : 0;
  }
  auto const div = static_cast<std::int32_t>(1U << bits);
  if (raw >= 0) {
    return static_cast<int>(raw / div);
  }
  std::int32_t const q = raw / div;
  if ((raw % div) == 0) {
    return static_cast<int>(q);
  }
  return static_cast<int>(q - 1);
}

template <typename Mant, typename X>
constexpr Mant NormalizeMantissa(X x, int& exponent_out) {
  int e = 0;
  X m = x;
  const X one_x = X::FromRuntimeInteger(1);
  const X two_x = X::FromRuntimeInteger(2);

  for (int guard = 0; guard < 64 && m >= two_x; ++guard) {
    m = DivTo<X>(m, two_x);
    ++e;
  }
  for (int guard = 0;
       guard < 64 &&
       m.RawValue() != static_cast<typename X::rep_value_type>(0) && m < one_x;
       ++guard) {
    m = AddTo<X>(m, m);
    --e;
  }

  exponent_out = e;
  return cast_fixed<Mant>(m);
}

template <typename Target>
  requires is_fixed_point_v<Target>
constexpr Target apply_pow2_int(Target acc, int int_part) {
  Target const two = Target::FromRuntimeInteger(2);
  if (int_part > 0) {
    for (int i = 0; i < int_part && i < 31; ++i) {
      acc = AddTo<Target>(acc, acc);
    }
    return acc;
  }
  if (int_part < 0) {
    for (int i = 0; i < -int_part && i < 31; ++i) {
      acc = DivTo<Target>(acc, two);
    }
  }
  return acc;
}

template <typename Target>
  requires is_fixed_point_v<Target>
constexpr Target from_clamped_i32(std::int32_t raw) {
  using RV = typename Target::rep_value_type;
  if constexpr (std::is_signed_v<RV>) {
    auto const min_raw = static_cast<std::int32_t>(Target::kRawMin);
    auto const max_raw = static_cast<std::int32_t>(Target::kRawMax);
    if (raw < min_raw) {
      raw = min_raw;
    }
    if (raw > max_raw) {
      raw = max_raw;
    }
    return Target::FromRaw(
        fixed_point_internal::RepFromRawValue<typename Target::rep_type>(
            static_cast<RV>(raw)));
  } else {
    if (raw < 0) {
      return Target::FromRaw(Target::kRawMin);
    }
    auto const u = static_cast<std::uint32_t>(raw);
    if (u > static_cast<std::uint32_t>(Target::kRawMax)) {
      return Target::FromRaw(Target::kRawMax);
    }
    return Target::FromRaw(
        fixed_point_internal::RepFromRawValue<typename Target::rep_type>(
            static_cast<RV>(u)));
  }
}

// Precondition: x > 0.
template <typename Target, typename X, typename Policy>
  requires internal::is_fixed_point_v<X> && internal::is_fixed_point_v<Target>
constexpr Target Log2To(X x) {
  if (!std::is_constant_evaluated()) {
    assert(x.RawValue() != typename X::rep_value_type{0});
  }

  using Mant = FixedPoint<std::uint32_t, 4>;
  using Log = Target;

  int exponent = 0;
  Mant mantissa = NormalizeMantissa<Mant>(x, exponent);

  Log result = Log::FromRuntimeInteger(exponent);
  const Mant two_m = Mant::FromRuntimeInteger(2);

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
  static_assert(sizeof(typename Target::rep_value_type) <= 4);
  using Log = typename Policy::log_type;
  using Mant = typename Policy::mant_type;
  static_assert(sizeof(typename Log::rep_value_type) <= 4);
  static_assert(sizeof(typename Mant::rep_value_type) <= 4);

  const Log ly = cast_fixed<Log>(y);
  const int int_part = FloorLogical32(ly);
  Log frac = SubTo<Log>(ly, Log::FromRuntimeInteger(int_part));

  Target acc = Target::FromRuntimeInteger(1);
  for (int i = 1; i <= Policy::kExp2FractionBits; ++i) {
    const Log bit = InvPow2Log<Log, Policy::kExp2FractionBits>(i);
    if (bit.RawValue() == typename Log::rep_value_type{0}) {
      break;
    }
    if (frac >= bit) {
      acc = MulTo<Target>(
          acc, cast_fixed<Target>(
                   Exp2Factor<Mant, Policy::kExp2FractionBits>(i)));
      frac = SubTo<Log>(frac, bit);
    }
  }

  return apply_pow2_int(acc, int_part);
}

}  // namespace internal

template <typename Log>
  requires internal::is_fixed_point_v<Log>
constexpr Log ScaleLogByInt(Log x, int n) {
  static_assert(sizeof(typename Log::rep_value_type) <= 4);
  if (n == 0 || x.RawValue() == static_cast<typename Log::rep_value_type>(0)) {
    return internal::from_clamped_i32<Log>(0);
  }
  bool const negative =
      (std::is_signed_v<typename Log::rep_value_type> &&
       x.RawValue() < static_cast<typename Log::rep_value_type>(0)) !=
      (n < 0);
  std::uint32_t a = 0;
  if constexpr (std::is_signed_v<typename Log::rep_value_type>) {
    a = integer_math::AbsI32ToU32(static_cast<std::int32_t>(x.RawValue()));
  } else {
    a = static_cast<std::uint32_t>(x.RawValue());
  }
  std::uint32_t const nu =
      n < 0 ? integer_math::AbsI32ToU32(static_cast<std::int32_t>(n))
            : static_cast<std::uint32_t>(n);
  std::uint32_t hi = 0;
  std::uint32_t lo = 0;
  integer_math::MulU32Wide(a, nu, hi, lo);
  if (hi != 0U) {
    return negative ? Log::FromRaw(Log::kRawMin) : Log::FromRaw(Log::kRawMax);
  }
  if (lo > static_cast<std::uint32_t>(
           std::numeric_limits<std::int32_t>::max())) {
    return negative ? Log::FromRaw(Log::kRawMin) : Log::FromRaw(Log::kRawMax);
  }
  std::int32_t out = static_cast<std::int32_t>(lo);
  if (negative) {
    out = -out;
  }
  return internal::from_clamped_i32<Log>(out);
}

template <typename Log>
  requires internal::is_fixed_point_v<Log>
constexpr Log DivLogByInt(Log x, int n) {
  static_assert(sizeof(typename Log::rep_value_type) <= 4);
  if (n == 0) {
    return x.RawValue() >= static_cast<typename Log::rep_value_type>(0)
               ? Log::FromRaw(Log::kRawMax)
               : Log::FromRaw(Log::kRawMin);
  }
  auto const num = static_cast<std::int32_t>(x.RawValue());
  auto const den = static_cast<std::int32_t>(n);
  return internal::from_clamped_i32<Log>(
      fixed_point_internal::RoundDivNearest(num, den));
}

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

template <typename T>
  requires internal::is_fixed_point_v<T>
constexpr T Sqrt(T x) {
  return internal::SqrtNewton(x);
}

template <typename T, typename Policy = DefaultFixedMathPolicy>
  requires internal::is_fixed_point_v<T>
constexpr T Log2(T x) {
  return Log2To<T, T, Policy>(x);
}

template <typename T, typename Policy = DefaultFixedMathPolicy>
  requires internal::is_fixed_point_v<T>
constexpr T Exp2(T y) {
  return Exp2To<T, T, Policy>(y);
}

template <typename Target, typename Base,
          typename Policy = DefaultFixedMathPolicy>
  requires internal::is_fixed_point_v<Target> &&
           internal::is_fixed_point_v<Base>
constexpr Target PowIntTo(Base base, int n) {
  if (n == 0) {
    return Target::FromRuntimeInteger(1);
  }
  using Log = typename Policy::log_type;
  Log const lb = Log2To<Log, Base, Policy>(base);
  Log const nlog = ScaleLogByInt(lb, n);
  return Exp2To<Target, Log, Policy>(nlog);
}

template <typename T, typename Policy = DefaultFixedMathPolicy>
  requires internal::is_fixed_point_v<T>
constexpr T PowInt(T base, int n) {
  return PowIntTo<T, T, Policy>(base, n);
}

template <typename Target, typename Base, typename Exp, typename Policy>
  requires internal::is_fixed_point_v<Target> &&
           internal::is_fixed_point_v<Base> &&
           internal::is_fixed_point_v<Exp>
constexpr Target PowTo(Base base, Exp exponent) {
  using Log = typename Policy::log_type;
  static_assert(sizeof(typename Log::rep_value_type) <= 4);
  Log const lb = Log2To<Log, Base, Policy>(base);
  auto const exp_one = static_cast<std::int32_t>(
      Exp::FromRuntimeInteger(1).RawValue());
  if (exp_one == 0) {
    return Target::FromRuntimeInteger(1);
  }
  bool const lneg = std::is_signed_v<typename Log::rep_value_type> &&
                    lb.RawValue() <
                        static_cast<typename Log::rep_value_type>(0);
  bool const eneg =
      std::is_signed_v<typename Exp::rep_value_type> &&
      exponent.RawValue() < static_cast<typename Exp::rep_value_type>(0);
  bool const negative = lneg != eneg;
  std::uint32_t a = 0;
  std::uint32_t f = 0;
  std::uint32_t d = 0;
  if constexpr (std::is_signed_v<typename Log::rep_value_type>) {
    a = integer_math::AbsI32ToU32(static_cast<std::int32_t>(lb.RawValue()));
  } else {
    a = static_cast<std::uint32_t>(lb.RawValue());
  }
  if constexpr (std::is_signed_v<typename Exp::rep_value_type>) {
    f = integer_math::AbsI32ToU32(
        static_cast<std::int32_t>(exponent.RawValue()));
  } else {
    f = static_cast<std::uint32_t>(exponent.RawValue());
  }
  d = integer_math::AbsI32ToU32(exp_one);
  std::uint32_t mag = 0;
  if (!integer_math::MulDivU32Nearest(a, f, d, mag)) {
    mag = static_cast<std::uint32_t>(std::numeric_limits<std::int32_t>::max());
  }
  std::int32_t prod = 0;
  if (mag > static_cast<std::uint32_t>(
           std::numeric_limits<std::int32_t>::max())) {
    prod = negative ? std::numeric_limits<std::int32_t>::min()
                    : std::numeric_limits<std::int32_t>::max();
  } else {
    prod = static_cast<std::int32_t>(mag);
    if (negative) {
      prod = -prod;
    }
  }
  Log const y = internal::from_clamped_i32<Log>(prod);
  return Exp2To<Target, Log, Policy>(y);
}

template <typename T, typename Policy = DefaultFixedMathPolicy>
  requires internal::is_fixed_point_v<T>
constexpr T Pow(T base, T exponent) {
  return PowTo<T, T, T, Policy>(base, exponent);
}

template <typename T, typename Policy = DefaultFixedMathPolicy>
  requires internal::is_fixed_point_v<T>
constexpr T NthRoot(T x, int n) {
  if (n <= 0) {
    return T::FromRuntimeInteger(1);
  }
  using Log = typename Policy::log_type;
  Log const lx = Log2To<Log, T, Policy>(x);
  return Exp2To<T, Log, Policy>(DivLogByInt(lx, n));
}

template <typename Target, typename Base, typename X, typename Policy>
  requires internal::is_fixed_point_v<Target> &&
           internal::is_fixed_point_v<Base> &&
           internal::is_fixed_point_v<X>
constexpr Target LogTo(Base base, X x) {
  using Log = typename Policy::log_type;
  Log const lx = Log2To<Log, X, Policy>(x);
  Log const lb = Log2To<Log, Base, Policy>(base);
  return DivTo<Target>(lx, lb);
}

}  // namespace ae::fixed_math

#endif  // AE_NUMERIC_FIXED_MATH_H_
