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

#ifndef NUMERIC_DECIMAL_H_
#define NUMERIC_DECIMAL_H_

#include <cstdint>
#include <type_traits>

namespace ae {

template <std::int64_t Num, std::int64_t Den = 1>
struct Ratio {
  static constexpr std::int64_t kNum = Num;
  static constexpr std::int64_t kDen = Den;
};

template <typename T>
inline constexpr bool kIsRatioV = false;

template <std::int64_t Num, std::int64_t Den>
inline constexpr bool kIsRatioV<Ratio<Num, Den>> = true;

template <std::int64_t Mantissa, int Exponent10 = 0>
struct Decimal {
  static constexpr std::int64_t kMantissa = Mantissa;
  static constexpr int kExponent10 = Exponent10;
};

template <typename T>
inline constexpr bool kIsDecimalV = false;

template <std::int64_t M, int E>
inline constexpr bool kIsDecimalV<Decimal<M, E>> = true;

template <typename T>
inline constexpr bool kIsNumberRatioV = kIsRatioV<T> || kIsDecimalV<T>;

constexpr std::int64_t Abs64(std::int64_t value) {
  return value < 0 ? -value : value;
}

constexpr std::int64_t Gcd64(std::int64_t a, std::int64_t b) {
  a = Abs64(a);
  b = Abs64(b);
  while (b != 0) {
    const auto t = a % b;
    a = b;
    b = t;
  }
  return a;
}

constexpr std::uint64_t Pow10u(unsigned n) {
  std::uint64_t value = 1;
  for (unsigned i = 0; i < n; ++i) {
    value *= 10;
  }
  return value;
}

constexpr std::int64_t Pow10i(unsigned n) {
  return static_cast<std::int64_t>(Pow10u(n));
}

constexpr std::int64_t NormalizeRatioNum(std::int64_t num, std::int64_t den) {
  if (den < 0) {
    num = -num;
    den = -den;
  }
  if (num == 0 || den == 0) {
    return 0;
  }
  return num / Gcd64(num, den);
}

constexpr std::int64_t NormalizeRatioDen(std::int64_t num, std::int64_t den) {
  if (den < 0) {
    num = -num;
    den = -den;
  }
  if (num == 0) {
    return 1;
  }
  if (den == 0) {
    return 0;
  }
  return den / Gcd64(num, den);
}

template <typename T>
struct NumberRatio;

template <std::int64_t Num, std::int64_t Den>
struct NumberRatio<Ratio<Num, Den>> {
  static constexpr std::int64_t num = NormalizeRatioNum(Num, Den);
  static constexpr std::int64_t den = NormalizeRatioDen(Num, Den);
  static constexpr bool kIsPositive = (num > 0) && (den > 0);
};

template <std::int64_t M, int E>
struct NumberRatio<Decimal<M, E>> {
 private:
  static constexpr std::int64_t kScaledNum =
      E >= 0 ? M * Pow10i(static_cast<unsigned>(E))
             : M;
  static constexpr std::int64_t kScaledDen =
      E >= 0 ? 1 : Pow10i(static_cast<unsigned>(-E));

 public:
  static constexpr std::int64_t num =
      NormalizeRatioNum(kScaledNum, kScaledDen);
  static constexpr std::int64_t den =
      NormalizeRatioDen(kScaledNum, kScaledDen);
  static constexpr bool kIsPositive = (num > 0) && (den > 0);
};

template <typename D>
struct DecimalTraits;

template <std::int64_t M, int E>
struct DecimalTraits<Decimal<M, E>> {
  static constexpr bool kIsNegative = (M < 0);
  using Abs = Decimal<(M < 0 ? -M : M), E>;
};

}  // namespace ae

#endif  // NUMERIC_DECIMAL_H_
