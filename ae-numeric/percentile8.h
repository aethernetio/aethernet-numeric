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

#ifndef AE_NUMERIC_PERCENTILE8_H_
#define AE_NUMERIC_PERCENTILE8_H_

// Purpose: Pack a percentile in [30, 99.99] U {100} into one byte by encoding
// the complementary tail probability (100 - percentile) with ae::Exponential.
//
// Motivation: soft-timeout / RTT reliability knobs need fine resolution near
// p99..p99.99 and coarser resolution near p30, without runtime floating point.
//
// Physical storage is the Exponential wire code for the tail percent. Code 0
// is the exact special value percentile=100 (tail=0). Positive codes map the
// geometric range [0.01, 70] percent of tail.

#include <cstddef>
#include <cstdint>
#include <optional>

#include "ae-numeric/exponential.h"
#include "ae-numeric/fixed_point.h"

namespace ae {

namespace percentile8_internal {

inline void Percentile8ConstantOutOfRange() {}

}  // namespace percentile8_internal

// Decoded tail magnitude in percent of probability mass beyond the percentile.
// Max 100 covers the declared tail domain [0.01, 70] with spare headroom.
// uint16 at Max=100 yields kScaleExp=-9 (1/512), which can plateau adjacent
// Exponential codes after Exp2 rounding; uint32 keeps decode strictly monotone
// while remaining well under the quantization of the 1-byte Exponential map.
using Percentile8TailRuntime = FixedPoint<std::uint32_t, 100.0>;

using Percentile8TailCodec =
    Exponential<Percentile8TailRuntime, std::uint8_t, 0.01, 70.0>;

// 1-byte timeout multiplier (declared Max=3.0 → typically Q2.6, step 1/64).
using TimeoutFactor8 = FixedPoint<std::uint8_t, 3.0>;

class Percentile8 {
 public:
  using TailRuntime = Percentile8TailRuntime;
  using TailCodec = Percentile8TailCodec;
  using wire_type = std::uint8_t;

  static constexpr double kMinPercent = 30.0;
  static constexpr double kMaxPercentExclusive = 99.99;
  static constexpr double kExactHundred = 100.0;
  static constexpr double kMinTailPercent = 0.01;
  static constexpr double kMaxTailPercent = 70.0;

  constexpr Percentile8() = default;

  static constexpr Percentile8 FromCode(wire_type code) {
    return Percentile8{code};
  }

  // Compile-time construction from a logical percentile percent.
  // Domain: [30.0, 99.99] U {100.0}. Out-of-range constants are ill-formed.
  static consteval Percentile8 FromPercent(double percentile) {
    if (percentile == kExactHundred) {
      return FromCode(0);
    }
    if (!(percentile >= kMinPercent && percentile <= kMaxPercentExclusive)) {
      percentile8_internal::Percentile8ConstantOutOfRange();
    }
    const double tail = kExactHundred - percentile;
    if (!(tail >= kMinTailPercent && tail <= kMaxTailPercent)) {
      percentile8_internal::Percentile8ConstantOutOfRange();
    }
    return FromCode(TailCodec::FromDouble(tail).CodeValue());
  }

  // Runtime encode from an already-decoded FixedPoint tail percent.
  // Accepts exact 0 (→ code 0 / p100) and [0.01, 70]; otherwise nullopt.
  static constexpr std::optional<Percentile8> TryFromTailPercent(
      TailRuntime tail) {
    constexpr auto zero = TailRuntime::FromInteger(0);
    constexpr auto min_tail = TailRuntime::FromDouble(0.01);
    constexpr auto max_tail = TailRuntime::FromDouble(70.0);
    if (tail == zero) {
      return FromCode(0);
    }
    if (tail < min_tail || tail > max_tail) {
      return std::nullopt;
    }
    return FromCode(TailCodec::FromRuntime(tail).CodeValue());
  }

  [[nodiscard]] constexpr wire_type Code() const { return code_; }

  [[nodiscard]] constexpr TailRuntime TailPercent() const {
    return TailCodec::FromCode(code_).ToRuntime();
  }

  [[nodiscard]] constexpr bool IsExactHundred() const { return code_ == 0; }

  constexpr bool operator==(Percentile8 other) const {
    return code_ == other.code_;
  }
  constexpr bool operator!=(Percentile8 other) const {
    return code_ != other.code_;
  }
  // Higher percentile ⇒ smaller tail code for positive codes; code 0 (p100)
  // is the highest logical percentile and compares greatest.
  constexpr bool operator<(Percentile8 other) const {
    if (code_ == other.code_) {
      return false;
    }
    if (code_ == 0) {
      return false;
    }
    if (other.code_ == 0) {
      return true;
    }
    return code_ > other.code_;
  }
  constexpr bool operator>(Percentile8 other) const { return other < *this; }
  constexpr bool operator<=(Percentile8 other) const { return !(*this > other); }
  constexpr bool operator>=(Percentile8 other) const { return !(*this < other); }

 private:
  explicit constexpr Percentile8(wire_type code) : code_{code} {}

  wire_type code_{0};
};

static_assert(sizeof(Percentile8) == 1);
static_assert(sizeof(TimeoutFactor8) == 1);
static_assert(Percentile8::FromPercent(100.0).Code() == 0);
static_assert(Percentile8::FromPercent(30.0).Code() != 0);
static_assert(Percentile8::FromPercent(99.9).Code() !=
              Percentile8::FromPercent(99.99).Code());

// Integer-only rank for a sorted sample buffer of size sample_count.
// Uses ceil(M * (1 - q)) = M - floor(M * q) with q = TailPercent/100.
// Returns an index in [0, sample_count) suitable for a 0-based sorted array.
[[nodiscard]] constexpr std::size_t PercentileIndex(std::size_t sample_count,
                                                    Percentile8 percentile) {
  if (sample_count == 0) {
    return 0;
  }
  if (sample_count == 1) {
    return 0;
  }

  const std::uint64_t m = static_cast<std::uint64_t>(sample_count - 1);
  if (percentile.IsExactHundred()) {
    return static_cast<std::size_t>(m);
  }

  using Tail = Percentile8::TailRuntime;
  static_assert(Tail::kScaleExp < 0,
                "PercentileIndex assumes fractional FixedPoint scale");
  constexpr int frac_bits = -Tail::kScaleExp;
  constexpr std::uint64_t scale = std::uint64_t{1} << frac_bits;
  constexpr std::uint64_t den = 100ull * scale;

  const std::uint64_t tail_raw =
      static_cast<std::uint64_t>(percentile.TailPercent().RawValue());
  // floor(M * tail_percent / 100) with nearest-free trunc for non-negative.
  const std::uint64_t floor_mq = (m * tail_raw) / den;
  return static_cast<std::size_t>(m - floor_mq);
}

// Integer percentile 0..100 rank: ceil(M * p / 100), no floating point.
[[nodiscard]] constexpr std::size_t PercentileIndexInteger(
    std::size_t sample_count, std::size_t percentile_0_100) {
  if (sample_count == 0) {
    return 0;
  }
  if (sample_count == 1 || percentile_0_100 == 0) {
    return 0;
  }
  if (percentile_0_100 >= 100) {
    return sample_count - 1;
  }
  const std::uint64_t m = static_cast<std::uint64_t>(sample_count - 1);
  // ceil(M * p / 100) = (M * p + 99) / 100
  return static_cast<std::size_t>((m * percentile_0_100 + 99ull) / 100ull);
}

}  // namespace ae

#endif  // AE_NUMERIC_PERCENTILE8_H_
