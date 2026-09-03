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

#ifndef AE_NUMERIC_PERCENTILE_H_
#define AE_NUMERIC_PERCENTILE_H_

// Purpose: Runtime-only percentile policy value stored as the complementary
// tail probability (100 - percentile) in a 16-bit FixedPoint.
//
// Logical domain: percentile in [30.0, 100.0] ⇒ tail in [0.0, 70.0].
// No Exponential / Log2 / Exp2 — storage is the FixedPoint tail itself.

#include <cstddef>
#include <cstdint>
#include <optional>

#include "ae-numeric/fixed_point.h"

namespace ae {

namespace percentile_internal {

inline void PercentileConstantOutOfRange() {}

}  // namespace percentile_internal

// Tail percent of probability mass beyond the percentile (0 = exact p100).
using PercentileTail = FixedPoint<std::uint16_t, 100.0>;

// 1-byte timeout multiplier (declared Max=3.0 → typically Q2.6, step 1/64).
using TimeoutFactor8 = FixedPoint<std::uint8_t, 3.0>;

class Percentile {
 public:
  using tail_type = PercentileTail;

  static constexpr double kMinPercent = 30.0;
  static constexpr double kMaxPercent = 100.0;
  static constexpr double kMaxTailPercent = 70.0;

  constexpr Percentile() = default;

  // Compile-time construction from a logical percentile percent.
  // Domain: [30.0, 100.0]. Out-of-range constants are ill-formed.
  static consteval Percentile FromPercent(double percentile) {
    if (!(percentile >= kMinPercent && percentile <= kMaxPercent)) {
      percentile_internal::PercentileConstantOutOfRange();
    }
    const double tail = kMaxPercent - percentile;
    if (!(tail >= 0.0 && tail <= kMaxTailPercent)) {
      percentile_internal::PercentileConstantOutOfRange();
    }
    return Percentile{tail_type::FromDouble(tail)};
  }

  // Runtime encode from an already-decoded FixedPoint tail percent.
  // Accepts [0, 70]; otherwise nullopt (no silent clamp).
  static constexpr std::optional<Percentile> TryFromTailPercent(
      tail_type tail) {
    constexpr auto max_tail = tail_type::FromInteger(70);
    if (tail > max_tail) {
      return std::nullopt;
    }
    return Percentile{tail};
  }

  [[nodiscard]] constexpr tail_type TailPercent() const { return tail_; }

  [[nodiscard]] constexpr bool IsExactHundred() const {
    return tail_ == tail_type::FromInteger(0);
  }

  constexpr bool operator==(Percentile other) const {
    return tail_ == other.tail_;
  }
  constexpr bool operator!=(Percentile other) const {
    return tail_ != other.tail_;
  }
  // Higher percentile ⇒ smaller tail.
  constexpr bool operator<(Percentile other) const {
    return tail_ > other.tail_;
  }
  constexpr bool operator>(Percentile other) const { return other < *this; }
  constexpr bool operator<=(Percentile other) const {
    return !(*this > other);
  }
  constexpr bool operator>=(Percentile other) const {
    return !(*this < other);
  }

 private:
  explicit constexpr Percentile(tail_type tail) : tail_{tail} {}

  tail_type tail_{};
};

static_assert(sizeof(Percentile) == 2);
static_assert(sizeof(TimeoutFactor8) == 1);
static_assert(Percentile::FromPercent(100.0).IsExactHundred());
static_assert(Percentile::FromPercent(99.9) !=
              Percentile::FromPercent(99.99));
static_assert(Percentile::FromPercent(30.0) < Percentile::FromPercent(100.0));

// Integer-only rank for a sorted sample buffer of size sample_count.
// Uses ceil(M * (1 - q)) = M - floor(M * q) with q = TailPercent/100.
// Returns an index in [0, sample_count) suitable for a 0-based sorted array.
[[nodiscard]] constexpr std::size_t PercentileIndex(std::size_t sample_count,
                                                    Percentile percentile) {
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

  using Tail = Percentile::tail_type;
  static_assert(Tail::kScaleExp < 0,
                "PercentileIndex assumes fractional FixedPoint scale");
  constexpr int frac_bits = -Tail::kScaleExp;
  constexpr std::uint64_t scale = std::uint64_t{1} << frac_bits;
  constexpr std::uint64_t den = 100ull * scale;

  const std::uint64_t tail_raw =
      static_cast<std::uint64_t>(percentile.TailPercent().RawValue());
  // floor(M * tail_percent / 100) with trunc for non-negative.
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

#endif  // AE_NUMERIC_PERCENTILE_H_
