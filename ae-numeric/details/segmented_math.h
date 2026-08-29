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

#ifndef AE_NUMERIC_DETAILS_SEGMENTED_MATH_H_
#define AE_NUMERIC_DETAILS_SEGMENTED_MATH_H_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

#include <gcem.hpp>

#include "ae-numeric/decimal.h"
#include "ae-numeric/details/segmented_format.h"
#include "ae-numeric/fixed_point.h"
#include "ae-numeric/integer_math.h"

namespace ae::seg::segmented_math_internal {

struct SegFixedMathPolicy {
  using log_type = FixedPoint<std::int32_t, 16.0>;
  using mant_type = FixedPoint<std::uint16_t, 4.0>;
  using mul_intermediate_type = std::int64_t;
  static constexpr int kLogIterations = 16;
  static constexpr int kExp2FractionBits = 16;
};

inline void SegmentedSpecError() {}

// High-resolution ratio (r, q ≈ 1). Geometric decode uses SegPow
// exponentiation-by-squaring (max 256 covers q^N up to ~23). Exponential
// decode uses a Q30 integer ratio (kSegRatioQ) instead.
using SegPow = FixedPoint<std::uint32_t, 256.0>;

inline constexpr int kSegRatioQ = 30;

consteval std::int32_t RatioToQ30(double r) {
  double const scaled = r * static_cast<double>(std::uint64_t{1} << kSegRatioQ);
  if (scaled < 1.0 ||
      scaled > static_cast<double>(std::numeric_limits<std::int32_t>::max())) {
    SegmentedSpecError();
    return 0;
  }
  return static_cast<std::int32_t>(scaled + 0.5);
}

template <typename T>
consteval double ValueToDouble() {
  if constexpr (kIsDecimalV<T>) {
    double const mag = static_cast<double>(
        T::kMantissa < 0 ? -T::kMantissa : T::kMantissa);
    double const scaled =
        T::kExponent10 >= 0
            ? mag * static_cast<double>(Pow10u(static_cast<unsigned>(
                        T::kExponent10)))
            : mag / static_cast<double>(Pow10u(static_cast<unsigned>(
                        -T::kExponent10)));
    return T::kMantissa < 0 ? -scaled : scaled;
  } else if constexpr (kIsRatioV<T>) {
    return static_cast<double>(T::kNum) / static_cast<double>(T::kDen);
  } else {
    SegmentedSpecError();
    return 0.0;
  }
}

consteval double GeomSum(double q, int n) {
  if (n <= 0) {
    return 0.0;
  }
  if (gcem::abs(q - 1.0) < 1.0e-18) {
    return static_cast<double>(n);
  }
  return (gcem::pow(q, n) - 1.0) / (q - 1.0);
}

// Solve (q^n - 1)/(q-1) = S for q > 1.
consteval double SolveQForGeomSum(int n, double sum) {
  if (n <= 0 || sum <= 0.0) {
    SegmentedSpecError();
    return 1.0;
  }
  double lo = 1.0 + 1.0e-18;
  double hi = 2.0;
  for (int i = 0; i < 40 && GeomSum(hi, n) < sum; ++i) {
    hi *= 2.0;
  }
  for (int i = 0; i < 80; ++i) {
    double const mid = 0.5 * (lo + hi);
    if (GeomSum(mid, n) < sum) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  return 0.5 * (lo + hi);
}

consteval double ExpRatio(double begin, double end, int intervals) {
  if (begin <= 0.0 || end <= 0.0 || intervals <= 0) {
    SegmentedSpecError();
    return 1.0;
  }
  return gcem::pow(end / begin, 1.0 / static_cast<double>(intervals));
}

struct AutoSplitResult {
  int n1 = 0;
  int n2 = 0;
  double r1 = 1.0;
  double r2 = 1.0;
};

consteval AutoSplitResult AutoSplitTwoExp(double begin, double mid, double end,
                                          int total_intervals) {
  AutoSplitResult best{};
  bool have = false;
  double best_jump = 0.0;
  double best_err = 0.0;
  for (int n1 = 1; n1 < total_intervals; ++n1) {
    int const n2 = total_intervals - n1;
    double const r1 = ExpRatio(begin, mid, n1);
    double const r2 = ExpRatio(mid, end, n2);
    double const step_before = mid * (1.0 - 1.0 / r1);
    double const step_after = mid * (r2 - 1.0);
    double const smaller =
        step_before < step_after ? step_before : step_after;
    if (smaller <= 0.0) {
      continue;
    }
    double const jump = gcem::abs(step_after - step_before) / smaller;
    double const err1 = gcem::sqrt(r1) - 1.0;
    double const err2 = gcem::sqrt(r2) - 1.0;
    double const max_err = err1 > err2 ? err1 : err2;
    bool const better = !have || jump < best_jump - 1.0e-18 ||
                        (gcem::abs(jump - best_jump) <= 1.0e-18 &&
                         (max_err < best_err - 1.0e-18 ||
                          (gcem::abs(max_err - best_err) <= 1.0e-18 &&
                           n1 < best.n1)));
    if (better) {
      have = true;
      best_jump = jump;
      best_err = max_err;
      best.n1 = n1;
      best.n2 = n2;
      best.r1 = r1;
      best.r2 = r2;
    }
  }
  if (!have) {
    SegmentedSpecError();
  }
  return best;
}

struct ContExpResult {
  int intervals = 0;
  int last_1 = 0;
  int last_2 = 0;
  double r = 1.0;
};

consteval double LogErr(double got, double want) {
  return gcem::abs(gcem::log(got / want));
}

consteval int RoundNearestNonneg(double x) {
  if (x < 0.0) {
    return 0;
  }
  return static_cast<int>(x + 0.5);
}

// Fill the 1-byte tier (last code = last_1_max, typically 254), then choose
// total intervals so nearest-code region boundaries sit as close as possible
// to the requested physical cuts.
consteval ContExpResult OptimizeContinuousExp(double vmin, double vmax,
                                              double cut1, double cut2,
                                              int last_1_max, int min_n,
                                              int max_n, int max_last_2) {
  ContExpResult best{};
  bool have = false;
  double best_cut = 0.0;
  double best_rel = 0.0;
  int const i1 = last_1_max;
  for (int n = min_n; n <= max_n; ++n) {
    if (n <= i1 + 1) {
      continue;
    }
    double const r = ExpRatio(vmin, vmax, n);
    double const span_log = gcem::log(vmax / vmin);
    double const i2f = static_cast<double>(n) * gcem::log(cut2 / vmin) /
                           span_log -
                       0.5;
    int const i2 = RoundNearestNonneg(i2f);
    if (i2 <= i1 || i2 >= n || i2 > max_last_2) {
      continue;
    }
    double const b1 = vmin * gcem::pow(r, static_cast<double>(i1) + 0.5);
    double const b2 = vmin * gcem::pow(r, static_cast<double>(i2) + 0.5);
    double const cut =
        LogErr(b1, cut1) > LogErr(b2, cut2) ? LogErr(b1, cut1) : LogErr(b2, cut2);
    double const rel = gcem::sqrt(r) - 1.0;
    bool const better =
        !have || cut < best_cut - 1.0e-18 ||
        (gcem::abs(cut - best_cut) <= 1.0e-18 &&
         (rel < best_rel - 1.0e-18 ||
          (gcem::abs(rel - best_rel) <= 1.0e-18 && n < best.intervals)));
    if (better) {
      have = true;
      best_cut = cut;
      best_rel = rel;
      best.intervals = n;
      best.last_1 = i1;
      best.last_2 = i2;
      best.r = r;
    }
  }
  if (!have) {
    SegmentedSpecError();
  }
  return best;
}

consteval std::uint64_t TwoTierMaxU8(std::uint32_t b0) {
  return (255ULL - b0 - 1ULL) * 256ULL + b0 + 1ULL + 255ULL;
}

consteval std::uint64_t ThreeTierMaxU8(std::uint32_t b0, std::uint32_t b1) {
  std::uint64_t const two = TwoTierMaxU8(b0);
  constexpr std::uint64_t kWord2 = 65536ULL;
  return (two - b1 - 1ULL) * kWord2 + b1 + 1ULL + (kWord2 - 1ULL);
}

consteval int CeilPositive(double x) {
  int const i = static_cast<int>(x);
  if (static_cast<double>(i) < x) {
    return i + 1;
  }
  return i;
}

consteval int MinRampIntervals(double span, double step0, double max_err) {
  if (span <= 0.0 || step0 <= 0.0 || max_err <= 0.0) {
    SegmentedSpecError();
    return 1;
  }
  double const last = 2.0 * max_err;
  double const den = step0 + last;
  if (den <= 0.0) {
    SegmentedSpecError();
    return 1;
  }
  int n = CeilPositive(2.0 * span / den);
  if (n < 1) {
    n = 1;
  }
  return n;
}

consteval std::uint64_t MixHash(std::uint64_t h, std::uint64_t v) {
  h ^= v;
  h *= 1099511628211ULL;
  return h;
}

}  // namespace ae::seg::segmented_math_internal

#endif  // AE_NUMERIC_DETAILS_SEGMENTED_MATH_H_
