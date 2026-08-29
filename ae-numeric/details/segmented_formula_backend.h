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

#ifndef AE_NUMERIC_DETAILS_SEGMENTED_FORMULA_BACKEND_H_
#define AE_NUMERIC_DETAILS_SEGMENTED_FORMULA_BACKEND_H_

#include <cstdint>
#include <limits>

#include "ae-numeric/details/segmented_compiler.h"
#include "ae-numeric/details/segmented_math.h"
#include "ae-numeric/fixed_point.h"
#include "ae-numeric/integer_math.h"

namespace ae::seg::segmented_formula_internal {

using segmented_compiler_internal::CompiledSegment;
using SegPow = segmented_math_internal::SegPow;

constexpr std::int64_t ClampI64(std::int64_t v, std::int64_t lo,
                               std::int64_t hi) {
  if (v < lo) {
    return lo;
  }
  if (v > hi) {
    return hi;
  }
  return v;
}

template <typename T>
constexpr T FromI64Raw(std::int64_t raw) {
  auto const clamped = ClampI64(raw, static_cast<std::int64_t>(T::kRawMin),
                                static_cast<std::int64_t>(T::kRawMax));
  return T::FromRaw(
      fixed_point_internal::RepFromRawValue<typename T::rep_type>(
          static_cast<typename T::rep_value_type>(clamped)));
}

constexpr SegPow PowFromRatioRaw(std::int32_t ratio_raw) {
  return FromI64Raw<SegPow>(static_cast<std::int64_t>(ratio_raw));
}

// Exponentiation by squaring in SegPow. e == 0 => 1. O(log e) muls, no heap.
constexpr SegPow PowUint(SegPow base, unsigned e) {
  SegPow result = SegPow::FromRuntimeInteger(1);
  SegPow b = base;
  while (e > 0U) {
    if ((e & 1U) != 0U) {
      result = MulTo<SegPow>(result, b);
    }
    e >>= 1U;
    if (e > 0U) {
      b = MulTo<SegPow>(b, b);
    }
  }
  return result;
}

constexpr std::int64_t LerpRaw(std::int64_t a, std::int64_t b, int i, int n) {
  if (n <= 0 || i <= 0) {
    return a;
  }
  if (i >= n) {
    return b;
  }
  std::int64_t const diff = b - a;
  bool const neg = diff < 0;
  std::uint64_t out = 0;
  if (!integer_math::MulDivU64Nearest(integer_math::AbsI64ToU64(diff),
                                      static_cast<std::uint64_t>(i),
                                      static_cast<std::uint64_t>(n), out)) {
    return i >= n / 2 ? b : a;
  }
  auto const mag = static_cast<std::int64_t>(out);
  return neg ? a - mag : a + mag;
}

constexpr std::uint64_t RatioPowQ30(std::uint64_t base, unsigned e) {
  std::uint64_t const one =
      std::uint64_t{1} << segmented_math_internal::kSegRatioQ;
  std::uint64_t result = one;
  std::uint64_t b = base;
  while (e > 0U) {
    if ((e & 1U) != 0U) {
      std::uint64_t out = 0;
      if (!integer_math::MulDivU64Nearest(result, b, one, out)) {
        return std::numeric_limits<std::uint64_t>::max();
      }
      result = out;
    }
    e >>= 1U;
    if (e > 0U) {
      std::uint64_t out = 0;
      if (!integer_math::MulDivU64Nearest(b, b, one, out)) {
        return std::numeric_limits<std::uint64_t>::max();
      }
      b = out;
    }
  }
  return result;
}

template <typename RT>
constexpr std::int64_t ExpValueRaw(CompiledSegment const& s, int math_i) {
  if (math_i <= 0) {
    return s.curve_begin_raw;
  }
  if (math_i >= s.intervals) {
    return s.curve_end_raw;
  }
  if (s.ratio_raw <= 0 || s.curve_begin_raw <= 0) {
    return LerpRaw(s.curve_begin_raw, s.curve_end_raw, math_i, s.intervals);
  }
  std::uint64_t const p =
      RatioPowQ30(static_cast<std::uint64_t>(s.ratio_raw),
                  static_cast<unsigned>(math_i));
  std::uint64_t const one =
      std::uint64_t{1} << segmented_math_internal::kSegRatioQ;
  std::uint64_t out = 0;
  if (!integer_math::MulDivU64Nearest(
          integer_math::AbsI64ToU64(s.curve_begin_raw), p, one, out)) {
    return s.curve_end_raw;
  }
  if (out > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())) {
    return s.curve_end_raw;
  }
  return static_cast<std::int64_t>(out);
}

constexpr std::int64_t LinearValueRaw(CompiledSegment const& s, int math_i) {
  if (math_i <= 0) {
    return s.curve_begin_raw;
  }
  if (math_i >= s.intervals) {
    return s.curve_end_raw;
  }
  std::int64_t const n = math_i;
  std::int64_t raw = s.curve_begin_raw;
  raw += n * s.step0_raw;
  raw += s.delta_raw * n * (n - 1) / 2;
  return raw;
}

constexpr std::int64_t GeomValueRaw(CompiledSegment const& s, int math_i) {
  if (math_i <= 0) {
    return s.curve_begin_raw;
  }
  if (math_i >= s.intervals) {
    return s.curve_end_raw;
  }
  std::int64_t const span = s.curve_end_raw - s.curve_begin_raw;
  if (s.ratio_raw <= 0 || span == 0) {
    return LerpRaw(s.curve_begin_raw, s.curve_end_raw, math_i, s.intervals);
  }
  int const n = s.from_upper != 0 ? (s.intervals - math_i) : math_i;
  SegPow const q = PowFromRatioRaw(s.ratio_raw);
  SegPow const qn = PowUint(q, static_cast<unsigned>(n));
  SegPow const qN = PowUint(q, static_cast<unsigned>(s.intervals));
  std::int64_t const one =
      static_cast<std::int64_t>(SegPow::FromRuntimeInteger(1).RawValue());
  std::int64_t const num = static_cast<std::int64_t>(qn.RawValue()) - one;
  std::int64_t const den = static_cast<std::int64_t>(qN.RawValue()) - one;
  if (den == 0 || num < 0) {
    return LerpRaw(s.curve_begin_raw, s.curve_end_raw, math_i, s.intervals);
  }
  bool const neg = span < 0;
  std::uint64_t out = 0;
  if (!integer_math::MulDivU64Nearest(integer_math::AbsI64ToU64(span),
                                      integer_math::AbsI64ToU64(num),
                                      integer_math::AbsI64ToU64(den), out)) {
    return LerpRaw(s.curve_begin_raw, s.curve_end_raw, math_i, s.intervals);
  }
  auto const mag = static_cast<std::int64_t>(out);
  std::int64_t const offset = neg ? -mag : mag;
  if (s.from_upper != 0) {
    return s.curve_end_raw - offset;
  }
  return s.curve_begin_raw + offset;
}

template <typename RT>
constexpr std::int64_t DecodeMathRaw(CompiledSegment const& s, int math_i) {
  if (s.curve_kind == CurveKind::kExponentialValues) {
    return ExpValueRaw<RT>(s, math_i);
  }
  if (s.curve_kind == CurveKind::kGeometricStep) {
    return GeomValueRaw(s, math_i);
  }
  if (s.curve_kind == CurveKind::kLinearStepRamp) {
    return LinearValueRaw(s, math_i);
  }
  return LerpRaw(s.curve_begin_raw, s.curve_end_raw, math_i, s.intervals);
}

template <typename RT>
constexpr std::int64_t DecodeRankRaw(CompiledSegment const* segs, int nseg,
                                     std::uint32_t rank) {
  for (int i = 0; i < nseg; ++i) {
    CompiledSegment const& s = segs[i];
    if (rank >= s.wire_code_begin &&
        rank < s.wire_code_begin + s.code_count) {
      int const local = static_cast<int>(rank - s.wire_code_begin);
      int const math_i = s.math_first + local;
      return DecodeMathRaw<RT>(s, math_i);
    }
  }
  return segs[0].curve_begin_raw;
}

constexpr int LinearApproxIndex(CompiledSegment const& s, std::int64_t raw) {
  std::int64_t const span = s.curve_end_raw - s.curve_begin_raw;
  if (span == 0 || s.intervals <= 0) {
    return 0;
  }
  std::uint64_t out = 0;
  std::int64_t const pos = raw - s.curve_begin_raw;
  integer_math::MulDivU64Nearest(integer_math::AbsI64ToU64(pos),
                                 static_cast<std::uint64_t>(s.intervals),
                                 integer_math::AbsI64ToU64(span), out);
  return static_cast<int>(out);
}

template <typename RT>
constexpr int ClosestMathIndex(CompiledSegment const& s, std::int64_t raw) {
  int const first = s.math_first;
  int const last = s.math_first + static_cast<int>(s.code_count) - 1;
  if (last <= first) {
    return first;
  }
  int lo = first;
  int hi = last;
  while (lo < hi) {
    int const mid = lo + (hi - lo + 1) / 2;
    if (DecodeMathRaw<RT>(s, mid) <= raw) {
      lo = mid;
    } else {
      hi = mid - 1;
    }
  }
  return lo;
}

template <typename RT>
constexpr int ApproxIndex(CompiledSegment const& s, std::int64_t raw) {
  if (s.intervals <= 1) {
    return 0;
  }
  if (s.curve_kind == CurveKind::kExponentialValues ||
      s.curve_kind == CurveKind::kGeometricStep) {
    return ClosestMathIndex<RT>(s, raw);
  }
  if (s.curve_kind == CurveKind::kLinearStepRamp && s.delta_raw != 0) {
    std::int64_t const a = s.delta_raw;
    std::int64_t const b = 2 * s.step0_raw - s.delta_raw;
    std::int64_t const cc = 2 * (s.curve_begin_raw - raw);
    std::int64_t disc = b * b - 4 * a * cc;
    if (disc < 0) {
      disc = 0;
    }
    std::uint64_t const root =
        integer_math::SqrtU64(static_cast<std::uint64_t>(disc));
    std::int64_t const den = 2 * a;
    if (den == 0) {
      return 0;
    }
    std::int64_t const num = -b + static_cast<std::int64_t>(root);
    return static_cast<int>(num / den);
  }
  return LinearApproxIndex(s, raw);
}

template <typename RT>
constexpr std::uint32_t EncodeRaw(CompiledSegment const* segs, int nseg,
                                  std::uint32_t code_count, std::int64_t raw) {
  std::uint32_t best = 0;
  std::uint64_t best_d = std::numeric_limits<std::uint64_t>::max();
  for (int si = 0; si < nseg; ++si) {
    CompiledSegment const& s = segs[si];
    int approx = ApproxIndex<RT>(s, raw);
    int const last = s.math_first + static_cast<int>(s.code_count) - 1;
    if (approx < s.math_first) {
      approx = s.math_first;
    }
    if (approx > last) {
      approx = last;
    }
    int const lo_j = (approx < s.math_first + 4) ? s.math_first : (approx - 4);
    int const hi_j = (approx + 4 > last) ? last : (approx + 4);
    for (int j = lo_j; j <= hi_j; ++j) {
      std::int64_t const dec = DecodeMathRaw<RT>(s, j);
      std::uint64_t const d = integer_math::AbsI64ToU64(dec - raw);
      std::uint32_t const rank =
          s.wire_code_begin + static_cast<std::uint32_t>(j - s.math_first);
      if (d < best_d || (d == best_d && rank < best)) {
        best_d = d;
        best = rank;
      }
    }
  }
  if (best >= code_count) {
    return code_count - 1U;
  }
  return best;
}

}  // namespace ae::seg::segmented_formula_internal

#endif  // AE_NUMERIC_DETAILS_SEGMENTED_FORMULA_BACKEND_H_
