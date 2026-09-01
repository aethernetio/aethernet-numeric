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

// Purpose: Provide the runtime analytical encode/decode implementation
// for compiled SegmentedNumber formats.
//
// Motivation: embedded codecs must avoid per-code lookup tables and
// searches whose cost grows with the number of representable values.
//
// Analogous concept: an analytical inverse/forward quantizer backend.
// Implementation lives in ae::seg::detail, selected by compute::Formula.
//
// How it works: uniform curves use arithmetic, exponential and geometric
// curves use FixedPoint Log2/Exp2, and step ramps use a quadratic inverse
// with 32-bit integer sqrt. An analytical index estimate is followed by
// a small bounded neighbor check to select the nearest canonical rank.
// No heap or rank-indexed value table is used.


#include <cassert>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <utility>

#include "ae-numeric/details/segmented_compiler.h"
#include "ae-numeric/details/segmented_math.h"
#include "ae-numeric/fixed_math.h"
#include "ae-numeric/fixed_point.h"
#include "ae-numeric/integer_math.h"

namespace ae::seg::detail {

#if defined(_MSC_VER)
#define AE_SEG_NOINLINE __declspec(noinline)
#else
#define AE_SEG_NOINLINE __attribute__((noinline))
#endif

inline constexpr int kNeighborRadius = 3;
inline constexpr int kRampRefineRadius = 2;

using StoredRaw = std::uint32_t;
using WorkRaw = std::int32_t;

template <typename RT>
constexpr typename RT::rep_value_type UnpackRtRaw(StoredRaw v) {
  using RV = typename RT::rep_value_type;
  if constexpr (kRtSigned<RT>) {
    return static_cast<RV>(static_cast<std::int32_t>(v));
  } else {
    return static_cast<RV>(v);
  }
}

template <bool kSigned>
constexpr bool StoredLess(StoredRaw a, StoredRaw b) {
  if constexpr (kSigned) {
    return static_cast<std::int32_t>(a) < static_cast<std::int32_t>(b);
  } else {
    return a < b;
  }
}

template <bool kSigned>
constexpr std::uint32_t AbsStoredDiff(StoredRaw a, StoredRaw b) {
  if constexpr (kSigned) {
    auto const ia = static_cast<std::int32_t>(a);
    auto const ib = static_cast<std::int32_t>(b);
    auto const ua = static_cast<std::uint32_t>(ia);
    auto const ub = static_cast<std::uint32_t>(ib);
    return ia >= ib ? ua - ub : ub - ua;
  } else {
    return a >= b ? a - b : b - a;
  }
}

template <typename T>
constexpr T FromStoredRaw(StoredRaw raw) {
  return T::FromRaw(
      fixed_point_internal::RepFromRawValue<typename T::rep_type>(
          UnpackRtRaw<T>(raw)));
}

template <typename RT>
constexpr SegWork WorkFromStored(StoredRaw raw) {
  return ConvertFixed<SegWork>(FromStoredRaw<RT>(raw));
}

template <typename RT>
constexpr WorkRaw DeltaToWork(StoredRaw packed) {
  auto const src = static_cast<std::int32_t>(packed);
  return fixed_point_internal::ConvertRawScaleTo<WorkRaw>(
      src, RT::kScaleExp, SegWork::kScaleExp, SegWork::kRawMin,
      SegWork::kRawMax);
}

constexpr int RoundDivLog(SegLog num, SegLog den) {
  auto const d = den.RawValue();
  if (d == static_cast<typename SegLog::rep_value_type>(0)) {
    return 0;
  }
  return static_cast<int>(
      fixed_point_internal::RoundDivNearest(num.RawValue(), d));
}

template <bool kSigned>
constexpr StoredRaw AddMag(StoredRaw base, std::uint32_t mag, bool subtract) {
  if constexpr (kSigned) {
    auto const b = static_cast<std::int32_t>(base);
    auto const m = static_cast<std::int32_t>(mag);
    return static_cast<StoredRaw>(subtract ? b - m : b + m);
  } else {
    return subtract ? base - mag : base + mag;
  }
}

template <bool kSigned>
constexpr StoredRaw LerpRaw(StoredRaw a, StoredRaw b, int i, int n) {
  if (n <= 0 || i <= 0) {
    return a;
  }
  if (i >= n) {
    return b;
  }
  bool const neg = StoredLess<kSigned>(b, a);
  std::uint32_t out = 0;
  if (!integer_math::MulDivU32Nearest(AbsStoredDiff<kSigned>(a, b),
                                      static_cast<std::uint32_t>(i),
                                      static_cast<std::uint32_t>(n), out)) {
    return i >= n / 2 ? b : a;
  }
  return AddMag<kSigned>(a, out, neg);
}

template <bool kSigned>
constexpr StoredRaw AvoidEndpointCollision(StoredRaw raw, StoredRaw begin,
                                           StoredRaw end, int math_i,
                                           int intervals) {
  if (math_i <= 0 || math_i >= intervals) {
    return raw;
  }
  bool const rising = !StoredLess<kSigned>(end, begin);
  if (raw == end) {
    return AddMag<kSigned>(raw, 1U, rising);
  }
  if (raw == begin) {
    return AddMag<kSigned>(raw, 1U, !rising);
  }
  return raw;
}

template <typename RT>
constexpr StoredRaw Exp2PosToStored(SegLog arg) {
  SegPosWork const w = Exp2Pos(arg);
  using RV = typename RT::rep_value_type;
  RV const aligned = fixed_point_internal::ConvertRawScaleTo<RV>(
      w.RawValue(), SegPosWork::kScaleExp, RT::kScaleExp, RT::kRawMin,
      RT::kRawMax);
  return PackRtRaw<RT>(aligned);
}

template <bool kSigned>
constexpr StoredRaw LinearValueAt(StoredRaw begin, StoredRaw end,
                                  WorkRaw step0_w, WorkRaw delta_w, int math_i,
                                  int intervals) {
  if (math_i <= 0) {
    return begin;
  }
  if (math_i >= intervals) {
    return end;
  }
  std::uint32_t const n = static_cast<std::uint32_t>(math_i);
  std::uint32_t const n_all = static_cast<std::uint32_t>(intervals);
  std::uint32_t const pair = n * (n - 1U) / 2U;
  std::uint32_t const den_pair = n_all * (n_all - 1U) / 2U;
  std::uint32_t num_a = 0;
  std::uint32_t num_b = 0;
  std::uint32_t den_a = 0;
  std::uint32_t den_b = 0;
  if (!integer_math::MulU32Checked(n, integer_math::AbsI32ToU32(step0_w),
                                   num_a) ||
      !integer_math::MulU32Checked(integer_math::AbsI32ToU32(delta_w), pair,
                                   num_b) ||
      !integer_math::MulU32Checked(n_all, integer_math::AbsI32ToU32(step0_w),
                                   den_a) ||
      !integer_math::MulU32Checked(integer_math::AbsI32ToU32(delta_w), den_pair,
                                   den_b)) {
    return end;
  }
  std::uint32_t num = 0;
  std::uint32_t den = 0;
  bool const step_delta_same = (step0_w < 0) == (delta_w < 0);
  if (step_delta_same) {
    if (!integer_math::AddU32Checked(num_a, num_b, num) ||
        !integer_math::AddU32Checked(den_a, den_b, den)) {
      return end;
    }
  } else {
    num = num_a >= num_b ? num_a - num_b : num_b - num_a;
    den = den_a >= den_b ? den_a - den_b : den_b - den_a;
  }
  if (den == 0U) {
    return begin;
  }
  std::uint32_t out = 0;
  if (!integer_math::MulDivU32Nearest(AbsStoredDiff<kSigned>(end, begin), num,
                                      den, out)) {
    return end;
  }
  return AddMag<kSigned>(begin, out, StoredLess<kSigned>(end, begin));
}

template <bool kSigned>
constexpr StoredRaw GeomValueAt(StoredRaw begin, StoredRaw end, SegLog log_q,
                                int math_i, int intervals, bool from_upper) {
  if (math_i <= 0) {
    return begin;
  }
  if (math_i >= intervals) {
    return end;
  }
  int const k = from_upper ? (intervals - math_i) : math_i;
  SegPosWork const w = GeomUnitWeightPos(log_q, k, intervals);
  SegPosWork const one = SegPosWork::FromRuntimeInteger(1);
  std::uint32_t off = 0;
  integer_math::MulDivU32Nearest(AbsStoredDiff<kSigned>(end, begin),
                                 w.RawValue(), one.RawValue(), off);
  bool const neg = StoredLess<kSigned>(end, begin);
  if (from_upper) {
    return AddMag<kSigned>(end, off, !neg);
  }
  return AddMag<kSigned>(begin, off, neg);
}

template <typename RT>
constexpr StoredRaw ExpValueAt(StoredRaw begin, StoredRaw end,
                               SegLog log2_begin, SegLog log2_r, int math_i,
                               int intervals) {
  if (math_i <= 0) {
    return begin;
  }
  if (math_i >= intervals) {
    return end;
  }
  SegLog const arg = AddTo<SegLog>(log2_begin, MulLogInt(log2_r, math_i));
  return Exp2PosToStored<RT>(arg);
}

template <bool kSigned>
constexpr int LinearApproxAt(StoredRaw begin, StoredRaw end, int intervals,
                             StoredRaw raw) {
  if (begin == end || intervals <= 0) {
    return 0;
  }
  std::uint32_t out = 0;
  integer_math::MulDivU32Nearest(AbsStoredDiff<kSigned>(raw, begin),
                                 static_cast<std::uint32_t>(intervals),
                                 AbsStoredDiff<kSigned>(end, begin), out);
  return static_cast<int>(out);
}

constexpr int ExpApproxPos(SegPosWork x, SegLog lr, SegLog lb) {
  if (x.RawValue() == static_cast<typename SegPosWork::rep_value_type>(0)) {
    return 0;
  }
  return RoundDivLog(SubTo<SegLog>(Log2Pos(x), lb), lr);
}

AE_SEG_NOINLINE constexpr int GeomApproxWork(SegWork x, SegWork begin,
                                             SegWork end, SegLog log_q,
                                             int intervals, int math_first,
                             bool from_upper) {
  if (log_q.RawValue() <= static_cast<typename SegLog::rep_value_type>(0)) {
    return 0;
  }
  SegWork const span = SubTo<SegWork>(end, begin);
  if (span.RawValue() == static_cast<typename SegWork::rep_value_type>(0)) {
    return math_first;
  }
  SegWork pos = from_upper ? SubTo<SegWork>(end, x) : SubTo<SegWork>(x, begin);
  SegWork const abs_span = WorkAbs(span);
  if (pos.RawValue() < static_cast<typename SegWork::rep_value_type>(0)) {
    pos = WorkZero();
  } else if (pos.RawValue() > abs_span.RawValue()) {
    pos = abs_span;
  }
  SegWork const t = DivTo<SegWork>(pos, abs_span);
  SegWork const qn_m1 = Exp2MinusOne(MulLogInt(log_q, intervals));
  SegWork const arg = AddTo<SegWork>(WorkOne(), MulWorkSame(t, qn_m1));
  if (!WorkPositive(arg)) {
    return from_upper ? intervals : 0;
  }
  int const k = RoundDivLog(Log2Work(arg), log_q);
  if (from_upper) {
    return intervals - k;
  }
  return k;
}

constexpr std::uint32_t LinearShapeMag(int i, std::int32_t step0,
                                       std::int32_t delta) {
  if (i <= 0) {
    return 0;
  }
  std::uint32_t const p =
      static_cast<std::uint32_t>(i) * static_cast<std::uint32_t>(i - 1) / 2U;
  std::uint32_t a = 0;
  std::uint32_t b = 0;
  if (!integer_math::MulU32Checked(static_cast<std::uint32_t>(i),
                                   integer_math::AbsI32ToU32(step0), a) ||
      !integer_math::MulU32Checked(integer_math::AbsI32ToU32(delta), p, b)) {
    return std::numeric_limits<std::uint32_t>::max();
  }
  std::uint32_t s = 0;
  bool const same = (step0 < 0) == (delta < 0);
  if (same) {
    if (!integer_math::AddU32Checked(a, b, s)) {
      return std::numeric_limits<std::uint32_t>::max();
    }
  } else {
    s = a >= b ? a - b : b - a;
  }
  return s;
}

constexpr bool DiscWide(std::uint32_t A, std::uint32_t B, std::uint32_t T,
                        bool at_neg, std::uint32_t& dhi, std::uint32_t& dlo) {
  std::uint32_t bb_hi = 0;
  std::uint32_t bb_lo = 0;
  integer_math::MulU32Wide(B, B, bb_hi, bb_lo);
  std::uint32_t at_hi = 0;
  std::uint32_t at_lo = 0;
  integer_math::MulU32Wide(A, T, at_hi, at_lo);
  if (!integer_math::ShlU32WideChecked(at_hi, at_lo, 3U)) {
    return false;
  }
  if (at_neg) {
    if (bb_hi < at_hi || (bb_hi == at_hi && bb_lo < at_lo)) {
      return false;
    }
    bool const borrow = bb_lo < at_lo;
    dlo = bb_lo - at_lo;
    dhi = bb_hi - at_hi - (borrow ? 1U : 0U);
    return true;
  }
  dlo = bb_lo + at_lo;
  std::uint32_t const carry = dlo < bb_lo ? 1U : 0U;
  dhi = bb_hi + at_hi + carry;
  if (dhi < bb_hi) {
    return false;
  }
  return true;
}

template <bool kSigned>
AE_SEG_NOINLINE constexpr int LinearRampApproxRuntime(
    StoredRaw begin, StoredRaw end, std::int32_t step0, std::int32_t delta,
    int intervals, StoredRaw raw) {
  if (delta == 0 || intervals <= 0) {
    return LinearApproxAt<kSigned>(begin, end, intervals, raw);
  }
  if (begin == end) {
    return 0;
  }
  std::uint32_t const n_all = static_cast<std::uint32_t>(intervals);
  std::uint32_t const pair = n_all * (n_all - 1U) / 2U;
  std::uint32_t abs_step0_n = 0;
  std::uint32_t abs_delta_pair = 0;
  if (!integer_math::MulU32Checked(n_all, integer_math::AbsI32ToU32(step0),
                                   abs_step0_n) ||
      !integer_math::MulU32Checked(integer_math::AbsI32ToU32(delta), pair,
                                   abs_delta_pair)) {
    return LinearApproxAt<kSigned>(begin, end, intervals, raw);
  }
  std::uint32_t abs_den = 0;
  if ((step0 < 0) == (delta < 0)) {
    if (!integer_math::AddU32Checked(abs_step0_n, abs_delta_pair, abs_den)) {
      return LinearApproxAt<kSigned>(begin, end, intervals, raw);
    }
  } else if (abs_step0_n >= abs_delta_pair) {
    abs_den = abs_step0_n - abs_delta_pair;
  } else {
    abs_den = abs_delta_pair - abs_step0_n;
  }
  if (abs_den == 0U) {
    return LinearApproxAt<kSigned>(begin, end, intervals, raw);
  }
  std::uint32_t T = 0;
  if (!integer_math::MulDivU32Nearest(
          abs_den, AbsStoredDiff<kSigned>(raw, begin),
          AbsStoredDiff<kSigned>(end, begin), T)) {
    return LinearApproxAt<kSigned>(begin, end, intervals, raw);
  }
  std::uint32_t const T_full = T;
  std::uint32_t A = integer_math::AbsI32ToU32(delta);
  std::int32_t const two_b = step0 + step0 - delta;
  std::uint32_t B = integer_math::AbsI32ToU32(two_b);
  bool const at_neg =
      (delta < 0) != (StoredLess<kSigned>(raw, begin) !=
                      StoredLess<kSigned>(end, begin));
  std::uint32_t dhi = 0;
  std::uint32_t dlo = 0;
  int guard = 0;
  while (!DiscWide(A, B, T, at_neg, dhi, dlo) && guard < 32) {
    A >>= 1U;
    B >>= 1U;
    T >>= 1U;
    ++guard;
    if ((A | B | T) == 0U) {
      return LinearApproxAt<kSigned>(begin, end, intervals, raw);
    }
  }
  if (!DiscWide(A, B, T, at_neg, dhi, dlo)) {
    return LinearApproxAt<kSigned>(begin, end, intervals, raw);
  }
  std::uint32_t const root = integer_math::SqrtU32Wide(dhi, dlo);
  std::uint32_t den = 0;
  if (!integer_math::AddU32Checked(B, root, den) || den == 0U) {
    return LinearApproxAt<kSigned>(begin, end, intervals, raw);
  }
  std::uint32_t n_u = 0;
  if (!integer_math::MulDivU32Nearest(T, 4U, den, n_u) ||
      n_u > static_cast<std::uint32_t>(std::numeric_limits<int>::max() / 2)) {
    return LinearApproxAt<kSigned>(begin, end, intervals, raw);
  }
  int n = static_cast<int>(n_u);
  int best = n;
  std::uint32_t best_e = std::numeric_limits<std::uint32_t>::max();
  for (int d = -kRampRefineRadius; d <= kRampRefineRadius; ++d) {
    int const cand = n + d;
    std::uint32_t const shape = LinearShapeMag(cand, step0, delta);
    std::uint32_t const e = shape >= T_full ? shape - T_full : T_full - shape;
    if (e < best_e) {
      best_e = e;
      best = cand;
    }
  }
  return best;
}

struct SegScalars {
  CurveKind kind = CurveKind::kUniformStep;
  std::uint32_t wire_begin = 0;
  std::uint32_t code_count = 0;
  int intervals = 0;
  int math_first = 0;
  StoredRaw begin = 0;
  StoredRaw end = 0;
  StoredRaw step0 = 0;
  StoredRaw delta = 0;
  SegLog log2_r = LogZero();
  SegLog log2_q = LogZero();
  SegLog log2_begin = LogZero();
  std::uint8_t from_upper = 0;
};

template <typename Spec, typename RT, int I>
consteval SegScalars MakeSegScalars() {
  CompiledSegment const p =
      detail::CompiledSegHolder<Spec, RT>::kAll
          [static_cast<std::size_t>(I)];
  SegScalars s{};
  s.kind = p.curve_kind;
  s.wire_begin = p.wire_code_begin;
  s.code_count = p.code_count;
  s.intervals = p.intervals;
  s.math_first = p.math_first;
  s.begin = p.curve_begin_raw;
  s.end = p.curve_end_raw;
  s.step0 = p.step0_raw;
  s.delta = p.delta_raw;
  s.log2_r = p.log2_r;
  s.log2_q = p.log2_q;
  s.log2_begin = p.log2_begin;
  s.from_upper = p.from_upper;
  return s;
}

template <typename Spec, typename RT, int I>
struct SegN {
  static constexpr SegScalars k = MakeSegScalars<Spec, RT, I>();
  static constexpr bool kSigned = kRtSigned<RT>;
  static constexpr CurveKind kKind = k.kind;
  static constexpr std::uint32_t kWireBegin = k.wire_begin;
  static constexpr std::uint32_t kCodeCount = k.code_count;
  static constexpr int kIntervals = k.intervals;
  static constexpr int kMathFirst = k.math_first;
  static constexpr StoredRaw kBegin = k.begin;
  static constexpr StoredRaw kEnd = k.end;
  static constexpr StoredRaw kStep0 = k.step0;
  static constexpr StoredRaw kDelta = k.delta;
  static constexpr WorkRaw kStep0W = DeltaToWork<RT>(k.step0);
  static constexpr WorkRaw kDeltaW = DeltaToWork<RT>(k.delta);
  static constexpr SegLog kLog2R = k.log2_r;
  static constexpr SegLog kLog2Q = k.log2_q;
  static constexpr SegLog kLog2Begin = k.log2_begin;
  static constexpr std::uint8_t kFromUpper = k.from_upper;
};

template <typename RT>
constexpr StoredRaw RtToStored(typename RT::rep_value_type v) {
  return PackRtRaw<RT>(v);
}

template <typename RT>
constexpr typename RT::rep_value_type StoredToRt(StoredRaw v) {
  return UnpackRtRaw<RT>(v);
}

template <typename RT, typename S>
constexpr StoredRaw DecodeMath(int math_i) {
  StoredRaw raw = 0;
  if constexpr (S::kKind == CurveKind::kExponentialValues) {
    raw = ExpValueAt<RT>(S::kBegin, S::kEnd, S::kLog2Begin, S::kLog2R, math_i,
                         S::kIntervals);
  } else if constexpr (S::kKind == CurveKind::kGeometricStep) {
    raw = GeomValueAt<S::kSigned>(S::kBegin, S::kEnd, S::kLog2Q, math_i,
                                 S::kIntervals, S::kFromUpper != 0);
  } else if constexpr (S::kKind == CurveKind::kLinearStepRamp) {
    raw = LinearValueAt<S::kSigned>(S::kBegin, S::kEnd, S::kStep0W, S::kDeltaW,
                                   math_i, S::kIntervals);
  } else {
    raw = LerpRaw<S::kSigned>(S::kBegin, S::kEnd, math_i, S::kIntervals);
  }
  return AvoidEndpointCollision<S::kSigned>(raw, S::kBegin, S::kEnd, math_i,
                                            S::kIntervals);
}

template <typename RT, typename S>
constexpr int ApproxIndex(StoredRaw raw) {
  if (S::kIntervals <= 1) {
    return 0;
  }
  if constexpr (S::kKind == CurveKind::kExponentialValues) {
    SegLog const lr = S::kLog2R;
    if (lr.RawValue() == static_cast<typename SegLog::rep_value_type>(0)) {
      return LinearApproxAt<S::kSigned>(S::kBegin, S::kEnd, S::kIntervals, raw);
    }
    return ExpApproxPos(ConvertFixed<SegPosWork>(FromStoredRaw<RT>(raw)), lr,
                        S::kLog2Begin);
  } else if constexpr (S::kKind == CurveKind::kGeometricStep) {
    return GeomApproxWork(WorkFromStored<RT>(raw),
                          WorkFromStored<RT>(S::kBegin),
                          WorkFromStored<RT>(S::kEnd), S::kLog2Q, S::kIntervals,
                          S::kMathFirst, S::kFromUpper != 0);
  } else if constexpr (S::kKind == CurveKind::kLinearStepRamp) {
    return LinearRampApproxRuntime<S::kSigned>(
        S::kBegin, S::kEnd, static_cast<std::int32_t>(S::kStep0),
        static_cast<std::int32_t>(S::kDelta), S::kIntervals, raw);
  } else {
    return LinearApproxAt<S::kSigned>(S::kBegin, S::kEnd, S::kIntervals, raw);
  }
}

template <typename RT, typename S>
constexpr void ConsiderRank(std::uint32_t& best, std::uint32_t& best_d,
                            int last, int j, StoredRaw raw) {
  if (j < S::kMathFirst || j > last) {
    return;
  }
  StoredRaw const dec = DecodeMath<RT, S>(j);
  std::uint32_t const d = AbsStoredDiff<S::kSigned>(dec, raw);
  std::uint32_t const rank =
      S::kWireBegin + static_cast<std::uint32_t>(j - S::kMathFirst);
  if (d < best_d || (d == best_d && rank < best)) {
    best_d = d;
    best = rank;
  }
}

template <typename Spec, typename RT, int I>
constexpr bool RankInSeg(std::uint32_t rank, StoredRaw& out) {
  using S = SegN<Spec, RT, I>;
  if (rank >= S::kWireBegin && rank < S::kWireBegin + S::kCodeCount) {
    int const local = static_cast<int>(rank - S::kWireBegin);
    out = DecodeMath<RT, S>(S::kMathFirst + local);
    return true;
  }
  return false;
}

template <typename Spec, typename RT>
constexpr StoredRaw DecodeRankSeq(std::uint32_t /*rank*/,
                                  std::integer_sequence<int>) {
  using S0 = SegN<Spec, RT, 0>;
  // Invalid rank is a caller contract violation; keep a debug assert only.
  // Do not call std::abort() — that would pull a libc dependency into the
  // mathematical footprint objects on bare-metal targets.
  assert(false && "invalid segmented rank");
  return S0::kBegin;
}

template <typename Spec, typename RT, int I, int... Rest>
constexpr StoredRaw DecodeRankSeq(std::uint32_t rank,
                                  std::integer_sequence<int, I, Rest...>) {
  StoredRaw out = 0;
  if (RankInSeg<Spec, RT, I>(rank, out)) {
    return out;
  }
  return DecodeRankSeq<Spec, RT>(rank, std::integer_sequence<int, Rest...>{});
}

template <typename Spec, typename RT, int I>
constexpr void AccSegRange(StoredRaw& mn, StoredRaw& mx) {
  using S = SegN<Spec, RT, I>;
  int const first = S::kMathFirst;
  int const last = first + static_cast<int>(S::kCodeCount) - 1;
  StoredRaw const a = DecodeMath<RT, S>(first);
  StoredRaw const b = DecodeMath<RT, S>(last);
  if (StoredLess<kRtSigned<RT>>(a, mn)) {
    mn = a;
  }
  if (StoredLess<kRtSigned<RT>>(b, mn)) {
    mn = b;
  }
  if (StoredLess<kRtSigned<RT>>(mx, a)) {
    mx = a;
  }
  if (StoredLess<kRtSigned<RT>>(mx, b)) {
    mx = b;
  }
}

template <typename Spec, typename RT>
constexpr void AccRangeSeq(StoredRaw& /*mn*/, StoredRaw& /*mx*/,
                           std::integer_sequence<int>) {}

template <typename Spec, typename RT, int I, int... Rest>
constexpr void AccRangeSeq(StoredRaw& mn, StoredRaw& mx,
                           std::integer_sequence<int, I, Rest...>) {
  AccSegRange<Spec, RT, I>(mn, mx);
  AccRangeSeq<Spec, RT>(mn, mx, std::integer_sequence<int, Rest...>{});
}

template <typename Spec, typename RT, int I>
constexpr void EncodeSeg(std::uint32_t& best, std::uint32_t& best_d,
                         StoredRaw raw) {
  using S = SegN<Spec, RT, I>;
  int approx = ApproxIndex<RT, S>(raw);
  int const last = S::kMathFirst + static_cast<int>(S::kCodeCount) - 1;
  if (approx < S::kMathFirst) {
    approx = S::kMathFirst;
  }
  if (approx > last) {
    approx = last;
  }
  int const lo_j = (approx < S::kMathFirst + kNeighborRadius)
                       ? S::kMathFirst
                       : (approx - kNeighborRadius);
  int const hi_j =
      (approx + kNeighborRadius > last) ? last : (approx + kNeighborRadius);
  for (int j = lo_j; j <= hi_j; ++j) {
    ConsiderRank<RT, S>(best, best_d, last, j, raw);
  }
  ConsiderRank<RT, S>(best, best_d, last, S::kMathFirst, raw);
  ConsiderRank<RT, S>(best, best_d, last, last, raw);
}

template <typename Spec, typename RT>
constexpr void EncodeRec(std::uint32_t& /*best*/, std::uint32_t& /*best_d*/,
                         StoredRaw /*raw*/, std::integer_sequence<int>) {}

template <typename Spec, typename RT, int I, int... Rest>
constexpr void EncodeRec(std::uint32_t& best, std::uint32_t& best_d,
                         StoredRaw raw,
                         std::integer_sequence<int, I, Rest...>) {
  EncodeSeg<Spec, RT, I>(best, best_d, raw);
  EncodeRec<Spec, RT>(best, best_d, raw, std::integer_sequence<int, Rest...>{});
}

template <typename Spec>
using SegIndexSeq =
    std::make_integer_sequence<int, PlanHolder<Spec>::kPlan.count>;

template <typename Spec, typename RT>
AE_SEG_NOINLINE constexpr typename RT::rep_value_type DecodeRankRaw(
    std::uint32_t rank) {
  return StoredToRt<RT>(DecodeRankSeq<Spec, RT>(rank, SegIndexSeq<Spec>{}));
}

template <typename Spec, typename RT>
AE_SEG_NOINLINE constexpr std::uint32_t EncodeRaw(
    std::uint32_t code_count, typename RT::rep_value_type raw) {
  std::uint32_t best = 0;
  std::uint32_t best_d = std::numeric_limits<std::uint32_t>::max();
  EncodeRec<Spec, RT>(best, best_d, RtToStored<RT>(raw), SegIndexSeq<Spec>{});
  if (best >= code_count) {
    return code_count - 1U;
  }
  return best;
}

template <typename Spec, typename RT>
constexpr typename RT::rep_value_type ScanRepresentable(bool want_min) {
  using S0 = SegN<Spec, RT, 0>;
  StoredRaw mn = DecodeMath<RT, S0>(S0::kMathFirst);
  StoredRaw mx = mn;
  AccRangeSeq<Spec, RT>(mn, mx, SegIndexSeq<Spec>{});
  return StoredToRt<RT>(want_min ? mn : mx);
}

template <typename Spec, typename RT, int I>
int RampIndexErrorOnCodes() {
  using S = SegN<Spec, RT, I>;
  if constexpr (S::kKind != CurveKind::kLinearStepRamp) {
    return 0;
  } else {
    int max_err = 0;
    int const last = S::kMathFirst + static_cast<int>(S::kCodeCount) - 1;
    for (int true_j = S::kMathFirst; true_j <= last; ++true_j) {
      StoredRaw const raw = DecodeMath<RT, S>(true_j);
      int const approx = LinearRampApproxRuntime<S::kSigned>(
          S::kBegin, S::kEnd, static_cast<std::int32_t>(S::kStep0),
          static_cast<std::int32_t>(S::kDelta), S::kIntervals, raw);
      int err = approx - true_j;
      if (err < 0) {
        err = -err;
      }
      if (err > max_err) {
        max_err = err;
      }
    }
    return max_err;
  }
}

template <typename Spec, typename RT, int I>
int RampIndexErrorDense() {
  using S = SegN<Spec, RT, I>;
  if constexpr (S::kKind != CurveKind::kLinearStepRamp) {
    return 0;
  } else {
    int max_err = 0;
    int const last = S::kMathFirst + static_cast<int>(S::kCodeCount) - 1;
    StoredRaw lo = S::kBegin;
    StoredRaw hi = S::kEnd;
    if (StoredLess<S::kSigned>(hi, lo)) {
      StoredRaw const t = lo;
      lo = hi;
      hi = t;
    }
    StoredRaw const span = AbsStoredDiff<S::kSigned>(hi, lo);
    StoredRaw step = span / 4096U;
    if (step < 1U) {
      step = 1U;
    }
    for (StoredRaw raw = lo; !StoredLess<S::kSigned>(hi, raw); ) {
      int const approx = LinearRampApproxRuntime<S::kSigned>(
          S::kBegin, S::kEnd, static_cast<std::int32_t>(S::kStep0),
          static_cast<std::int32_t>(S::kDelta), S::kIntervals, raw);
      int true_j = S::kMathFirst;
      std::uint32_t best_d = std::numeric_limits<std::uint32_t>::max();
      for (int j = S::kMathFirst; j <= last; ++j) {
        std::uint32_t const d =
            AbsStoredDiff<S::kSigned>(DecodeMath<RT, S>(j), raw);
        if (d < best_d) {
          best_d = d;
          true_j = j;
        }
      }
      int err = approx - true_j;
      if (err < 0) {
        err = -err;
      }
      if (err > max_err) {
        max_err = err;
      }
      if (raw > std::numeric_limits<StoredRaw>::max() - step) {
        break;
      }
      raw += step;
    }
    return max_err;
  }
}

template <typename Spec, typename RT>
int MaxRampIndexErrorRec(std::integer_sequence<int>, bool dense) {
  (void)dense;
  return 0;
}

template <typename Spec, typename RT, int I, int... Rest>
int MaxRampIndexErrorRec(std::integer_sequence<int, I, Rest...>, bool dense) {
  int const a = dense ? RampIndexErrorDense<Spec, RT, I>()
                    : RampIndexErrorOnCodes<Spec, RT, I>();
  int const b = MaxRampIndexErrorRec<Spec, RT>(
      std::integer_sequence<int, Rest...>{}, dense);
  return a > b ? a : b;
}

template <typename Spec, typename RT>
int MaxRampIndexError() {
  return MaxRampIndexErrorRec<Spec, RT>(SegIndexSeq<Spec>{}, false);
}

template <typename Spec, typename RT>
int MaxRampIndexErrorDenseInputs() {
  return MaxRampIndexErrorRec<Spec, RT>(SegIndexSeq<Spec>{}, true);
}

}  // namespace ae::seg::detail

#endif  // AE_NUMERIC_DETAILS_SEGMENTED_FORMULA_BACKEND_H_
