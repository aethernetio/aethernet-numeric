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

#include <cstdint>
#include <limits>
#include <type_traits>

#include "ae-numeric/decimal.h"
#include "ae-numeric/details/segmented_format.h"
#include "ae-numeric/fixed_math.h"
#include "ae-numeric/fixed_point.h"
#include "ae-numeric/integer_math.h"

namespace ae::seg::segmented_math_internal {

using SegPolicy = fixed_math::Segmented32MathPolicy;
// Max covers RX 24 h (86400 s). int32 is used for signed physical values
// (temperature). Positive Exp2/Log2 use uint32 for extra fraction bits.
using SegWork = FixedPoint<std::int32_t, 86400>;
using SegPosWork = FixedPoint<std::uint32_t, 86400>;
using SegRatio = FixedPoint<std::uint32_t, 8>;
using SegLog = SegPolicy::log_type;

static_assert(sizeof(typename SegWork::rep_value_type) <= 4);
static_assert(sizeof(typename SegPosWork::rep_value_type) <= 4);
static_assert(sizeof(typename SegRatio::rep_value_type) <= 4);
static_assert(sizeof(typename SegLog::rep_value_type) <= 4);

inline void SegmentedSpecError() {}

struct Rat {
  std::int32_t num = 0;
  std::int32_t den = 1;
};

template <typename T>
consteval Rat NumberToRat() {
  constexpr auto n = NumberRatio<T>::num;
  constexpr auto d = NumberRatio<T>::den;
  static_assert(n >= std::numeric_limits<std::int32_t>::min() &&
                n <= std::numeric_limits<std::int32_t>::max());
  static_assert(d >= std::numeric_limits<std::int32_t>::min() &&
                d <= std::numeric_limits<std::int32_t>::max());
  return {static_cast<std::int32_t>(n), static_cast<std::int32_t>(d)};
}

consteval std::int32_t AbsI32(std::int32_t v) {
  return v < 0 ? static_cast<std::int32_t>(-v) : v;
}

consteval bool RatLess(Rat a, Rat b) {
  // a.n/a.d < b.n/b.d  <=>  a.n*b.d < b.n*a.d (positive dens in our DSL)
  std::uint32_t an = integer_math::AbsI32ToU32(a.num);
  std::uint32_t bd = integer_math::AbsI32ToU32(b.den);
  std::uint32_t bn = integer_math::AbsI32ToU32(b.num);
  std::uint32_t ad = integer_math::AbsI32ToU32(a.den);
  bool const a_neg = a.num < 0;
  bool const b_neg = b.num < 0;
  if (a_neg != b_neg) {
    return a_neg;
  }
  int const cmp = integer_math::CmpMulU32(an, bd, bn, ad);
  return a_neg ? cmp > 0 : cmp < 0;
}

consteval bool RatAbsLess(Rat a, Rat b) {
  return integer_math::CmpMulU32(integer_math::AbsI32ToU32(a.num),
                                 integer_math::AbsI32ToU32(b.den),
                                 integer_math::AbsI32ToU32(b.num),
                                 integer_math::AbsI32ToU32(a.den)) < 0;
}

consteval Rat RatAbsMax(Rat a, Rat b) {
  return RatAbsLess(a, b) ? b : a;
}

consteval std::int32_t CeilAbsRat(Rat r) {
  std::int32_t const n = AbsI32(r.num);
  std::int32_t const d = r.den <= 0 ? 1 : r.den;
  std::int32_t const q = static_cast<std::int32_t>(
      (static_cast<std::uint32_t>(n) + static_cast<std::uint32_t>(d) - 1U) /
      static_cast<std::uint32_t>(d));
  return q < 1 ? 1 : q;
}

consteval SegWork WorkFromRat(Rat r) {
  return SegWork::FromRatio(r.num, r.den);
}

constexpr SegLog LogZero() {
  return SegLog::FromRuntimeInteger(0);
}

constexpr SegWork WorkZero() {
  return SegWork::FromRuntimeInteger(0);
}

constexpr SegWork WorkOne() {
  return SegWork::FromRuntimeInteger(1);
}

constexpr SegRatio RatioOne() {
  return SegRatio::FromRuntimeInteger(1);
}

constexpr SegLog WorkLogFromRaw(typename SegLog::rep_value_type raw) {
  return SegLog::FromRaw(
      fixed_point_internal::RepFromRawValue<typename SegLog::rep_type>(
          SegLog::ClampRaw(raw)));
}

constexpr SegWork WorkFromRaw(typename SegWork::rep_value_type raw) {
  return SegWork::FromRaw(
      fixed_point_internal::RepFromRawValue<typename SegWork::rep_type>(
          SegWork::ClampRaw(raw)));
}

template <typename To, typename From>
constexpr To ConvertFixed(From x) {
  static_assert(sizeof(typename From::rep_value_type) <= 4);
  static_assert(sizeof(typename To::rep_value_type) <= 4);
  using ToR = typename To::rep_value_type;
  ToR const aligned = fixed_point_internal::ConvertRawScaleTo<ToR>(
      x.RawValue(), From::kScaleExp, To::kScaleExp, To::kRawMin, To::kRawMax);
  return To::FromRaw(
      fixed_point_internal::RepFromRawValue<typename To::rep_type>(aligned));
}

consteval Rat RatSub(Rat a, Rat b) {
  // (a.n*b.d - b.n*a.d) / (a.d*b.d); all DSL bounds fit int32.
  std::uint32_t p1h = 0;
  std::uint32_t p1l = 0;
  std::uint32_t p2h = 0;
  std::uint32_t p2l = 0;
  integer_math::MulI32Wide(a.num, b.den, p1h, p1l);
  integer_math::MulI32Wide(b.num, a.den, p2h, p2l);
  // Subtract p2 from p1 in two's complement wide form.
  std::uint32_t nl = p1l - p2l;
  std::uint32_t const borrow = p1l < p2l ? 1U : 0U;
  std::uint32_t nh = p1h - p2h - borrow;
  bool const neg = (nh & 0x80000000U) != 0U;
  if (neg) {
    integer_math::NegU32Wide(nh, nl);
  }
  if (nh != 0U || nl > static_cast<std::uint32_t>(
                           std::numeric_limits<std::int32_t>::max())) {
    SegmentedSpecError();
    return {};
  }
  std::uint32_t dh = 0;
  std::uint32_t dl = 0;
  integer_math::MulI32Wide(a.den, b.den, dh, dl);
  if (dh != 0U || dl == 0U ||
      dl > static_cast<std::uint32_t>(
               std::numeric_limits<std::int32_t>::max())) {
    SegmentedSpecError();
    return {};
  }
  auto num = static_cast<std::int32_t>(nl);
  if (neg) {
    num = -num;
  }
  return {num, static_cast<std::int32_t>(dl)};
}

consteval int RoundRatQuotient(Rat span, Rat step) {
  if (step.num == 0 || step.den == 0 || span.den == 0) {
    SegmentedSpecError();
    return 1;
  }
  std::uint32_t nh = 0;
  std::uint32_t nl = 0;
  std::uint32_t dh = 0;
  std::uint32_t dl = 0;
  integer_math::MulU32Wide(integer_math::AbsI32ToU32(span.num),
                           integer_math::AbsI32ToU32(step.den), nh, nl);
  integer_math::MulU32Wide(integer_math::AbsI32ToU32(span.den),
                           integer_math::AbsI32ToU32(step.num), dh, dl);
  if (dh != 0U) {
    while (dh != 0U) {
      integer_math::ShrU32Wide(nh, nl, 1U, false);
      integer_math::ShrU32Wide(dh, dl, 1U, false);
    }
  }
  if (dl == 0U) {
    SegmentedSpecError();
    return 1;
  }
  std::uint32_t q = 0;
  std::uint32_t r = 0;
  if (!integer_math::DivU32Wide(nh, nl, dl, q, r) || q > 100000U) {
    SegmentedSpecError();
    return 1;
  }
  if (r >= dl - r && q != std::numeric_limits<std::uint32_t>::max()) {
    ++q;
  }
  if (q < 1U) {
    return 1;
  }
  return static_cast<int>(q);
}

constexpr SegWork MulWorkSame(SegWork a, SegWork b) {
  return fixed_point_internal::MulFixedPoint<std::int32_t, 86400>(a, b);
}

constexpr SegWork ScaleWorkByRatio(SegWork w, SegRatio r) {
  // Multiply in-place: do not ConvertFixed<SegWork>(r). SegWork Max is 86400,
  // so a ratio near 1 would keep only ~15 significant bits after conversion.
  return fixed_point_internal::MulFixedPoint<std::int32_t, 86400>(w, r);
}

constexpr SegWork ScaleWorkByInt(SegWork w, int n) {
  if (n == 0 || w.RawValue() == static_cast<typename SegWork::rep_value_type>(0)) {
    return WorkZero();
  }
  bool const negative = (w.RawValue() < 0) != (n < 0);
  std::uint32_t hi = 0;
  std::uint32_t lo = 0;
  integer_math::MulU32Wide(integer_math::AbsI32ToU32(w.RawValue()),
                           integer_math::AbsI32ToU32(static_cast<std::int32_t>(n)),
                           hi, lo);
  if (hi != 0U ||
      lo > static_cast<std::uint32_t>(std::numeric_limits<std::int32_t>::max())) {
    return negative ? SegWork::FromRaw(SegWork::kRawMin)
                    : SegWork::FromRaw(SegWork::kRawMax);
  }
  auto out = static_cast<std::int32_t>(lo);
  if (negative) {
    out = -out;
  }
  return WorkFromRaw(out);
}

constexpr bool WorkPositive(SegWork x) {
  return x.RawValue() > static_cast<typename SegWork::rep_value_type>(0);
}

constexpr bool WorkNonPositive(SegWork x) {
  return x.RawValue() <= static_cast<typename SegWork::rep_value_type>(0);
}

constexpr SegWork WorkAbs(SegWork x) {
  if (x.RawValue() < static_cast<typename SegWork::rep_value_type>(0)) {
    return SubTo<SegWork>(WorkZero(), x);
  }
  return x;
}

consteval int WorkToNearestInt(SegWork x) {
  auto const one = WorkOne().RawValue();
  if (one == 0) {
    return 0;
  }
  return static_cast<int>(fixed_point_internal::RoundDivNearest(
      x.RawValue(), one));
}

consteval int CeilDivWork(SegWork num, SegWork den) {
  auto const a = num.RawValue();
  auto const b = den.RawValue();
  if (b <= 0 || a <= 0) {
    SegmentedSpecError();
    return 1;
  }
  std::uint32_t const ua = integer_math::AbsI32ToU32(a);
  std::uint32_t const ub = integer_math::AbsI32ToU32(b);
  std::uint32_t q = ua / ub;
  if (ua % ub != 0U) {
    if (q == std::numeric_limits<std::uint32_t>::max()) {
      SegmentedSpecError();
      return 100000;
    }
    ++q;
  }
  if (q < 1U) {
    return 1;
  }
  if (q > 100000U) {
    SegmentedSpecError();
    return 100000;
  }
  return static_cast<int>(q);
}

constexpr SegLog Log2Work(SegWork x) {
  if (x.RawValue() <= static_cast<typename SegWork::rep_value_type>(0)) {
    return SegLog::FromRaw(SegLog::kRawMin);
  }
  return fixed_math::Log2To<SegLog, SegPosWork, SegPolicy>(
      ConvertFixed<SegPosWork>(x));
}

constexpr SegLog Log2Pos(SegPosWork x) {
  if (x.RawValue() == static_cast<typename SegPosWork::rep_value_type>(0)) {
    return SegLog::FromRaw(SegLog::kRawMin);
  }
  return fixed_math::Log2To<SegLog, SegPosWork, SegPolicy>(x);
}

constexpr SegLog Log2Ratio(SegRatio x) {
  return fixed_math::Log2To<SegLog, SegRatio, SegPolicy>(x);
}

consteval SegLog Log2OfRat(Rat r) {
  if (r.num <= 0 || r.den <= 0) {
    SegmentedSpecError();
    return SegLog::FromRuntimeInteger(0);
  }
  SegLog const ln =
      Log2Pos(SegPosWork::FromInteger(r.num));
  SegLog const ld =
      Log2Pos(SegPosWork::FromInteger(r.den));
  return SubTo<SegLog>(ln, ld);
}

constexpr SegPosWork Exp2Pos(SegLog y) {
  return fixed_math::Exp2To<SegPosWork, SegLog, SegPolicy>(y);
}

constexpr SegWork Exp2Work(SegLog y) {
  return ConvertFixed<SegWork>(Exp2Pos(y));
}

constexpr SegRatio Exp2Ratio(SegLog y) {
  return fixed_math::Exp2To<SegRatio, SegLog, SegPolicy>(y);
}

constexpr SegLog MulLogInt(SegLog x, int n) {
  return fixed_math::ScaleLogByInt(x, n);
}

constexpr SegLog DivLogInt(SegLog x, int n) {
  return fixed_math::DivLogByInt(x, n);
}

constexpr SegLog AbsLog(SegLog x) {
  if (x.RawValue() < static_cast<typename SegLog::rep_value_type>(0)) {
    return SubTo<SegLog>(SegLog::FromRuntimeInteger(0), x);
  }
  return x;
}

consteval SegLog Log2RFromEndpoints(Rat begin, Rat end, int intervals) {
  if (intervals <= 0) {
    SegmentedSpecError();
    return SegLog::FromRuntimeInteger(0);
  }
  SegLog const span = SubTo<SegLog>(Log2OfRat(end), Log2OfRat(begin));
  return DivLogInt(span, intervals);
}

consteval SegRatio ExpRatioOf(Rat begin, Rat end, int intervals) {
  return Exp2Ratio(Log2RFromEndpoints(begin, end, intervals));
}

constexpr bool LogMulSaturates(SegLog x, int n) {
  if (n <= 1) {
    return false;
  }
  auto const raw = x.RawValue();
  if (raw <= static_cast<typename SegLog::rep_value_type>(0)) {
    return false;
  }
  auto const maxr = static_cast<std::uint32_t>(SegLog::kRawMax);
  auto const nu = static_cast<std::uint32_t>(n);
  return static_cast<std::uint32_t>(raw) > maxr / nu;
}

constexpr SegPosWork Exp2PosMinusOne(SegLog y) {
  SegPosWork const e = Exp2Pos(y);
  SegPosWork const one = SegPosWork::FromRuntimeInteger(1);
  if (e.RawValue() > one.RawValue()) {
    return SubTo<SegPosWork>(e, one);
  }
  return SegPosWork::FromRuntimeInteger(0);
}

constexpr SegWork Exp2MinusOne(SegLog y) {
  return ConvertFixed<SegWork>(Exp2PosMinusOne(y));
}

// (q^k - 1) / (q^n - 1) as a SegWork in [0, 1]. Avoids dividing by (q - 1).
constexpr SegPosWork GeomUnitWeightPos(SegLog log2_q, int k, int n) {
  if (n <= 0 || k <= 0) {
    return SegPosWork::FromRuntimeInteger(0);
  }
  if (k >= n) {
    return SegPosWork::FromRuntimeInteger(1);
  }
  if (log2_q.RawValue() <= static_cast<typename SegLog::rep_value_type>(0) ||
      LogMulSaturates(log2_q, n)) {
    return SegPosWork::FromRatio(k, n);
  }
  SegPosWork const den = Exp2PosMinusOne(MulLogInt(log2_q, n));
  if (den.RawValue() == static_cast<typename SegPosWork::rep_value_type>(0)) {
    return SegPosWork::FromRatio(k, n);
  }
  SegPosWork const num = Exp2PosMinusOne(MulLogInt(log2_q, k));
  if (num.RawValue() == static_cast<typename SegPosWork::rep_value_type>(0)) {
    return SegPosWork::FromRuntimeInteger(0);
  }
  return DivTo<SegPosWork>(num, den);
}

constexpr SegWork GeomUnitWeight(SegLog log2_q, int k, int n) {
  return ConvertFixed<SegWork>(GeomUnitWeightPos(log2_q, k, n));
}

constexpr SegWork GeomInterp(SegWork begin, SegWork end, SegLog log2_q, int i,
                             int n, bool from_upper) {
  if (n <= 0 || i <= 0) {
    return begin;
  }
  if (i >= n) {
    return end;
  }
  SegWork const span = SubTo<SegWork>(end, begin);
  int const k = from_upper ? (n - i) : i;
  SegWork const w = GeomUnitWeight(log2_q, k, n);
  SegWork const offset = MulWorkSame(span, w);
  if (from_upper) {
    return SubTo<SegWork>(end, offset);
  }
  return AddTo<SegWork>(begin, offset);
}

constexpr SegWork GeomSumFromLog(SegLog log2_q, int n) {
  if (n <= 0) {
    return WorkZero();
  }
  if (log2_q.RawValue() <= static_cast<typename SegLog::rep_value_type>(0)) {
    return SegWork::FromRuntimeInteger(n);
  }
  if (LogMulSaturates(log2_q, n)) {
    return SegWork::FromRaw(SegWork::kRawMax);
  }
  SegWork const den = Exp2MinusOne(log2_q);
  if (!WorkPositive(den)) {
    return SegWork::FromRuntimeInteger(n);
  }
  SegWork const num = Exp2MinusOne(MulLogInt(log2_q, n));
  if (!WorkPositive(num)) {
    return WorkZero();
  }
  return DivTo<SegWork>(num, den);
}

consteval SegWork GeomSumByTerms(SegRatio q, int n) {
  if (n <= 0) {
    return WorkZero();
  }
  SegWork acc = WorkZero();
  SegWork term = WorkOne();
  for (int i = 0; i < n; ++i) {
    SegWork const next = AddTo<SegWork>(acc, term);
    if (next.RawValue() < acc.RawValue() && WorkPositive(term)) {
      return SegWork::FromRaw(SegWork::kRawMax);
    }
    acc = next;
    if (i + 1 == n) {
      break;
    }
    term = ScaleWorkByRatio(term, q);
    if (term.RawValue() == static_cast<typename SegWork::rep_value_type>(0)) {
      break;
    }
  }
  return acc;
}

consteval SegWork GeomSumQ(SegRatio q, int n) {
  return GeomSumFromLog(Log2Ratio(q), n);
}

consteval SegRatio RatioFromRawU32(std::uint32_t raw) {
  if (raw > static_cast<std::uint32_t>(SegRatio::kRawMax)) {
    raw = static_cast<std::uint32_t>(SegRatio::kRawMax);
  }
  return SegRatio::FromRaw(
      fixed_point_internal::RepFromRawValue<typename SegRatio::rep_type>(
          static_cast<typename SegRatio::rep_value_type>(raw)));
}

consteval SegRatio SolveQForGeomSum(int n, SegWork sum) {
  if (n <= 0 || !WorkPositive(sum)) {
    SegmentedSpecError();
    return RatioOne();
  }
  SegWork const n_as_work = SegWork::FromRuntimeInteger(n);
  if (!(n_as_work < sum)) {
    return RatioOne();
  }
  SegRatio lo = RatioOne();
  SegRatio hi = SegRatio::FromRatio(3, 2);
  for (int i = 0; i < 8 && GeomSumByTerms(hi, n) < sum; ++i) {
    std::uint32_t hi2 = 0;
    if (!integer_math::MulU32Checked(hi.RawValue(), 2U, hi2)) {
      hi = SegRatio::FromRaw(SegRatio::kRawMax);
      break;
    }
    hi = RatioFromRawU32(hi2);
  }
  for (int i = 0; i < 40; ++i) {
    std::uint32_t const lv = lo.RawValue();
    std::uint32_t const hv = hi.RawValue();
    std::uint32_t const mid_raw = lv + (hv - lv) / 2U;
    SegRatio const mid = RatioFromRawU32(mid_raw);
    if (GeomSumByTerms(mid, n) < sum) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  std::uint32_t const lv = lo.RawValue();
  std::uint32_t const hv = hi.RawValue();
  return RatioFromRawU32(lv + (hv - lv) / 2U);
}

struct AutoSplitResult {
  int n1 = 0;
  int n2 = 0;
  SegLog log2_r1 = LogZero();
  SegLog log2_r2 = LogZero();
  SegRatio r1 = RatioOne();
  SegRatio r2 = RatioOne();
};

struct JumpScore {
  std::uint32_t abs_d = 0;
  std::uint32_t min_s = 1;
};

consteval JumpScore MakeJumpScoreRatio(SegRatio step_before,
                                        SegRatio step_after) {
  std::uint32_t const a = step_before.RawValue();
  std::uint32_t const b = step_after.RawValue();
  JumpScore s{};
  s.abs_d = a > b ? a - b : b - a;
  s.min_s = (a < b ? a : b);
  if (s.min_s == 0) {
    s.min_s = 1;
  }
  return s;
}

consteval bool JumpLess(JumpScore a, JumpScore b) {
  return integer_math::CmpMulU32(a.abs_d, b.min_s, b.abs_d, a.min_s) < 0;
}

consteval bool BetterAutoSplit(bool have, JumpScore jump, JumpScore best_jump,
                               SegRatio err, SegRatio best_err, int n1,
                               int best_n1) {
  if (!have) {
    return true;
  }
  if (JumpLess(jump, best_jump)) {
    return true;
  }
  if (JumpLess(best_jump, jump)) {
    return false;
  }
  if (err < best_err) {
    return true;
  }
  if (best_err < err) {
    return false;
  }
  return n1 < best_n1;
}

consteval AutoSplitResult AutoSplitTwoExp(Rat begin, Rat mid, Rat end,
                                          int total_intervals) {
  AutoSplitResult best{};
  bool have = false;
  JumpScore best_jump{};
  SegRatio best_err = RatioOne();
  SegLog const log_b = Log2OfRat(begin);
  SegLog const log_m = Log2OfRat(mid);
  SegLog const log_e = Log2OfRat(end);
  for (int n1 = 1; n1 < total_intervals; ++n1) {
    int const n2 = total_intervals - n1;
    SegLog const log_r1 = DivLogInt(SubTo<SegLog>(log_m, log_b), n1);
    SegLog const log_r2 = DivLogInt(SubTo<SegLog>(log_e, log_m), n2);
    SegRatio const r1 = Exp2Ratio(log_r1);
    SegRatio const r2 = Exp2Ratio(log_r2);
    if (!(RatioOne() < r1) || !(RatioOne() < r2)) {
      continue;
    }
    SegRatio const step_before =
        DivTo<SegRatio>(SubTo<SegRatio>(r1, RatioOne()), r1);
    SegRatio const step_after = SubTo<SegRatio>(r2, RatioOne());
    if (step_before.RawValue() == 0 || step_after.RawValue() == 0) {
      continue;
    }
    JumpScore const jump = MakeJumpScoreRatio(step_before, step_after);
    SegRatio const err1 = SubTo<SegRatio>(r1, RatioOne());
    SegRatio const err2 = SubTo<SegRatio>(r2, RatioOne());
    SegRatio const max_err = err1 > err2 ? err1 : err2;
    if (BetterAutoSplit(have, jump, best_jump, max_err, best_err, n1,
                        best.n1)) {
      have = true;
      best_jump = jump;
      best_err = max_err;
      best.n1 = n1;
      best.n2 = n2;
      best.log2_r1 = log_r1;
      best.log2_r2 = log_r2;
      best.r1 = Exp2Ratio(log_r1);
      best.r2 = Exp2Ratio(log_r2);
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
  SegLog log2_r = LogZero();
  SegRatio r = RatioOne();
};

consteval int RoundNearestNonnegLog(SegLog x) {
  if (x.RawValue() < static_cast<typename SegLog::rep_value_type>(0)) {
    return 0;
  }
  auto const one = SegLog::FromRuntimeInteger(1).RawValue();
  if (one == 0) {
    return 0;
  }
  auto const q = fixed_point_internal::RoundDivNearest(x.RawValue(), one);
  if (q < 0) {
    return 0;
  }
  return static_cast<int>(q);
}

consteval ContExpResult OptimizeContinuousExp(Rat vmin, Rat vmax, Rat cut1,
                                                Rat cut2, int last_1_max,
                                                int min_n, int max_n,
                                                int max_last_2) {
  ContExpResult best{};
  bool have = false;
  SegLog best_cut = SegLog::FromRaw(SegLog::kRawMax);
  SegLog best_rel = SegLog::FromRaw(SegLog::kRawMax);
  int const i1 = last_1_max;
  SegLog const log_vmin = Log2OfRat(vmin);
  SegLog const log_vmax = Log2OfRat(vmax);
  SegLog const log_cut1 = Log2OfRat(cut1);
  SegLog const log_cut2 = Log2OfRat(cut2);
  SegLog const span_log = SubTo<SegLog>(log_vmax, log_vmin);
  auto const span_raw = span_log.RawValue();
  for (int n = min_n; n <= max_n; ++n) {
    if (n <= i1 + 1 || span_raw == 0) {
      continue;
    }
    SegLog const log2_r = DivLogInt(span_log, n);
    SegLog const half = DivLogInt(log2_r, 2);
    auto const dcut_raw =
        SubTo<SegLog>(log_cut2, log_vmin).RawValue();
    std::uint32_t const n_u = static_cast<std::uint32_t>(n);
    std::uint32_t const dcut_u = integer_math::AbsI32ToU32(dcut_raw);
    std::uint32_t const span_u = integer_math::AbsI32ToU32(span_raw);
    if (span_u == 0U || span_u > (std::numeric_limits<std::uint32_t>::max() >> 1U)) {
      continue;
    }
    std::uint32_t hi = 0;
    std::uint32_t lo = 0;
    integer_math::MulU32Wide(n_u, dcut_u, hi, lo);
    if (!integer_math::ShlU32WideChecked(hi, lo, 1U)) {
      continue;
    }
    if (lo < span_u) {
      if (hi == 0U) {
        continue;
      }
      --hi;
      lo -= span_u;
    } else {
      lo -= span_u;
    }
    std::uint32_t q = 0;
    std::uint32_t r = 0;
    std::uint32_t const den = span_u << 1U;
    if (!integer_math::DivU32Wide(hi, lo, den, q, r)) {
      continue;
    }
    if (r >= den - r && q != std::numeric_limits<std::uint32_t>::max()) {
      ++q;
    }
    int const i2 = static_cast<int>(q);
    if (i2 <= i1 || i2 >= n || i2 > max_last_2) {
      continue;
    }
    SegLog const b1_log =
        AddTo<SegLog>(log_vmin, AddTo<SegLog>(MulLogInt(log2_r, i1), half));
    SegLog const b2_log =
        AddTo<SegLog>(log_vmin, AddTo<SegLog>(MulLogInt(log2_r, i2), half));
    SegLog const e1 = AbsLog(SubTo<SegLog>(b1_log, log_cut1));
    SegLog const e2 = AbsLog(SubTo<SegLog>(b2_log, log_cut2));
    SegLog const cut = e1 > e2 ? e1 : e2;
    SegLog const rel = log2_r;
    bool const better =
        !have || cut < best_cut ||
        (cut == best_cut &&
         (rel < best_rel || (rel == best_rel && n < best.intervals)));
    if (better) {
      have = true;
      best_cut = cut;
      best_rel = rel;
      best.intervals = n;
      best.last_1 = i1;
      best.last_2 = i2;
      best.log2_r = log2_r;
    }
  }
  if (!have) {
    SegmentedSpecError();
  }
  best.r = Exp2Ratio(best.log2_r);
  return best;
}

consteval std::uint32_t TwoTierMaxU8(std::uint32_t b0) {
  return (255U - b0 - 1U) * 256U + b0 + 1U + 255U;
}

consteval std::uint32_t ThreeTierMaxU8(std::uint32_t b0, std::uint32_t b1) {
  std::uint32_t const two = TwoTierMaxU8(b0);
  std::uint32_t const n = (two > b1 + 1U) ? (two - b1 - 1U) : 0U;
  if (n > (std::numeric_limits<std::uint32_t>::max() >> 16U)) {
    return std::numeric_limits<std::uint32_t>::max();
  }
  std::uint32_t acc = n << 16U;
  std::uint32_t out = 0;
  if (!integer_math::AddU32Checked(acc, b1 + 1U + 65535U, out)) {
    return std::numeric_limits<std::uint32_t>::max();
  }
  return out;
}

consteval int MinRampIntervals(SegWork span, SegWork step0, SegWork max_err) {
  if (!WorkPositive(span) || !WorkPositive(step0) ||
      !WorkPositive(max_err)) {
    SegmentedSpecError();
    return 1;
  }
  std::uint32_t const span_u =
      integer_math::AbsI32ToU32(span.RawValue());
  std::uint32_t const step0_u =
      integer_math::AbsI32ToU32(step0.RawValue());
  std::uint32_t const err_u =
      integer_math::AbsI32ToU32(max_err.RawValue());
  std::uint32_t last_lim = 0;
  if (!integer_math::AddU32Checked(err_u, err_u, last_lim)) {
    SegmentedSpecError();
    return 1;
  }
  std::uint32_t den = 0;
  if (!integer_math::AddU32Checked(step0_u, last_lim, den) || den == 0U) {
    SegmentedSpecError();
    return 1;
  }
  std::uint32_t two_span = 0;
  if (!integer_math::AddU32Checked(span_u, span_u, two_span)) {
    SegmentedSpecError();
    return 1;
  }
  std::uint32_t n_u = two_span / den;
  if (two_span % den != 0U) {
    ++n_u;
  }
  int n = n_u < 1U ? 1 : static_cast<int>(n_u);
  for (int guard = 0; guard < 4096; ++guard) {
    std::uint32_t mean = 0;
    if (!integer_math::RoundDivU32(two_span, static_cast<std::uint32_t>(n),
                                   mean)) {
      break;
    }
    std::int32_t const last_step =
        static_cast<std::int32_t>(mean) - static_cast<std::int32_t>(step0_u);
    if (last_step > static_cast<std::int32_t>(last_lim)) {
      ++n;
      continue;
    }
    if (last_step > 0) {
      break;
    }
    if (n <= 1) {
      break;
    }
    --n;
  }
  return n;
}

// Schema-hash metadata only. kSchemaHash is computed at compile time and is
// not part of the SegmentedNumber encode/decode mathematical path. The FNV-1a
// mix uses a 64-bit state so existing golden hashes stay bit-identical; this
// is not runtime arithmetic.
consteval std::uint64_t MixHash(std::uint64_t h, std::uint64_t v) {
  h ^= v;
  h *= 1099511628211ULL;
  return h;
}

consteval std::uint64_t MixRat(std::uint64_t h, Rat r) {
  h = MixHash(h, static_cast<std::uint64_t>(r.num));
  h = MixHash(h, static_cast<std::uint64_t>(r.den));
  return h;
}

}  // namespace ae::seg::segmented_math_internal

#endif  // AE_NUMERIC_DETAILS_SEGMENTED_MATH_H_
