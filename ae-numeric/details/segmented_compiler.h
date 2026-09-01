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

#ifndef AE_NUMERIC_DETAILS_SEGMENTED_COMPILER_H_
#define AE_NUMERIC_DETAILS_SEGMENTED_COMPILER_H_

// Purpose: Compile a declarative SegmentedNumber schema into validated
// curve coefficients, canonical ranks, wire tiers, and schema metadata.
//
// Motivation: users should describe physical requirements, while the
// library derives a compact specialized codec and rejects impossible or
// ambiguous layouts at compile time.
//
// Analogous concepts: a small type-level compiler and quantizer design
// pass. Implementation lives in ae::seg::detail; use only through
// segmented_number.h and seg::Compile<Spec>.
//
// How it works: it flattens curve drafts, validates ranges and endpoint
// ownership, solves interval counts and coefficients, assigns ranks,
// derives uint8_t/TieredInt boundaries, and computes schema identity.
// CompiledSegment is universal compile-time metadata; its sizeof is not
// per-segment flash cost, and runtime instances do not store it.


#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

#include "ae-numeric/details/segmented_curves.h"
#include "ae-numeric/details/segmented_format.h"
#include "ae-numeric/details/segmented_math.h"
#include "ae-numeric/fixed_math.h"
#include "ae-numeric/fixed_point.h"
#include "ae-numeric/integer_math.h"
#include "ae-numeric/tiered_int.h"

namespace ae::seg::detail {

template <typename>
inline constexpr int kMaxBytesTag = -1;
template <std::size_t N>
inline constexpr int kMaxBytesTag<wire::MaxBytes<N>> = static_cast<int>(N);

template <typename T>
consteval void TakeMaxBytes(int& v) {
  if constexpr (kMaxBytesTag<T> >= 0) {
    v = kMaxBytesTag<T>;
  }
}

template <typename Cell, typename... Opts>
consteval int PolicyMaxBytes(wire::AutoTiered<Cell, Opts...> const*) {
  int v = 8;
  (TakeMaxBytes<Opts>(v), ...);
  return v;
}

template <typename WirePolicy>
consteval int FormatMaxBytes() {
  return PolicyMaxBytes(static_cast<WirePolicy const*>(nullptr));
}

consteval bool GeomFromUpper(CurveDraft const& d) {
  return d.step_mode == StepMode::kUpperExplicit ||
         d.step_mode == StepMode::kUpperInherit;
}

consteval SegWork GeomUpperStep(CurveDraft const& d) {
  if (d.step_mode == StepMode::kUpperInherit) {
    return d.last_step;
  }
  if (WorkPositive(d.specified_step)) {
    return d.specified_step;
  }
  return d.last_step;
}

consteval SegWork LerpWork(SegWork a, SegWork b, int i, int n) {
  if (n <= 0 || i <= 0) {
    return a;
  }
  if (i >= n) {
    return b;
  }
  SegWork const span = SubTo<SegWork>(b, a);
  SegWork const t = ConvertFixed<SegWork>(SegPosWork::FromRatio(i, n));
  return AddTo<SegWork>(a, MulWorkSame(span, t));
}

consteval SegWork DecodeMath(CurveDraft const& d, int i) {
  if (d.intervals <= 0) {
    return d.begin;
  }
  if (i <= 0) {
    return d.begin;
  }
  if (i >= d.intervals) {
    return d.end;
  }
  if (d.kind == CurveKind::kUniformStep ||
      d.kind == CurveKind::kUniformValues) {
    return LerpWork(d.begin, d.end, i, d.intervals);
  }
  if (d.kind == CurveKind::kExponentialValues) {
    SegLog const arg =
        AddTo<SegLog>(Log2OfRat(d.begin_rat), MulLogInt(d.log2_r, i));
    return Exp2Work(arg);
  }
  if (d.kind == CurveKind::kGeometricStep) {
    return GeomInterp(d.begin, d.end, d.log2_q, i, d.intervals,
                      GeomFromUpper(d));
  }
  std::uint32_t const n_u = static_cast<std::uint32_t>(i);
  std::uint32_t const n_all = static_cast<std::uint32_t>(d.intervals);
  std::uint32_t const pair_i = n_u * (n_u - 1U) / 2U;
  std::uint32_t const pair_n = n_all * (n_all - 1U) / 2U;
  SegWork const num = AddTo<SegWork>(
      ScaleWorkByInt(d.step0, i),
      ScaleWorkByInt(d.delta, static_cast<int>(pair_i)));
  SegWork const den =
      AddTo<SegWork>(ScaleWorkByInt(d.step0, d.intervals),
                     ScaleWorkByInt(d.delta, static_cast<int>(pair_n)));
  if (den.RawValue() == static_cast<typename SegWork::rep_value_type>(0)) {
    return d.begin;
  }
  SegWork const span = SubTo<SegWork>(d.end, d.begin);
  return AddTo<SegWork>(d.begin, MulWorkSame(span, DivTo<SegWork>(num, den)));
}

consteval SegWork FirstAbsStep(CurveDraft const& d) {
  return WorkAbs(SubTo<SegWork>(DecodeMath(d, 1), DecodeMath(d, 0)));
}

consteval SegWork LastAbsStep(CurveDraft const& d) {
  return WorkAbs(SubTo<SegWork>(DecodeMath(d, d.intervals),
                                DecodeMath(d, d.intervals - 1)));
}

consteval void AssignOwnership(CurveDraft* d, int n) {
  if (n <= 0) {
    SegmentedSpecError();
    return;
  }
  d[0].own_begin = true;
  d[n - 1].own_end = true;
  for (int i = 0; i < n - 1; ++i) {
    if (d[i].end_rat.num != d[i + 1].begin_rat.num ||
        d[i].end_rat.den != d[i + 1].begin_rat.den) {
      if (RatLess(d[i + 1].begin_rat, d[i].end_rat) ||
          RatLess(d[i].end_rat, d[i + 1].begin_rat)) {
        SegmentedSpecError();
      }
    }
    int const bi = d[i].bytes == 0 ? 1 : d[i].bytes;
    int const bj = d[i + 1].bytes == 0 ? 1 : d[i + 1].bytes;
    if (bi < bj) {
      d[i].own_end = true;
      d[i + 1].own_begin = false;
    } else if (bi > bj) {
      d[i].own_end = false;
      d[i + 1].own_begin = true;
    } else {
      d[i].own_end = true;
      d[i + 1].own_begin = false;
    }
  }
  if (n == 1) {
    d[0].own_end = true;
  }
}

consteval int StoredOf(CurveDraft const& d) {
  return d.intervals + (d.own_begin ? 1 : 0) + (d.own_end ? 1 : 0) - 1;
}

consteval SegRatio SolveQFromStepRat(int intervals, SegWork span, SegWork step0,
                                       Rat span_r, Rat step_r) {
  if (step_r.num == 0 || step_r.den == 0) {
    return SolveQForGeomSum(intervals, DivTo<SegWork>(span, step0));
  }
  std::int32_t const sn = span_r.num < 0 ? -span_r.num : span_r.num;
  std::int32_t const sd = span_r.den < 0 ? -span_r.den : span_r.den;
  std::int32_t const tn = step_r.num < 0 ? -step_r.num : step_r.num;
  std::int32_t const td = step_r.den < 0 ? -step_r.den : step_r.den;
  return SolveQForGeomSum(intervals, WorkFromRat({sn * td, sd * tn}));
}

consteval void ComputeCoeffsKnownN(CurveDraft& d) {
  if (d.intervals <= 0) {
    return;
  }
  SegWork const span = SubTo<SegWork>(d.end, d.begin);
  if (d.kind == CurveKind::kUniformStep ||
      d.kind == CurveKind::kUniformValues) {
    Rat const span_r = RatSub(d.end_rat, d.begin_rat);
    std::int32_t const den = span_r.den * d.intervals;
    if (den == 0) {
      SegmentedSpecError();
      return;
    }
    d.step0 = SegWork::FromRatio(span_r.num, den);
    d.last_step = d.step0;
    d.delta = WorkZero();
    d.r = RatioOne();
    d.q = RatioOne();
    return;
  }
  if (d.kind == CurveKind::kExponentialValues) {
    d.log2_r = Log2RFromEndpoints(d.begin_rat, d.end_rat, d.intervals);
    d.r = Exp2Ratio(d.log2_r);
    d.step0 = ScaleWorkByRatio(d.begin, SubTo<SegRatio>(d.r, RatioOne()));
    SegRatio const rel_last =
        DivTo<SegRatio>(SubTo<SegRatio>(d.r, RatioOne()), d.r);
    d.last_step = ScaleWorkByRatio(d.end, rel_last);
    return;
  }
  if (d.kind == CurveKind::kGeometricStep) {
    Rat const span_r = RatSub(d.end_rat, d.begin_rat);
    if (d.step_mode == StepMode::kLowerExplicit &&
        WorkPositive(d.specified_step)) {
      d.step0 = d.specified_step;
      d.q = SolveQFromStepRat(d.intervals, span, d.step0, span_r,
                               d.specified_step_rat);
      d.log2_q = Log2Ratio(d.q);
    } else if (d.step_mode == StepMode::kUpperExplicit &&
               WorkPositive(d.specified_step)) {
      d.last_step = d.specified_step;
      d.q = SolveQFromStepRat(d.intervals, span, d.step0, span_r,
                               d.specified_step_rat);
      d.log2_q = Log2Ratio(d.q);
    } else if (d.step_mode == StepMode::kLowerInherit &&
               WorkPositive(d.step0)) {
      d.q = SolveQForGeomSum(d.intervals, DivTo<SegWork>(span, d.step0));
      d.log2_q = Log2Ratio(d.q);
    } else if (d.step_mode == StepMode::kUpperInherit &&
               WorkPositive(d.last_step)) {
      d.q = SolveQForGeomSum(d.intervals, DivTo<SegWork>(span, d.last_step));
      d.log2_q = Log2Ratio(d.q);
    }
    if (d.intervals >= 1) {
      SegWork const qpow =
          Exp2Work(MulLogInt(d.log2_q, d.intervals - 1));
      if (GeomFromUpper(d) && WorkPositive(d.last_step)) {
        d.step0 = MulWorkSame(d.last_step, qpow);
      } else if (WorkPositive(d.step0)) {
        d.last_step = MulWorkSame(d.step0, qpow);
      }
    }
    return;
  }
  if (d.kind == CurveKind::kLinearStepRamp) {
    if (d.step_mode == StepMode::kLowerExplicit) {
      d.step0 = d.specified_step;
    }
    if (d.step_mode == StepMode::kUpperExplicit) {
      d.last_step = d.specified_step;
    }
    Rat const span_r = RatSub(d.end_rat, d.begin_rat);
    SegWork const mean = SegWork::FromRatio(
        span_r.num * 2, span_r.den * d.intervals);
    if (WorkPositive(d.step0) && !WorkPositive(d.last_step) &&
        d.intervals > 0) {
      d.last_step = SubTo<SegWork>(mean, d.step0);
    }
    if (WorkPositive(d.last_step) && !WorkPositive(d.step0) &&
        d.intervals > 0) {
      d.step0 = SubTo<SegWork>(mean, d.last_step);
    }
    if (d.intervals > 1 && WorkPositive(d.step0) && WorkPositive(d.last_step)) {
      d.delta = DivTo<SegWork>(SubTo<SegWork>(d.last_step, d.step0),
                               SegWork::FromRuntimeInteger(d.intervals - 1));
    }
    if (d.has_max_err_lower &&
        AddTo<SegWork>(d.max_err_lower, d.max_err_lower) < d.step0) {
      SegmentedSpecError();
    }
    if (d.has_max_err_upper &&
        AddTo<SegWork>(d.max_err_upper, d.max_err_upper) < d.last_step) {
      SegmentedSpecError();
    }
  }
}

consteval bool TryInherit(CurveDraft* d, int n) {
  bool changed = false;
  for (int i = 0; i < n; ++i) {
    if (d[i].step_mode == StepMode::kLowerInherit && i > 0) {
      SegWork prev_last = d[i - 1].last_step;
      if (!WorkPositive(prev_last) && d[i - 1].intervals > 0) {
        prev_last = LastAbsStep(d[i - 1]);
      }
      if (WorkPositive(prev_last)) {
        d[i].step0 = prev_last;
        changed = true;
      }
    }
    if (d[i].step_mode == StepMode::kUpperInherit && i + 1 < n) {
      SegWork nxt = WorkZero();
      if (WorkPositive(d[i + 1].step0)) {
        nxt = d[i + 1].step0;
      } else if (d[i + 1].intervals > 0 &&
                 (d[i + 1].kind == CurveKind::kUniformValues ||
                  d[i + 1].kind == CurveKind::kUniformStep ||
                  d[i + 1].kind == CurveKind::kExponentialValues)) {
        nxt = FirstAbsStep(d[i + 1]);
      }
      if (WorkPositive(nxt)) {
        d[i].last_step = nxt;
        changed = true;
      }
    }
  }
  return changed;
}

struct LogicalPlan {
  std::array<CurveDraft, kMaxDrafts> segs{};
  int count = 0;
  std::uint32_t n1 = 0;
  std::uint32_t n2 = 0;
  std::uint32_t n4 = 0;
  std::uint32_t n8 = 0;
  std::uint32_t code_count = 0;
  int max_bytes = 1;
  std::int32_t max_abs_ceil = 1;
  Rat declared_min{};
  Rat declared_max{};
  // Compile-time FNV-1a schema identity only — not encode/decode math.
  std::uint64_t schema_hash = 0;
};

consteval void AssignWire(CurveDraft* d, int n, LogicalPlan& plan) {
  std::uint32_t next = 0;
  int const order[4] = {1, 2, 4, 8};
  std::uint32_t* counts[4] = {&plan.n1, &plan.n2, &plan.n4, &plan.n8};
  for (int t = 0; t < 4; ++t) {
    for (int i = 0; i < n; ++i) {
      if (d[i].bytes == order[t]) {
        d[i].wire_begin = next;
        if (d[i].stored <= 0) {
          d[i].stored = StoredOf(d[i]);
          d[i].math_first = d[i].own_begin ? 0 : 1;
        } else if (!d[i].own_begin && d[i].math_first == 0) {
          d[i].math_first = 1;
        }
        if (d[i].stored <= 0) {
          SegmentedSpecError();
        }
        int const last_math = d[i].math_first + d[i].stored - 1;
        d[i].phys_begin = DecodeMath(d[i], d[i].math_first);
        d[i].phys_end = DecodeMath(d[i], last_math);
        next += static_cast<std::uint32_t>(d[i].stored);
        *counts[t] += static_cast<std::uint32_t>(d[i].stored);
      }
    }
  }
  plan.code_count = next;
  if (plan.code_count == 0) {
    SegmentedSpecError();
  }
}

consteval void PushContExpSlice(LogicalPlan& out, int& w, CurveDraft const& s,
                                int math_lo, int math_hi, int bytes,
                                int stored) {
  CurveDraft c = s;
  c.is_cont_exp = false;
  c.bytes = bytes;
  c.math_first = math_lo;
  c.own_begin = true;
  c.own_end = true;
  c.stored = stored;
  c.phys_begin = DecodeMath(s, math_lo);
  c.phys_end = DecodeMath(s, math_hi);
  out.segs[static_cast<std::size_t>(w)] = c;
  ++w;
}

consteval LogicalPlan SplitContExp(LogicalPlan in) {
  LogicalPlan out{};
  out.max_bytes = in.max_bytes;
  out.max_abs_ceil = in.max_abs_ceil;
  out.declared_min = in.declared_min;
  out.declared_max = in.declared_max;
  int w = 0;
  for (int i = 0; i < in.count; ++i) {
    CurveDraft const& s = in.segs[static_cast<std::size_t>(i)];
    if (!s.is_cont_exp) {
      out.segs[static_cast<std::size_t>(w++)] = s;
      continue;
    }
    int const n = s.intervals;
    int const a = s.last_1;
    int const b = s.last_2;
    PushContExpSlice(out, w, s, 0, a, 1, a + 1);
    PushContExpSlice(out, w, s, a + 1, b, 2, b - a);
    PushContExpSlice(out, w, s, b + 1, n, 4, n - b);
  }
  out.count = w;
  AssignWire(out.segs.data(), out.count, out);
  return out;
}

consteval std::uint64_t HashDraft(std::uint64_t h, CurveDraft const& d) {
  h = MixHash(h, static_cast<std::uint64_t>(d.kind));
  h = MixRat(h, d.begin_rat);
  h = MixRat(h, d.end_rat);
  h = MixHash(h, static_cast<std::uint64_t>(d.intervals));
  h = MixHash(h, static_cast<std::uint64_t>(d.bytes));
  h = MixHash(h, static_cast<std::uint64_t>(d.step_mode));
  h = MixRat(h, d.specified_step_rat);
  h = MixHash(h, d.fill_tier ? 1U : 0U);
  h = MixHash(h, d.min_intervals ? 1U : 0U);
  h = MixHash(h, d.has_max_err_upper ? 1U : 0U);
  h = MixRat(h, d.max_err_upper_rat);
  h = MixHash(h, d.has_max_err_lower ? 1U : 0U);
  h = MixRat(h, d.max_err_lower_rat);
  h = MixHash(h, d.is_cont_exp ? 1U : 0U);
  h = MixRat(h, d.cut1_rat);
  h = MixRat(h, d.cut2_rat);
  h = MixHash(h, static_cast<std::uint64_t>(d.last_1));
  h = MixHash(h, static_cast<std::uint64_t>(d.last_2));
  h = MixHash(h, d.own_begin ? 1U : 0U);
  h = MixHash(h, d.own_end ? 1U : 0U);
  h = MixHash(h, static_cast<std::uint64_t>(d.stored));
  h = MixHash(h, static_cast<std::uint64_t>(d.math_first));
  return h;
}

template <typename LayoutT>
consteval LogicalPlan MakeUnsplitPlan(int max_bytes) {
  LogicalPlan plan{};
  plan.max_bytes = max_bytes;
  int idx = 0;
  int as_id = 1;
  FlattenLayout<LayoutT>::Fill(plan.segs.data(), idx, as_id);
  plan.count = idx;
  if (plan.count <= 0 || plan.count > kMaxDrafts) {
    SegmentedSpecError();
  }

  plan.declared_min = plan.segs[0].begin_rat;
  plan.declared_max = plan.segs[0].end_rat;
  Rat max_abs = RatAbsMax(plan.segs[0].begin_rat, plan.segs[0].end_rat);
  for (int i = 0; i < plan.count; ++i) {
    CurveDraft& d = plan.segs[static_cast<std::size_t>(i)];
    if (d.bytes > max_bytes && d.bytes != 0) {
      SegmentedSpecError();
    }
    max_abs = RatAbsMax(max_abs, d.begin_rat);
    max_abs = RatAbsMax(max_abs, d.end_rat);
    if (RatLess(d.begin_rat, plan.declared_min)) {
      plan.declared_min = d.begin_rat;
    }
    if (RatLess(plan.declared_max, d.end_rat)) {
      plan.declared_max = d.end_rat;
    }
  }
  plan.max_abs_ceil = CeilAbsRat(max_abs);

  for (int i = 0; i < plan.count; ++i) {
    CurveDraft& d = plan.segs[static_cast<std::size_t>(i)];
    if (d.autosplit_id != 0 && d.kind == CurveKind::kExponentialValues &&
        d.intervals < 0) {
      int j = i;
      while (j < plan.count &&
             plan.segs[static_cast<std::size_t>(j)].autosplit_id ==
                 d.autosplit_id) {
        ++j;
      }
      if (j - i != 2) {
        SegmentedSpecError();
      }
      CurveDraft& a = plan.segs[static_cast<std::size_t>(i)];
      CurveDraft& b = plan.segs[static_cast<std::size_t>(i + 1)];
      if (a.end_rat.num != b.begin_rat.num ||
          a.end_rat.den != b.begin_rat.den) {
        SegmentedSpecError();
      }
      auto const sp = AutoSplitTwoExp(
          a.begin_rat, a.end_rat, b.end_rat, a.total_values - 1);
      a.intervals = sp.n1;
      b.intervals = sp.n2;
      a.log2_r = sp.log2_r1;
      b.log2_r = sp.log2_r2;
      a.r = sp.r1;
      b.r = sp.r2;
      i = j - 1;
    }
  }

  for (int i = 0; i < plan.count; ++i) {
    CurveDraft& d = plan.segs[static_cast<std::size_t>(i)];
    if (d.is_cont_exp) {
      bool const has4 = max_bytes >= 4;
      int const last1 = (max_bytes >= 2) ? 254 : 255;
      int const max_last2 =
          has4 ? static_cast<int>(TwoTierMaxU8(
                                    static_cast<std::uint32_t>(last1)) -
                                1U)
               : last1 + 1;
      auto const ce = OptimizeContinuousExp(d.begin_rat, d.end_rat, d.cut1_rat,
                                            d.cut2_rat, last1, last1 + 2, 1100,
                                            max_last2);
      d.intervals = ce.intervals;
      d.last_1 = ce.last_1;
      d.last_2 = ce.last_2;
      d.log2_r = ce.log2_r;
      d.r = ce.r;
    }
    if (d.kind == CurveKind::kUniformStep && d.intervals < 0 &&
        (WorkPositive(d.specified_step) || d.specified_step_rat.num != 0)) {
      d.intervals = RoundRatQuotient(RatSub(d.end_rat, d.begin_rat),
                                     d.specified_step_rat);
      if (d.intervals < 1) {
        SegmentedSpecError();
      }
    }
  }

  AssignOwnership(plan.segs.data(), plan.count);

  bool has2 = false;
  bool has4 = false;
  for (int i = 0; i < plan.count; ++i) {
    if (plan.segs[static_cast<std::size_t>(i)].is_cont_exp) {
      has2 = true;
      has4 = max_bytes >= 4;
    }
    if (plan.segs[static_cast<std::size_t>(i)].bytes == 2) {
      has2 = true;
    }
    if (plan.segs[static_cast<std::size_t>(i)].bytes == 4) {
      has4 = true;
    }
  }

  int n1_known = 0;
  for (int i = 0; i < plan.count; ++i) {
    CurveDraft& d = plan.segs[static_cast<std::size_t>(i)];
    if (d.bytes == 1 && d.intervals >= 0 && !d.fill_tier) {
      n1_known += StoredOf(d);
    }
    if (d.is_cont_exp) {
      n1_known += d.last_1 + 1;
    }
  }

  for (int i = 0; i < plan.count; ++i) {
    CurveDraft& d = plan.segs[static_cast<std::size_t>(i)];
    if (!d.fill_tier) {
      continue;
    }
    int cap = 0;
    if (d.bytes == 1) {
      cap = has2 ? 255 : 256;
      cap -= n1_known;
    } else if (d.bytes == 2) {
      std::uint32_t const b0 =
          n1_known > 0 ? static_cast<std::uint32_t>(n1_known - 1) : 0;
      std::uint32_t const tmax = TwoTierMaxU8(b0);
      cap = has4 ? static_cast<int>(tmax - n1_known)
                 : static_cast<int>(tmax + 1U - n1_known);
    }
    if (cap < 1) {
      SegmentedSpecError();
    }
    d.stored = cap;
    d.intervals = cap - (d.own_begin ? 1 : 0) - (d.own_end ? 1 : 0) + 1;
  }

  for (int pass = 0; pass < 2; ++pass) {
    for (int i = 0; i < plan.count; ++i) {
      ComputeCoeffsKnownN(plan.segs[static_cast<std::size_t>(i)]);
    }
    TryInherit(plan.segs.data(), plan.count);
  }

  for (int i = 0; i < plan.count; ++i) {
    CurveDraft& d = plan.segs[static_cast<std::size_t>(i)];
    if (d.min_intervals) {
      if (!WorkPositive(d.step0) && i > 0) {
        d.step0 = LastAbsStep(plan.segs[static_cast<std::size_t>(i - 1)]);
      }
      if (!WorkPositive(d.step0) || !d.has_max_err_upper) {
        SegmentedSpecError();
      }
      d.intervals = MinRampIntervals(SubTo<SegWork>(d.end, d.begin), d.step0,
                                      d.max_err_upper);
    }
  }

  for (int pass = 0; pass < 1; ++pass) {
    for (int i = 0; i < plan.count; ++i) {
      ComputeCoeffsKnownN(plan.segs[static_cast<std::size_t>(i)]);
    }
  }

  for (int i = 0; i < plan.count; ++i) {
    CurveDraft const& d = plan.segs[static_cast<std::size_t>(i)];
    if (d.intervals < 1 && !d.is_cont_exp) {
      SegmentedSpecError();
    }
    if (d.step_mode == StepMode::kLowerInherit && !WorkPositive(d.step0)) {
      SegmentedSpecError();
    }
  }

  plan.schema_hash =
      MixHash(0xcbf29ce484222325ULL, static_cast<std::uint64_t>(plan.count));
  plan.schema_hash =
      MixHash(plan.schema_hash, static_cast<std::uint64_t>(max_bytes));
  plan.schema_hash = MixRat(plan.schema_hash, plan.declared_min);
  plan.schema_hash = MixRat(plan.schema_hash, plan.declared_max);
  for (int i = 0; i < plan.count; ++i) {
    plan.schema_hash =
        HashDraft(plan.schema_hash, plan.segs[static_cast<std::size_t>(i)]);
  }
  return plan;
}

template <typename Spec>
consteval LogicalPlan CompileLogical() {
  constexpr int kMaxB = FormatMaxBytes<typename Spec::wire_policy>();
  auto unsplit = MakeUnsplitPlan<typename Spec::layout_type>(kMaxB);
  auto plan = SplitContExp(unsplit);
  if (plan.n1 > 256U) {
    SegmentedSpecError();
  }
  if (plan.n2 > 0 && plan.n1 > 255U) {
    SegmentedSpecError();
  }
  if (plan.n1 == 0) {
    SegmentedSpecError();
  }
  plan.max_abs_ceil = unsplit.max_abs_ceil;
  plan.declared_min = unsplit.declared_min;
  plan.declared_max = unsplit.declared_max;
  plan.schema_hash = MixHash(unsplit.schema_hash, plan.code_count);
  plan.schema_hash = MixHash(plan.schema_hash, plan.n1);
  plan.schema_hash = MixHash(plan.schema_hash, plan.n2);
  plan.schema_hash = MixHash(plan.schema_hash, plan.n4);
  plan.schema_hash = MixHash(plan.schema_hash, plan.n8);
  for (int i = 0; i < plan.count; ++i) {
    plan.schema_hash =
        HashDraft(plan.schema_hash, plan.segs[static_cast<std::size_t>(i)]);
  }
  return plan;
}

struct CompiledSegment {
  std::uint32_t physical_begin_raw = 0;
  std::uint32_t physical_end_raw = 0;
  std::uint32_t wire_code_begin = 0;
  std::uint32_t code_count = 0;
  CurveKind curve_kind = CurveKind::kUniformStep;
  std::uint8_t wire_bytes = 1;
  std::int32_t intervals = 0;
  std::int32_t math_first = 0;
  std::uint32_t curve_begin_raw = 0;
  std::uint32_t curve_end_raw = 0;
  std::uint32_t step0_raw = 0;
  std::uint32_t last_step_raw = 0;
  std::uint32_t delta_raw = 0;
  SegLog log2_r = LogZero();
  SegLog log2_q = LogZero();
  SegLog log2_begin = LogZero();
  SegLog log2_end = LogZero();
  std::uint8_t from_upper = 0;
};

template <int Kind, std::uint32_t A, std::uint32_t B, std::uint32_t C>
struct WireSelKind;

template <std::uint32_t A, std::uint32_t B, std::uint32_t C>
struct WireSelKind<0, A, B, C> {
  using type = std::uint8_t;
};

template <std::uint32_t A, std::uint32_t B, std::uint32_t C>
struct WireSelKind<1, A, B, C> {
  using type = TieredInt<std::uint8_t, A>;
};

template <std::uint32_t A, std::uint32_t B, std::uint32_t C>
struct WireSelKind<2, A, B, C> {
  using type = TieredInt<std::uint8_t, A, B>;
};

template <std::uint32_t A, std::uint32_t B, std::uint32_t C>
struct WireSelKind<3, A, B, C> {
  using type = TieredInt<std::uint8_t, A, B, C>;
};

template <typename Spec>
struct PlanHolder {
  static constexpr LogicalPlan kPlan = CompileLogical<Spec>();
};

template <typename Spec>
inline constexpr int kWireKind =
    (PlanHolder<Spec>::kPlan.n2 == 0 && PlanHolder<Spec>::kPlan.n4 == 0 &&
     PlanHolder<Spec>::kPlan.n8 == 0)
        ? 0
        : ((PlanHolder<Spec>::kPlan.n4 == 0 && PlanHolder<Spec>::kPlan.n8 == 0)
               ? 1
               : (PlanHolder<Spec>::kPlan.n8 == 0 ? 2 : 3));

template <typename Spec>
using WireTypeOf = typename WireSelKind<
    kWireKind<Spec>,
    PlanHolder<Spec>::kPlan.n1 - 1U,
    PlanHolder<Spec>::kPlan.n1 + PlanHolder<Spec>::kPlan.n2 - 1U,
    PlanHolder<Spec>::kPlan.n1 + PlanHolder<Spec>::kPlan.n2 +
        PlanHolder<Spec>::kPlan.n4 - 1U>::type;

template <typename Spec>
inline constexpr auto kMaxAbsBound = PlanHolder<Spec>::kPlan.max_abs_ceil;

template <bool IsFloating, typename Spec>
struct LogicalTypeSel;

template <typename Spec>
struct LogicalTypeSel<false, Spec> {
  using type =
      FixedPoint<typename Spec::runtime_policy::rep, kMaxAbsBound<Spec>>;
};

template <typename Spec>
struct LogicalTypeSel<true, Spec> {
  using type = FixedPoint<std::int32_t, kMaxAbsBound<Spec>>;
};

template <typename Spec>
using LogicalTypeOf = typename LogicalTypeSel<
    kIsFloatingRuntimePolicy<typename Spec::runtime_policy>, Spec>::type;

template <typename Spec>
using FixedRuntimeOf = LogicalTypeOf<Spec>;

template <typename RT>
inline constexpr bool kRtSigned =
    std::is_signed_v<typename RT::rep_value_type>;

template <typename RT>
constexpr std::uint32_t PackRtRaw(typename RT::rep_value_type v) {
  if constexpr (kRtSigned<RT>) {
    return static_cast<std::uint32_t>(static_cast<std::int32_t>(v));
  } else {
    return static_cast<std::uint32_t>(v);
  }
}

template <typename RT>
consteval bool StoredRawLess(std::uint32_t a, std::uint32_t b) {
  if constexpr (kRtSigned<RT>) {
    return static_cast<std::int32_t>(a) < static_cast<std::int32_t>(b);
  } else {
    return a < b;
  }
}

template <typename RT>
consteval std::uint32_t StoredRawAbsDiff(std::uint32_t a, std::uint32_t b) {
  if constexpr (kRtSigned<RT>) {
    auto const ia = static_cast<std::int32_t>(a);
    auto const ib = static_cast<std::int32_t>(b);
    auto const ua = static_cast<std::uint32_t>(ia);
    auto const ub = static_cast<std::uint32_t>(ib);
    return ia >= ib ? ua - ub : ub - ua;
  } else {
    return a >= b ? a - b : b - a;
  }
}

template <typename RT>
consteval std::uint32_t RawFromWork(SegWork w) {
  return PackRtRaw<RT>(ConvertFixed<RT>(w).RawValue());
}

template <typename RT>
consteval std::uint32_t RawFromRat(Rat r) {
  return PackRtRaw<RT>(RT::FromRatio(r.num, r.den).RawValue());
}

template <typename RT>
consteval std::uint32_t RawAt(CurveDraft const& d, int i) {
  if (i <= 0) {
    return RawFromRat<RT>(d.begin_rat);
  }
  if (i >= d.intervals) {
    return RawFromRat<RT>(d.end_rat);
  }
  std::uint32_t raw = RawFromWork<RT>(DecodeMath(d, i));
  std::uint32_t const b = RawFromRat<RT>(d.begin_rat);
  std::uint32_t const e = RawFromRat<RT>(d.end_rat);
  bool const rising = !StoredRawLess<RT>(e, b);
  if (raw == e) {
    if constexpr (kRtSigned<RT>) {
      auto v = static_cast<std::int32_t>(raw);
      v += rising ? -1 : 1;
      raw = static_cast<std::uint32_t>(v);
    } else if (rising) {
      raw -= 1U;
    } else {
      raw += 1U;
    }
  } else if (raw == b) {
    if constexpr (kRtSigned<RT>) {
      auto v = static_cast<std::int32_t>(raw);
      v += rising ? 1 : -1;
      raw = static_cast<std::uint32_t>(v);
    } else if (rising) {
      raw += 1U;
    } else {
      raw -= 1U;
    }
  }
  return raw;
}

template <typename RT>
consteval CompiledSegment CompileOne(CurveDraft const& d) {
  CompiledSegment c{};
  c.physical_begin_raw = RawAt<RT>(d, d.math_first);
  c.physical_end_raw = RawAt<RT>(d, d.math_first + d.stored - 1);
  if (StoredRawLess<RT>(c.physical_end_raw, c.physical_begin_raw)) {
    auto const t = c.physical_begin_raw;
    c.physical_begin_raw = c.physical_end_raw;
    c.physical_end_raw = t;
  }
  c.wire_code_begin = d.wire_begin;
  c.code_count = static_cast<std::uint32_t>(d.stored);
  c.curve_kind = d.kind;
  c.wire_bytes = static_cast<std::uint8_t>(d.bytes);
  c.intervals = d.intervals;
  c.math_first = d.math_first;
  c.curve_begin_raw = RawFromRat<RT>(d.begin_rat);
  c.curve_end_raw = RawFromRat<RT>(d.end_rat);
  c.step0_raw = RawFromWork<RT>(d.step0);
  c.last_step_raw = RawFromWork<RT>(d.last_step);
  c.delta_raw = RawFromWork<RT>(d.delta);
  if (d.kind == CurveKind::kLinearStepRamp && d.intervals > 0) {
    std::uint32_t const span =
        StoredRawAbsDiff<RT>(c.curve_end_raw, c.curve_begin_raw);
    std::uint32_t const n = static_cast<std::uint32_t>(d.intervals);
    std::uint32_t mean2u = 0;
    if (!integer_math::MulDivU32Nearest(span, 2U, n, mean2u) ||
        mean2u > static_cast<std::uint32_t>(
                     std::numeric_limits<std::int32_t>::max())) {
      SegmentedSpecError();
    }
    bool const rising =
        !StoredRawLess<RT>(c.curve_end_raw, c.curve_begin_raw);
    std::int32_t const mean2 =
        rising ? static_cast<std::int32_t>(mean2u)
               : -static_cast<std::int32_t>(mean2u);
    auto const last_i = static_cast<std::int32_t>(c.last_step_raw);
    auto const step0_i = static_cast<std::int32_t>(c.step0_raw);
    bool const keep_last = d.step_mode == StepMode::kUpperExplicit ||
                           d.step_mode == StepMode::kUpperInherit;
    std::int32_t new_step0 = step0_i;
    std::int32_t new_last = last_i;
    if (keep_last) {
      new_step0 = mean2 - last_i;
    } else {
      new_last = mean2 - step0_i;
    }
    c.step0_raw = static_cast<std::uint32_t>(new_step0);
    c.last_step_raw = static_cast<std::uint32_t>(new_last);
    if (n > 1U) {
      c.delta_raw = static_cast<std::uint32_t>(
          fixed_point_internal::RoundDivNearest(
              new_last - new_step0, static_cast<std::int32_t>(n - 1U)));
    }
  }
  c.from_upper = GeomFromUpper(d) ? 1 : 0;
  if (d.kind == CurveKind::kExponentialValues && d.begin_rat.num > 0) {
    c.log2_r = d.log2_r;
    c.log2_begin = Log2OfRat(d.begin_rat);
    c.log2_end = Log2OfRat(d.end_rat);
  }
  if (d.kind == CurveKind::kGeometricStep) {
    c.log2_q = d.log2_q;
  }
  return c;
}

template <typename Spec, typename RT>
consteval std::array<CompiledSegment, kMaxDrafts> MakeCompiledSegments() {
  constexpr LogicalPlan kPlan = PlanHolder<Spec>::kPlan;
  std::array<CompiledSegment, kMaxDrafts> out{};
  for (int i = 0; i < kPlan.count; ++i) {
    CompiledSegment const c =
        CompileOne<RT>(kPlan.segs[static_cast<std::size_t>(i)]);
    std::uint32_t const span =
        StoredRawAbsDiff<RT>(c.physical_end_raw, c.physical_begin_raw);
    if (c.code_count > span + 1U) {
      SegmentedSpecError();
    }
    out[static_cast<std::size_t>(i)] = c;
  }
  for (int i = 1; i < kPlan.count; ++i) {
    CompiledSegment const key = out[static_cast<std::size_t>(i)];
    int j = i;
    while (j > 0 &&
           (StoredRawLess<RT>(
                key.physical_begin_raw,
                out[static_cast<std::size_t>(j - 1)].physical_begin_raw) ||
            (out[static_cast<std::size_t>(j - 1)].physical_begin_raw ==
                 key.physical_begin_raw &&
             out[static_cast<std::size_t>(j - 1)].wire_code_begin >
                 key.wire_code_begin))) {
      out[static_cast<std::size_t>(j)] = out[static_cast<std::size_t>(j - 1)];
      --j;
    }
    out[static_cast<std::size_t>(j)] = key;
  }
  return out;
}

template <typename Spec, typename RT>
struct CompiledSegHolder {
  static constexpr std::array<CompiledSegment, kMaxDrafts> kAll =
      MakeCompiledSegments<Spec, RT>();
};

template <typename Spec, typename RT, std::size_t N>
consteval std::array<CompiledSegment, N> MakeExactCompiledSegments() {
  constexpr auto all = CompiledSegHolder<Spec, RT>::kAll;
  std::array<CompiledSegment, N> out{};
  for (std::size_t i = 0; i < N; ++i) {
    out[i] = all[i];
  }
  return out;
}

consteval std::size_t PackedDescriptorBytes(CurveKind kind) {
  if (kind == CurveKind::kExponentialValues) {
    return 32;
  }
  if (kind == CurveKind::kGeometricStep) {
    return 32;
  }
  if (kind == CurveKind::kLinearStepRamp) {
    return 40;
  }
  return 24;
}

template <typename Spec>
consteval std::size_t FormulaCoefficientBytes() {
  constexpr LogicalPlan kPlan = PlanHolder<Spec>::kPlan;
  std::size_t n = 0;
  for (int i = 0; i < kPlan.count; ++i) {
    n += PackedDescriptorBytes(kPlan.segs[static_cast<std::size_t>(i)].kind);
  }
  return n;
}

}  // namespace ae::seg::detail

#endif  // AE_NUMERIC_DETAILS_SEGMENTED_COMPILER_H_
