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

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

#include <gcem.hpp>

#include "ae-numeric/details/segmented_curves.h"
#include "ae-numeric/details/segmented_format.h"
#include "ae-numeric/details/segmented_math.h"
#include "ae-numeric/fixed_math.h"
#include "ae-numeric/fixed_point.h"
#include "ae-numeric/tiered_int.h"

namespace ae::seg::segmented_compiler_internal {

using segmented_curves_internal::CurveDraft;
using segmented_curves_internal::FlattenLayout;
using segmented_curves_internal::kMaxDrafts;
using segmented_curves_internal::StepMode;
using segmented_math_internal::AutoSplitTwoExp;
using segmented_math_internal::ExpRatio;
using segmented_math_internal::GeomSum;
using segmented_math_internal::MinRampIntervals;
using segmented_math_internal::MixHash;
using segmented_math_internal::OptimizeContinuousExp;
using segmented_math_internal::SegmentedSpecError;
using segmented_math_internal::SolveQForGeomSum;
using segmented_math_internal::ThreeTierMaxU8;
using segmented_math_internal::TwoTierMaxU8;

inline constexpr double kEps = 1.0e-12;

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

consteval bool NearlyEqual(double a, double b) {
  double const s = gcem::abs(a) > gcem::abs(b) ? gcem::abs(a) : gcem::abs(b);
  double const tol = kEps * (s > 1.0 ? s : 1.0);
  return gcem::abs(a - b) <= tol;
}

consteval int RoundN(double x) {
  if (x < 0.0) {
    return static_cast<int>(x - 0.5);
  }
  return static_cast<int>(x + 0.5);
}

consteval bool GeomFromUpper(CurveDraft const& d) {
  return d.step_mode == StepMode::kUpperExplicit ||
         d.step_mode == StepMode::kUpperInherit;
}

consteval double GeomUpperStep(CurveDraft const& d) {
  if (d.step_mode == StepMode::kUpperInherit) {
    return d.last_step;
  }
  if (d.specified_step != 0.0) {
    return d.specified_step;
  }
  return d.last_step;
}

consteval double DecodeMath(CurveDraft const& d, int i) {
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
    return d.begin + (d.end - d.begin) *
                         static_cast<double>(i) /
                         static_cast<double>(d.intervals);
  }
  if (d.kind == CurveKind::kExponentialValues) {
    return d.begin * gcem::pow(d.r, static_cast<double>(i));
  }
  if (d.kind == CurveKind::kGeometricStep) {
    if (GeomFromUpper(d)) {
      double const se = GeomUpperStep(d);
      return d.end - se * GeomSum(d.q, d.intervals - i);
    }
    return d.begin + d.step0 * GeomSum(d.q, i);
  }
  return d.begin + static_cast<double>(i) * d.step0 +
         d.delta * static_cast<double>(i) * static_cast<double>(i - 1) / 2.0;
}

consteval double FirstAbsStep(CurveDraft const& d) {
  return gcem::abs(DecodeMath(d, 1) - DecodeMath(d, 0));
}

consteval double LastAbsStep(CurveDraft const& d) {
  return gcem::abs(DecodeMath(d, d.intervals) - DecodeMath(d, d.intervals - 1));
}

consteval void AssignOwnership(CurveDraft* d, int n) {
  if (n <= 0) {
    SegmentedSpecError();
    return;
  }
  d[0].own_begin = true;
  d[n - 1].own_end = true;
  for (int i = 0; i < n - 1; ++i) {
    if (!NearlyEqual(d[i].end, d[i + 1].begin)) {
      if (d[i + 1].begin < d[i].end) {
        SegmentedSpecError();
      } else {
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

consteval bool HasBytes(CurveDraft const* d, int n, int bytes) {
  for (int i = 0; i < n; ++i) {
    if (d[i].bytes == bytes || (d[i].is_cont_exp && bytes >= 1)) {
      if (d[i].is_cont_exp) {
        return bytes == 1 || bytes == 2 || bytes == 4;
      }
      if (d[i].bytes == bytes) {
        return true;
      }
    }
  }
  return false;
}

consteval void ComputeCoeffsKnownN(CurveDraft& d) {
  if (d.intervals <= 0) {
    return;
  }
  double const span = d.end - d.begin;
  if (d.kind == CurveKind::kUniformStep ||
      d.kind == CurveKind::kUniformValues) {
    d.step0 = span / static_cast<double>(d.intervals);
    d.last_step = d.step0;
    d.delta = 0.0;
    d.r = 1.0;
    d.q = 1.0;
    return;
  }
  if (d.kind == CurveKind::kExponentialValues) {
    d.r = ExpRatio(d.begin, d.end, d.intervals);
    d.step0 = d.begin * (d.r - 1.0);
    d.last_step = d.end * (1.0 - 1.0 / d.r);
    return;
  }
  if (d.kind == CurveKind::kGeometricStep) {
    if (d.step_mode == StepMode::kLowerExplicit && d.specified_step > 0.0) {
      d.step0 = d.specified_step;
      d.q = SolveQForGeomSum(d.intervals, span / d.step0);
      d.last_step = d.step0 * gcem::pow(d.q, d.intervals - 1);
    } else if (d.step_mode == StepMode::kUpperExplicit &&
               d.specified_step > 0.0) {
      d.last_step = d.specified_step;
      d.q = SolveQForGeomSum(d.intervals, span / d.last_step);
      d.step0 = d.last_step * gcem::pow(d.q, d.intervals - 1);
    } else if (d.step_mode == StepMode::kLowerInherit && d.step0 > 0.0) {
      d.q = SolveQForGeomSum(d.intervals, span / d.step0);
      d.last_step = d.step0 * gcem::pow(d.q, d.intervals - 1);
    } else if (d.step_mode == StepMode::kUpperInherit && d.last_step > 0.0) {
      d.q = SolveQForGeomSum(d.intervals, span / d.last_step);
      d.step0 = d.last_step * gcem::pow(d.q, d.intervals - 1);
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
    if (d.step0 > 0.0 && d.last_step > 0.0) {
      double const mean = 2.0 * span / static_cast<double>(d.intervals);
      (void)mean;
    }
    if (d.step0 > 0.0 && d.last_step <= 0.0 && d.intervals > 0) {
      d.last_step = 2.0 * span / static_cast<double>(d.intervals) - d.step0;
    }
    if (d.last_step > 0.0 && d.step0 <= 0.0 && d.intervals > 0 &&
        d.step_mode != StepMode::kLowerInherit) {
      d.step0 = 2.0 * span / static_cast<double>(d.intervals) - d.last_step;
    }
    if (d.intervals > 1 && d.step0 > 0.0 && d.last_step > 0.0) {
      d.delta = (d.last_step - d.step0) /
                static_cast<double>(d.intervals - 1);
    }
    if (d.has_max_err_lower && d.step0 > 2.0 * d.max_err_lower + 1.0e-15) {
      SegmentedSpecError();
    }
    if (d.has_max_err_upper && d.last_step > 2.0 * d.max_err_upper + 1.0e-12) {
      SegmentedSpecError();
    }
  }
}

consteval bool TryInherit(CurveDraft* d, int n) {
  bool changed = false;
  for (int i = 0; i < n; ++i) {
    if (d[i].step_mode == StepMode::kLowerInherit && i > 0) {
      if (d[i - 1].last_step > 0.0) {
        d[i].step0 = d[i - 1].last_step;
        changed = true;
      }
    }
    if (d[i].step_mode == StepMode::kUpperInherit && i + 1 < n) {
      double nxt = 0.0;
      if (d[i + 1].step0 > 0.0) {
        nxt = d[i + 1].step0;
      } else if (d[i + 1].intervals > 0 &&
                 (d[i + 1].kind == CurveKind::kUniformValues ||
                  d[i + 1].kind == CurveKind::kUniformStep ||
                  d[i + 1].kind == CurveKind::kExponentialValues)) {
        nxt = FirstAbsStep(d[i + 1]);
      }
      if (nxt > 0.0) {
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
  double max_abs = 0.0;
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
          // FillTier pre-assigns stored but leaves math_first at 0.
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

consteval LogicalPlan SplitContExp(LogicalPlan in) {
  LogicalPlan out{};
  out.max_bytes = in.max_bytes;
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
    auto push_slice = [&](int math_lo, int math_hi, int bytes, int stored) {
      CurveDraft c = s;
      c.is_cont_exp = false;
      c.bytes = bytes;
      c.math_first = math_lo;
      c.own_begin = true;
      c.own_end = true;
      c.stored = stored;
      c.phys_begin = DecodeMath(s, math_lo);
      c.phys_end = DecodeMath(s, math_hi);
      out.segs[static_cast<std::size_t>(w++)] = c;
    };
    push_slice(0, a, 1, a + 1);
    push_slice(a + 1, b, 2, b - a);
    push_slice(b + 1, n, 4, n - b);
  }
  out.count = w;
  AssignWire(out.segs.data(), out.count, out);
  return out;
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

  for (int i = 0; i < plan.count; ++i) {
    CurveDraft& d = plan.segs[static_cast<std::size_t>(i)];
    if (d.bytes > max_bytes && d.bytes != 0) {
      SegmentedSpecError();
    }
    double const mag =
        gcem::abs(d.begin) > gcem::abs(d.end) ? gcem::abs(d.begin)
                                              : gcem::abs(d.end);
    if (mag > plan.max_abs) {
      plan.max_abs = mag;
    }
  }

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
      if (!NearlyEqual(a.end, b.begin)) {
        SegmentedSpecError();
      }
      auto const sp = AutoSplitTwoExp(a.begin, a.end, b.end, a.total_values - 1);
      a.intervals = sp.n1;
      b.intervals = sp.n2;
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
          has4 ? static_cast<int>(TwoTierMaxU8(static_cast<std::uint32_t>(last1)) -
                                  1U)
               : last1 + 1;
      auto const ce = OptimizeContinuousExp(d.begin, d.end, d.cut1, d.cut2, last1,
                                            last1 + 2, 2500, max_last2);
      d.intervals = ce.intervals;
      d.last_1 = ce.last_1;
      d.last_2 = ce.last_2;
      d.r = ce.r;
    }
    if (d.kind == CurveKind::kUniformStep && d.intervals < 0 &&
        d.specified_step > 0.0) {
      double const n = (d.end - d.begin) / d.specified_step;
      d.intervals = RoundN(n);
      if (d.intervals < 1) {
        SegmentedSpecError();
      }
    }
  }

  AssignOwnership(plan.segs.data(), plan.count);

  bool has2 = false;
  bool has4 = false;
  bool has8 = false;
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
    if (plan.segs[static_cast<std::size_t>(i)].bytes == 8) {
      has8 = true;
    }
  }
  (void)has8;

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
      std::uint64_t const tmax = TwoTierMaxU8(b0);
      cap = has4 ? static_cast<int>(tmax - static_cast<std::uint64_t>(n1_known))
                 : static_cast<int>(tmax + 1U -
                                    static_cast<std::uint64_t>(n1_known));
    }
    if (cap < 1) {
      SegmentedSpecError();
    }
    d.stored = cap;
    d.intervals = cap - (d.own_begin ? 1 : 0) - (d.own_end ? 1 : 0) + 1;
  }

  for (int pass = 0; pass < 8; ++pass) {
    for (int i = 0; i < plan.count; ++i) {
      ComputeCoeffsKnownN(plan.segs[static_cast<std::size_t>(i)]);
    }
    TryInherit(plan.segs.data(), plan.count);
  }

  for (int i = 0; i < plan.count; ++i) {
    CurveDraft& d = plan.segs[static_cast<std::size_t>(i)];
    if (d.min_intervals) {
      if (d.step0 <= 0.0 || !d.has_max_err_upper) {
        SegmentedSpecError();
      }
      d.intervals = MinRampIntervals(d.end - d.begin, d.step0, d.max_err_upper);
    }
  }

  for (int pass = 0; pass < 4; ++pass) {
    for (int i = 0; i < plan.count; ++i) {
      ComputeCoeffsKnownN(plan.segs[static_cast<std::size_t>(i)]);
    }
  }

  for (int i = 0; i < plan.count; ++i) {
    CurveDraft const& d = plan.segs[static_cast<std::size_t>(i)];
    if (d.intervals < 1 && !d.is_cont_exp) {
      SegmentedSpecError();
    }
    if (d.step_mode == StepMode::kLowerInherit && d.step0 <= 0.0) {
      SegmentedSpecError();
    }
  }

  plan.schema_hash = MixHash(0xcbf29ce484222325ULL,
                             static_cast<std::uint64_t>(plan.count));
  for (int i = 0; i < plan.count; ++i) {
    CurveDraft const& d = plan.segs[static_cast<std::size_t>(i)];
    plan.schema_hash = MixHash(plan.schema_hash,
                               static_cast<std::uint64_t>(d.intervals));
    plan.schema_hash = MixHash(plan.schema_hash,
                               static_cast<std::uint64_t>(d.bytes));
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
  plan.max_abs = unsplit.max_abs;
  plan.schema_hash = MixHash(unsplit.schema_hash, plan.code_count);
  return plan;
}

struct CompiledSegment {
  std::int64_t physical_begin_raw = 0;
  std::int64_t physical_end_raw = 0;
  std::uint32_t wire_code_begin = 0;
  std::uint32_t code_count = 0;
  CurveKind curve_kind = CurveKind::kUniformStep;
  std::uint8_t wire_bytes = 1;
  std::int32_t intervals = 0;
  std::int32_t math_first = 0;
  std::int64_t curve_begin_raw = 0;
  std::int64_t curve_end_raw = 0;
  std::int64_t step0_raw = 0;
  std::int64_t last_step_raw = 0;
  std::int64_t delta_raw = 0;
  std::int32_t log2_r_raw = 0;
  std::int32_t log2_q_raw = 0;
  std::int32_t log2_begin_raw = 0;
  std::int32_t ratio_raw = 0;
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
inline constexpr double kMaxAbsBound = PlanHolder<Spec>::kPlan.max_abs == 0.0
                                           ? 1.0
                                           : PlanHolder<Spec>::kPlan.max_abs;

template <bool IsFloating, typename Spec>
struct LogicalTypeSel;

template <typename Spec>
struct LogicalTypeSel<false, Spec> {
  using type = FixedPoint<typename Spec::runtime_policy::rep, kMaxAbsBound<Spec>>;
};

template <typename Spec>
struct LogicalTypeSel<true, Spec> {
  using type = FixedPoint<std::int32_t, kMaxAbsBound<Spec>>;
};

template <typename Spec>
using LogicalTypeOf =
    typename LogicalTypeSel<kIsFloatingRuntimePolicy<typename Spec::runtime_policy>,
                            Spec>::type;

template <typename Spec>
using FixedRuntimeOf = LogicalTypeOf<Spec>;

template <typename RT>
consteval std::int64_t RawAt(CurveDraft const& d, int i) {
  return static_cast<std::int64_t>(RT::FromDouble(DecodeMath(d, i)).RawValue());
}

template <typename RT>
consteval CompiledSegment CompileOne(CurveDraft const& d) {
  using Log = segmented_math_internal::SegFixedMathPolicy::log_type;
  CompiledSegment c{};
  c.physical_begin_raw = RawAt<RT>(d, d.math_first);
  c.physical_end_raw = RawAt<RT>(d, d.math_first + d.stored - 1);
  if (c.physical_end_raw < c.physical_begin_raw) {
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
  c.curve_begin_raw = RawAt<RT>(d, 0);
  c.curve_end_raw = RawAt<RT>(d, d.intervals);
  if (d.intervals >= 1) {
    c.step0_raw = RawAt<RT>(d, 1) - RawAt<RT>(d, 0);
    c.last_step_raw =
        RawAt<RT>(d, d.intervals) - RawAt<RT>(d, d.intervals - 1);
  }
  if (d.intervals >= 2) {
    c.delta_raw = (RawAt<RT>(d, 2) - RawAt<RT>(d, 1)) - c.step0_raw;
  }
  c.from_upper = GeomFromUpper(d) ? 1 : 0;
  if (d.kind == CurveKind::kExponentialValues && d.begin > 0.0 && d.r > 0.0) {
    c.log2_r_raw = static_cast<std::int32_t>(
        Log::FromDouble(gcem::log(d.r) / gcem::log(2.0)).RawValue());
    c.log2_begin_raw = static_cast<std::int32_t>(
        Log::FromDouble(gcem::log(d.begin) / gcem::log(2.0)).RawValue());
    c.ratio_raw = segmented_math_internal::RatioToQ30(d.r);
  }
  if (d.kind == CurveKind::kGeometricStep && d.q > 1.0) {
    c.log2_q_raw = static_cast<std::int32_t>(
        Log::FromDouble(gcem::log(d.q) / gcem::log(2.0)).RawValue());
    c.ratio_raw = static_cast<std::int32_t>(
        segmented_math_internal::SegPow::FromDouble(d.q).RawValue());
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
    std::int64_t span = c.physical_end_raw - c.physical_begin_raw;
    if (span < 0) {
      span = -span;
    }
    if (c.code_count > static_cast<std::uint32_t>(span) + 1U) {
      SegmentedSpecError();
    }
    out[static_cast<std::size_t>(i)] = c;
  }
  for (int i = 1; i < kPlan.count; ++i) {
    CompiledSegment const key = out[static_cast<std::size_t>(i)];
    int j = i;
    while (j > 0 &&
           (out[static_cast<std::size_t>(j - 1)].physical_begin_raw >
                key.physical_begin_raw ||
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

}  // namespace ae::seg::segmented_compiler_internal

#endif  // AE_NUMERIC_DETAILS_SEGMENTED_COMPILER_H_
