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

#ifndef AE_NUMERIC_DETAILS_SEGMENTED_CURVES_H_
#define AE_NUMERIC_DETAILS_SEGMENTED_CURVES_H_

#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "ae-numeric/details/segmented_format.h"
#include "ae-numeric/details/segmented_math.h"

namespace ae::seg::segmented_curves_internal {

using segmented_math_internal::SegmentedSpecError;
using segmented_math_internal::ValueToDouble;

inline constexpr int kMaxDrafts = 16;

enum class StepMode : std::uint8_t {
  kNone = 0,
  kLowerExplicit = 1,
  kUpperExplicit = 2,
  kLowerInherit = 3,
  kUpperInherit = 4,
};

struct CurveDraft {
  CurveKind kind = CurveKind::kUniformStep;
  double begin = 0.0;
  double end = 0.0;
  int bytes = 1;
  int intervals = -1;
  StepMode step_mode = StepMode::kNone;
  double specified_step = 0.0;
  bool fill_tier = false;
  bool min_intervals = false;
  double max_err_upper = 0.0;
  bool has_max_err_upper = false;
  double max_err_lower = 0.0;
  bool has_max_err_lower = false;
  int autosplit_id = 0;
  int total_values = 0;
  bool is_cont_exp = false;
  double cut1 = 0.0;
  double cut2 = 0.0;
  int last_1 = -1;
  int last_2 = -1;
  double r = 1.0;
  double q = 1.0;
  double step0 = 0.0;
  double last_step = 0.0;
  double delta = 0.0;
  bool own_begin = false;
  bool own_end = false;
  int stored = 0;
  int math_first = 0;
  std::uint32_t wire_begin = 0;
  double phys_begin = 0.0;
  double phys_end = 0.0;
};

template <typename>
inline constexpr bool kIsRangeV = false;
template <typename B, typename E>
inline constexpr bool kIsRangeV<Range<B, E>> = true;

template <typename>
inline constexpr bool kIsPlaceV = false;
template <typename B>
inline constexpr bool kIsPlaceV<Place<B>> = true;

template <typename>
inline constexpr int kBytesOf = -1;
template <std::size_t N>
inline constexpr int kBytesOf<Bytes<N>> = static_cast<int>(N);

template <typename>
inline constexpr int kIntervalsOf = -1;
template <std::size_t N>
inline constexpr int kIntervalsOf<Intervals<N>> = static_cast<int>(N);

template <typename>
inline constexpr int kTotalValuesOf = -1;
template <std::size_t N>
inline constexpr int kTotalValuesOf<TotalValues<N>> = static_cast<int>(N);

template <typename>
inline constexpr bool kIsStepV = false;
template <typename V>
inline constexpr bool kIsStepV<Step<V>> = true;

template <typename>
inline constexpr bool kIsStepAtLowerV = false;
template <typename V>
inline constexpr bool kIsStepAtLowerV<StepAtLower<V>> = true;

template <typename>
inline constexpr bool kIsStepAtUpperV = false;
template <typename V>
inline constexpr bool kIsStepAtUpperV<StepAtUpper<V>> = true;

template <typename>
inline constexpr bool kIsMaxErrLowerV = false;
template <typename V>
inline constexpr bool kIsMaxErrLowerV<MaxAbsErrorAtLower<V>> = true;

template <typename>
inline constexpr bool kIsMaxErrUpperV = false;
template <typename V>
inline constexpr bool kIsMaxErrUpperV<MaxAbsErrorAtUpper<V>> = true;

template <typename>
inline constexpr bool kIsApproxCutV = false;
template <typename V, typename B>
inline constexpr bool kIsApproxCutV<ApproximateCut<V, B>> = true;

template <typename>
inline constexpr bool kIsRestV = false;
template <typename B>
inline constexpr bool kIsRestV<Rest<B>> = true;

template <typename>
inline constexpr bool kIsWireCutsV = false;
template <typename... C>
inline constexpr bool kIsWireCutsV<WireCuts<C...>> = true;

template <typename>
inline constexpr bool kIsMathCurveV = false;
template <typename... O>
inline constexpr bool kIsMathCurveV<UniformStep<O...>> = true;
template <typename... O>
inline constexpr bool kIsMathCurveV<UniformValues<O...>> = true;
template <typename... O>
inline constexpr bool kIsMathCurveV<ExponentialValues<O...>> = true;
template <typename... O>
inline constexpr bool kIsMathCurveV<GeometricStep<O...>> = true;
template <typename... O>
inline constexpr bool kIsMathCurveV<LinearStepRamp<O...>> = true;

struct OptAcc {
  double begin = 0.0;
  double end = 0.0;
  bool has_range = false;
  int bytes = -1;
  int intervals = -1;
  int total_values = -1;
  StepMode step_mode = StepMode::kNone;
  double specified_step = 0.0;
  bool fill_tier = false;
  bool min_intervals = false;
  double max_err_upper = 0.0;
  bool has_max_err_upper = false;
  double max_err_lower = 0.0;
  bool has_max_err_lower = false;
  double cut1 = 0.0;
  double cut2 = 0.0;
  int ncuts = 0;
};

template <typename B, typename E>
consteval void ApplyOne(OptAcc& a, Range<B, E>) {
  a.begin = ValueToDouble<B>();
  a.end = ValueToDouble<E>();
  a.has_range = true;
}

template <typename B>
consteval void ApplyOne(OptAcc& a, Place<B>) {
  a.bytes = kBytesOf<B>;
}

template <std::size_t N>
consteval void ApplyOne(OptAcc& a, Intervals<N>) {
  a.intervals = static_cast<int>(N);
}

template <std::size_t N>
consteval void ApplyOne(OptAcc& a, TotalValues<N>) {
  a.total_values = static_cast<int>(N);
}

consteval void ApplyOne(OptAcc& a, FillTier) { a.fill_tier = true; }

consteval void ApplyOne(OptAcc& a, MinimumIntervals) { a.min_intervals = true; }

template <typename V>
consteval void ApplyOne(OptAcc& a, Step<V>) {
  a.specified_step = ValueToDouble<V>();
  a.step_mode = StepMode::kLowerExplicit;
}

template <typename V>
consteval void ApplyOne(OptAcc& a, StepAtLower<V>) {
  if constexpr (std::is_same_v<V, InheritStep>) {
    a.step_mode = StepMode::kLowerInherit;
  } else {
    a.step_mode = StepMode::kLowerExplicit;
    a.specified_step = ValueToDouble<V>();
  }
}

template <typename V>
consteval void ApplyOne(OptAcc& a, StepAtUpper<V>) {
  if constexpr (std::is_same_v<V, InheritStep>) {
    a.step_mode = StepMode::kUpperInherit;
  } else {
    a.step_mode = StepMode::kUpperExplicit;
    a.specified_step = ValueToDouble<V>();
  }
}

template <typename V>
consteval void ApplyOne(OptAcc& a, MaxAbsErrorAtLower<V>) {
  a.max_err_lower = ValueToDouble<V>();
  a.has_max_err_lower = true;
}

template <typename V>
consteval void ApplyOne(OptAcc& a, MaxAbsErrorAtUpper<V>) {
  a.max_err_upper = ValueToDouble<V>();
  a.has_max_err_upper = true;
}

template <typename V, typename B>
consteval void ApplyOne(OptAcc& a, ApproximateCut<V, B>) {
  if (a.ncuts == 0) {
    a.cut1 = ValueToDouble<V>();
  } else {
    a.cut2 = ValueToDouble<V>();
  }
  ++a.ncuts;
}

template <typename B>
consteval void ApplyOne(OptAcc& /*a*/, Rest<B>) {}

template <typename... C>
consteval void ApplyOne(OptAcc& a, WireCuts<C...>) {
  (ApplyOne(a, C{}), ...);
}

consteval void ApplyOne(OptAcc& /*a*/, ExactEndpoints) {}
consteval void ApplyOne(OptAcc& /*a*/, OptimizeCuts) {}
template <typename P>
consteval void ApplyOne(OptAcc& /*a*/, EndpointPolicy<P>) {}
template <typename P>
consteval void ApplyOne(OptAcc& /*a*/, Allocate<P>) {}
template <typename T>
consteval void ApplyOne(OptAcc& /*a*/, T) {}

template <typename... Opts>
consteval OptAcc ParseOpts() {
  OptAcc a{};
  (ApplyOne(a, Opts{}), ...);
  return a;
}

consteval CurveDraft DraftFromAcc(CurveKind kind, OptAcc const& a, int bytes_fallback) {
  CurveDraft d{};
  d.kind = kind;
  if (!a.has_range) {
    SegmentedSpecError();
  }
  d.begin = a.begin;
  d.end = a.end;
  if (d.end < d.begin) {
    SegmentedSpecError();
  }
  d.bytes = a.bytes >= 0 ? a.bytes : bytes_fallback;
  if (d.bytes != 1 && d.bytes != 2 && d.bytes != 4 && d.bytes != 8) {
    SegmentedSpecError();
  }
  d.intervals = a.intervals;
  d.step_mode = a.step_mode;
  d.specified_step = a.specified_step;
  d.fill_tier = a.fill_tier;
  d.min_intervals = a.min_intervals;
  d.max_err_upper = a.max_err_upper;
  d.has_max_err_upper = a.has_max_err_upper;
  d.max_err_lower = a.max_err_lower;
  d.has_max_err_lower = a.has_max_err_lower;
  d.cut1 = a.cut1;
  d.cut2 = a.cut2;
  return d;
}

template <typename T>
struct DraftsOf;

template <typename... Opts>
struct DraftsOf<UniformStep<Opts...>> {
  static constexpr int kCount = 1;
  static consteval void Fill(CurveDraft* out, int& i, int /*as_id*/, int bytes_fb) {
    out[i++] = DraftFromAcc(CurveKind::kUniformStep, ParseOpts<Opts...>(), bytes_fb);
  }
};

template <typename... Opts>
struct DraftsOf<UniformValues<Opts...>> {
  static constexpr int kCount = 1;
  static consteval void Fill(CurveDraft* out, int& i, int /*as_id*/, int bytes_fb) {
    out[i++] =
        DraftFromAcc(CurveKind::kUniformValues, ParseOpts<Opts...>(), bytes_fb);
  }
};

template <typename... Opts>
struct DraftsOf<ExponentialValues<Opts...>> {
  static constexpr int kCount = 1;
  static consteval void Fill(CurveDraft* out, int& i, int /*as_id*/, int bytes_fb) {
    CurveDraft d =
        DraftFromAcc(CurveKind::kExponentialValues, ParseOpts<Opts...>(), bytes_fb);
    if (d.begin <= 0.0 || d.end <= 0.0) {
      SegmentedSpecError();
    }
    out[i++] = d;
  }
};

template <typename... Opts>
struct DraftsOf<GeometricStep<Opts...>> {
  static constexpr int kCount = 1;
  static consteval void Fill(CurveDraft* out, int& i, int /*as_id*/, int bytes_fb) {
    out[i++] =
        DraftFromAcc(CurveKind::kGeometricStep, ParseOpts<Opts...>(), bytes_fb);
  }
};

template <typename... Opts>
struct DraftsOf<LinearStepRamp<Opts...>> {
  static constexpr int kCount = 1;
  static consteval void Fill(CurveDraft* out, int& i, int /*as_id*/, int bytes_fb) {
    out[i++] =
        DraftFromAcc(CurveKind::kLinearStepRamp, ParseOpts<Opts...>(), bytes_fb);
  }
};

template <typename... Opts>
struct DraftsOf<ContinuousExponential<Opts...>> {
  static constexpr int kCount = 1;
  static consteval void Fill(CurveDraft* out, int& i, int /*as_id*/, int bytes_fb) {
    CurveDraft d = DraftFromAcc(CurveKind::kExponentialValues,
                                ParseOpts<Opts...>(), bytes_fb);
    if (d.begin <= 0.0 || d.end <= 0.0) {
      SegmentedSpecError();
    }
    d.is_cont_exp = true;
    d.bytes = 0;
    out[i++] = d;
  }
};

template <typename T>
consteval int CurveDraftCount() {
  if constexpr (kIsMathCurveV<T>) {
    return DraftsOf<T>::kCount;
  } else {
    return 0;
  }
}

template <typename... Opts>
struct CountAutoCurves {
  static constexpr int value = (CurveDraftCount<Opts>() + ... + 0);
};

template <typename T>
consteval void FillIfCurve(CurveDraft* out, int& i, int as_id, int bytes) {
  if constexpr (kIsMathCurveV<T>) {
    DraftsOf<T>::Fill(out, i, as_id, bytes);
  }
}

template <typename... Opts>
struct DraftsOf<AutoSplit<Opts...>> {
  static constexpr int kCount = CountAutoCurves<Opts...>::value;
  static consteval void Fill(CurveDraft* out, int& i, int as_id, int /*bytes_fb*/) {
    OptAcc const pack = ParseOpts<Opts...>();
    int const bytes = pack.bytes;
    int const total = pack.total_values;
    if (bytes < 0 || total < 2 || kCount < 2) {
      SegmentedSpecError();
    }
    int const start = i;
    (FillIfCurve<Opts>(out, i, as_id, bytes), ...);
    for (int k = start; k < i; ++k) {
      out[k].autosplit_id = as_id;
      out[k].total_values = total;
      out[k].bytes = bytes;
    }
  }
};

template <typename... Items>
struct FlattenLayout;

template <>
struct FlattenLayout<> {
  static constexpr int kCount = 0;
  static consteval void Fill(CurveDraft*, int&, int&) {}
};

template <typename Head, typename... Tail>
struct FlattenLayout<Head, Tail...> {
  static constexpr int kCount =
      DraftsOf<Head>::kCount + FlattenLayout<Tail...>::kCount;
  static consteval void Fill(CurveDraft* out, int& i, int& next_as) {
    DraftsOf<Head>::Fill(out, i, next_as, 1);
    ++next_as;
    FlattenLayout<Tail...>::Fill(out, i, next_as);
  }
};

template <typename... Items>
struct FlattenLayout<Layout<Items...>> : FlattenLayout<Items...> {};

}  // namespace ae::seg::segmented_curves_internal

#endif  // AE_NUMERIC_DETAILS_SEGMENTED_CURVES_H_
