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

#ifndef NUMERIC_EXPONENTIAL_MATH_POLICY_H_
#define NUMERIC_EXPONENTIAL_MATH_POLICY_H_

#include <cstdint>
#include <limits>
#include <type_traits>

#include <gcem.hpp>

#include "numeric/fixed_point.h"
#include "numeric/numeric_traits.h"
#include "numeric/runtime_numeric_traits.h"

namespace ae {
namespace exponential_internal {

template <typename Raw>
struct MulIntermediate {
  using type = std::conditional_t<
      (sizeof(Raw) <= 2), std::int32_t,
      std::conditional_t<
          (sizeof(Raw) <= 4), std::int64_t,
#ifdef __SIZEOF_INT128__
          __int128_t
#else
          std::int64_t
#endif
          >>;
};

inline constexpr int ClampInt(int value, int lo, int hi) {
  return value < lo ? lo : (value > hi ? hi : value);
}

template <typename RuntimeT>
consteval double RuntimeObservedQuantum() {
  if constexpr (std::is_floating_point_v<RuntimeT>) {
    return static_cast<double>(std::numeric_limits<RuntimeT>::epsilon());
  } else if constexpr (numeric_traits<RuntimeT>::kIsFixedPoint) {
    if constexpr (RuntimeT::kFractionBits > 0) {
      return 1.0 / static_cast<double>(std::uint64_t{1}
                                       << static_cast<unsigned>(
                                              RuntimeT::kFractionBits));
    }
    return 1.0;
  } else {
    return 1.0;
  }
}

template <typename Rep, auto Max, double BoundaryMag, double MinMag,
          double MinQuantum>
struct WorkCandidateOk {
  using Candidate = FixedPoint<Rep, Max>;

  static constexpr bool kFitsBoundary =
      BoundaryMag <= static_cast<double>(Max) &&
      static_cast<double>(Max) <= gcem::max(BoundaryMag * 4.0, 1.0);
  static constexpr double kStep =
      Candidate::kFractionBits > 0
          ? (1.0 / static_cast<double>(std::uint64_t{1}
                                        << static_cast<unsigned>(
                                               Candidate::kFractionBits)))
          : 1.0;
  static constexpr bool kQuantumOk = kStep <= MinQuantum;
  static constexpr bool kMinRepresentable =
      Candidate::FromDouble(MinMag).raw_value() >
          typename Candidate::rep_value_type{0} &&
      Candidate::FromDouble(MinMag * 10.0) > Candidate::FromDouble(MinMag);
  static constexpr bool kRepWidthOk =
      sizeof(Rep) >= sizeof(std::uint32_t) || MinMag >= 0.01;
  static constexpr bool kValue =
      kFitsBoundary && kQuantumOk && kMinRepresentable && kRepWidthOk;
};

template <typename Rep, auto Max, double BoundaryMag, double MinMag,
          double MinQuantum>
inline constexpr bool WorkCandidateOkV =
    WorkCandidateOk<Rep, Max, BoundaryMag, MinMag, MinQuantum>::kValue;

template <typename Rep, auto Max, double BoundaryMag, double MinMag,
          double MinQuantum>
struct WorkChoice {
  static constexpr bool kOk =
      WorkCandidateOkV<Rep, Max, BoundaryMag, MinMag, MinQuantum>;
  using type = FixedPoint<Rep, Max>;
};

template <typename... Choices>
struct PickFirstWork;

template <typename Head, typename... Tail>
struct PickFirstWork<Head, Tail...> {
  using type = std::conditional_t<Head::kOk, typename Head::type,
                                   typename PickFirstWork<Tail...>::type>;
};

template <>
struct PickFirstWork<> {
  using type = FixedPoint<std::uint32_t, 65536.0>;
};

template <double BoundaryMag, double MinMag, double MinQuantum>
struct PickWorkType {
  using type = typename PickFirstWork<
      WorkChoice<std::uint16_t, 1.0, BoundaryMag, MinMag, MinQuantum>,
      WorkChoice<std::uint16_t, 2.0, BoundaryMag, MinMag, MinQuantum>,
      WorkChoice<std::uint16_t, 4.0, BoundaryMag, MinMag, MinQuantum>,
      WorkChoice<std::uint16_t, 8.0, BoundaryMag, MinMag, MinQuantum>,
      WorkChoice<std::uint16_t, 16.0, BoundaryMag, MinMag, MinQuantum>,
      WorkChoice<std::uint16_t, 32.0, BoundaryMag, MinMag, MinQuantum>,
      WorkChoice<std::uint16_t, 64.0, BoundaryMag, MinMag, MinQuantum>,
      WorkChoice<std::uint16_t, 256.0, BoundaryMag, MinMag, MinQuantum>,
      WorkChoice<std::uint32_t, 64.0, BoundaryMag, MinMag, MinQuantum>,
      WorkChoice<std::uint32_t, 256.0, BoundaryMag, MinMag, MinQuantum>,
      WorkChoice<std::uint32_t, 1024.0, BoundaryMag, MinMag, MinQuantum>,
      WorkChoice<std::uint32_t, 4096.0, BoundaryMag, MinMag, MinQuantum>,
      WorkChoice<std::uint32_t, 8192.0, BoundaryMag, MinMag, MinQuantum>,
      WorkChoice<std::uint32_t, 65536.0, BoundaryMag, MinMag,
                 MinQuantum>>::type;
};

// TODO: The codec mapping is unit-stable, but `log_type` selection still uses
// absolute log extent to support the current absolute-log implementation.
// A future cleanup should switch interpolation to relative log delta:
//   log_delta = log2(value) - log2(MinMagnitude)
// so log_type depends only on log_span = log2(BoundaryMagnitude / MinMagnitude).
template <double LogSpan, double AbsLogExtent>
struct PickLogType {
  using type = std::conditional_t<
      (LogSpan <= 16.0 && AbsLogExtent <= 16.0),
      FixedPoint<std::int16_t, 16.0>,
      std::conditional_t<
          (LogSpan <= 32.0 && AbsLogExtent <= 32.0),
          FixedPoint<std::int16_t, 32.0>,
          std::conditional_t<
              (LogSpan <= 64.0 && AbsLogExtent <= 64.0),
              FixedPoint<std::int16_t, 64.0>,
              FixedPoint<std::int32_t, 1024.0>>>>;
};

template <typename RuntimeT, double BoundaryMag, double MinMag,
          double MinQuantum>
struct SelectWorkType {
  using type =
      typename PickWorkType<BoundaryMag, MinMag, MinQuantum>::type;
};

template <typename RuntimeT, auto MinMagnitude, auto BoundaryMagnitude,
          auto BoundaryCode, typename WireValue, bool IsSigned>
struct ExponentialMathPolicy {
  static constexpr double kMinMag = static_cast<double>(MinMagnitude);
  static constexpr double kBoundaryMag = static_cast<double>(BoundaryMagnitude);
  static constexpr double kLogSpan =
      gcem::log2(kBoundaryMag / kMinMag);
  static constexpr double kAbsLogMinValue = gcem::log2(kMinMag);
  static constexpr double kAbsLogMaxValue = gcem::log2(kBoundaryMag);
  static constexpr double kAbsLogExtent =
      gcem::max(gcem::abs(kAbsLogMinValue), gcem::abs(kAbsLogMaxValue));

  static constexpr WireValue kPositiveBoundaryCode =
      static_cast<WireValue>(BoundaryCode) -
      static_cast<WireValue>(static_cast<WireValue>(BoundaryCode) %
                             WireValue{2});
  static constexpr WireValue kMaxMagnitudeIndex =
      IsSigned ? static_cast<WireValue>(kPositiveBoundaryCode / WireValue{2})
               : static_cast<WireValue>(BoundaryCode);

  static constexpr double kLogStep =
      static_cast<double>(static_cast<int>(kMaxMagnitudeIndex) - 1) > 0.0
          ? (kLogSpan / static_cast<double>(static_cast<int>(
                            kMaxMagnitudeIndex) -
                        1))
          : kLogSpan;

  static constexpr double kMathErrorBudgetCodes = 0.25;
  static constexpr double kAllowedLogError =
      kLogStep * kMathErrorBudgetCodes;
  static constexpr int kDerivedIterations = static_cast<int>(gcem::ceil(
      -gcem::log2(kAllowedLogError > 0.0 ? kAllowedLogError : 1.0)));
  static constexpr int kMinIterations = 4;
  static constexpr int kMaxIterations = 10;
  static constexpr int kLogIterations =
      ClampInt(kDerivedIterations, kMinIterations, kMaxIterations);
  static constexpr int kExp2FractionBits = kLogIterations;

  using log_type = typename PickLogType<kLogSpan, kAbsLogExtent>::type;
  using abs_log_type = log_type;
  using mant_type = FixedPoint<std::uint16_t, 4.0>;

  static constexpr double kHalfCodeRelError =
      gcem::pow(2.0, kLogStep / 2.0) - 1.0;
  static constexpr double kAllowedWorkAbsErrorAtMin =
      kMinMag * kHalfCodeRelError * 0.25;
  static constexpr double kRuntimeObservedQuantum =
      RuntimeObservedQuantum<RuntimeT>();
  static constexpr double kMinWorkQuantum =
      gcem::max(kAllowedWorkAbsErrorAtMin, kRuntimeObservedQuantum);

  using picked_work_type =
      typename PickWorkType<kBoundaryMag, kMinMag, kMinWorkQuantum>::type;
  using work_type = typename SelectWorkType<RuntimeT, kBoundaryMag, kMinMag,
                                            kMinWorkQuantum>::type;

  using mul_intermediate_type =
      typename MulIntermediate<typename work_type::rep_value_type>::type;
};

}  // namespace exponential_internal
}  // namespace ae

#endif  // NUMERIC_EXPONENTIAL_MATH_POLICY_H_
