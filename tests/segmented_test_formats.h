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

#ifndef AE_NUMERIC_TESTS_SEGMENTED_TEST_FORMATS_H_
#define AE_NUMERIC_TESTS_SEGMENTED_TEST_FORMATS_H_

#include <cstdint>
#include <type_traits>

#include <ae-numeric/decimal.h>
#include <ae-numeric/segmented_number.h>

namespace ae::test_segmented_formats {

template <std::int64_t M, int E = 0>
using D = Decimal<M, E>;

using RssiSpec = seg::Format<
    seg::runtime::Fixed<std::int8_t>,
    seg::wire::AutoTiered<std::uint8_t, seg::wire::MaxBytes<1>>,
    seg::compute::Formula,
    seg::Layout<seg::UniformStep<seg::Range<D<-127>, D<0>>, seg::Step<D<1>>,
                                 seg::Place<seg::Bytes<1>>>>>;
using Rssi = seg::Compile<RssiSpec>;

using TemperatureSpec = seg::Format<
    seg::runtime::Fixed<std::int16_t>,
    seg::wire::AutoTiered<std::uint8_t, seg::wire::MaxBytes<2>>,
    seg::compute::Formula,
    seg::Layout<
        seg::GeometricStep<seg::Range<D<-40>, D<10>>, seg::Intervals<349>,
                           seg::StepAtUpper<D<1, -1>>,
                           seg::Place<seg::Bytes<2>>>,
        seg::UniformStep<seg::Range<D<10>, D<352, -1>>, seg::Step<D<1, -1>>,
                         seg::Place<seg::Bytes<1>>>,
        seg::GeometricStep<seg::Range<D<352, -1>, D<125>>, seg::Intervals<419>,
                           seg::StepAtLower<D<1, -1>>,
                           seg::Place<seg::Bytes<2>>>>>;
using Temperature = seg::Compile<TemperatureSpec>;

using HumiditySpec = seg::Format<
    seg::runtime::Fixed<std::uint16_t>,
    seg::wire::AutoTiered<std::uint8_t, seg::wire::MaxBytes<1>>,
    seg::compute::Formula,
    seg::Layout<
        seg::LinearStepRamp<seg::Range<D<0>, D<20>>, seg::Intervals<45>,
                            seg::StepAtUpper<seg::InheritStep>,
                            seg::Place<seg::Bytes<1>>>,
        seg::UniformValues<seg::Range<D<20>, D<80>>, seg::Intervals<168>,
                           seg::Place<seg::Bytes<1>>>,
        seg::LinearStepRamp<seg::Range<D<80>, D<100>>, seg::Intervals<42>,
                            seg::StepAtLower<seg::InheritStep>,
                            seg::Place<seg::Bytes<1>>>>>;
using Humidity = seg::Compile<HumiditySpec>;

using Co2Spec = seg::Format<
    seg::runtime::Fixed<std::uint16_t>,
    seg::wire::AutoTiered<std::uint8_t, seg::wire::MaxBytes<4>>,
    seg::compute::Formula,
    seg::Layout<seg::ContinuousExponential<
        seg::Range<D<380>, D<32000>>, seg::ExactEndpoints,
        seg::WireCuts<seg::ApproximateCut<D<1500>, seg::Bytes<1>>,
                      seg::ApproximateCut<D<5000>, seg::Bytes<2>>,
                      seg::Rest<seg::Bytes<4>>>,
        seg::OptimizeCuts>>>;
using Co2 = seg::Compile<Co2Spec>;

using RxWindowSpec = seg::Format<
    seg::runtime::Fixed<std::uint32_t>,
    seg::wire::AutoTiered<std::uint8_t, seg::wire::MaxBytes<4>>,
    seg::compute::Formula,
    seg::Layout<
        seg::AutoSplit<seg::TotalValues<255>, seg::Place<seg::Bytes<1>>,
                       seg::Objective<seg::ContinuousAbsoluteStep,
                                      seg::MinimaxRelativeError>,
                       seg::ExponentialValues<seg::Range<D<1, -2>, D<1>>>,
                       seg::ExponentialValues<seg::Range<D<1>, D<60>>>>,
        seg::GeometricStep<seg::Range<D<60>, D<3600>>, seg::FillTier,
                           seg::StepAtLower<seg::InheritStep>,
                           seg::Place<seg::Bytes<2>>>,
        seg::LinearStepRamp<seg::Range<D<3600>, D<86400>>,
                            seg::MinimumIntervals,
                            seg::StepAtLower<seg::InheritStep>,
                            seg::MaxAbsErrorAtUpper<D<10>>,
                            seg::Place<seg::Bytes<4>>>>>;
using RxWindow = seg::Compile<RxWindowSpec>;

using BatteryVoltageSpec = seg::Format<
    seg::runtime::Fixed<std::uint16_t>,
    seg::wire::AutoTiered<std::uint8_t, seg::wire::MaxBytes<1>>,
    seg::compute::Formula,
    seg::Layout<
        seg::GeometricStep<seg::Range<D<215, -2>, D<275, -2>>,
                           seg::Intervals<130>, seg::StepAtUpper<D<2, -3>>,
                           seg::Place<seg::Bytes<1>>>,
        seg::UniformStep<seg::Range<D<275, -2>, D<300, -2>>,
                         seg::Step<D<2, -3>>,
                         seg::Place<seg::Bytes<1>>>>>;
using Battery = seg::Compile<BatteryVoltageSpec>;

using ConnectDurationSpec = seg::Format<
    seg::runtime::Fixed<std::uint32_t>,
    seg::wire::AutoTiered<std::uint8_t, seg::wire::MaxBytes<1>>,
    seg::compute::Formula,
    seg::Layout<seg::AutoSplit<
        seg::TotalValues<256>, seg::Place<seg::Bytes<1>>,
        seg::Objective<seg::ContinuousAbsoluteStep, seg::MinimaxRelativeError>,
        seg::ExponentialValues<seg::Range<D<2, -1>, D<2>>>,
        seg::ExponentialValues<seg::Range<D<2>, D<60>>>>>>;
using ConnectDuration = seg::Compile<ConnectDurationSpec>;

template <typename Num>
typename Num::wire_type WireFromRank(std::uint32_t rank) {
  return ae::seg::detail::RankToWire<typename Num::wire_type>(rank);
}

}  // namespace ae::test_segmented_formats

#endif  // AE_NUMERIC_TESTS_SEGMENTED_TEST_FORMATS_H_
