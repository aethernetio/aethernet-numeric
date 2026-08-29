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

#ifndef AE_NUMERIC_TESTS_FOOTPRINT_COMPONENT_FORMATS_H_
#define AE_NUMERIC_TESTS_FOOTPRINT_COMPONENT_FORMATS_H_

#include <cstdint>

#include <ae-numeric/decimal.h>
#include <ae-numeric/segmented_number.h>

namespace ae::fp_component {

template <std::int64_t M, int E = 0>
using D = Decimal<M, E>;

using Uniform = seg::Compile<seg::Format<
    seg::runtime::Fixed<std::int8_t>,
    seg::wire::AutoTiered<std::uint8_t, seg::wire::MaxBytes<1>>,
    seg::compute::Formula,
    seg::Layout<seg::UniformStep<seg::Range<D<-127>, D<0>>, seg::Step<D<1>>,
                                 seg::Place<seg::Bytes<1>>>>>>;

using Ramp = seg::Compile<seg::Format<
    seg::runtime::Fixed<std::uint16_t>,
    seg::wire::AutoTiered<std::uint8_t, seg::wire::MaxBytes<1>>,
    seg::compute::Formula,
    seg::Layout<seg::LinearStepRamp<
        seg::Range<D<0>, D<20>>, seg::Intervals<45>,
        seg::StepAtUpper<seg::InheritStep>,
        seg::Place<seg::Bytes<1>>>>>>;

using Exponential = seg::Compile<seg::Format<
    seg::runtime::Fixed<std::uint32_t>,
    seg::wire::AutoTiered<std::uint8_t, seg::wire::MaxBytes<1>>,
    seg::compute::Formula,
    seg::Layout<seg::ExponentialValues<
        seg::Range<D<2, -1>, D<2>>, seg::Intervals<102>,
        seg::Place<seg::Bytes<1>>>>>>;

using Geometric = seg::Compile<seg::Format<
    seg::runtime::Fixed<std::uint16_t>,
    seg::wire::AutoTiered<std::uint8_t, seg::wire::MaxBytes<1>>,
    seg::compute::Formula,
    seg::Layout<seg::GeometricStep<
        seg::Range<D<215, -2>, D<275, -2>>, seg::Intervals<130>,
        seg::StepAtUpper<D<2, -3>>, seg::Place<seg::Bytes<1>>>>>>;

using Tiered = seg::Compile<seg::Format<
    seg::runtime::Fixed<std::int16_t>,
    seg::wire::AutoTiered<std::uint8_t, seg::wire::MaxBytes<2>>,
    seg::compute::Formula,
    seg::Layout<
        seg::UniformStep<seg::Range<D<0>, D<200>>, seg::Step<D<1>>,
                         seg::Place<seg::Bytes<1>>>,
        seg::UniformStep<seg::Range<D<200>, D<400>>, seg::Step<D<1>>,
                         seg::Place<seg::Bytes<2>>>>>>;

using Combined = seg::Compile<seg::Format<
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
                            seg::Place<seg::Bytes<1>>>>>>;

}  // namespace ae::fp_component

#endif  // AE_NUMERIC_TESTS_FOOTPRINT_COMPONENT_FORMATS_H_
