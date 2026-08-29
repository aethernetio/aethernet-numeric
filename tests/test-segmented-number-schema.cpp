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

#include <unity.h>

#include <cstdint>
#include <type_traits>

#include <ae-numeric/decimal.h>
#include <ae-numeric/segmented_number_floating_runtime.h>
#include <ae-numeric/segmented_number_wire_io.h>

#include "segmented_test_formats.h"

namespace ae::test_segmented_number_schema {

using test_segmented_formats::Co2;
using test_segmented_formats::Rssi;
using test_segmented_formats::RssiSpec;
using test_segmented_formats::Temperature;
using test_segmented_formats::TemperatureSpec;

template <std::int64_t M, int E = 0>
using D = ae::Decimal<M, E>;

using RssiAgain = seg::Compile<RssiSpec>;

using RssiFloatSpec = seg::Format<
    seg::runtime::Floating<double>,
    seg::wire::AutoTiered<std::uint8_t, seg::wire::MaxBytes<1>>,
    seg::compute::Formula, typename RssiSpec::layout_type>;
using RssiFloat = seg::Compile<RssiFloatSpec>;

using TemperatureFloatSpec = seg::Format<
    seg::runtime::Floating<double>,
    seg::wire::AutoTiered<std::uint8_t, seg::wire::MaxBytes<2>>,
    seg::compute::Formula, typename TemperatureSpec::layout_type>;
using TemperatureFloat = seg::Compile<TemperatureFloatSpec>;

using RssiFewerCodes = seg::Compile<seg::Format<
    seg::runtime::Fixed<std::int8_t>,
    seg::wire::AutoTiered<std::uint8_t, seg::wire::MaxBytes<1>>,
    seg::compute::Formula,
    seg::Layout<seg::UniformStep<seg::Range<D<-127>, D<0>>, seg::Step<D<2>>,
                             seg::Place<seg::Bytes<1>>>>>>;

using RssiWiderRange = seg::Compile<seg::Format<
    seg::runtime::Fixed<std::int8_t>,
    seg::wire::AutoTiered<std::uint8_t, seg::wire::MaxBytes<1>>,
    seg::compute::Formula,
    seg::Layout<seg::UniformStep<seg::Range<D<-120>, D<0>>, seg::Step<D<1>>,
                             seg::Place<seg::Bytes<1>>>>>>;

using Co2AltCuts = seg::Compile<seg::Format<
    seg::runtime::Fixed<std::uint16_t>,
    seg::wire::AutoTiered<std::uint8_t, seg::wire::MaxBytes<4>>,
    seg::compute::Formula,
    seg::Layout<seg::ContinuousExponential<
        seg::Range<D<380>, D<32000>>, seg::ExactEndpoints,
        seg::WireCuts<seg::ApproximateCut<D<1600>, seg::Bytes<1>>,
                      seg::ApproximateCut<D<5000>, seg::Bytes<2>>,
                      seg::Rest<seg::Bytes<4>>>,
        seg::OptimizeCuts>>>>;

static_assert(Rssi::kSchemaHash == RssiAgain::kSchemaHash);
static_assert(Rssi::kSchemaHash == RssiFloat::kSchemaHash);
static_assert(Temperature::kSchemaHash == TemperatureFloat::kSchemaHash);
static_assert(Rssi::kSchemaHash != RssiFewerCodes::kSchemaHash);
static_assert(Rssi::kSchemaHash != RssiWiderRange::kSchemaHash);
static_assert(Co2::kSchemaHash != Co2AltCuts::kSchemaHash);

void test_SameFormatSameHash() {
  TEST_ASSERT_EQUAL_UINT64(Rssi::kSchemaHash, RssiAgain::kSchemaHash);
}

void test_FixedFloatingSameHash() {
  TEST_ASSERT_EQUAL_UINT64(Rssi::kSchemaHash, RssiFloat::kSchemaHash);
  TEST_ASSERT_EQUAL_UINT64(Temperature::kSchemaHash,
                           TemperatureFloat::kSchemaHash);
}

void test_IntervalChangeChangesHash() {
  TEST_ASSERT(Rssi::kSchemaHash != RssiFewerCodes::kSchemaHash);
}

void test_RangeChangeChangesHash() {
  TEST_ASSERT(Rssi::kSchemaHash != RssiWiderRange::kSchemaHash);
}

void test_WireCutChangeChangesHash() {
  TEST_ASSERT(Co2::kSchemaHash != Co2AltCuts::kSchemaHash);
}

void test_TryDecodeRejectsInvalidRank() {
  auto const bad = Rssi::TryDecode(static_cast<std::uint8_t>(200));
  TEST_ASSERT_FALSE(bad.has_value());
  auto const ok = Rssi::TryDecode(static_cast<std::uint8_t>(0));
  TEST_ASSERT(ok.has_value());
}

}  // namespace ae::test_segmented_number_schema

int test_segmented_number_schema() {
  UNITY_BEGIN();
  RUN_TEST(ae::test_segmented_number_schema::test_SameFormatSameHash);
  RUN_TEST(ae::test_segmented_number_schema::test_FixedFloatingSameHash);
  RUN_TEST(ae::test_segmented_number_schema::test_IntervalChangeChangesHash);
  RUN_TEST(ae::test_segmented_number_schema::test_RangeChangeChangesHash);
  RUN_TEST(ae::test_segmented_number_schema::test_WireCutChangeChangesHash);
  RUN_TEST(ae::test_segmented_number_schema::test_TryDecodeRejectsInvalidRank);
  return UNITY_END();
}
