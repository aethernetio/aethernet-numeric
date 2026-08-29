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

#include <ae-numeric/segmented_number_floating_runtime.h>
#include <ae-numeric/segmented_number_wire_io.h>

#include "segmented_test_formats.h"

namespace ae::test_segmented_number_floating_runtime {

using test_segmented_formats::RssiSpec;
using test_segmented_formats::TemperatureSpec;

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

static_assert(sizeof(RssiFloat) == sizeof(double));
static_assert(RssiFloat::kCodeCount == test_segmented_formats::Rssi::kCodeCount);
static_assert(std::is_same_v<RssiFloat::wire_type, std::uint8_t>);
static_assert(TemperatureFloat::kCodeCount ==
              test_segmented_formats::Temperature::kCodeCount);
static_assert(std::is_same_v<TemperatureFloat::wire_type,
                             test_segmented_formats::Temperature::wire_type>);

void test_RssiFloatingWireMatchesFixed() {
  using Fixed = test_segmented_formats::Rssi;
  for (int v = -127; v <= 0; ++v) {
    auto const f = Fixed::TryFromRuntime(Fixed::runtime_type::FromInteger(v));
    auto const d = RssiFloat::TryFromRuntime(static_cast<double>(v));
    TEST_ASSERT(f.has_value());
    TEST_ASSERT(d.has_value());
    std::uint8_t a[2] = {};
    std::uint8_t b[2] = {};
    TEST_ASSERT_EQUAL_UINT(Fixed::Serialize(*f, a), RssiFloat::Serialize(*d, b));
    TEST_ASSERT_EQUAL_HEX8(a[0], b[0]);
  }
}

void test_TemperatureFloatingEndpoints() {
  using Fixed = test_segmented_formats::Temperature;
  double const pts[] = {-40.0, 10.0, 35.2, 125.0};
  for (double x : pts) {
    auto const d = TemperatureFloat::TryFromRuntime(x);
    TEST_ASSERT(d.has_value());
    std::uint8_t buf[4] = {};
    std::size_t const n = TemperatureFloat::Serialize(*d, buf);
    TEST_ASSERT(n > 0);
    auto const back = TemperatureFloat::Deserialize(buf, n);
    TEST_ASSERT_EQUAL_UINT(n, back.bytes_read);
    auto const enc = TemperatureFloat::TryEncode(d->Value());
    auto const fenc = Fixed::TryEncode(Fixed::Decode(*enc));
    TEST_ASSERT(enc.has_value());
    TEST_ASSERT(fenc.has_value());
    TEST_ASSERT_EQUAL_UINT(static_cast<std::uint32_t>(*enc),
                           static_cast<std::uint32_t>(*fenc));
  }
}

}  // namespace ae::test_segmented_number_floating_runtime

int test_segmented_number_floating_runtime() {
  UNITY_BEGIN();
  RUN_TEST(ae::test_segmented_number_floating_runtime::
               test_RssiFloatingWireMatchesFixed);
  RUN_TEST(ae::test_segmented_number_floating_runtime::
               test_TemperatureFloatingEndpoints);
  return UNITY_END();
}
