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

#include <ae-numeric/segmented_number_wire_io.h>
#include <ae-numeric/wire_io.h>

#include "segmented_test_formats.h"

namespace ae::test_segmented_number_wire {

using test_segmented_formats::Co2;
using test_segmented_formats::Rssi;
using test_segmented_formats::RxWindow;
using test_segmented_formats::Temperature;

template <typename Num>
std::size_t SerializedSize(std::uint32_t rank) {
    auto const n = Num::FromWire(test_segmented_formats::WireFromRank<Num>(rank));
  std::uint8_t buf[8] = {};
  return Num::Serialize(n, buf);
}

void test_RssiWireTraits() {
  TEST_ASSERT_EQUAL_UINT(1U, MaxWireBytes<Rssi>());
  std::uint8_t buf[1] = {};
  auto const n = Rssi::TryFromRuntime(Rssi::runtime_type::FromInteger(-10));
  TEST_ASSERT(n.has_value());
  std::size_t const wrote = wire_traits<Rssi>::Serialize(*n, buf);
  TEST_ASSERT_EQUAL_UINT(1U, wrote);
  auto const back = wire_traits<Rssi>::Deserialize(buf, wrote);
  TEST_ASSERT_EQUAL_UINT(1U, back.bytes_read);
  TEST_ASSERT(*n == back.value);
}

void test_TemperatureTierSizes() {
  TEST_ASSERT_EQUAL_UINT(1U, SerializedSize<Temperature>(0));
  TEST_ASSERT_EQUAL_UINT(1U, SerializedSize<Temperature>(252));
  TEST_ASSERT_EQUAL_UINT(2U, SerializedSize<Temperature>(253));
  TEST_ASSERT_EQUAL_UINT(2U, SerializedSize<Temperature>(1020));
}

void test_Co2TierSizes() {
  TEST_ASSERT_EQUAL_UINT(1U, SerializedSize<Co2>(0));
  TEST_ASSERT_EQUAL_UINT(1U, SerializedSize<Co2>(254));
  TEST_ASSERT_EQUAL_UINT(2U, SerializedSize<Co2>(255));
  TEST_ASSERT_EQUAL_UINT(2U, SerializedSize<Co2>(477));
  TEST_ASSERT_EQUAL_UINT(4U, SerializedSize<Co2>(478));
  TEST_ASSERT_EQUAL_UINT(4U, SerializedSize<Co2>(821));
}

void test_RxWindowTierSizes() {
  TEST_ASSERT_EQUAL_UINT(1U, SerializedSize<RxWindow>(0));
  TEST_ASSERT_EQUAL_UINT(1U, SerializedSize<RxWindow>(254));
  TEST_ASSERT_EQUAL_UINT(2U, SerializedSize<RxWindow>(255));
  TEST_ASSERT_EQUAL_UINT(2U, SerializedSize<RxWindow>(509));
  TEST_ASSERT_EQUAL_UINT(4U, SerializedSize<RxWindow>(510));
  TEST_ASSERT_EQUAL_UINT(4U, SerializedSize<RxWindow>(3045));
}

void test_TruncatedInput() {
  std::uint8_t buf[8] = {};
  std::size_t const tbytes =
      Temperature::Serialize(Temperature::FromWire(Temperature::wire_type{253}),
                             buf);
  TEST_ASSERT_EQUAL_UINT(2U, tbytes);
  auto const t = Temperature::Deserialize(buf, 1);
  TEST_ASSERT_EQUAL_UINT(0U, t.bytes_read);

  std::size_t const cbytes =
      Co2::Serialize(Co2::FromWire(Co2::wire_type{478}), buf);
  TEST_ASSERT_EQUAL_UINT(4U, cbytes);
  auto const c = Co2::Deserialize(buf, 2);
  TEST_ASSERT_EQUAL_UINT(0U, c.bytes_read);
}

void test_UnusedCo2Rank() {
  Co2::wire_type const w{2000};
  std::uint8_t buf[8] = {};
  std::size_t const n = wire_traits<Co2::wire_type>::Serialize(w, buf);
  TEST_ASSERT(n > 0);
  auto const r = Co2::Deserialize(buf, n);
  TEST_ASSERT_EQUAL_UINT(0U, r.bytes_read);
}

}  // namespace ae::test_segmented_number_wire

int test_segmented_number_wire() {
  UNITY_BEGIN();
  RUN_TEST(ae::test_segmented_number_wire::test_RssiWireTraits);
  RUN_TEST(ae::test_segmented_number_wire::test_TemperatureTierSizes);
  RUN_TEST(ae::test_segmented_number_wire::test_Co2TierSizes);
  RUN_TEST(ae::test_segmented_number_wire::test_RxWindowTierSizes);
  RUN_TEST(ae::test_segmented_number_wire::test_TruncatedInput);
  RUN_TEST(ae::test_segmented_number_wire::test_UnusedCo2Rank);
  return UNITY_END();
}
