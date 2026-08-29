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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <type_traits>

#include <ae-numeric/numeric_traits.h>
#include <ae-numeric/segmented_number_wire_io.h>

#include "segmented_test_formats.h"

namespace ae::test_segmented_number_core {

using test_segmented_formats::Rssi;
using test_segmented_formats::Temperature;

template <typename RT>
double AsDouble(RT const& v) {
  return std::ldexp(static_cast<double>(v.RawValue()), RT::kScaleExp);
}

template <typename Num>
std::size_t WireSizeOf(typename Num::runtime_type const& v) {
  std::uint8_t buf[8] = {};
  auto const n = Num::TryFromRuntime(v);
  TEST_ASSERT(n.has_value());
  return Num::Serialize(*n, buf);
}

template <typename Num>
void CheckRankRoundTrip() {
  for (std::uint32_t rank = 0; rank < static_cast<std::uint32_t>(Num::kCodeCount);
       ++rank) {
    typename Num::wire_type const w =
        test_segmented_formats::WireFromRank<Num>(rank);
    auto const decoded = Num::Decode(w);
    auto const enc = Num::TryEncode(decoded);
    TEST_ASSERT(enc.has_value());
    TEST_ASSERT_EQUAL_UINT(rank, static_cast<std::uint32_t>(*enc));
  }
}

static_assert(sizeof(Rssi) == sizeof(Rssi::runtime_type));
static_assert(Rssi::kCodeCount == 128);
static_assert(std::is_same_v<Rssi::wire_type, std::uint8_t>);
static_assert(Rssi::kMaxWireBytes == 1);
static_assert(Rssi::kLookupTableBytes == 0);
static_assert(sizeof(Temperature) == sizeof(Temperature::runtime_type));
static_assert(Temperature::kCodeCount == 1021);
static_assert(Temperature::kOneByteCount == 253);
static_assert(Temperature::kTwoByteCount == 768);
static_assert(Temperature::kMaxWireBytes == 2);
static_assert(std::is_same_v<Temperature::wire_type,
                             ae::TieredInt<std::uint8_t, 252>>);
static_assert(numeric_traits<Rssi>::kIsSegmented);

void test_RssiLayout() {
  TEST_ASSERT_EQUAL_UINT(128U, Rssi::kCodeCount);
  TEST_ASSERT_EQUAL_UINT(1U, Rssi::kMaxWireBytes);
  TEST_ASSERT_EQUAL_UINT(128U, Rssi::kOneByteCount);
  TEST_ASSERT_EQUAL_UINT(0U, Rssi::kTwoByteCount);
}

void test_RssiEndpoints() {
  auto const lo = Rssi::TryFromRuntime(Rssi::runtime_type::FromInteger(-127));
  auto const hi = Rssi::TryFromRuntime(Rssi::runtime_type::FromInteger(0));
  TEST_ASSERT(lo.has_value());
  TEST_ASSERT(hi.has_value());
  TEST_ASSERT_EQUAL(-127, lo->Value().RawValue());
  TEST_ASSERT_EQUAL(0, hi->Value().RawValue());
}

void test_RssiRoundTrip() { CheckRankRoundTrip<Rssi>(); }

void test_RssiUnusedRank() {
  std::uint8_t buf[1] = {200};
  auto const r = Rssi::Deserialize(buf, 1);
  TEST_ASSERT_EQUAL_UINT(0U, r.bytes_read);
  std::uint8_t empty[1] = {0};
  auto const t = Rssi::Deserialize(empty, 0);
  TEST_ASSERT_EQUAL_UINT(0U, t.bytes_read);
}

void test_RssiOutOfRange() {
  auto const too_hi =
      Rssi::TryFromRuntime(Rssi::runtime_type::FromInteger(1));
  TEST_ASSERT_FALSE(too_hi.has_value());
  auto const sat = Rssi::Saturating(Rssi::runtime_type::FromInteger(1));
  TEST_ASSERT_EQUAL(0, sat.Value().RawValue());
}

void test_RssiFractional() {
  auto const n =
      Rssi::TryFromRuntime(Rssi::runtime_type::FromRatio(-1275, 10));
  TEST_ASSERT(n.has_value());
  TEST_ASSERT_DOUBLE_WITHIN(0.5, -127.5, AsDouble(n->Value()));
}

void test_TemperatureGolden() {
  TEST_ASSERT_EQUAL_UINT(1021U, Temperature::kCodeCount);
  TEST_ASSERT_EQUAL_UINT(253U, Temperature::kOneByteCount);
  TEST_ASSERT_EQUAL_UINT(768U, Temperature::kTwoByteCount);
  TEST_ASSERT_EQUAL_UINT(2U, Temperature::kMaxWireBytes);
  auto const& p = Temperature::Logical();
  TEST_ASSERT_EQUAL(3, p.count);
  TEST_ASSERT_DOUBLE_WITHIN(1.0e-9, 1.0019571403660685, p.segs[0].q);
  TEST_ASSERT_DOUBLE_WITHIN(1.0e-9, 1.0032825748092233, p.segs[2].q);
  TEST_ASSERT_DOUBLE_WITHIN(1.0e-6, 0.1974705407, p.segs[0].step0);
  TEST_ASSERT_DOUBLE_WITHIN(1.0e-6, 0.3934835786, p.segs[2].last_step);
}

void test_TemperatureEndpointsAndWire() {
  using RT = Temperature::runtime_type;
  TEST_ASSERT_EQUAL_UINT(2U, WireSizeOf<Temperature>(RT::FromInteger(-40)));
  TEST_ASSERT_EQUAL_UINT(1U, WireSizeOf<Temperature>(RT::FromInteger(10)));
  TEST_ASSERT_EQUAL_UINT(1U, WireSizeOf<Temperature>(RT::FromRatio(352, 10)));
  TEST_ASSERT_EQUAL_UINT(1U, WireSizeOf<Temperature>(RT::FromInteger(20)));
  TEST_ASSERT_EQUAL_UINT(2U, WireSizeOf<Temperature>(RT::FromInteger(125)));
  TEST_ASSERT_EQUAL_UINT(2U, WireSizeOf<Temperature>(RT::FromRatio(99, 10)));
  TEST_ASSERT_EQUAL_UINT(2U, WireSizeOf<Temperature>(RT::FromRatio(353, 10)));

  auto const a = Temperature::TryFromRuntime(RT::FromInteger(-40));
  auto const b = Temperature::TryFromRuntime(RT::FromInteger(10));
  auto const c = Temperature::TryFromRuntime(RT::FromRatio(352, 10));
  auto const d = Temperature::TryFromRuntime(RT::FromInteger(125));
  TEST_ASSERT(a && b && c && d);
  TEST_ASSERT_DOUBLE_WITHIN(1.0e-3, -40.0, AsDouble(a->Value()));
  TEST_ASSERT_DOUBLE_WITHIN(1.0e-3, 10.0, AsDouble(b->Value()));
  TEST_ASSERT_DOUBLE_WITHIN(1.0e-3, 35.2, AsDouble(c->Value()));
  TEST_ASSERT_DOUBLE_WITHIN(1.0e-3, 125.0, AsDouble(d->Value()));
}

void test_TemperatureQuantization() {
  using RT = Temperature::runtime_type;
  auto max_err = [](double lo, double hi, int steps) {
    double m = 0.0;
    for (int i = 0; i <= steps; ++i) {
      double const x = lo + (hi - lo) * static_cast<double>(i) /
                                static_cast<double>(steps);
      auto const v = RT::FromRatio(static_cast<std::int64_t>(std::llround(x * 100.0)),
                                   100);
      auto const n = Temperature::TryFromRuntime(v);
      TEST_ASSERT(n.has_value());
      m = std::max(m, std::fabs(AsDouble(n->Value()) - x));
    }
    return m;
  };
  TEST_ASSERT(max_err(9.9, 10.1, 40) <= 0.06);
  TEST_ASSERT(max_err(35.1, 35.3, 40) <= 0.06);
  TEST_ASSERT(max_err(-40.0, -39.7, 40) <= 0.11);
  TEST_ASSERT(max_err(124.6, 125.0, 40) <= 0.22);
}

void test_TemperatureRoundTrip() { CheckRankRoundTrip<Temperature>(); }

}  // namespace ae::test_segmented_number_core

int test_segmented_number_core() {
  UNITY_BEGIN();
  RUN_TEST(ae::test_segmented_number_core::test_RssiLayout);
  RUN_TEST(ae::test_segmented_number_core::test_RssiEndpoints);
  RUN_TEST(ae::test_segmented_number_core::test_RssiRoundTrip);
  RUN_TEST(ae::test_segmented_number_core::test_RssiUnusedRank);
  RUN_TEST(ae::test_segmented_number_core::test_RssiOutOfRange);
  RUN_TEST(ae::test_segmented_number_core::test_RssiFractional);
  RUN_TEST(ae::test_segmented_number_core::test_TemperatureGolden);
  RUN_TEST(ae::test_segmented_number_core::test_TemperatureEndpointsAndWire);
  RUN_TEST(ae::test_segmented_number_core::test_TemperatureQuantization);
  RUN_TEST(ae::test_segmented_number_core::test_TemperatureRoundTrip);
  return UNITY_END();
}
