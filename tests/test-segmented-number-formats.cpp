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

#include <ae-numeric/segmented_number_wire_io.h>
#include <ae-numeric/tiered_int.h>

#include "segmented_test_formats.h"

namespace ae::test_segmented_number_formats {

using test_segmented_formats::Battery;
using test_segmented_formats::Co2;
using test_segmented_formats::ConnectDuration;
using test_segmented_formats::Humidity;
using test_segmented_formats::Rssi;
using test_segmented_formats::RxWindow;
using test_segmented_formats::Temperature;

template <typename RT>
double AsDouble(RT const& v) {
  return std::ldexp(static_cast<double>(v.RawValue()), RT::kScaleExp);
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

template <typename Num>
void CheckUniqueRaws() {
  std::int64_t raws[4096];
  static_assert(Num::kCodeCount <= 4096);
  for (std::uint32_t i = 0; i < static_cast<std::uint32_t>(Num::kCodeCount);
       ++i) {
    raws[i] = static_cast<std::int64_t>(
        Num::Decode(test_segmented_formats::WireFromRank<Num>(i)).RawValue());
  }
  std::uint32_t dup = 0;
  for (std::uint32_t i = 0; i < static_cast<std::uint32_t>(Num::kCodeCount);
       ++i) {
    for (std::uint32_t j = i + 1; j < static_cast<std::uint32_t>(Num::kCodeCount);
         ++j) {
      if (raws[i] == raws[j]) {
        ++dup;
      }
    }
  }
  TEST_ASSERT_EQUAL_UINT(0U, dup);
}

template <typename Num>
void CheckSerializeRoundTrip() {
  for (std::uint32_t rank = 0; rank < static_cast<std::uint32_t>(Num::kCodeCount);
       ++rank) {
    auto const n =
        Num::FromWire(test_segmented_formats::WireFromRank<Num>(rank));
    std::uint8_t buf[8] = {};
    std::size_t const wrote = Num::Serialize(n, buf);
    TEST_ASSERT(wrote > 0);
    TEST_ASSERT(wrote <= Num::kMaxWireBytes);
    auto const back = Num::Deserialize(buf, wrote);
    TEST_ASSERT_EQUAL_UINT(wrote, back.bytes_read);
    TEST_ASSERT(n == back.value);
  }
}

void test_HumidityGolden() {
  TEST_ASSERT_EQUAL_UINT(256U, Humidity::kCodeCount);
  TEST_ASSERT_EQUAL_UINT(1U, Humidity::kMaxWireBytes);
  auto const& p = Humidity::Logical();
  TEST_ASSERT_EQUAL(45, p.segs[0].intervals);
  TEST_ASSERT_EQUAL(168, p.segs[1].intervals);
  TEST_ASSERT_EQUAL(42, p.segs[2].intervals);
  TEST_ASSERT_DOUBLE_WITHIN(1.0e-8, 60.0 / 168.0, p.segs[1].step0);
  TEST_ASSERT_DOUBLE_WITHIN(1.0e-6, 0.5317460317, p.segs[0].step0);
  TEST_ASSERT_DOUBLE_WITHIN(1.0e-6, 0.5952380952, p.segs[2].last_step);
}

void test_HumidityErrors() {
  using RT = Humidity::runtime_type;
  auto max_err = [](double lo, double hi, int steps) {
    double m = 0.0;
    for (int i = 0; i <= steps; ++i) {
      double const x = lo + (hi - lo) * static_cast<double>(i) /
                                static_cast<double>(steps);
      auto const v = RT::FromRatio(static_cast<std::int64_t>(std::llround(x * 100.0)),
                                   100);
      auto const n = Humidity::TryFromRuntime(v);
      TEST_ASSERT(n.has_value());
      m = std::max(m, std::fabs(AsDouble(n->Value()) - x));
    }
    return m;
  };
  TEST_ASSERT(max_err(0.0, 0.6, 40) <= 0.29);
  TEST_ASSERT(max_err(19.8, 20.2, 40) <= 0.20);
  TEST_ASSERT(max_err(49.8, 50.2, 40) <= 0.20);
  TEST_ASSERT(max_err(79.8, 80.2, 40) <= 0.20);
  TEST_ASSERT(max_err(99.4, 100.0, 40) <= 0.32);
}

void test_Co2Golden() {
  TEST_ASSERT_EQUAL_UINT(822U, Co2::kCodeCount);
  TEST_ASSERT_EQUAL_UINT(255U, Co2::kOneByteCount);
  TEST_ASSERT_EQUAL_UINT(223U, Co2::kTwoByteCount);
  TEST_ASSERT_EQUAL_UINT(4U, Co2::kMaxWireBytes);
  auto const& p = Co2::Logical();
  TEST_ASSERT_EQUAL(821, p.segs[0].intervals);
  TEST_ASSERT_EQUAL(254, p.segs[0].last_1);
  TEST_ASSERT_EQUAL(477, p.segs[0].last_2);
  TEST_ASSERT_DOUBLE_WITHIN(1.0e-9, 1.005414508222845, p.segs[0].r);
}

void test_Co2DecodedCuts() {
  auto at = [](std::uint32_t rank) {
    return AsDouble(Co2::Decode(Co2::wire_type{rank}));
  };
  TEST_ASSERT_DOUBLE_WITHIN(0.5, 380.0, at(0));
  TEST_ASSERT_DOUBLE_WITHIN(2.0, 1497.7907686, at(254));
  TEST_ASSERT_DOUBLE_WITHIN(2.0, 1505.9005690, at(255));
  TEST_ASSERT_DOUBLE_WITHIN(4.0, 4993.6617217, at(477));
  TEST_ASSERT_DOUBLE_WITHIN(4.0, 5020.6999442, at(478));
  TEST_ASSERT_DOUBLE_WITHIN(1.0, 32000.0, at(821));
}

void test_RxWindowGolden() {
  auto const& p = RxWindow::Logical();
  TEST_ASSERT_EQUAL(132, p.segs[0].intervals);
  TEST_ASSERT_EQUAL(122, p.segs[1].intervals);
  TEST_ASSERT_EQUAL(255, p.segs[2].intervals);
  TEST_ASSERT_EQUAL(2536, p.segs[3].intervals);
  TEST_ASSERT_EQUAL_UINT(255U, RxWindow::kOneByteCount);
  TEST_ASSERT_EQUAL_UINT(255U, RxWindow::kTwoByteCount);
  TEST_ASSERT_EQUAL_UINT(3046U, RxWindow::kCodeCount);
  TEST_ASSERT_EQUAL_UINT(4U, RxWindow::kMaxWireBytes);
  TEST_ASSERT_DOUBLE_WITHIN(1.0e-9, 1.0355033664891309, p.segs[0].r);
  TEST_ASSERT_DOUBLE_WITHIN(1.0e-9, 1.0341296978352505, p.segs[1].r);
  TEST_ASSERT_DOUBLE_WITHIN(1.0e-6, 1.012401106168161, p.segs[2].q);
}

void test_BatteryGolden() {
  TEST_ASSERT_EQUAL_UINT(256U, Battery::kCodeCount);
  auto const& p = Battery::Logical();
  TEST_ASSERT_EQUAL(130, p.segs[0].intervals);
  TEST_ASSERT_EQUAL(125, p.segs[1].intervals);
  TEST_ASSERT_DOUBLE_WITHIN(1.0e-9, 1.0116049124714404, p.segs[0].q);
  TEST_ASSERT_DOUBLE_WITHIN(1.0e-7, 0.0088601265, p.segs[0].step0);
}

void test_ConnectDurationGolden() {
  TEST_ASSERT_EQUAL_UINT(256U, ConnectDuration::kCodeCount);
  auto const& p = ConnectDuration::Logical();
  TEST_ASSERT_EQUAL(102, p.segs[0].intervals);
  TEST_ASSERT_EQUAL(153, p.segs[1].intervals);
  TEST_ASSERT_DOUBLE_WITHIN(1.0e-9, 1.0228310927967654, p.segs[0].r);
  TEST_ASSERT_DOUBLE_WITHIN(1.0e-9, 1.0224789769119687, p.segs[1].r);
}

void test_RoundTripAllFormats() {
  CheckRankRoundTrip<Humidity>();
  CheckRankRoundTrip<Co2>();
  CheckRankRoundTrip<Battery>();
  CheckRankRoundTrip<ConnectDuration>();
  CheckRankRoundTrip<RxWindow>();
}

void test_UniqueAndSerializeSmall() {
  CheckUniqueRaws<Humidity>();
  CheckUniqueRaws<Battery>();
  CheckUniqueRaws<ConnectDuration>();
  CheckUniqueRaws<RxWindow>();
  CheckSerializeRoundTrip<Humidity>();
  CheckSerializeRoundTrip<Battery>();
  CheckSerializeRoundTrip<ConnectDuration>();
}

void test_Co2UniqueAndSerialize() {
  CheckUniqueRaws<Co2>();
  CheckSerializeRoundTrip<Co2>();
}

template <typename Num>
double MaxAbsErrorRatio(std::int64_t den, std::int64_t n0, std::int64_t n1,
                        std::int64_t step) {
  double m = 0.0;
  for (std::int64_t n = n0; n <= n1; n += step) {
    auto const v = Num::runtime_type::FromRatio(n, den);
    auto const got = Num::TryFromRuntime(v);
    if (!got.has_value()) {
      continue;
    }
    double const x = static_cast<double>(n) / static_cast<double>(den);
    m = std::max(m, std::fabs(AsDouble(got->Value()) - x));
  }
  return m;
}

void test_DenseSampling() {
  TEST_ASSERT(MaxAbsErrorRatio<Temperature>(100, -4000, 12500, 1) <= 0.22);
  TEST_ASSERT(MaxAbsErrorRatio<Humidity>(100, 0, 10000, 1) <= 0.32);
  TEST_ASSERT(MaxAbsErrorRatio<Rssi>(10, -1270, 0, 1) <= 0.51);
  TEST_ASSERT(MaxAbsErrorRatio<Battery>(10000, 21500, 30000, 1) <= 0.005);
  TEST_ASSERT(MaxAbsErrorRatio<ConnectDuration>(1000, 200, 60000, 1) <= 0.70);
  double co2_rel = 0.0;
  for (int ppm = 380; ppm <= 32000; ++ppm) {
    auto const v = Co2::runtime_type::FromInteger(ppm);
    auto const n = Co2::TryFromRuntime(v);
    TEST_ASSERT(n.has_value());
    double const got = AsDouble(n->Value());
    double const rel = std::fabs(got - static_cast<double>(ppm)) /
                       static_cast<double>(ppm);
    co2_rel = std::max(co2_rel, rel);
  }
  TEST_ASSERT(co2_rel <= 0.0035);
  TEST_ASSERT(MaxAbsErrorRatio<RxWindow>(1, 1, 86400, 1) <= 23.0);
  TEST_ASSERT(MaxAbsErrorRatio<RxWindow>(1, 86000, 86400, 1) <= 12.0);
}

}  // namespace ae::test_segmented_number_formats

int test_segmented_number_formats() {
  UNITY_BEGIN();
  RUN_TEST(ae::test_segmented_number_formats::test_HumidityGolden);
  RUN_TEST(ae::test_segmented_number_formats::test_HumidityErrors);
  RUN_TEST(ae::test_segmented_number_formats::test_Co2Golden);
  RUN_TEST(ae::test_segmented_number_formats::test_Co2DecodedCuts);
  RUN_TEST(ae::test_segmented_number_formats::test_RxWindowGolden);
  RUN_TEST(ae::test_segmented_number_formats::test_BatteryGolden);
  RUN_TEST(ae::test_segmented_number_formats::test_ConnectDurationGolden);
  RUN_TEST(ae::test_segmented_number_formats::test_RoundTripAllFormats);
  RUN_TEST(ae::test_segmented_number_formats::test_UniqueAndSerializeSmall);
  RUN_TEST(ae::test_segmented_number_formats::test_Co2UniqueAndSerialize);
  RUN_TEST(ae::test_segmented_number_formats::test_DenseSampling);
  return UNITY_END();
}
