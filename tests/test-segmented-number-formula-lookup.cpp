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

#include "segmented_test_formats.h"

namespace ae::test_segmented_number_formula_lookup {

using test_segmented_formats::Battery;
using test_segmented_formats::BatteryLookup;
using test_segmented_formats::Co2;
using test_segmented_formats::Co2Lookup;
using test_segmented_formats::ConnectDuration;
using test_segmented_formats::ConnectLookup;
using test_segmented_formats::Humidity;
using test_segmented_formats::HumidityLookup;
using test_segmented_formats::Rssi;
using test_segmented_formats::RssiLookup;
using test_segmented_formats::RxWindow;
using test_segmented_formats::RxWindowLookup;
using test_segmented_formats::Temperature;
using test_segmented_formats::TemperatureLookup;

static_assert(Rssi::kLookupTableBytes == 0);
static_assert(RssiLookup::kLookupTableBytes > 0);
static_assert(Temperature::kLookupTableBytes == 0);
static_assert(TemperatureLookup::kLookupTableBytes > 0);

template <typename Formula, typename Lookup>
void CompareAllRanks() {
  TEST_ASSERT_EQUAL_UINT(Formula::kCodeCount, Lookup::kCodeCount);
  TEST_ASSERT_EQUAL_UINT(Formula::kOneByteCount, Lookup::kOneByteCount);
  TEST_ASSERT_EQUAL_UINT(Formula::kMaxWireBytes, Lookup::kMaxWireBytes);
  for (std::uint32_t rank = 0;
       rank < static_cast<std::uint32_t>(Formula::kCodeCount); ++rank) {
    auto const wf = test_segmented_formats::WireFromRank<Formula>(rank);
    auto const wl = test_segmented_formats::WireFromRank<Lookup>(rank);
    auto const df = Formula::Decode(wf);
    auto const dl = Lookup::Decode(wl);
    TEST_ASSERT_EQUAL(df.RawValue(), dl.RawValue());
    auto const ef = Formula::TryEncode(df);
    auto const el = Lookup::TryEncode(dl);
    TEST_ASSERT(ef.has_value());
    TEST_ASSERT(el.has_value());
    TEST_ASSERT_EQUAL_UINT(static_cast<std::uint32_t>(*ef),
                           static_cast<std::uint32_t>(*el));
  }
}

void test_RssiFormulaLookup() { CompareAllRanks<Rssi, RssiLookup>(); }

void test_HumidityFormulaLookup() {
  CompareAllRanks<Humidity, HumidityLookup>();
}

void test_BatteryFormulaLookup() { CompareAllRanks<Battery, BatteryLookup>(); }

void test_ConnectFormulaLookup() {
  CompareAllRanks<ConnectDuration, ConnectLookup>();
}

void test_TemperatureFormulaLookup() {
  CompareAllRanks<Temperature, TemperatureLookup>();
}

void test_Co2FormulaLookup() { CompareAllRanks<Co2, Co2Lookup>(); }

void test_RxWindowFormulaLookup() {
  CompareAllRanks<RxWindow, RxWindowLookup>();
}

}  // namespace ae::test_segmented_number_formula_lookup

int test_segmented_number_formula_lookup() {
  UNITY_BEGIN();
  RUN_TEST(ae::test_segmented_number_formula_lookup::test_RssiFormulaLookup);
  RUN_TEST(
      ae::test_segmented_number_formula_lookup::test_HumidityFormulaLookup);
  RUN_TEST(ae::test_segmented_number_formula_lookup::test_BatteryFormulaLookup);
  RUN_TEST(
      ae::test_segmented_number_formula_lookup::test_ConnectFormulaLookup);
  RUN_TEST(
      ae::test_segmented_number_formula_lookup::test_TemperatureFormulaLookup);
  RUN_TEST(ae::test_segmented_number_formula_lookup::test_Co2FormulaLookup);
  RUN_TEST(
      ae::test_segmented_number_formula_lookup::test_RxWindowFormulaLookup);
  return UNITY_END();
}
