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

#include "segmented_test_formats.h"

namespace ae::test_segmented_number_size {

using test_segmented_formats::Battery;
using test_segmented_formats::Co2;
using test_segmented_formats::ConnectDuration;
using test_segmented_formats::Humidity;
using test_segmented_formats::Rssi;
using test_segmented_formats::RxWindow;
using test_segmented_formats::Temperature;

static_assert(sizeof(Rssi) == sizeof(Rssi::runtime_type));
static_assert(sizeof(Temperature) == sizeof(Temperature::runtime_type));
static_assert(sizeof(Humidity) == sizeof(Humidity::runtime_type));
static_assert(sizeof(Co2) == sizeof(Co2::runtime_type));
static_assert(sizeof(RxWindow) == sizeof(RxWindow::runtime_type));
static_assert(sizeof(Battery) == sizeof(Battery::runtime_type));
static_assert(sizeof(ConnectDuration) == sizeof(ConnectDuration::runtime_type));
static_assert(MaxWireBytes<Rssi>() == 1);
static_assert(MaxWireBytes<Temperature>() == 2);
static_assert(MaxWireBytes<Co2>() == 4);

void test_FootprintConstants() {
  TEST_ASSERT_EQUAL_UINT(sizeof(Rssi::runtime_type), sizeof(Rssi));
  TEST_ASSERT_EQUAL_UINT(1U, sizeof(Rssi::wire_type));
  TEST_ASSERT(Temperature::kFormulaCoefficientBytes > 0);
  TEST_ASSERT_EQUAL_UINT(3U, Temperature::kSegmentCount);
  TEST_ASSERT_EQUAL_UINT(1U, Rssi::kSegmentCount);
}

}  // namespace ae::test_segmented_number_size

int test_segmented_number_size() {
  UNITY_BEGIN();
  RUN_TEST(ae::test_segmented_number_size::test_FootprintConstants);
  return UNITY_END();
}
