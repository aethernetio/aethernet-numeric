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

#include <unity.h>

#include <cstdint>

#include "numeric/exponential.h"
#include "numeric/fixed_point.h"
#include "numeric/text_io.h"
#include "numeric/tiered_int.h"

namespace ae::test_text_io {

using T = TieredInt<std::uint8_t, 254>;
static_assert(numeric_traits<T>::kIsIntegerLike);

void test_TieredInt() {
  TEST_ASSERT_EQUAL_STRING("123", ToString(T{123}).c_str());

  T parsed{};
  TEST_ASSERT(FromString("123", parsed));
  TEST_ASSERT_EQUAL(123, parsed.value_);
  TEST_ASSERT_FALSE(FromString("152915291529152915299", parsed));
}

void test_FixedPointQ71() {
  using F100 = FixedPoint<std::uint8_t, 100.0>;

  TEST_ASSERT_EQUAL_STRING("10", ToString(F100::FromInteger(10)).c_str());
  TEST_ASSERT_EQUAL_STRING("0.5", ToString(F100::FromRaw(1)).c_str());
  TEST_ASSERT_EQUAL_STRING("127.5", ToString(F100::FromRaw(255)).c_str());

  F100 x = F100::FromInteger(0);
  TEST_ASSERT(FromString("10.5", x));
  TEST_ASSERT_EQUAL(F100::FromDouble(10.5).raw_value(), x.raw_value());
}

void test_FixedPointQ53() {
  using F30 = FixedPoint<std::uint8_t, 30.0>;

  TEST_ASSERT_EQUAL_STRING("0.125", ToString(F30::FromRaw(1)).c_str());
  TEST_ASSERT_EQUAL_STRING("10", ToString(F30::FromInteger(10)).c_str());

  F30 y = F30::FromInteger(0);
  TEST_ASSERT(FromString("0.125", y));
  TEST_ASSERT_EQUAL(1, y.raw_value());
}

void test_SignedFixedPoint() {
  using S100 = FixedPoint<std::int8_t, 100.0>;
  using SFrac = FixedPoint<std::int8_t, 63.0>;

  TEST_ASSERT_EQUAL_STRING("-10", ToString(S100::FromInteger(-10)).c_str());
  TEST_ASSERT_EQUAL_STRING("-0.5", ToString(SFrac::FromRaw(-1)).c_str());

  S100 s = S100::FromInteger(0);
  TEST_ASSERT(FromString("-10.5", s));
  TEST_ASSERT(s < S100::FromInteger(0));
}

void test_LargePositiveScale() {
  using Huge = FixedPoint<std::uint8_t, 1000000000.0>;

  const auto text = ToString(Huge::FromRaw(1));
  TEST_ASSERT(text.size() > 1);
}

void test_Exponential() {
  using Runtime = FixedPoint<std::uint32_t, 60.0>;
  using Wire = TieredInt<std::uint8_t, 249, 1529>;
  using E = Exponential<Runtime, Wire, 0.001, 60.0, 1529>;

  constexpr auto e = E::from_double(1.0);
  const auto text = ToString(e);
  TEST_ASSERT(!text.empty());

  E parsed{};
  TEST_ASSERT(FromString("1.0", parsed));
  TEST_ASSERT(parsed > E::from_double(0.9));
  TEST_ASSERT(parsed < E::from_double(1.1));
}

}  // namespace ae::test_text_io

int test_text_io() {
  UNITY_BEGIN();
  RUN_TEST(ae::test_text_io::test_TieredInt);
  RUN_TEST(ae::test_text_io::test_FixedPointQ71);
  RUN_TEST(ae::test_text_io::test_FixedPointQ53);
  RUN_TEST(ae::test_text_io::test_SignedFixedPoint);
  RUN_TEST(ae::test_text_io::test_LargePositiveScale);
  RUN_TEST(ae::test_text_io::test_Exponential);
  return UNITY_END();
}
