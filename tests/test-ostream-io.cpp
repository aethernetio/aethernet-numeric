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

#include <sstream>

#include "numeric/ostream_io.h"

namespace ae::test_ostream_io {

void test_TieredInt() {
  using T = TieredInt<std::uint8_t, 254>;

  std::ostringstream os;
  os << T{123};

  TEST_ASSERT_EQUAL_STRING("123", os.str().c_str());
}

void test_TieredIntSigned() {
  using S = TieredInt<std::int8_t, 10, 20>;

  std::ostringstream os;
  os << S{-5};

  TEST_ASSERT_EQUAL_STRING("-5", os.str().c_str());
}

void test_FixedPointQ71() {
  using F = FixedPoint<std::uint8_t, 100.0>;

  {
    std::ostringstream os;
    os << F::FromRaw(1);

    TEST_ASSERT_EQUAL_STRING("0.5", os.str().c_str());
  }

  {
    std::ostringstream os;
    os << F::FromInteger(10);

    TEST_ASSERT_EQUAL_STRING("10", os.str().c_str());
  }
}

void test_FixedPointSigned() {
  using S = FixedPoint<std::int8_t, 63.0>;

  std::ostringstream os;
  os << S::FromRaw(-1);

  TEST_ASSERT_EQUAL_STRING("-0.5", os.str().c_str());
}

void test_FixedPointTieredIntRep() {
  using Raw = TieredInt<std::uint8_t, 254>;
  using F = FixedPoint<Raw, 60.0>;

  std::ostringstream os;
  os << F::FromInteger(30);

  TEST_ASSERT_EQUAL_STRING("30", os.str().c_str());
}

}  // namespace ae::test_ostream_io

int test_ostream_io() {
  UNITY_BEGIN();
  RUN_TEST(ae::test_ostream_io::test_TieredInt);
  RUN_TEST(ae::test_ostream_io::test_TieredIntSigned);
  RUN_TEST(ae::test_ostream_io::test_FixedPointQ71);
  RUN_TEST(ae::test_ostream_io::test_FixedPointSigned);
  RUN_TEST(ae::test_ostream_io::test_FixedPointTieredIntRep);
  return UNITY_END();
}
