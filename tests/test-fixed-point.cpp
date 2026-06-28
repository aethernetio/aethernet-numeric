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

#include "numeric/fixed_point.h"
#include "numeric/tiered_int.h"

namespace ae::test_fixed_point {

using F = FixedPoint<std::uint8_t, 123.5>;
static_assert(!numeric_traits<F>::kIsSigned);
static_assert(F::FromInteger(0).raw_value() == 0);
static_assert(F::FromDouble(123.5).raw_value() == 255);

using Huge = FixedPoint<std::uint8_t, 1000000000.0>;
static_assert(Huge::FromInteger(1000000000).raw_value() == 255);

using Tiny = FixedPoint<std::uint8_t, 0.0000000001>;
static_assert(Tiny::FromDouble(0.0000000001).raw_value() == 255);

using S = FixedPoint<std::int8_t, 10.0>;
static_assert(S::kRawMin == -127);
static_assert(S::kRawMax == 127);
static_assert(S::FromInteger(0).raw_value() == 0);
static_assert(S::FromInteger(10).raw_value() == 127);
static_assert(S::FromInteger(-10).raw_value() == -127);

using C = FixedPoint<std::int8_t, 10.0>;
static_assert(C::FromInteger(1) < C::FromInteger(2));

using A = FixedPoint<std::uint8_t, 10.0>;
static_assert(A::FromInteger(1) + A::FromInteger(2) > A::FromInteger(2));

using U = FixedPoint<std::uint8_t, 10.0>;
static_assert(U::FromInteger(100).raw_value() == U::kRawMax);
static_assert(U::FromInteger(-100).raw_value() == U::kRawMin);

using Raw = TieredInt<std::uint8_t, 254>;
using Compact = FixedPoint<Raw, 60.0>;
static_assert(numeric_traits<Compact>::kIsFixedPoint);
static_assert(Compact::FromInteger(0).raw_value() == 0);
static_assert(Compact::FromInteger(60).raw_value() == Raw::kUpper);

void test_RuntimeRoundTrip() {
  const auto f = F::FromInteger(61);
  TEST_ASSERT_EQUAL(126, f.raw_value());

  const auto sum = A::FromInteger(1) + A::FromInteger(2);
  TEST_ASSERT(sum > A::FromInteger(2));
  TEST_ASSERT(sum > A::FromInteger(1));

  const auto diff = A::FromInteger(5) - A::FromInteger(2);
  TEST_ASSERT(diff > A::FromInteger(2));

  const auto neg = -S::FromInteger(3);
  TEST_ASSERT(neg.raw_value() < 0);
}

}  // namespace ae::test_fixed_point

int test_fixed_point() {
  UNITY_BEGIN();
  RUN_TEST(ae::test_fixed_point::test_RuntimeRoundTrip);
  return UNITY_END();
}
