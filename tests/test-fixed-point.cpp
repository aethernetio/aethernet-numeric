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
#include <limits>
#include <type_traits>

#include "numeric/fixed_point.h"
#include "numeric/tiered_int.h"

namespace ae::test_fixed_point {

using T = TieredInt<std::uint8_t, 254>;
static_assert(numeric_traits<T>::kIsIntegerLike);
static_assert(IntegralStorage<T>);

using F30 = FixedPoint<std::uint8_t, 30.0>;
static_assert(F30::kScaleExp == -3);
static_assert(F30::kFractionBits == 3);

using F100 = FixedPoint<std::uint8_t, 100.0>;
static_assert(F100::kScaleExp == -1);
static_assert(F100::kFractionBits == 1);
static_assert(F100::FromInteger(100).raw_value() == 200);
static_assert(F100::FromDouble(127.5).raw_value() == 255);

using F130 = FixedPoint<std::uint8_t, 130.0>;
static_assert(F130::kScaleExp == 0);
static_assert(F130::kFractionBits == 0);

using S100 = FixedPoint<std::int8_t, 100.0>;
static_assert(S100::kRawMin == -127);
static_assert(S100::kRawMax == 127);

using S = FixedPoint<std::int8_t, 10.0>;
static_assert(S::FromRaw(std::numeric_limits<std::int8_t>::min()).raw_value() ==
              S::kRawMin);

using A = FixedPoint<std::uint8_t, 30.0>;
using B = FixedPoint<std::uint8_t, 100.0>;

constexpr auto a = A::FromInteger(10);
constexpr auto b = B::FromInteger(20);
constexpr auto r = a + b;

static_assert(std::is_same_v<decltype(r), const FixedPoint<std::uint8_t, 130.0>> ||
              std::is_same_v<decltype(r), FixedPoint<std::uint8_t, 130.0>>);
static_assert(r == FixedPoint<std::uint8_t, 130.0>::FromInteger(30));
static_assert(decltype(r)::kScaleExp == 0);

constexpr auto b_as_a = B::Cast<A>(B::FromInteger(10));
static_assert(b_as_a == A::FromInteger(10));

using Dst = FixedPoint<std::uint8_t, 20.0>;
constexpr auto div = div_to<Dst>(B::FromInteger(100), B::FromInteger(10));
static_assert(div == Dst::FromInteger(10));

using Raw = TieredInt<std::uint8_t, 254>;
using Compact = FixedPoint<Raw, 60.0>;
static_assert(numeric_traits<Compact>::kIsFixedPoint);
static_assert(Compact::FromInteger(60).raw_value() <= Raw::kUpper);

void test_RuntimeRoundTrip() {
  const auto sum = A::FromInteger(10) + B::FromInteger(20);
  TEST_ASSERT((sum == FixedPoint<std::uint8_t, 130.0>::FromInteger(30)));

  const auto cast = B::Cast<A>(B::FromInteger(10));
  TEST_ASSERT(cast == A::FromInteger(10));

  const auto quotient = div_to<Dst>(B::FromInteger(100), B::FromInteger(10));
  TEST_ASSERT(quotient == Dst::FromInteger(10));
}

}  // namespace ae::test_fixed_point

int test_fixed_point() {
  UNITY_BEGIN();
  RUN_TEST(ae::test_fixed_point::test_RuntimeRoundTrip);
  return UNITY_END();
}
