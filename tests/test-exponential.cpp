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
#include "numeric/tiered_int.h"

namespace ae::test_exponential {

using Runtime = FixedPoint<std::uint32_t, 60.0>;
using Wire = TieredInt<std::uint8_t, 254>;
using E = Exponential<Runtime, Wire, 0.001, 60.0, 254>;

static_assert(numeric_traits<E>::kIsExponential);
static_assert(!numeric_traits<E>::kIsSigned);

static_assert(E::from_runtime(Runtime::FromInteger(0)).code_value() == 0);

constexpr auto one_ms = E::from_double(0.001);
static_assert(one_ms.code_value() == 1);

constexpr auto max_v = E::from_double(60.0);
static_assert(max_v.code_value() == 254);

static_assert(E::from_double(0.001) < E::from_double(0.01));
static_assert(E::from_double(1.0) < E::from_double(60.0));

constexpr auto e1 = E::from_double(1.0);
constexpr auto r1 = e1.to_runtime();

static_assert(r1 > Runtime::FromDouble(0.9));
static_assert(r1 < Runtime::FromDouble(1.1));

static_assert(E::from_double(1000.0).code_value() == 254);

using SRuntime = FixedPoint<std::int32_t, 60.0>;
using SE = Exponential<SRuntime, Wire, 0.001, 60.0, 254>;

static_assert(numeric_traits<SE>::kIsSigned);

constexpr auto neg = SE::from_double(-1.0);
constexpr auto zero = SE::from_double(0.0);
constexpr auto pos = SE::from_double(1.0);

static_assert(neg < zero);
static_assert(zero < pos);
static_assert(neg.is_negative());
static_assert(pos.is_positive());
static_assert(!zero.is_negative());
static_assert(!zero.is_positive());

static_assert((-pos).is_negative());
static_assert((-neg).is_positive());
static_assert(neg.abs() == pos.abs());

static_assert(SE::from_double(-10.0) < SE::from_double(-1.0));
static_assert(SE::from_double(-1.0) < SE::from_double(0.0));
static_assert(SE::from_double(1.0) < SE::from_double(10.0));

constexpr auto low = SE::from_double(-1.0);
constexpr auto high = SE::from_double(1.0);
constexpr auto v = SE::from_double(10.0);

static_assert(clamp(v, low, high) == high);
static_assert(min(low, high) == low);
static_assert(max(low, high) == high);

void test_RuntimeRoundTrip() {
  const auto encoded = E::from_runtime(Runtime::FromInteger(0));
  TEST_ASSERT_EQUAL(0, encoded.code_value());

  const auto decoded = E::from_double(1.0).to_runtime();
  TEST_ASSERT(decoded > Runtime::FromDouble(0.9));
  TEST_ASSERT(decoded < Runtime::FromDouble(1.1));

  const auto saturated = E::from_double(1000.0);
  TEST_ASSERT_EQUAL(254, saturated.code_value());

  const auto signed_neg = SE::from_double(-1.0);
  TEST_ASSERT(signed_neg.is_negative());
  TEST_ASSERT((-SE::from_double(1.0)).is_negative());
}

}  // namespace ae::test_exponential

int test_exponential() {
  UNITY_BEGIN();
  RUN_TEST(ae::test_exponential::test_RuntimeRoundTrip);
  return UNITY_END();
}
