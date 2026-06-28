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
#include "numeric/runtime_numeric_traits.h"
#include "numeric/tiered_int.h"

namespace ae::test_exponential {

static_assert(runtime_numeric_traits<std::int32_t>::kIsSupported);
static_assert(runtime_numeric_traits<std::int32_t>::kIsSigned);

using F = FixedPoint<std::uint8_t, 100.0>;
static_assert(runtime_numeric_traits<F>::kIsSupported);
static_assert(!runtime_numeric_traits<F>::kIsSigned);

using Runtime = FixedPoint<std::uint32_t, 60.0>;
using Wire = TieredInt<std::uint8_t, 254>;
using E = Exponential<Runtime, Wire, 0.001, 60.0>;

static_assert(E::kBoundaryCode == numeric_traits<Wire>::kRawMax);

using Partial = Exponential<Runtime, Wire, 0.001, 60.0, 200>;
static_assert(Partial::kBoundaryCode == 200);

static_assert(numeric_traits<E>::kIsExponential);
static_assert(!numeric_traits<E>::kIsSigned);

static_assert(E::from_runtime(Runtime::FromInteger(0)).code_value() == 0);

constexpr auto one_ms = E::from_double(0.001);
static_assert(one_ms.code_value() == 1);

constexpr auto max_v = E::from_double(60.0);
static_assert(max_v.code_value() == E::kBoundaryCode);

static_assert(E::from_double(0.001) < E::from_double(0.01));
static_assert(E::from_double(1.0) < E::from_double(60.0));

constexpr auto e1 = E::from_double(1.0);
constexpr auto r1 = e1.to_runtime();

static_assert(r1 > Runtime::FromDouble(0.9));
static_assert(r1 < Runtime::FromDouble(1.1));

static_assert(E::from_double(1000.0).code_value() == E::kBoundaryCode);

using SRuntime = FixedPoint<std::int32_t, 60.0>;
using SE = Exponential<SRuntime, Wire, 0.001, 60.0>;

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

// ---- Final logical-value construction semantics ----

// Logical-value construction (consteval): E{1}, E{10}, E{0.001}.
constexpr E el1{1};
constexpr E el10{10};
constexpr E el_min{0.001};
static_assert(el1.code_value() == E::FromRuntimeInteger(1).code_value());
static_assert(el10.code_value() == E::FromRuntimeInteger(10).code_value());
static_assert(el1.code_value() != 0);
static_assert(el10.code_value() != 0);
static_assert(el_min.code_value() != 0);
static_assert(el1.code_value() < el10.code_value());

// Raw-code construction: E::Code(10) / E::FromCode(10).
static_assert(E::Code(10).code_value() == 10);
static_assert(E::FromCode(10).code_value() == 10);

// Accessors: code() returns stored code, value() decodes to RuntimeT.
static_assert(E::FromCode(0).code_value() == 0);
static_assert(E::FromCode(0).value() == Runtime::FromInteger(0));
static_assert(el1.value() > Runtime::FromDouble(0.9));
static_assert(el1.value() < Runtime::FromDouble(1.1));

// Runtime conversion APIs: clamp / checked.
static_assert(E::FromRuntimeInteger(123).code_value() == E::kBoundaryCode);
static_assert(E::Saturating(123).code_value() == E::kBoundaryCode);
static_assert(E::TryFromRuntimeInteger(10).has_value());
static_assert(!E::TryFromRuntimeInteger(123).has_value());

// Signed logical-value construction.
constexpr SE sel_neg{-1};
static_assert(sel_neg.is_negative());

void test_LogicalConstruction() {
  TEST_ASSERT(el1.code_value() != 0);
  TEST_ASSERT_EQUAL(10, E::Code(10).code_value());
  TEST_ASSERT_EQUAL(10, E::FromCode(10).code_value());

  const auto decoded = el1.value();
  TEST_ASSERT(decoded > Runtime::FromDouble(0.9));
  TEST_ASSERT(decoded < Runtime::FromDouble(1.1));
}

void test_RuntimeIntegerApis() {
  std::int64_t in_range = 10;
  std::int64_t out_of_range = 123;

  TEST_ASSERT_EQUAL(E::kBoundaryCode,
                    E::FromRuntimeInteger(out_of_range).code_value());
  TEST_ASSERT_EQUAL(E::kBoundaryCode,
                    E::Saturating(out_of_range).code_value());

  const auto ok = E::TryFromRuntimeInteger(in_range);
  TEST_ASSERT(ok.has_value());
  TEST_ASSERT(ok->code_value() != 0);

  const auto bad = E::TryFromRuntimeInteger(out_of_range);
  TEST_ASSERT_FALSE(bad.has_value());
}

void test_RuntimeRoundTrip() {
  const auto encoded = E::from_runtime(Runtime::FromInteger(0));
  TEST_ASSERT_EQUAL(0, encoded.code_value());

  const auto decoded = E::from_double(1.0).to_runtime();
  TEST_ASSERT(decoded > Runtime::FromDouble(0.9));
  TEST_ASSERT(decoded < Runtime::FromDouble(1.1));

  const auto saturated = E::from_double(1000.0);
  TEST_ASSERT_EQUAL(E::kBoundaryCode, saturated.code_value());

  const auto signed_neg = SE::from_double(-1.0);
  TEST_ASSERT(signed_neg.is_negative());
  TEST_ASSERT((-SE::from_double(1.0)).is_negative());
}

}  // namespace ae::test_exponential

int test_exponential() {
  UNITY_BEGIN();
  RUN_TEST(ae::test_exponential::test_LogicalConstruction);
  RUN_TEST(ae::test_exponential::test_RuntimeIntegerApis);
  RUN_TEST(ae::test_exponential::test_RuntimeRoundTrip);
  return UNITY_END();
}
