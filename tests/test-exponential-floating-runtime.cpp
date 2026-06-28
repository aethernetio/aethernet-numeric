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
#include "numeric/exponential_floating_runtime.h"
#include "numeric/exponential_wire_io.h"
#include "numeric/tiered_int.h"
#include "numeric/wire_io.h"

namespace ae::test_exponential_floating_runtime {

static_assert(runtime_numeric_traits<float>::kIsSupported);
static_assert(runtime_numeric_traits<double>::kIsSupported);

using Wire = TieredInt<std::uint8_t, 254>;
using EFloat = Exponential<float, Wire, 0.001f, 60.0f>;
using EDouble = Exponential<double, Wire, 0.001, 60.0>;

static_assert(EFloat::kIsSigned);
static_assert(EFloat::kBoundaryCode == numeric_traits<Wire>::kRawMax);

static_assert(EDouble::kIsSigned);
static_assert(EDouble::kBoundaryCode == numeric_traits<Wire>::kRawMax);

constexpr auto one_ms = EFloat::from_double(0.001);
static_assert(one_ms.code_value() == 2);

constexpr auto max_v = EFloat::from_double(60.0);
static_assert(max_v.code_value() == EFloat::kPositiveBoundaryCode);

void test_FloatFromDoubleToRuntime() {
  const auto e = EFloat::from_runtime(1.0f);
  const float r = e.to_runtime();

  TEST_ASSERT(r > 0.9f);
  TEST_ASSERT(r < 1.1f);
}

void test_FloatSignedOrdering() {
  const auto neg = EFloat::from_runtime(-1.0f);
  const auto zero = EFloat::from_runtime(0.0f);
  const auto pos = EFloat::from_runtime(1.0f);

  TEST_ASSERT(neg < zero);
  TEST_ASSERT(zero < pos);
  TEST_ASSERT(neg.is_negative());
  TEST_ASSERT(pos.is_positive());
  TEST_ASSERT_EQUAL(pos.abs().code_value(), neg.abs().code_value());
}

void test_DoublePrecisionSmoke() {
  const auto e = EDouble::from_runtime(1.0);
  const double r = e.to_runtime();

  TEST_ASSERT(r > 0.9);
  TEST_ASSERT(r < 1.1);
}

void test_WireSerialization() {
  std::uint8_t buf[MaxWireBytes<EFloat>()] = {};
  const auto n = Serialize(EFloat::from_runtime(1.0f), buf);
  const auto restored = Deserialize<EFloat>(buf, n).value;

  TEST_ASSERT_EQUAL(EFloat::from_runtime(1.0f).code_value(),
                    restored.code_value());
}

}  // namespace ae::test_exponential_floating_runtime

int test_exponential_floating_runtime() {
  UNITY_BEGIN();
  RUN_TEST(ae::test_exponential_floating_runtime::test_FloatFromDoubleToRuntime);
  RUN_TEST(ae::test_exponential_floating_runtime::test_FloatSignedOrdering);
  RUN_TEST(ae::test_exponential_floating_runtime::test_DoublePrecisionSmoke);
  RUN_TEST(ae::test_exponential_floating_runtime::test_WireSerialization);
  return UNITY_END();
}
