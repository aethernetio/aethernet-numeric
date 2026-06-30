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

#include "numeric/exponential.h"
#include "numeric/exponential_floating_runtime.h"
#include "numeric/exponential_wire_io.h"
#include "numeric/tiered_int.h"
#include "numeric/wire_io.h"

namespace ae::test_exponential_floating_runtime {

static_assert(runtime_numeric_traits<float>::kIsSupported);
static_assert(runtime_numeric_traits<double>::kIsSupported);

using Wire = TieredInt<std::uint8_t, 249, 1529>;
using EFloat = Exponential<float, Wire, 0.001f, 60.0f>;
using EDouble = Exponential<double, Wire, 0.001, 60.0>;

static_assert(EFloat::kIsSigned);
static_assert(EFloat::kBoundaryCode == 1529);

static_assert(EDouble::kIsSigned);
static_assert(EDouble::kBoundaryCode == 1529);

constexpr auto one_ms = EFloat::FromDouble(0.001);
static_assert(one_ms.CodeValue() == 2);

constexpr auto max_v = EFloat::FromDouble(60.0);
static_assert(max_v.CodeValue() == EFloat::kPositiveBoundaryCode);

void test_FloatFromDoubleToRuntime() {
  const auto e = EFloat::FromRuntime(1.0f);
  const float r = e.ToRuntime();

  TEST_ASSERT(r > 0.9f);
  TEST_ASSERT(r < 1.1f);
}

void test_FloatSignedOrdering() {
  const auto neg = EFloat::FromRuntime(-1.0f);
  const auto zero = EFloat::FromRuntime(0.0f);
  const auto pos = EFloat::FromRuntime(1.0f);

  TEST_ASSERT(neg < zero);
  TEST_ASSERT(zero < pos);
  TEST_ASSERT(neg.IsNegative());
  TEST_ASSERT(pos.IsPositive());
  TEST_ASSERT_EQUAL(pos.Abs().CodeValue(), neg.Abs().CodeValue());
}

void test_DoublePrecisionSmoke() {
  const auto e = EDouble::FromRuntime(1.0);
  const double r = e.ToRuntime();

  TEST_ASSERT(r > 0.9);
  TEST_ASSERT(r < 1.1);
}

void test_WireSerialization() {
  std::uint8_t buf[MaxWireBytes<EFloat>()] = {};
  const auto n = Serialize(EFloat::FromRuntime(1.0f), buf);
  const auto restored = Deserialize<EFloat>(buf, n).value;

  TEST_ASSERT_EQUAL(EFloat::FromRuntime(1.0f).CodeValue(),
                    restored.CodeValue());
}

}  // namespace ae::test_exponential_floating_runtime

int test_exponential_floating_runtime() {
  UNITY_BEGIN();
  RUN_TEST(
      ae::test_exponential_floating_runtime::test_FloatFromDoubleToRuntime);
  RUN_TEST(ae::test_exponential_floating_runtime::test_FloatSignedOrdering);
  RUN_TEST(ae::test_exponential_floating_runtime::test_DoublePrecisionSmoke);
  RUN_TEST(ae::test_exponential_floating_runtime::test_WireSerialization);
  return UNITY_END();
}
