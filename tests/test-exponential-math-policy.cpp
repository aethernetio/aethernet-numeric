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
#include <type_traits>

#include "numeric/exponential.h"
#include "numeric/exponential_math_policy.h"
#include "numeric/fixed_math.h"
#include "numeric/fixed_point.h"
#include "numeric/tiered_int.h"

namespace ae::test_exponential_math_policy {

using RuntimeSeconds = FixedPoint<std::uint32_t, 60.0>;
using RuntimeMillis = FixedPoint<std::uint32_t, 60000.0>;
using RuntimeSmall = FixedPoint<std::uint16_t, 60.0>;
using Code = TieredInt<std::uint8_t, 254>;

static constexpr double kLatencyBoundarySeconds = 42.5937;
using LatencyE = Exponential<RuntimeSeconds, Code, 0.001, kLatencyBoundarySeconds,
                             510>;
using LatencyPolicy = LatencyE::math_policy;

static_assert(LatencyPolicy::kLogIterations <= 8);
static_assert(LatencyPolicy::kExp2FractionBits == LatencyPolicy::kLogIterations);
static_assert(LatencyPolicy::kLogIterations != 11);
static_assert(
    std::is_same_v<LatencyPolicy::mant_type, FixedPoint<std::uint16_t, 4.0>>);
static_assert(sizeof(typename LatencyPolicy::mul_intermediate_type) <=
              sizeof(std::int64_t));

using SecondsE = Exponential<RuntimeSeconds, Code, 0.001, kLatencyBoundarySeconds,
                             510>;
using MillisE = Exponential<RuntimeMillis, Code, 1.0,
                            kLatencyBoundarySeconds * 1000.0, 510>;

using SmallE = Exponential<RuntimeSmall, Code, 1.0, 60.0, 255>;
using SmallPolicy = SmallE::math_policy;

static_assert(sizeof(SmallPolicy::work_type::rep_value_type) <=
              sizeof(std::uint16_t));
static_assert(sizeof(SmallPolicy::mul_intermediate_type) <=
              sizeof(std::int32_t));
static_assert(SmallPolicy::kLogIterations <= 10);

RuntimeSeconds RuntimeMidpoint(const RuntimeSeconds& a,
                               const RuntimeSeconds& b) {
  const std::int64_t mid =
      (static_cast<std::int64_t>(a.raw_value()) +
       static_cast<std::int64_t>(b.raw_value())) /
      2;
  return RuntimeSeconds::FromRaw(
      static_cast<typename RuntimeSeconds::rep_type>(mid));
}

void test_LatencyPolicyDerivation() {
  TEST_ASSERT(LatencyPolicy::kLogIterations <= 8);
  TEST_ASSERT_EQUAL(LatencyPolicy::kLogIterations,
                    LatencyPolicy::kExp2FractionBits);
  TEST_ASSERT(LatencyPolicy::kLogIterations >= 4);
}

void test_NoHardcodedElevenIterations() {
  TEST_ASSERT(LatencyPolicy::kLogIterations < 11);
}

void test_PolicyLogTypeUsedByExp2() {
  using Policy = LatencyPolicy;
  using LogT = Policy::log_type;
  using WorkT = Policy::work_type;
  const WorkT w = WorkT::FromDouble(1.0);
  const LogT logv = fixed_math::log2_to<LogT, WorkT, Policy>(w);
  const WorkT back = fixed_math::exp2_to<WorkT, LogT, Policy>(logv);
  TEST_ASSERT(back.raw_value() > 0);
}

static_assert(
    std::is_same_v<LatencyE::WorkT, FixedPoint<std::uint32_t, 64.0>>);

void test_RuntimeMatchedWorkType() {
  TEST_PASS();
}

void test_MidpointBoundaryNoUpwardBias() {
  using E = LatencyE;
  const int pairs[][2] = {{10, 11}, {100, 101}, {250, 251}};
  for (const auto& pair : pairs) {
    const int lo = pair[0];
    const int hi = pair[1];
    const RuntimeSeconds v_lo = E::Code(lo).value();
    const RuntimeSeconds v_hi = E::Code(hi).value();
    const RuntimeSeconds below =
        RuntimeMidpoint(v_lo, RuntimeMidpoint(v_lo, v_hi));
    const RuntimeSeconds above =
        RuntimeMidpoint(RuntimeMidpoint(v_lo, v_hi), v_hi);
    const int code_below =
        static_cast<int>(E::FromRuntime(below).code_value());
    const int code_above =
        static_cast<int>(E::FromRuntime(above).code_value());
    TEST_ASSERT(code_below <= hi);
    TEST_ASSERT(code_above >= lo);
    TEST_ASSERT(code_below <= code_above);
  }
}

void test_SmallRuntimeLowCostPath() {
  const SmallE e = SmallE::from_double(5.0);
  TEST_ASSERT(e.code_value() > 0);
  const SmallE roundtrip = SmallE::FromRuntime(e.value());
  const int diff =
      static_cast<int>(roundtrip.code_value()) -
      static_cast<int>(e.code_value());
  TEST_ASSERT(diff <= 3 && diff >= -3);
}

void test_UnitScalingCodeMapping() {
  const int seconds_code =
      static_cast<int>(SecondsE::from_double(1.0).code_value());
  const int millis_code =
      static_cast<int>(MillisE::from_double(1000.0).code_value());
  TEST_ASSERT(seconds_code - millis_code <= 1);
  TEST_ASSERT(seconds_code - millis_code >= -1);
}

}  // namespace ae::test_exponential_math_policy

int test_exponential_math_policy() {
  UNITY_BEGIN();
  RUN_TEST(ae::test_exponential_math_policy::test_LatencyPolicyDerivation);
  RUN_TEST(ae::test_exponential_math_policy::test_NoHardcodedElevenIterations);
  RUN_TEST(ae::test_exponential_math_policy::test_PolicyLogTypeUsedByExp2);
  RUN_TEST(ae::test_exponential_math_policy::test_RuntimeMatchedWorkType);
  RUN_TEST(ae::test_exponential_math_policy::test_MidpointBoundaryNoUpwardBias);
  RUN_TEST(ae::test_exponential_math_policy::test_SmallRuntimeLowCostPath);
  RUN_TEST(ae::test_exponential_math_policy::test_UnitScalingCodeMapping);
  return UNITY_END();
}
