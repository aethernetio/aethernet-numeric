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

#include "numeric/fixed_math.h"
#include "numeric/fixed_point.h"

namespace ae::test_fixed_math {

using Runtime = FixedPoint<std::uint32_t, 60.0>;
using SmallRuntime = FixedPoint<std::uint16_t, 60.0>;
using Log = FixedPoint<std::int16_t, 16.0>;

template <typename T>
bool NearRaw(T a, T b, std::int64_t max_raw_diff) {
  const std::int64_t diff = static_cast<std::int64_t>(a.raw_value()) -
                            static_cast<std::int64_t>(b.raw_value());
  return diff <= max_raw_diff && diff >= -max_raw_diff;
}

constexpr Log log2_3 = Log::FromDouble(1.584962500721156);
constexpr Log log2_5 = Log::FromDouble(2.321928094887362);
constexpr Log log2_10 = Log::FromDouble(3.321928094887362);
constexpr Log log2_30 = Log::FromDouble(4.906890595608519);
constexpr Log log2_60 = Log::FromDouble(5.906890595608519);

constexpr Runtime one{1};
constexpr Log zero = fixed_math::log2_to<Log>(one);
static_assert(zero == Log{0});

constexpr Runtime two = fixed_math::exp2_to<Runtime>(Log{1});
static_assert(two == Runtime{2});

void test_Log2PowersOfTwo() {
  TEST_ASSERT(NearRaw(fixed_math::log2_to<Log>(Runtime{1}), Log{0}, 1));
  TEST_ASSERT(NearRaw(fixed_math::log2_to<Log>(Runtime{2}), Log{1}, 1));
  TEST_ASSERT(NearRaw(fixed_math::log2_to<Log>(Runtime{4}), Log{2}, 1));
  TEST_ASSERT(NearRaw(fixed_math::log2_to<Log>(Runtime{8}), Log{3}, 1));
  TEST_ASSERT(NearRaw(fixed_math::log2_to<Log>(Runtime{16}), Log{4}, 1));
}

void test_Exp2PowersOfTwo() {
  TEST_ASSERT(NearRaw(fixed_math::exp2_to<Runtime>(Log{0}), Runtime{1}, 2));
  TEST_ASSERT(NearRaw(fixed_math::exp2_to<Runtime>(Log{1}), Runtime{2}, 2));
  TEST_ASSERT(NearRaw(fixed_math::exp2_to<Runtime>(Log{2}), Runtime{4}, 2));
  TEST_ASSERT(NearRaw(fixed_math::exp2_to<Runtime>(Log{3}), Runtime{8}, 4));
  TEST_ASSERT(NearRaw(fixed_math::exp2_to<Runtime>(Log{4}), Runtime{16}, 8));
}

void test_Log2FractionalValues() {
  TEST_ASSERT(NearRaw(fixed_math::log2_to<Log>(Runtime{3}), log2_3, 4));
  TEST_ASSERT(NearRaw(fixed_math::log2_to<Log>(Runtime{5}), log2_5, 4));
  TEST_ASSERT(NearRaw(fixed_math::log2_to<Log>(Runtime{10}), log2_10, 4));
  TEST_ASSERT(NearRaw(fixed_math::log2_to<Log>(Runtime{30}), log2_30, 4));
  TEST_ASSERT(NearRaw(fixed_math::log2_to<Log>(Runtime{60}), log2_60, 4));
}

void test_Exp2FractionalValues() {
  TEST_ASSERT(NearRaw(fixed_math::exp2_to<Runtime>(log2_3), Runtime{3}, 150000));
  TEST_ASSERT(NearRaw(fixed_math::exp2_to<Runtime>(log2_5), Runtime{5}, 100000));
  TEST_ASSERT(NearRaw(fixed_math::exp2_to<Runtime>(log2_10), Runtime{10}, 200000));
  TEST_ASSERT(NearRaw(fixed_math::exp2_to<Runtime>(log2_30), Runtime{30}, 2000000));
  TEST_ASSERT(NearRaw(fixed_math::exp2_to<Runtime>(log2_60), Runtime{60}, 4000000));
}

void test_RoundTrip() {
  const Runtime values[] = {Runtime{1}, Runtime{2}, Runtime{3}, Runtime{5},
                            Runtime{10}, Runtime{30}, Runtime{60}};
  const std::int64_t tolerances[] = {2, 2, 150000, 100000, 200000, 2500000,
                                     5000000};
  for (std::size_t i = 0; i < sizeof(values) / sizeof(values[0]); ++i) {
    const Runtime x = values[i];
    const Log lx = fixed_math::log2_to<Log>(x);
    const Runtime y = fixed_math::exp2_to<Runtime>(lx);
    TEST_ASSERT(NearRaw(y, x, tolerances[i]));
  }
}

void test_SmallRuntimeRoundTrip() {
  const Log lx = fixed_math::log2_to<Log>(SmallRuntime{10});
  const SmallRuntime y = fixed_math::exp2_to<SmallRuntime>(lx);
  TEST_ASSERT(NearRaw(y, SmallRuntime{10}, 32));
}

void test_FromClampedRawClampsLargeValue() {
  using Target = FixedPoint<std::uint16_t, 60.0>;
  constexpr auto clamped =
      fixed_math::internal::from_clamped_raw<Target>(std::int64_t{1} << 40);
  TEST_ASSERT_EQUAL(static_cast<std::int64_t>(Target::kRawMax),
                    static_cast<std::int64_t>(clamped.raw_value()));
}

}  // namespace ae::test_fixed_math

int test_fixed_math() {
  UNITY_BEGIN();
  RUN_TEST(ae::test_fixed_math::test_Log2PowersOfTwo);
  RUN_TEST(ae::test_fixed_math::test_Exp2PowersOfTwo);
  RUN_TEST(ae::test_fixed_math::test_Log2FractionalValues);
  RUN_TEST(ae::test_fixed_math::test_Exp2FractionalValues);
  RUN_TEST(ae::test_fixed_math::test_RoundTrip);
  RUN_TEST(ae::test_fixed_math::test_SmallRuntimeRoundTrip);
  RUN_TEST(ae::test_fixed_math::test_FromClampedRawClampsLargeValue);
  return UNITY_END();
}
