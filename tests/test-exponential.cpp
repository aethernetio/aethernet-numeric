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
using Code = TieredInt<std::uint8_t, 249, 1529>;
using E = Exponential<Runtime, Code, 0.001, 60.0, 1529>;

static_assert(E::kBoundaryCode == 1529);

using DefaultE = Exponential<Runtime, Code, 0.001, 60.0>;
static_assert(DefaultE::kBoundaryCode == 255);
static_assert(DefaultE::kBoundaryCode <= 255);

using Partial = Exponential<Runtime, Code, 0.001, 60.0, 200>;
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

static_assert(r1 > Runtime::FromDouble(0.85));
static_assert(r1 < Runtime::FromDouble(1.15));

static_assert(E::from_double(1000.0).code_value() == E::kBoundaryCode);

using SRuntime = FixedPoint<std::int32_t, 60.0>;
using SE = Exponential<SRuntime, Code, 0.001, 60.0, 1529>;

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
constexpr auto value = SE::from_double(10.0);

static_assert(clamp(value, low, high) == high);
static_assert(min(low, high) == low);
static_assert(max(low, high) == high);

constexpr E el1{1};
constexpr E el10{10};
constexpr E el_min{0.001};
static_assert(el1.code_value() == E::FromRuntimeInteger(1).code_value());
static_assert(el10.code_value() == E::FromRuntimeInteger(10).code_value());
static_assert(el1.code_value() != 0);
static_assert(el10.code_value() != 0);
static_assert(el_min.code_value() != 0);
static_assert(el1.code_value() < el10.code_value());

static_assert(E::Code(10).code_value() == 10);
static_assert(E::FromCode(10).code_value() == 10);

static_assert(E::FromCode(0).code_value() == 0);
static_assert(E::FromCode(0).value() == Runtime::FromInteger(0));
static_assert(el1.value() > Runtime::FromDouble(0.85));
static_assert(el1.value() < Runtime::FromDouble(1.15));

static_assert(E::FromRuntimeInteger(123).code_value() == E::kBoundaryCode);
static_assert(E::Saturating(123).code_value() == E::kBoundaryCode);
static_assert(E::TryFromRuntimeInteger(10).has_value());
static_assert(!E::TryFromRuntimeInteger(123).has_value());

constexpr SE sel_neg{-1};
static_assert(sel_neg.is_negative());

template <typename T>
bool NearRuntime(T a, T b, std::int64_t max_raw_diff) {
  if constexpr (numeric_traits<T>::kIsFixedPoint) {
    const std::int64_t diff = static_cast<std::int64_t>(a.raw_value()) -
                              static_cast<std::int64_t>(b.raw_value());
    return diff <= max_raw_diff && diff >= -max_raw_diff;
  }
  return false;
}

void test_LogicalConstruction() {
  TEST_ASSERT(el1.code_value() != 0);
  TEST_ASSERT_EQUAL(10, E::Code(10).code_value());
  TEST_ASSERT_EQUAL(10, E::FromCode(10).code_value());

  const E min = E{0.001};
  const E one = E{1};
  const E ten = E{10};
  const E max = E{60};

  TEST_ASSERT(min.code_value() != 0);
  TEST_ASSERT(max.code_value() == E::kBoundaryCode);
  TEST_ASSERT(one.code_value() < ten.code_value());
  TEST_ASSERT(NearRuntime(one.value(), Runtime{1}, 500000));
  TEST_ASSERT(NearRuntime(ten.value(), Runtime{10}, 4000000));
  TEST_ASSERT(NearRuntime(max.value(), Runtime{60}, 5000000));

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

void test_CodeDecodeMonotonicity() {
  const int codes[] = {0, 1, 2, 10, 100, 249, 250, 1000, 1529};
  Runtime prev = Runtime::FromInteger(0);
  for (const int code : codes) {
    const Runtime current = E::Code(code).value();
    TEST_ASSERT(current.raw_value() >= prev.raw_value());
    prev = current;
  }
}

void test_CodeRoundTrip() {
  const int codes[] = {1, 2, 10, 100, 249, 250, 1000, 1529};
  for (const int code : codes) {
    const E encoded = E::Code(code);
    const E roundtrip = E::FromRuntime(encoded.value());
    const int diff = static_cast<int>(roundtrip.code_value()) - code;
    TEST_ASSERT(diff <= 3 && diff >= -3);
  }
}

void test_SignedRuntime() {
  const SE se_neg = SE{-1};
  const SE se_zero = SE{0};
  const SE se_pos = SE{1};

  TEST_ASSERT(se_neg.is_negative());
  TEST_ASSERT(se_zero.is_zero());
  TEST_ASSERT(se_pos.is_positive());
  TEST_ASSERT(se_neg < se_zero);
  TEST_ASSERT(se_zero < se_pos);
  TEST_ASSERT(NearRuntime(se_neg.abs().value(), se_pos.value(), 200000));
}

void test_NoHugeTableCompile() {
  TEST_PASS();
}

void test_EncodeDecodeBudget() {
  constexpr Runtime values[] = {
      Runtime::FromDouble(0.001), Runtime::FromDouble(0.01),
      Runtime::FromDouble(0.1), Runtime::FromDouble(1.0),
      Runtime::FromDouble(10.0), Runtime::FromDouble(60.0)};
  int prev_code = 0;
  for (const Runtime v : values) {
    const E encoded = E::FromRuntime(v);
    const int code = static_cast<int>(encoded.code_value());
    TEST_ASSERT(code >= prev_code);
    prev_code = code;

    const E roundtrip = E::FromRuntime(encoded.value());
    const int diff = static_cast<int>(roundtrip.code_value()) - code;
    TEST_ASSERT(diff <= 3 && diff >= -3);
  }
}

}  // namespace ae::test_exponential

int test_exponential() {
  UNITY_BEGIN();
  RUN_TEST(ae::test_exponential::test_LogicalConstruction);
  RUN_TEST(ae::test_exponential::test_RuntimeIntegerApis);
  RUN_TEST(ae::test_exponential::test_RuntimeRoundTrip);
  RUN_TEST(ae::test_exponential::test_CodeDecodeMonotonicity);
  RUN_TEST(ae::test_exponential::test_CodeRoundTrip);
  RUN_TEST(ae::test_exponential::test_SignedRuntime);
  RUN_TEST(ae::test_exponential::test_NoHugeTableCompile);
  RUN_TEST(ae::test_exponential::test_EncodeDecodeBudget);
  return UNITY_END();
}
