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

using F = FixedPoint<std::uint8_t, 100.0>;

static_assert(F::kScaleExp == -1);
static_assert(F::kFractionBits == 1);

// 1. logical constructors from integers
static_assert(F{10}.raw_value() == 20);
static_assert(F{20}.raw_value() == 40);
static_assert(F::FromInteger(100).raw_value() == 200);

// 2. consteval floating constructors
static_assert(F{0.5}.raw_value() == 1);

// 3. raw constructor
static_assert(F::FromRaw(10).raw_value() == 10);
static_assert(F::Raw(10).raw_value() == 10);

// 4. declared Max bound checks (compile-time ok / fail)
static_assert(F{100}.raw_value() == 200);
static_assert(
    fixed_point_internal::LogicalBoundChecker<100.0, false, 100, 1>::kOk);

// 5. F{10} + F{20} -> FixedPoint<uint8_t, 200.0>
constexpr auto sum_same = F{10} + F{20};
static_assert(
    std::is_same_v<std::remove_cv_t<decltype(sum_same)>,
                   FixedPoint<std::uint8_t, 200.0>>);
static_assert(sum_same.raw_value() == 30);
static_assert(decltype(sum_same)::kScaleExp == 0);

// integer literal operations
constexpr auto sum_int = F{45} + 55;
static_assert(std::is_same_v<std::remove_cv_t<decltype(sum_int)>,
                             FixedPoint<std::uint8_t, 200.0>>);
static_assert(sum_int.raw_value() == 100);

// 6. A{10} + B{100} -> FixedPoint<uint16_t, 1030.0>
using A = FixedPoint<std::uint8_t, 30.0>;
using B = FixedPoint<std::uint16_t, 1000.0>;
constexpr auto mixed = A{10} + B{100};
static_assert(
    std::is_same_v<std::remove_cv_t<decltype(mixed)>,
                   FixedPoint<std::uint16_t, 1030.0>>);
static_assert((mixed.raw_value() >> (-decltype(mixed)::kScaleExp)) == 110);

// 7. multiplication result MaxA * MaxB
using MulA = FixedPoint<std::uint16_t, 1000.0>;
using MulB = FixedPoint<std::uint16_t, 0.001>;
constexpr auto product = MulA{1000} * MulB{0.001};
static_assert(
    std::is_same_v<std::remove_cv_t<decltype(product)>,
                   FixedPoint<std::uint16_t, 1.0>>);

// 8. mul_to<Target>
using MulTarget = FixedPoint<std::uint16_t, 1.0>;
constexpr auto mul_target =
    mul_to<MulTarget>(MulA{1000}, MulB{0.001});
static_assert(std::is_same_v<std::remove_cv_t<decltype(mul_target)>, MulTarget>);

// scale / cast / div
using F30 = FixedPoint<std::uint8_t, 30.0>;
using F100 = FixedPoint<std::uint8_t, 100.0>;
static_assert(F30::kScaleExp == -3);
static_assert(F100::kScaleExp == -1);

using F130 = FixedPoint<std::uint8_t, 130.0>;
static_assert(F130::kScaleExp == 0);

using S100 = FixedPoint<std::int8_t, 100.0>;
static_assert(S100::kRawMin == -127);
static_assert(S100::kRawMax == 127);

using S = FixedPoint<std::int8_t, 10.0>;
static_assert(S::FromRaw(std::numeric_limits<std::int8_t>::min()).raw_value() ==
              S::kRawMin);

constexpr auto b_as_a = F100::Cast<F30>(F100{10});
static_assert(b_as_a == F30{10});

using Dst = FixedPoint<std::uint8_t, 20.0>;
constexpr auto div = div_to<Dst>(F100{100}, F100{10});
static_assert(div == Dst{10});

using Raw = TieredInt<std::uint8_t, 254>;
using Compact = FixedPoint<Raw, 60.0>;
static_assert(numeric_traits<Compact>::kIsFixedPoint);
static_assert(Compact{60}.raw_value() <= Raw::kUpper);

using Small = FixedPoint<std::uint8_t, 0.0000000001>;
using Large = FixedPoint<std::uint8_t, 1000000000.0>;

constexpr auto s = Small::FromRaw(255);
constexpr auto l = Small::Cast<Large>(s);
static_assert(l.raw_value() >= 0);

constexpr auto cast_back = Large::Cast<Small>(l);
static_assert(cast_back.raw_value() <= Small::kRawMax);

// PromoteRep same-rep
static_assert(std::is_same_v<PromoteRep<std::uint8_t, std::uint8_t>::type,
                             std::uint8_t>);

void test_LogicalConstructors() {
  TEST_ASSERT_EQUAL_UINT8(20, F{10}.raw_value());
  TEST_ASSERT_EQUAL_UINT8(40, F{20}.raw_value());
  TEST_ASSERT_EQUAL_UINT8(10, F::FromRaw(10).raw_value());
}

void test_AddSameRep() {
  const auto r = F{10} + F{20};
  TEST_ASSERT_EQUAL_UINT8(30, r.raw_value());
  TEST_ASSERT((std::is_same_v<std::remove_cv_t<decltype(r)>,
                               FixedPoint<std::uint8_t, 200.0>>));
}

void test_AddMixedRep() {
  const auto r = A{10} + B{100};
  TEST_ASSERT((std::is_same_v<std::remove_cv_t<decltype(r)>,
                               FixedPoint<std::uint16_t, 1030.0>>));
  TEST_ASSERT_EQUAL_UINT16(110, r.raw_value() >> (-decltype(r)::kScaleExp));
}

void test_Multiply() {
  const auto r = MulA{1000} * MulB{0.001};
  TEST_ASSERT(
      (std::is_same_v<std::remove_cv_t<decltype(r)>, FixedPoint<std::uint16_t, 1.0>>));
}

void test_MulTo() {
  const auto r = mul_to<MulTarget>(MulA{1000}, MulB{0.001});
  TEST_ASSERT((std::is_same_v<std::remove_cv_t<decltype(r)>, MulTarget>));
}

void test_IntegerAdd() {
  const auto ok = F{45} + 55;
  TEST_ASSERT_EQUAL_UINT8(100, ok.raw_value());
}

void test_RuntimeRoundTrip() {
  const auto sum = F30{10} + F100{20};
  TEST_ASSERT((sum == FixedPoint<std::uint8_t, 130.0>{30}));

  const auto cast = F100::Cast<F30>(F100{10});
  TEST_ASSERT(cast == F30{10});

  const auto quotient = div_to<Dst>(F100{100}, F100{10});
  TEST_ASSERT(quotient == Dst{10});
}

// 9. floating construction is consteval-only (verified by consteval ctor signature)
static_assert(F{0.5}.raw_value() == 1);

// Explicit runtime conversion APIs (clamp / checked), usable with non-constants.
static_assert(F::FromRuntimeInteger(45).raw_value() == 90);
static_assert(F::FromRuntimeInteger(123).raw_value() == 200);  // clamps to Max
static_assert(F::Saturating(123).raw_value() == 200);          // clamps to Max
static_assert(F::TryFromRuntimeInteger(45).has_value());
static_assert(F::TryFromRuntimeInteger(45)->raw_value() == 90);
static_assert(!F::TryFromRuntimeInteger(123).has_value());     // checked failure

void test_RuntimeIntegerApis() {
  std::int64_t runtime_in_range = 45;
  std::int64_t runtime_out_of_range = 123;

  TEST_ASSERT_EQUAL_UINT8(90,
                          F::FromRuntimeInteger(runtime_in_range).raw_value());
  TEST_ASSERT_EQUAL_UINT8(
      200, F::FromRuntimeInteger(runtime_out_of_range).raw_value());
  TEST_ASSERT_EQUAL_UINT8(200, F::Saturating(runtime_out_of_range).raw_value());

  const auto ok = F::TryFromRuntimeInteger(runtime_in_range);
  TEST_ASSERT(ok.has_value());
  TEST_ASSERT_EQUAL_UINT8(90, ok->raw_value());

  const auto bad = F::TryFromRuntimeInteger(runtime_out_of_range);
  TEST_ASSERT_FALSE(bad.has_value());
}

}  // namespace ae::test_fixed_point

int test_fixed_point() {
  UNITY_BEGIN();
  RUN_TEST(ae::test_fixed_point::test_LogicalConstructors);
  RUN_TEST(ae::test_fixed_point::test_AddSameRep);
  RUN_TEST(ae::test_fixed_point::test_AddMixedRep);
  RUN_TEST(ae::test_fixed_point::test_Multiply);
  RUN_TEST(ae::test_fixed_point::test_MulTo);
  RUN_TEST(ae::test_fixed_point::test_IntegerAdd);
  RUN_TEST(ae::test_fixed_point::test_RuntimeRoundTrip);
  RUN_TEST(ae::test_fixed_point::test_RuntimeIntegerApis);
  return UNITY_END();
}
