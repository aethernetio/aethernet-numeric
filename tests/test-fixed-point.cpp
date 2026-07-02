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
#include <limits>
#include <type_traits>

#include <ae-numeric/fixed_point.h>
#include <ae-numeric/tiered_int.h>

namespace ae::test_fixed_point {

using F = FixedPoint<std::uint8_t, 100.0>;

static_assert(F::kScaleExp == -1);
static_assert(F::kFractionBits == 1);

// 1. logical constructors from integers
static_assert(F{10}.RawValue() == 20);
static_assert(F{20}.RawValue() == 40);
static_assert(F::FromInteger(100).RawValue() == 200);

// 2. consteval floating constructors
static_assert(F{0.5}.RawValue() == 1);

// 3. raw constructor
static_assert(F::FromRaw(10).RawValue() == 10);
static_assert(F::Raw(10).RawValue() == 10);

// 4. declared Max bound checks (compile-time ok / fail)
static_assert(F{100}.RawValue() == 200);
static_assert(
    fixed_point_internal::LogicalBoundChecker<100.0, false, 100, 1>::kOk);

// 5. F{10} + F{20} -> FixedPoint<uint8_t, 200.0>
constexpr auto sum_same = F{10} + F{20};
static_assert(std::is_same_v<std::remove_cv_t<decltype(sum_same)>,
                             FixedPoint<std::uint8_t, 200.0>>);
static_assert(sum_same.RawValue() == 30);
static_assert(decltype(sum_same)::kScaleExp == 0);

// integer literal operations
constexpr auto sum_int = F{45} + 55;
static_assert(std::is_same_v<std::remove_cv_t<decltype(sum_int)>,
                             FixedPoint<std::uint8_t, 200.0>>);
static_assert(sum_int.RawValue() == 100);

// 6. A{10} + B{100} -> FixedPoint<uint16_t, 1030.0>
using A = FixedPoint<std::uint8_t, 30.0>;
using B = FixedPoint<std::uint16_t, 1000.0>;
constexpr auto mixed = A{10} + B{100};
static_assert(std::is_same_v<std::remove_cv_t<decltype(mixed)>,
                             FixedPoint<std::uint16_t, 1030.0>>);
static_assert((mixed.RawValue() >> (-decltype(mixed)::kScaleExp)) == 110);

// 7. multiplication result MaxA * MaxB
using MulA = FixedPoint<std::uint16_t, 1000.0>;
using MulB = FixedPoint<std::uint16_t, 0.001>;
constexpr auto product = MulA{1000} * MulB{0.001};
static_assert(std::is_same_v<std::remove_cv_t<decltype(product)>,
                             FixedPoint<std::uint16_t, 1.0>>);

// 8. MulTo<Target>
using MulTarget = FixedPoint<std::uint16_t, 1.0>;
constexpr auto mul_target = MulTo<MulTarget>(MulA{1000}, MulB{0.001});
static_assert(
    std::is_same_v<std::remove_cv_t<decltype(mul_target)>, MulTarget>);

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
static_assert(S::FromRaw(std::numeric_limits<std::int8_t>::min()).RawValue() ==
              S::kRawMin);

constexpr auto b_as_a = F100::Cast<F30>(F100{10});
static_assert(b_as_a == F30{10});

using Dst = FixedPoint<std::uint8_t, 20.0>;
constexpr auto div = DivTo<Dst>(F100{100}, F100{10});
static_assert(div == Dst{10});

using Raw = TieredInt<std::uint8_t, 254>;
using Compact = FixedPoint<Raw, 60.0>;
static_assert(numeric_traits<Compact>::kIsFixedPoint);
static_assert(Compact{60}.RawValue() <= Raw::kUpper);

using Small = FixedPoint<std::uint8_t, 0.0000000001>;
using Large = FixedPoint<std::uint8_t, 1000000000.0>;

constexpr auto s = Small::FromRaw(255);
constexpr auto l = Small::Cast<Large>(s);
static_assert(l.RawValue() == Large::kRawMin);

constexpr auto cast_back = Large::Cast<Small>(l);
static_assert(cast_back.RawValue() == Small::kRawMin);

// PromoteRep same-rep
static_assert(
    std::is_same_v<PromoteRep<std::uint8_t, std::uint8_t>::type, std::uint8_t>);
static_assert(std::is_same_v<PromoteRep<std::uint8_t, std::uint16_t>::type,
                             std::uint16_t>);
static_assert(
    std::is_same_v<PromoteRep<std::uint8_t, std::int8_t>::type, std::int16_t>);
static_assert(std::is_same_v<PromoteRep<std::uint16_t, std::int16_t>::type,
                             std::int32_t>);
static_assert(
    std::is_same_v<PromoteRep<std::uint8_t, std::int16_t>::type, std::int16_t>);

void test_LogicalConstructors() {
  TEST_ASSERT_EQUAL_UINT8(20, F{10}.RawValue());
  TEST_ASSERT_EQUAL_UINT8(40, F{20}.RawValue());
  TEST_ASSERT_EQUAL_UINT8(10, F::FromRaw(10).RawValue());
}

void test_AddSameRep() {
  const auto r = F{10} + F{20};
  TEST_ASSERT_EQUAL_UINT8(30, r.RawValue());
  TEST_ASSERT((std::is_same_v<std::remove_cv_t<decltype(r)>,
                              FixedPoint<std::uint8_t, 200.0>>));
}

void test_AddMixedRep() {
  const auto r = A{10} + B{100};
  TEST_ASSERT((std::is_same_v<std::remove_cv_t<decltype(r)>,
                              FixedPoint<std::uint16_t, 1030.0>>));
  TEST_ASSERT_EQUAL_UINT16(110, r.RawValue() >> (-decltype(r)::kScaleExp));
}

void test_Multiply() {
  const auto r = MulA{1000} * MulB{0.001};
  TEST_ASSERT((std::is_same_v<std::remove_cv_t<decltype(r)>,
                              FixedPoint<std::uint16_t, 1.0>>));
}

void test_MulTo() {
  const auto r = MulTo<MulTarget>(MulA{1000}, MulB{0.001});
  TEST_ASSERT((std::is_same_v<std::remove_cv_t<decltype(r)>, MulTarget>));
}

void test_IntegerAdd() {
  const auto ok = F{45} + 55;
  TEST_ASSERT_EQUAL_UINT8(100, ok.RawValue());
}

void test_RuntimeRoundTrip() {
  const auto sum = F30{10} + F100{20};
  TEST_ASSERT((sum == FixedPoint<std::uint8_t, 130.0>{30}));

  const auto cast = F100::Cast<F30>(F100{10});
  TEST_ASSERT(cast == F30{10});

  const auto quotient = DivTo<Dst>(F100{100}, F100{10});
  TEST_ASSERT(quotient == Dst{10});
}

// 9. floating construction is consteval-only (verified by consteval ctor
// signature)
static_assert(F{0.5}.RawValue() == 1);

// Explicit runtime conversion APIs (clamp / checked), usable with
// non-constants.
static_assert(F::FromRuntimeInteger(45).RawValue() == 90);
static_assert(F::FromRuntimeInteger(123).RawValue() == 200);  // clamps to Max
static_assert(F::Saturating(123).RawValue() == 200);          // clamps to Max
static_assert(F::TryFromRuntimeInteger(45).has_value());
static_assert(F::TryFromRuntimeInteger(45)->RawValue() == 90);
static_assert(!F::TryFromRuntimeInteger(123).has_value());  // checked failure

void test_RuntimeIntegerApis() {
  std::int64_t runtime_in_range = 45;
  std::int64_t runtime_out_of_range = 123;

  TEST_ASSERT_EQUAL_UINT8(90,
                          F::FromRuntimeInteger(runtime_in_range).RawValue());
  TEST_ASSERT_EQUAL_UINT8(
      200, F::FromRuntimeInteger(runtime_out_of_range).RawValue());
  TEST_ASSERT_EQUAL_UINT8(200, F::Saturating(runtime_out_of_range).RawValue());

  const auto ok = F::TryFromRuntimeInteger(runtime_in_range);
  TEST_ASSERT(ok.has_value());
  TEST_ASSERT_EQUAL_UINT8(90, ok->RawValue());

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
