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

#include <ae-numeric/exponential.h>
#include <ae-numeric/fixed_point.h>
#include <ae-numeric/text_io.h>
#include <ae-numeric/tiered_int.h>

namespace ae::test_text_io {

using T = TieredInt<std::uint8_t, 254>;
static_assert(numeric_traits<T>::kIsIntegerLike);

void test_TieredInt() {
  TEST_ASSERT_EQUAL_STRING("123", ToString(T{123}).c_str());

  T parsed{};
  TEST_ASSERT(FromString("123", parsed));
  TEST_ASSERT_EQUAL(123, parsed.value_);
  TEST_ASSERT_FALSE(FromString("152915291529152915299", parsed));
}

void test_FixedPointQ71() {
  using F100 = FixedPoint<std::uint8_t, 100.0>;

  TEST_ASSERT_EQUAL_STRING("10", ToString(F100::FromInteger(10)).c_str());
  TEST_ASSERT_EQUAL_STRING("0.5", ToString(F100::FromRaw(1)).c_str());
  TEST_ASSERT_EQUAL_STRING("127.5", ToString(F100::FromRaw(255)).c_str());

  F100 x = F100::FromInteger(0);
  TEST_ASSERT(FromString("10.5", x));
  TEST_ASSERT_EQUAL(F100::FromDouble(10.5).RawValue(), x.RawValue());
}

void test_FixedPointQ53() {
  using F30 = FixedPoint<std::uint8_t, 30.0>;

  TEST_ASSERT_EQUAL_STRING("0.125", ToString(F30::FromRaw(1)).c_str());
  TEST_ASSERT_EQUAL_STRING("10", ToString(F30::FromInteger(10)).c_str());

  F30 y = F30::FromInteger(0);
  TEST_ASSERT(FromString("0.125", y));
  TEST_ASSERT_EQUAL(1, y.RawValue());
}

void test_SignedFixedPoint() {
  using S100 = FixedPoint<std::int8_t, 100.0>;
  using SFrac = FixedPoint<std::int8_t, 63.0>;

  TEST_ASSERT_EQUAL_STRING("-10", ToString(S100::FromInteger(-10)).c_str());
  TEST_ASSERT_EQUAL_STRING("-0.5", ToString(SFrac::FromRaw(-1)).c_str());

  S100 s = S100::FromInteger(0);
  TEST_ASSERT(FromString("-10.5", s));
  TEST_ASSERT(s < S100::FromInteger(0));
}

void test_LargePositiveScale() {
  using Huge = FixedPoint<std::uint8_t, 1000000000.0>;

  const auto text = ToString(Huge::FromRaw(1));
  TEST_ASSERT(text.size() > 1);
}

void test_Exponential() {
  using Runtime = FixedPoint<std::uint32_t, 60.0>;
  using Wire = TieredInt<std::uint8_t, 249, 1529>;
  using E = Exponential<Runtime, Wire, 0.001, 60.0>;

  constexpr auto e = E::FromDouble(1.0);
  const auto text = ToString(e);
  TEST_ASSERT(!text.empty());

  E parsed{};
  TEST_ASSERT(FromString("1.0", parsed));
  TEST_ASSERT(parsed > E::FromDouble(0.9));
  TEST_ASSERT(parsed < E::FromDouble(1.1));
}

void test_RawFromRatioNearInt64Bounds() {
  std::uint8_t raw_u = 0;
  // 9223372036854775807 / 1 with scale 0 -> INT64_MAX, out of uint8 range.
  TEST_ASSERT_FALSE(text_io_internal::RawFromRatioInRange<std::uint8_t>(
      std::numeric_limits<std::int64_t>::max(), 1, 0, 0, 255, raw_u));

  std::int64_t raw64 = 0;
  TEST_ASSERT(text_io_internal::RawFromRatioInRange<std::int64_t>(
      std::numeric_limits<std::int64_t>::min(), 1, 0,
      std::numeric_limits<std::int64_t>::min(),
      std::numeric_limits<std::int64_t>::max(), raw64));
  TEST_ASSERT_EQUAL(std::numeric_limits<std::int64_t>::min(), raw64);

  // Positive scale: divide by large power of two near the edge.
  std::uint16_t raw16 = 0;
  TEST_ASSERT(text_io_internal::RawFromRatioInRange<std::uint16_t>(
      1000, 1, 3, 0, 65535, raw16));  // round(1000/8) = 125
  TEST_ASSERT_EQUAL(125, raw16);
}

void test_RawFromRatioLargeScaleExp() {
  std::uint32_t raw = 0;
  // scale_exp = -40: naive num<<40 overflows int64 for large num, but
  // round(3 * 2^40 / 2^40) = 3 still fits.
  TEST_ASSERT(text_io_internal::RawFromRatioInRange<std::uint32_t>(
      3, 1LL << 40, -40, 0, 1000, raw));
  TEST_ASSERT_EQUAL(3, raw);

  // scale_exp = +40: round(5 / 2^40) = 0
  TEST_ASSERT(text_io_internal::RawFromRatioInRange<std::uint32_t>(
      5, 1, 40, 0, 1000, raw));
  TEST_ASSERT_EQUAL(0, raw);

  // Half-up at large positive scale: round(1 / 2) with scale +1 => round(1/2)=1
  TEST_ASSERT(text_io_internal::RawFromRatioInRange<std::uint32_t>(
      1, 1, 1, 0, 1000, raw));
  TEST_ASSERT_EQUAL(1, raw);
}

void test_RawFromRatioNaiveMulOverflowButResultFits() {
  std::uint8_t raw = 0;
  // num close to INT64_MAX/2, shift 2: naive num*4 overflows signed 64-bit,
  // but round((INT64_MAX/2)*4 / (INT64_MAX/2)) = 4.
  constexpr std::int64_t kHalfMax =
      std::numeric_limits<std::int64_t>::max() / 2;
  TEST_ASSERT(text_io_internal::RawFromRatioInRange<std::uint8_t>(
      kHalfMax, kHalfMax, -2, 0, 255, raw));
  TEST_ASSERT_EQUAL(4, raw);

  // Same pattern for signed rep with negative value.
  std::int8_t sraw = 0;
  TEST_ASSERT(text_io_internal::RawFromRatioInRange<std::int8_t>(
      -kHalfMax, kHalfMax, -2, -128, 127, sraw));
  TEST_ASSERT_EQUAL(-4, sraw);
}

void test_RawFromRatioTrueOutOfRange() {
  std::uint8_t raw = 0;
  TEST_ASSERT_FALSE(text_io_internal::RawFromRatioInRange<std::uint8_t>(
      1000, 1, 0, 0, 255, raw));

  std::int8_t sraw = 0;
  TEST_ASSERT_FALSE(text_io_internal::RawFromRatioInRange<std::int8_t>(
      -1000, 1, 0, -50, 50, sraw));

  // Unsigned rejects negatives.
  TEST_ASSERT_FALSE(text_io_internal::RawFromRatioInRange<std::uint8_t>(
      -1, 1, 0, 0, 255, raw));
}

void test_RawFromRatioSignedUnsignedRep() {
  std::uint16_t u = 0;
  TEST_ASSERT(text_io_internal::RawFromRatioInRange<std::uint16_t>(
      3, 2, 0, 0, 65535, u));  // round(1.5) = 2
  TEST_ASSERT_EQUAL(2, u);

  std::int16_t s = 0;
  TEST_ASSERT(text_io_internal::RawFromRatioInRange<std::int16_t>(
      -3, 2, 0, -100, 100, s));  // round(-1.5) = -2
  TEST_ASSERT_EQUAL(-2, s);

  using F = FixedPoint<std::uint8_t, 100.0>;
  F parsed = F::FromInteger(0);
  TEST_ASSERT(FromString("10.25", parsed));
  TEST_ASSERT_EQUAL(F::FromDouble(10.5).RawValue(),
                    parsed.RawValue());  // Q7.1 step 0.5
}

}  // namespace ae::test_text_io

int test_text_io() {
  UNITY_BEGIN();
  RUN_TEST(ae::test_text_io::test_TieredInt);
  RUN_TEST(ae::test_text_io::test_FixedPointQ71);
  RUN_TEST(ae::test_text_io::test_FixedPointQ53);
  RUN_TEST(ae::test_text_io::test_SignedFixedPoint);
  RUN_TEST(ae::test_text_io::test_LargePositiveScale);
  RUN_TEST(ae::test_text_io::test_Exponential);
  RUN_TEST(ae::test_text_io::test_RawFromRatioNearInt64Bounds);
  RUN_TEST(ae::test_text_io::test_RawFromRatioLargeScaleExp);
  RUN_TEST(ae::test_text_io::test_RawFromRatioNaiveMulOverflowButResultFits);
  RUN_TEST(ae::test_text_io::test_RawFromRatioTrueOutOfRange);
  RUN_TEST(ae::test_text_io::test_RawFromRatioSignedUnsignedRep);
  return UNITY_END();
}
