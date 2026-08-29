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

#include <ae-numeric/integer_math.h>
#include <ae-numeric/text_io.h>

namespace ae::test_integer_math {

using namespace integer_math;

// Call through function pointers so GCC cannot constexpr-fold bodies away
// from gcov instrumentation of integer_math.h.
struct RuntimeFns {
  std::uint64_t (*abs_i64)(std::int64_t) = &AbsI64ToU64;
  std::uint64_t (*gcd)(std::uint64_t, std::uint64_t) = &GcdU64;
  bool (*mul)(std::uint64_t, std::uint64_t, std::uint64_t&) = &MulU64Checked;
  bool (*add)(std::uint64_t, std::uint64_t, std::uint64_t&) = &AddU64Checked;
  bool (*round_div)(std::uint64_t, std::uint64_t, std::uint64_t&) =
      &RoundDivU64;
  bool (*muldiv)(std::uint64_t, std::uint64_t, std::uint64_t, std::uint64_t&) =
      &MulDivU64Nearest;
  bool (*mul_pow2)(std::uint64_t, std::uint64_t, unsigned, std::uint64_t&) =
      &RoundMulPow2DivU64;
  bool (*div_pow2)(std::uint64_t, std::uint64_t, unsigned, std::uint64_t&) =
      &RoundDivPow2U64;
  std::uint64_t (*sqrt_u64)(std::uint64_t) = &SqrtU64;
};

RuntimeFns const& Fns() {
  static RuntimeFns const fns{};
  return fns;
}

void test_AbsI64ToU64() {
  auto const& f = Fns();
  TEST_ASSERT_EQUAL_UINT64(0U, f.abs_i64(0));
  TEST_ASSERT_EQUAL_UINT64(1U, f.abs_i64(1));
  TEST_ASSERT_EQUAL_UINT64(1U, f.abs_i64(-1));
  TEST_ASSERT_EQUAL_UINT64(
      static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()) + 1U,
      f.abs_i64(std::numeric_limits<std::int64_t>::min()));
}

void test_GcdU64() {
  auto const& f = Fns();
  TEST_ASSERT_EQUAL_UINT64(6U, f.gcd(54, 24));
  TEST_ASSERT_EQUAL_UINT64(1U, f.gcd(17, 13));
  TEST_ASSERT_EQUAL_UINT64(8U, f.gcd(0, 8));
  TEST_ASSERT_EQUAL_UINT64(8U, f.gcd(8, 0));
}

void test_MulU64Checked() {
  auto const& f = Fns();
  std::uint64_t out = 1;
  TEST_ASSERT(f.mul(0, 123, out));
  TEST_ASSERT_EQUAL_UINT64(0U, out);
  TEST_ASSERT(f.mul(6, 7, out));
  TEST_ASSERT_EQUAL_UINT64(42U, out);
  TEST_ASSERT_FALSE(
      f.mul(std::numeric_limits<std::uint64_t>::max(), 2, out));
  TEST_ASSERT(f.mul((std::uint64_t{1} << 32) - 1U,
                    (std::uint64_t{1} << 32) - 1U, out));
}

void test_AddU64Checked() {
  auto const& f = Fns();
  std::uint64_t out = 0;
  TEST_ASSERT(f.add(1, 2, out));
  TEST_ASSERT_EQUAL_UINT64(3U, out);
  TEST_ASSERT_FALSE(
      f.add(std::numeric_limits<std::uint64_t>::max(), 1, out));
}

void test_RoundDivU64() {
  auto const& f = Fns();
  std::uint64_t out = 0;
  TEST_ASSERT_FALSE(f.round_div(1, 0, out));
  TEST_ASSERT(f.round_div(0, 5, out));
  TEST_ASSERT_EQUAL_UINT64(0U, out);
  TEST_ASSERT(f.round_div(10, 5, out));
  TEST_ASSERT_EQUAL_UINT64(2U, out);
  TEST_ASSERT(f.round_div(10, 6, out));
  TEST_ASSERT_EQUAL_UINT64(2U, out);
  TEST_ASSERT(f.round_div(9, 6, out));
  TEST_ASSERT_EQUAL_UINT64(2U, out);
  TEST_ASSERT(f.round_div(8, 6, out));
  TEST_ASSERT_EQUAL_UINT64(1U, out);
  TEST_ASSERT(f.round_div(1, 2, out));
  TEST_ASSERT_EQUAL_UINT64(1U, out);
}

void test_MulDivU64NearestBasics() {
  auto const& f = Fns();
  std::uint64_t out = 0;
  TEST_ASSERT_FALSE(f.muldiv(1, 1, 0, out));
  TEST_ASSERT(f.muldiv(0, 99, 5, out));
  TEST_ASSERT_EQUAL_UINT64(0U, out);
  TEST_ASSERT(f.muldiv(7, 1, 1, out));
  TEST_ASSERT_EQUAL_UINT64(7U, out);
  TEST_ASSERT(f.muldiv(10, 3, 2, out));
  TEST_ASSERT_EQUAL_UINT64(15U, out);
  TEST_ASSERT(f.muldiv(5, 3, 2, out));
  TEST_ASSERT_EQUAL_UINT64(8U, out);
  TEST_ASSERT(f.muldiv(4, 3, 2, out));
  TEST_ASSERT_EQUAL_UINT64(6U, out);
  TEST_ASSERT(f.muldiv(3, 3, 2, out));
  TEST_ASSERT_EQUAL_UINT64(5U, out);
  TEST_ASSERT(f.muldiv(100, 1, 7, out));
  TEST_ASSERT_EQUAL_UINT64(14U, out);
}

void test_MulDivU64NearestOverflowPaths() {
  auto const& f = Fns();
  std::uint64_t out = 0;
  // a*f overflows uint64, but (a*f)/d fits via the q/r split.
  TEST_ASSERT(f.muldiv(std::uint64_t{1} << 32, std::uint64_t{1} << 32,
                       std::uint64_t{1} << 32, out));
  TEST_ASSERT_EQUAL_UINT64(std::uint64_t{1} << 32, out);

  // r*f overflows: forces the f = f_q*d + f_r fallback path.
  std::uint64_t const d = std::uint64_t{1} << 32;
  std::uint64_t const a = d - 1U;
  std::uint64_t const factor = std::uint64_t{1} << 33;
  TEST_ASSERT(f.muldiv(a, factor, d, out));
  TEST_ASSERT_EQUAL_UINT64((std::uint64_t{1} << 33) - 2U, out);

  // Non-zero f_r in the overflow fallback.
  std::uint64_t const factor2 = (std::uint64_t{1} << 33) + 7U;
  TEST_ASSERT(f.muldiv(a, factor2, d, out));
  TEST_ASSERT_EQUAL_UINT64(8589934597ULL, out);  // round(((2^32-1)*(2^33+7))/2^32)

  // Commute a/f on the overflow case.
  std::uint64_t out2 = 0;
  TEST_ASSERT(f.muldiv(factor, a, d, out2));
  TEST_ASSERT_EQUAL_UINT64((std::uint64_t{1} << 33) - 2U, out2);

  // Final quotient does not fit in uint64.
  TEST_ASSERT_FALSE(f.muldiv(std::numeric_limits<std::uint64_t>::max(), 2, 1,
                             out));
}

void test_MulDivU64NearestExhaustiveSmall() {
  auto const& f = Fns();
  for (std::uint64_t a = 0; a <= 20; ++a) {
    for (std::uint64_t factor = 0; factor <= 20; ++factor) {
      for (std::uint64_t d = 1; d <= 20; ++d) {
        std::uint64_t out = 0;
        TEST_ASSERT(f.muldiv(a, factor, d, out));
        std::uint64_t const expected = (a * factor + d / 2U) / d;
        TEST_ASSERT_EQUAL_UINT64(expected, out);
        std::uint64_t out2 = 0;
        TEST_ASSERT(f.muldiv(factor, a, d, out2));
        TEST_ASSERT_EQUAL_UINT64(out, out2);
      }
    }
  }
}

void test_RoundMulPow2DivU64() {
  auto const& f = Fns();
  std::uint64_t out = 0;
  TEST_ASSERT_FALSE(f.mul_pow2(1, 0, 3, out));
  TEST_ASSERT(f.mul_pow2(0, 5, 10, out));
  TEST_ASSERT_EQUAL_UINT64(0U, out);
  TEST_ASSERT(f.mul_pow2(3, 1, 0, out));
  TEST_ASSERT_EQUAL_UINT64(3U, out);
  TEST_ASSERT(f.mul_pow2(3, 2, 1, out));
  TEST_ASSERT_EQUAL_UINT64(3U, out);
  TEST_ASSERT(f.mul_pow2(6, 8, 3, out));
  TEST_ASSERT_EQUAL_UINT64(6U, out);
  TEST_ASSERT(f.mul_pow2(3, (std::uint64_t{1} << 40), 40, out));
  TEST_ASSERT_EQUAL_UINT64(3U, out);
  // den with many powers of two reduced against shift
  TEST_ASSERT(f.mul_pow2(5, 16, 5, out));  // 5*32/16 = 10
  TEST_ASSERT_EQUAL_UINT64(10U, out);
}

void test_RoundMulPow2DivU64HighRemBranch() {
  auto const& f = Fns();
  std::uint64_t out = 0;
  std::uint64_t const b = (std::uint64_t{1} << 63) + 5U;
  std::uint64_t const a = b - 1U;
  TEST_ASSERT(f.mul_pow2(a, b, 1, out));
  TEST_ASSERT_EQUAL_UINT64(2U, out);

  // sum < rem_hi carry bump: rem close to UINT64_MAX, lo large.
  std::uint64_t const b2 = (std::uint64_t{1} << 63) + 1U;
  std::uint64_t const a2 = b2 - 1U;  // rem = 2^63
  TEST_ASSERT(f.mul_pow2(a2, b2, 1, out));
}

void test_RoundDivPow2U64() {
  auto const& f = Fns();
  std::uint64_t out = 0;
  TEST_ASSERT_FALSE(f.div_pow2(1, 0, 1, out));
  TEST_ASSERT(f.div_pow2(0, 9, 5, out));
  TEST_ASSERT_EQUAL_UINT64(0U, out);
  TEST_ASSERT(f.div_pow2(16, 1, 2, out));
  TEST_ASSERT_EQUAL_UINT64(4U, out);
  TEST_ASSERT(f.div_pow2(1, 1, 1, out));
  TEST_ASSERT_EQUAL_UINT64(1U, out);
  TEST_ASSERT(f.div_pow2(1, 1, 2, out));
  TEST_ASSERT_EQUAL_UINT64(0U, out);

  std::uint64_t const big_b =
      (std::numeric_limits<std::uint64_t>::max() >> 2) + 3U;
  TEST_ASSERT(f.div_pow2(big_b, big_b, 3, out));
  TEST_ASSERT_EQUAL_UINT64(0U, out);

  TEST_ASSERT(f.div_pow2(1, 1, 64, out));
  TEST_ASSERT_EQUAL_UINT64(0U, out);
  TEST_ASSERT(f.div_pow2(1, 1, 65, out));
  TEST_ASSERT_EQUAL_UINT64(0U, out);
  TEST_ASSERT(f.div_pow2(std::numeric_limits<std::uint64_t>::max(), 1, 64,
                         out));
  TEST_ASSERT_EQUAL_UINT64(1U, out);

  // shift==64, odd numerator below half-threshold -> 0.
  TEST_ASSERT(f.div_pow2(3, 1, 64, out));
  TEST_ASSERT_EQUAL_UINT64(0U, out);

  // shift==64 and b too large for threshold path.
  TEST_ASSERT(f.div_pow2(1, 2, 64, out));
  TEST_ASSERT_EQUAL_UINT64(0U, out);

  // Wide-den path: b << shift does not fit in uint64.
  std::uint64_t const wide_b =
      (std::numeric_limits<std::uint64_t>::max() >> 3) + 1U;
  TEST_ASSERT(f.div_pow2(1, wide_b, 3, out));
  TEST_ASSERT_EQUAL_UINT64(0U, out);
  TEST_ASSERT(f.div_pow2(std::numeric_limits<std::uint64_t>::max(), wide_b, 3,
                         out));
  TEST_ASSERT_EQUAL_UINT64(1U, out);

  // Wide-den exact-half remainder (r == b - r) rounds up.
  // Keep a odd so gcd does not strip factors of two from an even b.
  std::uint64_t const wide_even =
      2U * ((std::numeric_limits<std::uint64_t>::max() >> 5) + 1U);
  TEST_ASSERT(wide_even > (std::numeric_limits<std::uint64_t>::max() >> 4));
  std::uint64_t const half_r = wide_even / 2U;
  std::uint64_t const a_half = (half_r << 4) + 1U;
  TEST_ASSERT(f.div_pow2(a_half, wide_even, 4, out));
  TEST_ASSERT_EQUAL_UINT64(1U, out);

  // Wide-den remainder below half -> no round up.
  TEST_ASSERT(f.div_pow2((1U << 4) + 1U, wide_even, 4, out));
  TEST_ASSERT_EQUAL_UINT64(0U, out);

  // Wide-den: r small but need fits; fraction decides rounding.
  std::uint64_t const wide_odd =
      (std::numeric_limits<std::uint64_t>::max() >> 4) + 1U;
  std::uint64_t const a_frac_up = (1U << 4) + ((1U << 4) - 1U);  // a_hi=1, frac=all1s
  TEST_ASSERT(f.div_pow2(a_frac_up, wide_odd, 4, out));
}

void test_RoundMulPow2Overflow() {
  auto const& f = Fns();
  std::uint64_t out = 0;
  // q doubles past uint64: start with q near max.
  TEST_ASSERT_FALSE(f.mul_pow2(std::numeric_limits<std::uint64_t>::max(), 1, 1,
                               out));
}

void test_MulDivTerm2Overflow() {
  auto const& f = Fns();
  std::uint64_t out = 0;
  // d > 2^32 so r*f_r can overflow in the r*f overflow fallback.
  std::uint64_t const d = (std::uint64_t{1} << 32) + 1U;
  std::uint64_t const a = d - 1U;
  std::uint64_t const factor = d - 1U;  // f_q=0, f_r=d-1, r*f_r overflows
  TEST_ASSERT_FALSE(f.muldiv(a, factor, d, out));
}

void test_RoundMulPow2DivExhaustiveSmall() {
  auto const& f = Fns();
  for (std::uint64_t a = 0; a <= 12; ++a) {
    for (std::uint64_t b = 1; b <= 12; ++b) {
      for (unsigned shift = 0; shift <= 6; ++shift) {
        std::uint64_t out = 0;
        TEST_ASSERT(f.mul_pow2(a, b, shift, out));
        std::uint64_t const expected = ((a << shift) + b / 2U) / b;
        TEST_ASSERT_EQUAL_UINT64(expected, out);
      }
    }
  }
}

void test_RoundDivPow2ExhaustiveSmall() {
  auto const& f = Fns();
  for (std::uint64_t a = 0; a <= 12; ++a) {
    for (std::uint64_t b = 1; b <= 12; ++b) {
      for (unsigned shift = 0; shift <= 4; ++shift) {
        std::uint64_t out = 0;
        TEST_ASSERT(f.div_pow2(a, b, shift, out));
        std::uint64_t const den = b << shift;
        std::uint64_t const expected = (a + den / 2U) / den;
        TEST_ASSERT_EQUAL_UINT64(expected, out);
      }
    }
  }
}

void test_RawFromRatioZeroAndSigns() {
  std::uint8_t u = 0;
  std::int8_t s = 0;

  TEST_ASSERT(text_io_internal::RawFromRatioInRange<std::uint8_t>(0, 5, 0, 0,
                                                                  255, u));
  TEST_ASSERT_EQUAL(0, u);

  TEST_ASSERT(text_io_internal::RawFromRatioInRange<std::int8_t>(9, 2, 0, -50,
                                                                 50, s));
  TEST_ASSERT_EQUAL(5, s);
  TEST_ASSERT(text_io_internal::RawFromRatioInRange<std::int8_t>(-9, 2, 0, -50,
                                                                 50, s));
  TEST_ASSERT_EQUAL(-5, s);
  TEST_ASSERT(text_io_internal::RawFromRatioInRange<std::int8_t>(9, -2, 0, -50,
                                                                 50, s));
  TEST_ASSERT_EQUAL(-5, s);
  TEST_ASSERT_FALSE(text_io_internal::RawFromRatioInRange<std::uint8_t>(
      -1, 1, 0, 0, 255, u));
  TEST_ASSERT_FALSE(text_io_internal::RawFromRatioInRange<std::uint8_t>(
      1, 0, 0, 0, 255, u));
}

void test_RawFromRatioInt64MinAndScales() {
  std::int64_t r64 = 0;
  TEST_ASSERT(text_io_internal::RawFromRatioInRange<std::int64_t>(
      std::numeric_limits<std::int64_t>::min(), 1, 0,
      std::numeric_limits<std::int64_t>::min(),
      std::numeric_limits<std::int64_t>::max(), r64));
  TEST_ASSERT_EQUAL(std::numeric_limits<std::int64_t>::min(), r64);

  std::uint8_t u = 0;
  TEST_ASSERT(text_io_internal::RawFromRatioInRange<std::uint8_t>(10, 4, 0, 0,
                                                                  255, u));
  TEST_ASSERT_EQUAL(3, u);

  std::uint32_t u32 = 0;
  TEST_ASSERT(text_io_internal::RawFromRatioInRange<std::uint32_t>(
      3, 1LL << 40, -40, 0, 1000, u32));
  TEST_ASSERT_EQUAL(3, u32);
  TEST_ASSERT(text_io_internal::RawFromRatioInRange<std::uint32_t>(5, 1, 40, 0,
                                                                   1000, u32));
  TEST_ASSERT_EQUAL(0, u32);
  TEST_ASSERT(text_io_internal::RawFromRatioInRange<std::uint32_t>(1, 1, 1, 0,
                                                                   1000, u32));
  TEST_ASSERT_EQUAL(1, u32);
}

void test_RawFromRatioGcdAndRounding() {
  std::uint16_t u = 0;
  // gcd reduce 100/25
  TEST_ASSERT(text_io_internal::RawFromRatioInRange<std::uint16_t>(100, 25, 0, 0,
                                                                   65535, u));
  TEST_ASSERT_EQUAL(4, u);
  // den powers of two with negative scale
  TEST_ASSERT(text_io_internal::RawFromRatioInRange<std::uint16_t>(5, 16, -5, 0,
                                                                   65535, u));
  TEST_ASSERT_EQUAL(10, u);
  // exact / below half / half / above half
  TEST_ASSERT(text_io_internal::RawFromRatioInRange<std::uint16_t>(10, 5, 0, 0,
                                                                   65535, u));
  TEST_ASSERT_EQUAL(2, u);
  TEST_ASSERT(text_io_internal::RawFromRatioInRange<std::uint16_t>(8, 6, 0, 0,
                                                                   65535, u));
  TEST_ASSERT_EQUAL(1, u);
  TEST_ASSERT(text_io_internal::RawFromRatioInRange<std::uint16_t>(3, 2, 0, 0,
                                                                   65535, u));
  TEST_ASSERT_EQUAL(2, u);
  TEST_ASSERT(text_io_internal::RawFromRatioInRange<std::uint16_t>(10, 6, 0, 0,
                                                                   65535, u));
  TEST_ASSERT_EQUAL(2, u);
}

void test_RawFromRatioOverflowButFitsAndBounds() {
  std::uint8_t u = 0;
  constexpr std::int64_t kHalfMax =
      std::numeric_limits<std::int64_t>::max() / 2;
  TEST_ASSERT(text_io_internal::RawFromRatioInRange<std::uint8_t>(
      kHalfMax, kHalfMax, -2, 0, 255, u));
  TEST_ASSERT_EQUAL(4, u);

  std::int8_t s = 0;
  TEST_ASSERT(text_io_internal::RawFromRatioInRange<std::int8_t>(
      -kHalfMax, kHalfMax, -2, -128, 127, s));
  TEST_ASSERT_EQUAL(-4, s);

  // den-side large positive scale where den<<shift overflows uint64
  std::uint32_t u32 = 0;
  constexpr std::int64_t big_den =
      static_cast<std::int64_t>((std::numeric_limits<std::uint64_t>::max() >> 2) +
                               3U);
  TEST_ASSERT(text_io_internal::RawFromRatioInRange<std::uint32_t>(
      big_den, big_den, 3, 0, 1000, u32));
  TEST_ASSERT_EQUAL(0, u32);

  // exact bounds
  TEST_ASSERT(text_io_internal::RawFromRatioInRange<std::uint8_t>(10, 1, 0, 10,
                                                                  20, u));
  TEST_ASSERT_EQUAL(10, u);
  TEST_ASSERT(text_io_internal::RawFromRatioInRange<std::uint8_t>(20, 1, 0, 10,
                                                                  20, u));
  TEST_ASSERT_EQUAL(20, u);
  TEST_ASSERT_FALSE(text_io_internal::RawFromRatioInRange<std::uint8_t>(
      9, 1, 0, 10, 20, u));
  TEST_ASSERT_FALSE(text_io_internal::RawFromRatioInRange<std::uint8_t>(
      21, 1, 0, 10, 20, u));
  TEST_ASSERT_FALSE(text_io_internal::RawFromRatioInRange<std::uint8_t>(
      1000, 1, 0, 0, 255, u));
}

void test_RawFromRatioExhaustiveSmall() {
  for (std::int64_t num = -8; num <= 8; ++num) {
    for (std::int64_t den = -8; den <= 8; ++den) {
      if (den == 0) {
        continue;
      }
      for (int scale = -3; scale <= 3; ++scale) {
        std::int16_t out = 0;
        bool const ok = text_io_internal::RawFromRatioInRange<std::int16_t>(
            num, den, scale, -200, 200, out);
        // Reference for small values via double (exact here).
        double logical = static_cast<double>(num) / static_cast<double>(den);
        if (scale < 0) {
          logical *= static_cast<double>(std::uint64_t{1}
                                         << static_cast<unsigned>(-scale));
        } else if (scale > 0) {
          logical /= static_cast<double>(std::uint64_t{1}
                                         << static_cast<unsigned>(scale));
        }
        double const rounded =
            logical >= 0.0 ? logical + 0.5 : logical - 0.5;
        auto const expected = static_cast<std::int64_t>(rounded);
        if (expected < -200 || expected > 200) {
          TEST_ASSERT_FALSE(ok);
        } else {
          TEST_ASSERT(ok);
          TEST_ASSERT_EQUAL(static_cast<std::int16_t>(expected), out);
        }
      }
    }
  }
}

void test_SqrtU64() {
  auto const& f = Fns();
  TEST_ASSERT_EQUAL_UINT(0U, f.sqrt_u64(0));
  TEST_ASSERT_EQUAL_UINT(1U, f.sqrt_u64(1));
  TEST_ASSERT_EQUAL_UINT(1U, f.sqrt_u64(3));
  TEST_ASSERT_EQUAL_UINT(2U, f.sqrt_u64(4));
  TEST_ASSERT_EQUAL_UINT(10U, f.sqrt_u64(100));
  TEST_ASSERT_EQUAL_UINT(255U, f.sqrt_u64(65535));
  TEST_ASSERT_EQUAL_UINT(65536U, f.sqrt_u64(65536ULL * 65536ULL));
  TEST_ASSERT_EQUAL_UINT(4294967295ULL, f.sqrt_u64(~0ULL));
}

}  // namespace ae::test_integer_math

int test_integer_math() {
  UNITY_BEGIN();
  RUN_TEST(ae::test_integer_math::test_AbsI64ToU64);
  RUN_TEST(ae::test_integer_math::test_GcdU64);
  RUN_TEST(ae::test_integer_math::test_MulU64Checked);
  RUN_TEST(ae::test_integer_math::test_AddU64Checked);
  RUN_TEST(ae::test_integer_math::test_RoundDivU64);
  RUN_TEST(ae::test_integer_math::test_MulDivU64NearestBasics);
  RUN_TEST(ae::test_integer_math::test_MulDivU64NearestOverflowPaths);
  RUN_TEST(ae::test_integer_math::test_MulDivU64NearestExhaustiveSmall);
  RUN_TEST(ae::test_integer_math::test_RoundMulPow2DivU64);
  RUN_TEST(ae::test_integer_math::test_RoundMulPow2DivU64HighRemBranch);
  RUN_TEST(ae::test_integer_math::test_RoundDivPow2U64);
  RUN_TEST(ae::test_integer_math::test_RoundMulPow2Overflow);
  RUN_TEST(ae::test_integer_math::test_MulDivTerm2Overflow);
  RUN_TEST(ae::test_integer_math::test_RoundMulPow2DivExhaustiveSmall);
  RUN_TEST(ae::test_integer_math::test_RoundDivPow2ExhaustiveSmall);
  RUN_TEST(ae::test_integer_math::test_RawFromRatioZeroAndSigns);
  RUN_TEST(ae::test_integer_math::test_RawFromRatioInt64MinAndScales);
  RUN_TEST(ae::test_integer_math::test_RawFromRatioGcdAndRounding);
  RUN_TEST(ae::test_integer_math::test_RawFromRatioOverflowButFitsAndBounds);
  RUN_TEST(ae::test_integer_math::test_RawFromRatioExhaustiveSmall);
  RUN_TEST(ae::test_integer_math::test_SqrtU64);
  return UNITY_END();
}
