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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include <ae-numeric/percentile.h>

namespace ae::test_percentile {

static_assert(sizeof(Percentile) == 2);
static_assert(sizeof(TimeoutFactor8) == 1);
static_assert(PercentileTail::kScaleExp < 0);
static_assert(TimeoutFactor8::kScaleExp == -6);

constexpr auto kP30 = Percentile::FromPercent(30.0);
constexpr auto kP50 = Percentile::FromPercent(50.0);
constexpr auto kP90 = Percentile::FromPercent(90.0);
constexpr auto kP95 = Percentile::FromPercent(95.0);
constexpr auto kP99 = Percentile::FromPercent(99.0);
constexpr auto kP999 = Percentile::FromPercent(99.9);
constexpr auto kP9999 = Percentile::FromPercent(99.99);
constexpr auto kP99999 = Percentile::FromPercent(99.999);
constexpr auto kP100 = Percentile::FromPercent(100.0);

static_assert(kP100.IsExactHundred());
static_assert(kP999 != kP9999);
static_assert(kP9999 != kP99999);
static_assert(kP30 < kP95);
static_assert(kP95 < kP99);
static_assert(kP99 < kP999);
static_assert(kP999 < kP9999);
static_assert(kP9999 < kP99999);
static_assert(kP99999 < kP100);

double TailToDouble(Percentile::tail_type tail) {
  const int frac = -Percentile::tail_type::kScaleExp;
  return static_cast<double>(tail.RawValue()) /
         static_cast<double>(std::uint64_t{1} << frac);
}

double EffectivePercentile(Percentile p) {
  return 100.0 - TailToDouble(p.TailPercent());
}

void ReportPoint(char const* name, double requested, Percentile p) {
  const double req_tail = 100.0 - requested;
  const double decoded_tail = TailToDouble(p.TailPercent());
  const double effective = EffectivePercentile(p);
  const double abs_err = std::abs(effective - requested);
  const double rel_err =
      (req_tail <= 0.0) ? 0.0
                        : std::abs(decoded_tail - req_tail) /
                              std::max(req_tail, 1e-18);
  std::printf(
      "  %-7s req=%.6f req_tail=%.6f raw=%u decoded_tail=%.9f "
      "effective=%.9f abs_err=%.9f tail_rel_err=%.6f%%\n",
      name, requested, req_tail,
      static_cast<unsigned>(p.TailPercent().RawValue()), decoded_tail,
      effective, abs_err, rel_err * 100.0);
  fflush(stdout);
}

void test_size_and_scales() {
  TEST_ASSERT_EQUAL_UINT(2, sizeof(Percentile));
  TEST_ASSERT_EQUAL_UINT(1, sizeof(TimeoutFactor8));
  TEST_ASSERT_EQUAL_INT(-9, PercentileTail::kScaleExp);
  TEST_ASSERT_EQUAL_INT(-6, TimeoutFactor8::kScaleExp);
  std::printf(
      "PercentileTail=FixedPoint<uint16_t,100> kScaleExp=%d resolution=1/%llu\n",
      PercentileTail::kScaleExp,
      1ull << static_cast<unsigned>(-PercentileTail::kScaleExp));
  std::printf("TimeoutFactor8 kScaleExp=%d step=1/%u\n",
              TimeoutFactor8::kScaleExp,
              1u << static_cast<unsigned>(-TimeoutFactor8::kScaleExp));
}

void test_named_accuracy() {
  ReportPoint("p30", 30.0, kP30);
  ReportPoint("p50", 50.0, kP50);
  ReportPoint("p90", 90.0, kP90);
  ReportPoint("p95", 95.0, kP95);
  ReportPoint("p99", 99.0, kP99);
  ReportPoint("p99.9", 99.9, kP999);
  ReportPoint("p99.99", 99.99, kP9999);
  ReportPoint("p99.999", 99.999, kP99999);
  ReportPoint("p100", 100.0, kP100);

  TEST_ASSERT_TRUE(kP100.IsExactHundred());
  TEST_ASSERT_TRUE(kP999 != kP9999);
  TEST_ASSERT_TRUE(kP9999 != kP99999);
  TEST_ASSERT_TRUE(kP30 < kP50);
  TEST_ASSERT_TRUE(kP99 < kP999);
}

void test_try_from_tail_bounds() {
  auto ok = Percentile::TryFromTailPercent(
      Percentile::tail_type::FromInteger(1));
  TEST_ASSERT_TRUE(ok.has_value());
  auto zero = Percentile::TryFromTailPercent(
      Percentile::tail_type::FromInteger(0));
  TEST_ASSERT_TRUE(zero.has_value());
  TEST_ASSERT_TRUE(zero->IsExactHundred());
  auto edge = Percentile::TryFromTailPercent(
      Percentile::tail_type::FromInteger(70));
  TEST_ASSERT_TRUE(edge.has_value());
  // One ULP above max declared tail (70).
  auto const max_tail = Percentile::tail_type::FromInteger(70);
  auto too_big = Percentile::TryFromTailPercent(
      Percentile::tail_type::FromRaw(
          static_cast<std::uint16_t>(max_tail.RawValue() + 1)));
  TEST_ASSERT_FALSE(too_big.has_value());
}

void test_percentile_index_large_n() {
  struct Case {
    std::size_t n;
    Percentile p;
    char const* name;
  };
  Case cases[] = {
      {100, kP95, "p95"},         {100, kP99, "p99"},
      {100, kP999, "p99.9"},      {100, kP9999, "p99.99"},
      {1000, kP95, "p95"},        {1000, kP99, "p99"},
      {1000, kP999, "p99.9"},     {1000, kP9999, "p99.99"},
      {10000, kP95, "p95"},       {10000, kP99, "p99"},
      {10000, kP999, "p99.9"},    {10000, kP9999, "p99.99"},
      {1000000, kP95, "p95"},     {1000000, kP99, "p99"},
      {1000000, kP999, "p99.9"},  {1000000, kP9999, "p99.99"},
  };

  for (auto const& c : cases) {
    const std::size_t got = PercentileIndex(c.n, c.p);
    // Reference uses the same quantized FixedPoint effective percentile.
    const double eff = EffectivePercentile(c.p) / 100.0;
    const double ideal = std::ceil(static_cast<double>(c.n - 1) * eff);
    const std::size_t ref = static_cast<std::size_t>(ideal);
    const std::int64_t delta =
        static_cast<std::int64_t>(got) - static_cast<std::int64_t>(ref);
    TEST_ASSERT_TRUE(got < c.n);
    TEST_ASSERT_TRUE(std::abs(delta) <= 1);
    std::printf("  N=%zu %s idx=%zu ref~%zu delta=%lld eff_p=%.9f\n", c.n,
                c.name, got, ref, static_cast<long long>(delta),
                EffectivePercentile(c.p));
  }

  TEST_ASSERT_TRUE(PercentileIndex(1000000, kP95) <
                   PercentileIndex(1000000, kP99));
  TEST_ASSERT_TRUE(PercentileIndex(1000000, kP99) <
                   PercentileIndex(1000000, kP999));
  TEST_ASSERT_TRUE(PercentileIndex(1000000, kP999) <
                   PercentileIndex(1000000, kP9999));
  TEST_ASSERT_EQUAL_UINT(PercentileIndexInteger(1000, 95),
                         PercentileIndex(1000, kP95));
  TEST_ASSERT_EQUAL_UINT(PercentileIndexInteger(1000, 99),
                         PercentileIndex(1000, kP99));
}

void test_timeout_factor_quantization() {
  constexpr auto f10 = TimeoutFactor8::FromDouble(1.0);
  constexpr auto f11 = TimeoutFactor8::FromDouble(1.1);
  constexpr auto f12 = TimeoutFactor8::FromDouble(1.2);
  constexpr auto f125 = TimeoutFactor8::FromDouble(1.25);
  constexpr auto f15 = TimeoutFactor8::FromDouble(1.5);
  constexpr auto f20 = TimeoutFactor8::FromDouble(2.0);
  struct Row {
    char const* name;
    double req;
    TimeoutFactor8 stored;
  };
  Row rows[] = {
      {"1.0", 1.0, f10},   {"1.1", 1.1, f11},   {"1.2", 1.2, f12},
      {"1.25", 1.25, f125}, {"1.5", 1.5, f15},  {"2.0", 2.0, f20},
  };
  const double step = 1.0 / 64.0;
  for (auto const& row : rows) {
    const double eff = static_cast<double>(row.stored.RawValue()) * step;
    std::printf("  factor req=%s stored_raw=%u effective=%.6f\n", row.name,
                static_cast<unsigned>(row.stored.RawValue()), eff);
    TEST_ASSERT_TRUE(std::abs(eff - row.req) <= step / 2.0 + 1e-12);
  }
  const std::uint32_t rtt_ms = 100;
  const std::uint32_t num =
      rtt_ms * static_cast<std::uint32_t>(f12.RawValue());
  const std::uint32_t timeout = (num + (1u << 5)) >> 6;
  std::printf("  RTT=100ms factor~1.2 timeout_ms=%u\n", timeout);
  TEST_ASSERT_EQUAL_UINT(120, timeout);
}

}  // namespace ae::test_percentile

int test_percentile() {
  UNITY_BEGIN();
  RUN_TEST(ae::test_percentile::test_size_and_scales);
  RUN_TEST(ae::test_percentile::test_named_accuracy);
  RUN_TEST(ae::test_percentile::test_try_from_tail_bounds);
  RUN_TEST(ae::test_percentile::test_percentile_index_large_n);
  RUN_TEST(ae::test_percentile::test_timeout_factor_quantization);
  return UNITY_END();
}
