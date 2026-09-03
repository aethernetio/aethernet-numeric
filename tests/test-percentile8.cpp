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
#include <vector>

#include <ae-numeric/percentile8.h>

namespace ae::test_percentile8 {

static_assert(sizeof(Percentile8) == 1);
static_assert(sizeof(TimeoutFactor8) == 1);
static_assert(Percentile8TailRuntime::kScaleExp < 0);
static_assert(TimeoutFactor8::kScaleExp == -6);

constexpr auto kP30 = Percentile8::FromPercent(30.0);
constexpr auto kP50 = Percentile8::FromPercent(50.0);
constexpr auto kP90 = Percentile8::FromPercent(90.0);
constexpr auto kP95 = Percentile8::FromPercent(95.0);
constexpr auto kP99 = Percentile8::FromPercent(99.0);
constexpr auto kP999 = Percentile8::FromPercent(99.9);
constexpr auto kP9999 = Percentile8::FromPercent(99.99);
constexpr auto kP100 = Percentile8::FromPercent(100.0);

static_assert(kP100.Code() == 0);
static_assert(kP999.Code() != kP9999.Code());
static_assert(kP30 < kP95);
static_assert(kP95 < kP99);
static_assert(kP99 < kP999);
static_assert(kP999 < kP9999);
static_assert(kP9999 < kP100);

double TailRuntimeToDouble(Percentile8::TailRuntime tail) {
  const int frac = -Percentile8::TailRuntime::kScaleExp;
  return static_cast<double>(tail.RawValue()) /
         static_cast<double>(std::uint64_t{1} << frac);
}

double EffectivePercentile(Percentile8 p) {
  if (p.IsExactHundred()) {
    return 100.0;
  }
  return 100.0 - TailRuntimeToDouble(p.TailPercent());
}

void ReportPoint(char const* name, double requested, Percentile8 p) {
  const double decoded_tail = TailRuntimeToDouble(p.TailPercent());
  const double effective = EffectivePercentile(p);
  const double req_tail = 100.0 - requested;
  const double rel_err =
      (requested >= 100.0) ? 0.0
                           : std::abs(decoded_tail - req_tail) /
                                 std::max(req_tail, 1e-12);
  std::printf(
      "  %-6s req=%.4f code=%u decoded_tail=%.6f effective=%.6f "
      "rel_tail_err=%.4f%%\n",
      name, requested, static_cast<unsigned>(p.Code()), decoded_tail, effective,
      rel_err * 100.0);
  fflush(stdout);
}

void test_size_and_scales() {
  TEST_ASSERT_EQUAL_UINT(1, sizeof(Percentile8));
  TEST_ASSERT_EQUAL_UINT(1, sizeof(TimeoutFactor8));
  TEST_ASSERT_TRUE(Percentile8TailRuntime::kScaleExp < 0);
  TEST_ASSERT_EQUAL_INT(-6, TimeoutFactor8::kScaleExp);
  std::printf("TailRuntime=FixedPoint<uint32_t,100> kScaleExp=%d resolution=1/%llu\n",
              Percentile8TailRuntime::kScaleExp,
              1ull << static_cast<unsigned>(-Percentile8TailRuntime::kScaleExp));
  std::printf("TimeoutFactor8 kScaleExp=%d step=1/%u\n", TimeoutFactor8::kScaleExp,
              1u << static_cast<unsigned>(-TimeoutFactor8::kScaleExp));
}

void test_named_encode_decode() {
  ReportPoint("p30", 30.0, kP30);
  ReportPoint("p50", 50.0, kP50);
  ReportPoint("p90", 90.0, kP90);
  ReportPoint("p95", 95.0, kP95);
  ReportPoint("p99", 99.0, kP99);
  ReportPoint("p99.9", 99.9, kP999);
  ReportPoint("p99.99", 99.99, kP9999);
  ReportPoint("p100", 100.0, kP100);

  TEST_ASSERT_EQUAL_UINT8(0, kP100.Code());
  TEST_ASSERT_TRUE(kP100.IsExactHundred());
  TEST_ASSERT_TRUE(kP999.Code() != kP9999.Code());
  TEST_ASSERT_TRUE(kP30.Code() > kP50.Code());
  TEST_ASSERT_TRUE(kP99.Code() > kP999.Code());
}

void test_code_space_monotonic_and_accuracy() {
  double max_neighbor_ratio = 1.0;
  double min_neighbor_ratio = 1e9;
  auto prev_tail = Percentile8::FromCode(1).TailPercent();
  auto const first_tail = prev_tail;
  for (unsigned c = 2; c <= 255; ++c) {
    auto const cur = Percentile8::FromCode(static_cast<std::uint8_t>(c));
    auto const cur_tail = cur.TailPercent();
    TEST_ASSERT_TRUE(cur_tail > prev_tail);
    const double a = TailRuntimeToDouble(prev_tail);
    const double b = TailRuntimeToDouble(cur_tail);
    const double ratio = b / a;
    max_neighbor_ratio = std::max(max_neighbor_ratio, ratio);
    min_neighbor_ratio = std::min(min_neighbor_ratio, ratio);
    prev_tail = cur_tail;
  }
  TEST_ASSERT_TRUE(prev_tail > first_tail);
  std::printf(
      "neighbor ratio min=%.6f max=%.6f (theory step~1.0355)\n",
      min_neighbor_ratio, max_neighbor_ratio);

  double max_rel = 0.0;
  std::vector<double> rels;
  rels.reserve(7000);
  for (int i = 0; i <= 6999; ++i) {
    const double p = 30.0 + (99.99 - 30.0) * (static_cast<double>(i) / 6999.0);
    const double req_tail = 100.0 - p;
    const auto milli =
        static_cast<std::int64_t>(req_tail * 100000.0 + 0.5);  // 1e-5
    auto const tail_fp =
        Percentile8::TailRuntime::FromRatio(milli, static_cast<std::int64_t>(100000));
    auto const encoded = Percentile8::TryFromTailPercent(tail_fp);
    TEST_ASSERT_TRUE(encoded.has_value());
    const double got_tail = TailRuntimeToDouble(encoded->TailPercent());
    const double rel = std::abs(got_tail - req_tail) / req_tail;
    max_rel = std::max(max_rel, rel);
    rels.push_back(rel);
  }
  std::nth_element(rels.begin(), rels.begin() + rels.size() / 2, rels.end());
  const double median_rel = rels[rels.size() / 2];
  std::printf("dense sweep max_rel_tail_err=%.4f%% median=%.4f%%\n",
              max_rel * 100.0, median_rel * 100.0);
  // Geometric neighbor step ≈ 3.55% ⇒ nearest quantization ≈ 1.8%. Extra error
  // is fixed Log2/Exp2 approximation. Do not tighten the assert to hide it.
  const double kTarget = 0.025;
  if (max_rel > kTarget) {
    std::printf(
        "NOTE: max relative tail error %.4f%% exceeds 2.5%% target; "
        "decomposition=exponential quantization + fixed Log2/Exp2 approx\n",
        max_rel * 100.0);
  }
  // Hard bound: half-step of geometric ratio (~1.8%) plus generous fixed-math
  // slack; keep the suite red only on pathological regression.
  TEST_ASSERT_TRUE(max_rel <= 0.05 + 1e-9);
  TEST_ASSERT_TRUE(median_rel <= 0.02 + 1e-9);
}

void test_try_from_tail_bounds() {
  auto ok =
      Percentile8::TryFromTailPercent(Percentile8::TailRuntime::FromDouble(1.0));
  TEST_ASSERT_TRUE(ok.has_value());
  auto zero =
      Percentile8::TryFromTailPercent(Percentile8::TailRuntime::FromInteger(0));
  TEST_ASSERT_TRUE(zero.has_value());
  TEST_ASSERT_EQUAL_UINT8(0, zero->Code());
  auto too_small = Percentile8::TryFromTailPercent(
      Percentile8::TailRuntime::FromDouble(0.001));
  TEST_ASSERT_FALSE(too_small.has_value());
  auto too_big = Percentile8::TryFromTailPercent(
      Percentile8::TailRuntime::FromDouble(80.0));
  TEST_ASSERT_FALSE(too_big.has_value());
}

void test_percentile_index_large_n() {
  struct Case {
    std::size_t n;
    Percentile8 p;
    char const* name;
  };
  Case cases[] = {
      {100, kP30, "p30"},       {100, kP95, "p95"},
      {100, kP99, "p99"},       {100, kP999, "p99.9"},
      {100, kP9999, "p99.99"},  {1000, kP30, "p30"},
      {1000, kP95, "p95"},      {1000, kP99, "p99"},
      {1000, kP999, "p99.9"},   {1000, kP9999, "p99.99"},
      {10000, kP30, "p30"},     {10000, kP95, "p95"},
      {10000, kP99, "p99"},     {10000, kP999, "p99.9"},
      {10000, kP9999, "p99.99"},{1000000, kP30, "p30"},
      {1000000, kP95, "p95"},   {1000000, kP99, "p99"},
      {1000000, kP999, "p99.9"},{1000000, kP9999, "p99.99"},
  };

  for (auto const& c : cases) {
    const std::size_t got = PercentileIndex(c.n, c.p);
    const double eff = EffectivePercentile(c.p) / 100.0;
    const double ideal = std::ceil(static_cast<double>(c.n - 1) * eff);
    const std::size_t ref = static_cast<std::size_t>(ideal);
    const std::int64_t delta =
        static_cast<std::int64_t>(got) - static_cast<std::int64_t>(ref);
    TEST_ASSERT_TRUE(got < c.n);
    // Rank math itself is exact for the quantized effective percentile; allow
    // only ±1 for host ceil float noise at huge N.
    TEST_ASSERT_TRUE(std::abs(delta) <= 1);
    std::printf("  N=%zu %s idx=%zu ref~%zu delta=%lld eff_p=%.6f\n", c.n,
                c.name, got, ref, static_cast<long long>(delta),
                EffectivePercentile(c.p));
  }

  TEST_ASSERT_TRUE(PercentileIndex(1000000, kP999) !=
                   PercentileIndex(1000000, kP9999));
  // Integer p95 matches Percentile8 p95 when quantization is close enough.
  const double eff95 = EffectivePercentile(kP95);
  TEST_ASSERT_TRUE(std::abs(eff95 - 95.0) < 0.5);
  TEST_ASSERT_EQUAL_UINT(PercentileIndexInteger(1000, 95),
                         PercentileIndex(1000, kP95));
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

}  // namespace ae::test_percentile8

int test_percentile8() {
  UNITY_BEGIN();
  RUN_TEST(ae::test_percentile8::test_size_and_scales);
  RUN_TEST(ae::test_percentile8::test_named_encode_decode);
  RUN_TEST(ae::test_percentile8::test_code_space_monotonic_and_accuracy);
  RUN_TEST(ae::test_percentile8::test_try_from_tail_bounds);
  RUN_TEST(ae::test_percentile8::test_percentile_index_large_n);
  RUN_TEST(ae::test_percentile8::test_timeout_factor_quantization);
  return UNITY_END();
}
