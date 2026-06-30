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
#include "numeric/exponential_wire_io.h"
#include "numeric/fixed_point.h"
#include "numeric/tiered_int.h"
#include "numeric/wire_io.h"

namespace ae::test_exponential_arithmetic {

using Runtime = FixedPoint<std::uint32_t, 60.0>;
// Two-byte max wire tier (not 4- or 8-byte TieredInt exponential configs).
using Code = TieredInt<std::uint8_t, 249>;
using E = Exponential<Runtime, Code, 0.001, 60.0, 249>;
// Tiered runtime: MinMagnitude must fit runtime precision (0.001 rounds to
// zero).
using TieredRuntime = FixedPoint<Code, 60.0>;
using TieredE = Exponential<TieredRuntime, Code, 0.25, 60.0, 249>;

static_assert(E::kBoundaryCode == 249);
static_assert(E::kBoundaryCode <= numeric_traits<Code>::kMaxBoundaryCode);

static_assert(E::Code(1).Value().RawValue() > 0);
static_assert(E::FromDouble(0.001).CodeValue() != 0);
static_assert(TieredE::FromDouble(0.25).CodeValue() != 0);

static_assert(sizeof(E) == sizeof(Code));
static_assert(sizeof(E) == sizeof(Code::ValueType));
static_assert(alignof(E) == alignof(Code));
static_assert(MaxWireBytes<E>() == MaxWireBytes<Code>());
static_assert(MaxWireBytes<E>() == 2);
static_assert(MaxWireBytes<E>() <= 2);
static_assert(Code::kMaxWireBytes == 2);

Runtime RuntimeMidpoint(const Runtime& a, const Runtime& b) {
  const std::int64_t mid = (static_cast<std::int64_t>(a.RawValue()) +
                            static_cast<std::int64_t>(b.RawValue())) /
                           2;
  return Runtime::FromRaw(static_cast<typename Runtime::rep_type>(mid));
}

int CodeRoundTripTolerance(int code) {
  return code < static_cast<int>(Code::kWireTier0) ? 1 : 3;
}

void test_SelectedCodeDecodeMonotonicity() {
  const int codes[] = {0, 1, 2, 5, 10, 50, 100, 200, 248, 249};
  Runtime prev = Runtime::FromInteger(0);
  for (const int code : codes) {
    const Runtime current = E::Code(code).Value();
    TEST_ASSERT(current.RawValue() >= prev.RawValue());
    prev = current;
  }
}

void test_CodeValueCodeRoundTrip() {
  const int codes[] = {1, 2, 10, 50, 100, 200, 248, 249};
  for (const int code : codes) {
    const E encoded = E::Code(code);
    const E roundtrip = E::FromRuntime(encoded.Value());
    const int diff = static_cast<int>(roundtrip.CodeValue()) -
                     static_cast<int>(encoded.CodeValue());
    const int tolerance = CodeRoundTripTolerance(code);
    TEST_ASSERT(diff <= tolerance && diff >= -tolerance);
  }
}

void test_AdjacentCodeMidpointBoundaries() {
  const int pairs[][2] = {{1, 2},     {2, 3},     {10, 11},   {50, 51},
                          {100, 101}, {200, 201}, {247, 248}, {248, 249}};
  for (const auto& pair : pairs) {
    const int lo = pair[0];
    const int hi = pair[1];
    const Runtime v_lo = E::Code(lo).Value();
    const Runtime v_hi = E::Code(hi).Value();
    TEST_ASSERT(v_lo.RawValue() < v_hi.RawValue());

    const Runtime mid = RuntimeMidpoint(v_lo, v_hi);
    const int encoded = static_cast<int>(E::FromRuntime(mid).CodeValue());
    TEST_ASSERT(encoded >= lo - 1);
    TEST_ASSERT(encoded <= hi + 1);
  }
}

void test_MinMagnitudeRepresentability() {
  TEST_ASSERT(E::Code(1).Value().RawValue() > 0);
  TEST_ASSERT(E::FromDouble(0.001).CodeValue() >= 1);
  const int reencoded =
      static_cast<int>(E::FromRuntime(E::Code(1).Value()).CodeValue());
  TEST_ASSERT(reencoded >= 0 && reencoded <= 2);

  TEST_ASSERT(TieredE::Code(1).Value().RawValue() > 0);
  TEST_ASSERT(TieredE::FromDouble(0.25).CodeValue() >= 1);
}

void test_ExactStorageAndMaxWireBytes() {
  TEST_ASSERT_EQUAL(sizeof(Code::ValueType), sizeof(E));
  TEST_ASSERT_EQUAL(sizeof(Code), sizeof(E));
  TEST_ASSERT_EQUAL(2, MaxWireBytes<E>());
  TEST_ASSERT_EQUAL(Code::kMaxWireBytes, MaxWireBytes<E>());
}

void test_WireTierByteCounts() {
  std::uint8_t buf[MaxWireBytes<E>()] = {};

  const auto check = [&](int code, std::size_t expected_bytes) {
    const E value = E::Code(code);
    const std::size_t n = Serialize(value, buf);
    TEST_ASSERT_EQUAL(expected_bytes, n);
    TEST_ASSERT_EQUAL(expected_bytes, SerializedSizeAt<E>(buf, n));
  };

  check(10, 1);
  check(249, 1);
}

void test_NoWideWireExponentialConfigs() {
  // This suite uses a two-byte-max TieredInt wire only; no 4- or 8-byte
  // configs.
  TEST_ASSERT(MaxWireBytes<E>() <= 2);
  TEST_ASSERT(Code::kMaxWireBytes <= 2);
  TEST_PASS();
}

}  // namespace ae::test_exponential_arithmetic

int test_exponential_arithmetic() {
  UNITY_BEGIN();
  RUN_TEST(
      ae::test_exponential_arithmetic::test_SelectedCodeDecodeMonotonicity);
  RUN_TEST(ae::test_exponential_arithmetic::test_CodeValueCodeRoundTrip);
  RUN_TEST(
      ae::test_exponential_arithmetic::test_AdjacentCodeMidpointBoundaries);
  RUN_TEST(ae::test_exponential_arithmetic::test_MinMagnitudeRepresentability);
  RUN_TEST(ae::test_exponential_arithmetic::test_ExactStorageAndMaxWireBytes);
  RUN_TEST(ae::test_exponential_arithmetic::test_WireTierByteCounts);
  RUN_TEST(ae::test_exponential_arithmetic::test_NoWideWireExponentialConfigs);
  return UNITY_END();
}
