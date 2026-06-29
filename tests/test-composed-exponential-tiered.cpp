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

#include <cstddef>
#include <cstdint>

#include "numeric/exponential.h"
#include "numeric/exponential_wire_io.h"
#include "numeric/fixed_point.h"
#include "numeric/packed_ring.h"
#include "numeric/tiered_int.h"
#include "numeric/wire_io.h"

namespace ae::test_composed_exponential_tiered {

template <typename T>
std::size_t WireSize(const T& value) {
  std::uint8_t buf[MaxWireBytes<T>()] = {};
  return Serialize(value, buf);
}

template <typename T>
bool WireRoundTripEqual(const T& value) {
  std::uint8_t buf[MaxWireBytes<T>()] = {};
  const auto n = Serialize(value, buf);
  const auto r = Deserialize<T>(buf, n);
  return r.bytes_read == n && r.value == value;
}

template <typename E>
bool ExponentialCodeWireRoundTrip(const E& value) {
  std::uint8_t buf[MaxWireBytes<E>()] = {};
  const auto n = Serialize(value, buf);
  const auto r = Deserialize<E>(buf, n);
  return r.bytes_read == n && r.value.code_value() == value.code_value();
}

using IntRuntime = FixedPoint<std::uint32_t, 60.0>;
using Code = TieredInt<std::uint8_t, 254>;
using RawRuntime = TieredInt<std::uint8_t, 254>;
using TieredRuntime = FixedPoint<RawRuntime, 60.0>;
using TieredE = Exponential<TieredRuntime, Code, 0.001, 60.0>;

static_assert(MaxWireBytes<TieredE>() == MaxWireBytes<Code>());
static_assert(sizeof(TieredE) == sizeof(Code));
static_assert(TieredE::Code(10).code_value() == 10);

// Multi-tier TieredInt cannot use 254 as the first boundary (no extension-header
// space). Use the same valid 4-tier config as test-tiered-int.cpp.
using FutureCode = TieredInt<std::uint8_t, 249, 1529, 16777215>;

// FutureCode::kUpper is huge; the default BoundaryCode (= kRawMax) would build an
// enormous compile-time magnitude table and hang the compiler (>60s). Use an
// explicit boundary code; codes in tests must stay within this boundary.
using FutureE = Exponential<IntRuntime, FutureCode, 0.001, 60.0, 254>;

static_assert(MaxWireBytes<FutureE>() == MaxWireBytes<FutureCode>());

// ---------------------------------------------------------------------------
// 3. Exponential with FixedPoint<TieredInt> runtime and TieredInt code
// ---------------------------------------------------------------------------

void test_ExponentialTieredIntFixedPointRuntime() {
  // FixedPoint<TieredInt>::FromDouble(0.001) rounds to zero at this scale, so
  // the encoded code is 0 rather than the minimum positive code.
  const TieredE te_min = TieredE::from_double(0.001);
  const TieredE te1 = TieredE::from_double(1.0);
  const TieredE te10 = TieredE::FromRuntimeInteger(10);

  TEST_ASSERT(te_min.is_zero());
  TEST_ASSERT_EQUAL(10, TieredE::Code(10).code_value());

  const auto decoded = te1.value();
  TEST_ASSERT(decoded > TieredRuntime::FromDouble(0.9));
  TEST_ASSERT(decoded < TieredRuntime::FromDouble(1.1));

  TEST_ASSERT(ExponentialCodeWireRoundTrip(TieredE::Code(10)));
  TEST_ASSERT(ExponentialCodeWireRoundTrip(te1));
  TEST_ASSERT(ExponentialCodeWireRoundTrip(te10));
}

// ---------------------------------------------------------------------------
// 5. Future-tier Code
// ---------------------------------------------------------------------------

void test_FutureTierCode() {
  TEST_ASSERT_EQUAL(1, WireSize(FutureCode{10}));
  TEST_ASSERT_EQUAL(1, WireSize(FutureCode{249}));
  TEST_ASSERT_EQUAL(2, WireSize(FutureCode{250}));
  TEST_ASSERT_EQUAL(4, WireSize(FutureCode{1530}));
  TEST_ASSERT_EQUAL(8, WireSize(FutureCode{16777216}));

  TEST_ASSERT(WireRoundTripEqual(FutureCode{1530}));
  TEST_ASSERT(ExponentialCodeWireRoundTrip(FutureE::Code(FutureCode{10})));
}

// ---------------------------------------------------------------------------
// 9. PackedRing<Exponential<FixedPoint<TieredInt>, TieredInt>>
// ---------------------------------------------------------------------------

void test_PackedRingExponentialTieredIntFixedPoint() {
  PackedRing<TieredE, 64> ring;

  TEST_ASSERT_TRUE(ring.push(TieredE::Code(10)));
  TEST_ASSERT_TRUE(ring.push(TieredE::Code(254)));
  TEST_ASSERT_TRUE(ring.push(TieredE::Code(255)));
  TEST_ASSERT_TRUE(ring.push(TieredE::Code(510)));

  TEST_ASSERT_EQUAL(6, ring.used_bytes());
  TEST_ASSERT_EQUAL(4, ring.size());
  TEST_ASSERT_EQUAL(10, static_cast<int>(ring.front().code_value()));

  TEST_ASSERT_TRUE(ring.pop_front());
  TEST_ASSERT_EQUAL(254, static_cast<int>(ring.front().code_value()));
  TEST_ASSERT_EQUAL(3, ring.size());

  const int expected[] = {254, 255, 510};
  int i = 0;
  for (const TieredE value : ring) {
    TEST_ASSERT_EQUAL(expected[i], static_cast<int>(value.code_value()));
    ++i;
  }
  TEST_ASSERT_EQUAL(3, i);
}

// ---------------------------------------------------------------------------
// 10. PackedRing with future-tier code
// ---------------------------------------------------------------------------

void test_PackedRingFutureTierCode() {
  PackedRing<FutureE, 32> ring;

  TEST_ASSERT_TRUE(ring.push(FutureE::Code(FutureCode{10})));
  TEST_ASSERT_TRUE(ring.push(FutureE::Code(FutureCode{250})));
  // Code 1530 exceeds FutureE's explicit BoundaryCode (254) and would clamp.
  TEST_ASSERT_TRUE(ring.push(FutureE::Code(FutureCode{254})));

  TEST_ASSERT_EQUAL(4, ring.used_bytes());
  TEST_ASSERT_EQUAL(3, ring.size());

  const int expected_codes[] = {10, 250, 254};
  int i = 0;
  for (const FutureE value : ring) {
    TEST_ASSERT_EQUAL(expected_codes[i],
                     static_cast<int>(value.code().value_));
    ++i;
  }
  TEST_ASSERT_EQUAL(3, i);
}

}  // namespace ae::test_composed_exponential_tiered

int test_composed_exponential_tiered() {
  UNITY_BEGIN();
  RUN_TEST(
      ae::test_composed_exponential_tiered::test_ExponentialTieredIntFixedPointRuntime);
  RUN_TEST(ae::test_composed_exponential_tiered::test_FutureTierCode);
  RUN_TEST(ae::test_composed_exponential_tiered::
               test_PackedRingExponentialTieredIntFixedPoint);
  RUN_TEST(ae::test_composed_exponential_tiered::test_PackedRingFutureTierCode);
  return UNITY_END();
}
