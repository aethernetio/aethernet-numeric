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

#include <cstddef>
#include <cstdint>

#include <ae-numeric/exponential.h>
#include <ae-numeric/exponential_wire_io.h>
#include <ae-numeric/fixed_point.h>
#include <ae-numeric/packed_ring.h>
#include <ae-numeric/tiered_int.h>
#include <ae-numeric/wire_io.h>

namespace ae::test_composed_exponential_tiered {

template <typename T>
std::size_t WireSize(const T& value) {
  std::uint8_t buf[MaxWireBytes<T>()] = {};
  return Serialize(value, buf);
}

template <typename E>
bool ExponentialCodeWireRoundTrip(const E& value) {
  std::uint8_t buf[MaxWireBytes<E>()] = {};
  const auto n = Serialize(value, buf);
  const auto r = Deserialize<E>(buf, n);
  return r.bytes_read == n && r.value.CodeValue() == value.CodeValue();
}

using IntRuntime = FixedPoint<std::uint32_t, 60.0>;
using Code = TieredInt<std::uint8_t, 249, 1529>;
using RawRuntime = TieredInt<std::uint8_t, 254>;
using TieredRuntime = FixedPoint<RawRuntime, 60.0>;
using TieredE = Exponential<TieredRuntime, Code, 0.25, 60.0>;

static_assert(MaxWireBytes<TieredE>() == MaxWireBytes<Code>());
static_assert(sizeof(TieredE) == sizeof(Code));
static_assert(TieredE::Code(10).CodeValue() == 10);

using FutureE = Exponential<IntRuntime, Code, 0.001, 60.0>;

static_assert(MaxWireBytes<FutureE>() == MaxWireBytes<Code>());

void test_ExponentialTieredIntFixedPointRuntime() {
  const TieredE te_min = TieredE::FromDouble(0.25);
  const TieredE te1 = TieredE::FromDouble(1.0);
  const TieredE te10 = TieredE::FromRuntimeInteger(10);

  TEST_ASSERT(te_min.CodeValue() != 0);
  TEST_ASSERT_EQUAL(10, TieredE::Code(10).CodeValue());

  const auto decoded = te1.Value();
  TEST_ASSERT(decoded > TieredRuntime::FromDouble(0.9));
  TEST_ASSERT(decoded < TieredRuntime::FromDouble(1.1));

  TEST_ASSERT(ExponentialCodeWireRoundTrip(TieredE::Code(10)));
  TEST_ASSERT(ExponentialCodeWireRoundTrip(te1));
  TEST_ASSERT(ExponentialCodeWireRoundTrip(te10));
}

void test_FutureTierCode() {
  TEST_ASSERT_EQUAL(1, WireSize(Code{10}));
  TEST_ASSERT_EQUAL(1, WireSize(Code{249}));
  TEST_ASSERT_EQUAL(2, WireSize(Code{250}));
  TEST_ASSERT_EQUAL(2, WireSize(Code{1529}));

  TEST_ASSERT(ExponentialCodeWireRoundTrip(FutureE::Code(Code{10})));
  TEST_ASSERT(ExponentialCodeWireRoundTrip(FutureE::Code(Code{250})));
}

void test_PackedRingExponentialTieredIntFixedPoint() {
  PackedRing<TieredE, 64> ring;

  TEST_ASSERT_TRUE(ring.push(TieredE::Code(10)));
  TEST_ASSERT_TRUE(ring.push(TieredE::Code(249)));
  TEST_ASSERT_TRUE(ring.push(TieredE::Code(250)));
  TEST_ASSERT_TRUE(ring.push(TieredE::Code(1529)));

  TEST_ASSERT_EQUAL(6, ring.UsedBytes());
  TEST_ASSERT_EQUAL(4, ring.size());
  TEST_ASSERT_EQUAL(10, static_cast<int>(ring.front().CodeValue()));

  TEST_ASSERT_TRUE(ring.PopFront());
  TEST_ASSERT_EQUAL(249, static_cast<int>(ring.front().CodeValue()));
  TEST_ASSERT_EQUAL(3, ring.size());

  const int expected[] = {249, 250, 1529};
  int i = 0;
  for (const TieredE value : ring) {
    TEST_ASSERT_EQUAL(expected[i], static_cast<int>(value.CodeValue()));
    ++i;
  }
  TEST_ASSERT_EQUAL(3, i);
}

void test_PackedRingTwoByteCode() {
  PackedRing<FutureE, 32> ring;

  TEST_ASSERT_TRUE(ring.push(FutureE::Code(Code{10})));
  TEST_ASSERT_TRUE(ring.push(FutureE::Code(Code{249})));
  TEST_ASSERT_TRUE(ring.push(FutureE::Code(Code{250})));
  TEST_ASSERT_TRUE(ring.push(FutureE::Code(Code{1529})));

  TEST_ASSERT_EQUAL(6, ring.UsedBytes());
  TEST_ASSERT_EQUAL(4, ring.size());

  const int expected_codes[] = {10, 249, 250, 1529};
  int i = 0;
  for (const FutureE value : ring) {
    TEST_ASSERT_EQUAL(expected_codes[i],
                      static_cast<int>(value.WireCode().value_));
    ++i;
  }
  TEST_ASSERT_EQUAL(4, i);
}

}  // namespace ae::test_composed_exponential_tiered

int test_composed_exponential_tiered() {
  UNITY_BEGIN();
  RUN_TEST(ae::test_composed_exponential_tiered::
               test_ExponentialTieredIntFixedPointRuntime);
  RUN_TEST(ae::test_composed_exponential_tiered::test_FutureTierCode);
  RUN_TEST(ae::test_composed_exponential_tiered::
               test_PackedRingExponentialTieredIntFixedPoint);
  RUN_TEST(ae::test_composed_exponential_tiered::test_PackedRingTwoByteCode);
  return UNITY_END();
}
