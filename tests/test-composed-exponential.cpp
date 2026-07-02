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

namespace ae::test_composed_exponential {

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
  return r.bytes_read == n && r.value.CodeValue() == value.CodeValue();
}

using IntRuntime = FixedPoint<std::uint32_t, 60.0>;
using Code = TieredInt<std::uint8_t, 249, 1529>;
using E = Exponential<IntRuntime, Code, 0.001, 60.0>;

static_assert(MaxWireBytes<E>() == MaxWireBytes<Code>());
static_assert(sizeof(E) == sizeof(Code));
static_assert(E::Code(10).CodeValue() == 10);
static_assert(E::FromCode(10).CodeValue() == 10);

void test_ExponentialIntegerFixedPointRuntime() {
  const E e_min = E::FromDouble(0.001);
  const E e1 = E::FromDouble(1.0);
  const E e10 = E::FromRuntimeInteger(10);

  TEST_ASSERT(e_min.CodeValue() != 0);
  TEST_ASSERT(e10.CodeValue() != e1.CodeValue());
  TEST_ASSERT_EQUAL(10, E::Code(10).CodeValue());
  TEST_ASSERT_EQUAL(10, E::FromCode(10).CodeValue());

  const auto decoded = e1.Value();
  TEST_ASSERT(decoded > IntRuntime::FromDouble(0.9));
  TEST_ASSERT(decoded < IntRuntime::FromDouble(1.1));
  TEST_ASSERT_EQUAL(10, E::Code(10).WireCode().value_);

  TEST_ASSERT(ExponentialCodeWireRoundTrip(E::Code(10)));
  TEST_ASSERT(ExponentialCodeWireRoundTrip(e1));
}

void test_TwoByteBoundaryCode() {
  TEST_ASSERT_EQUAL(1, WireSize(E::Code(Code{10})));
  TEST_ASSERT_EQUAL(1, WireSize(E::Code(Code{249})));
  TEST_ASSERT_EQUAL(2, WireSize(E::Code(Code{250})));
  TEST_ASSERT_EQUAL(2, WireSize(E::Code(Code{1529})));

  TEST_ASSERT(ExponentialCodeWireRoundTrip(E::Code(Code{10})));
  TEST_ASSERT(ExponentialCodeWireRoundTrip(E::Code(Code{250})));
  TEST_ASSERT(ExponentialCodeWireRoundTrip(E::Code(Code{1529})));
}

using SignedRuntime = FixedPoint<std::int32_t, 60.0>;
using SE = Exponential<SignedRuntime, Code, 0.001, 60.0>;

void test_SignedExponentialRuntime() {
  const SE se_neg = SE::FromDouble(-1.0);
  const SE se_zero = SE::FromDouble(0.0);
  const SE se_pos = SE::FromDouble(1.0);

  TEST_ASSERT(se_neg.IsNegative());
  TEST_ASSERT(se_zero.IsZero());
  TEST_ASSERT(se_pos.IsPositive());
  TEST_ASSERT(se_neg < se_zero);
  TEST_ASSERT(se_zero < se_pos);
  TEST_ASSERT(se_neg.Abs().CodeValue() == se_pos.CodeValue());
  TEST_ASSERT(ExponentialCodeWireRoundTrip(se_neg));
  TEST_ASSERT(ExponentialCodeWireRoundTrip(se_zero));
  TEST_ASSERT(ExponentialCodeWireRoundTrip(se_pos));
}

void test_PackedRingExponentialIntegerFixedPoint() {
  PackedRing<E, 64> ring;

  TEST_ASSERT_TRUE(ring.push(E::Code(Code{10})));
  TEST_ASSERT_TRUE(ring.push(E::Code(Code{249})));
  TEST_ASSERT_TRUE(ring.push(E::Code(Code{250})));
  TEST_ASSERT_TRUE(ring.push(E::Code(Code{1529})));

  TEST_ASSERT_EQUAL(6, ring.UsedBytes());
  TEST_ASSERT_EQUAL(4, ring.size());

  const int expected[] = {10, 249, 250, 1529};
  int i = 0;
  for (const E value : ring) {
    TEST_ASSERT_EQUAL(expected[i], static_cast<int>(value.CodeValue()));
    ++i;
  }
  TEST_ASSERT_EQUAL(4, i);
}

}  // namespace ae::test_composed_exponential

int test_composed_exponential() {
  UNITY_BEGIN();
  RUN_TEST(
      ae::test_composed_exponential::test_ExponentialIntegerFixedPointRuntime);
  RUN_TEST(ae::test_composed_exponential::test_TwoByteBoundaryCode);
  RUN_TEST(ae::test_composed_exponential::test_SignedExponentialRuntime);
  RUN_TEST(ae::test_composed_exponential::
               test_PackedRingExponentialIntegerFixedPoint);
  return UNITY_END();
}
