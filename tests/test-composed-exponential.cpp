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
  return r.bytes_read == n && r.value.code_value() == value.code_value();
}

using IntRuntime = FixedPoint<std::uint32_t, 60.0>;
using Code = TieredInt<std::uint8_t, 254>;
using E = Exponential<IntRuntime, Code, 0.001, 60.0>;

static_assert(MaxWireBytes<E>() == MaxWireBytes<Code>());
static_assert(sizeof(E) == sizeof(Code));
static_assert(E::Code(10).code_value() == 10);
static_assert(E::FromCode(10).code_value() == 10);

// ---------------------------------------------------------------------------
// 2. Exponential with integer FixedPoint runtime and TieredInt code
// ---------------------------------------------------------------------------

void test_ExponentialIntegerFixedPointRuntime() {
  const E e_min = E::from_double(0.001);
  const E e1 = E::from_double(1.0);
  const E e10 = E::FromRuntimeInteger(10);

  TEST_ASSERT(e_min.code_value() != 0);
  TEST_ASSERT(e10.code_value() != e1.code_value());
  TEST_ASSERT_EQUAL(10, E::Code(10).code_value());
  TEST_ASSERT_EQUAL(10, E::FromCode(10).code_value());

  const auto decoded = e1.value();
  TEST_ASSERT(decoded > IntRuntime::FromDouble(0.9));
  TEST_ASSERT(decoded < IntRuntime::FromDouble(1.1));
  TEST_ASSERT_EQUAL(10, E::Code(10).code().value_);

  TEST_ASSERT(ExponentialCodeWireRoundTrip(E::Code(10)));
  TEST_ASSERT(ExponentialCodeWireRoundTrip(e1));
}

// ---------------------------------------------------------------------------
// 4. Signed Exponential runtime
// ---------------------------------------------------------------------------

using SignedRuntime = FixedPoint<std::int32_t, 60.0>;
using SE = Exponential<SignedRuntime, Code, 0.001, 60.0>;

void test_SignedExponentialRuntime() {
  const SE se_neg = SE::from_double(-1.0);
  const SE se_zero = SE::from_double(0.0);
  const SE se_pos = SE::from_double(1.0);

  TEST_ASSERT(se_neg.is_negative());
  TEST_ASSERT(se_zero.is_zero());
  TEST_ASSERT(se_pos.is_positive());
  TEST_ASSERT(se_neg < se_zero);
  TEST_ASSERT(se_zero < se_pos);
  TEST_ASSERT(se_neg.abs().code_value() == se_pos.code_value());
  TEST_ASSERT(ExponentialCodeWireRoundTrip(se_neg));
  TEST_ASSERT(ExponentialCodeWireRoundTrip(se_zero));
  TEST_ASSERT(ExponentialCodeWireRoundTrip(se_pos));
}

// ---------------------------------------------------------------------------
// 8. PackedRing<Exponential<FixedPoint<integer>, TieredInt>>
// ---------------------------------------------------------------------------

void test_PackedRingExponentialIntegerFixedPoint() {
  PackedRing<E, 64> ring;

  TEST_ASSERT_TRUE(ring.push(E::Code(10)));
  TEST_ASSERT_TRUE(ring.push(E::Code(254)));
  TEST_ASSERT_TRUE(ring.push(E::Code(255)));
  TEST_ASSERT_TRUE(ring.push(E::Code(510)));

  TEST_ASSERT_EQUAL(6, ring.used_bytes());
  TEST_ASSERT_EQUAL(4, ring.size());

  const int expected[] = {10, 254, 255, 510};
  int i = 0;
  for (const E value : ring) {
    TEST_ASSERT_EQUAL(expected[i], static_cast<int>(value.code_value()));
    ++i;
  }
  TEST_ASSERT_EQUAL(4, i);
}

}  // namespace ae::test_composed_exponential

int test_composed_exponential() {
  UNITY_BEGIN();
  RUN_TEST(ae::test_composed_exponential::test_ExponentialIntegerFixedPointRuntime);
  RUN_TEST(ae::test_composed_exponential::test_SignedExponentialRuntime);
  RUN_TEST(ae::test_composed_exponential::test_PackedRingExponentialIntegerFixedPoint);
  return UNITY_END();
}
