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

#include <cstdint>
#include <stdexcept>

#include "numeric/exponential.h"
#include "numeric/fixed_point.h"
#include "numeric/tiered_int.h"
#include "numeric/wire_io.h"

namespace ae::test_wire_io {

void test_BuiltinInteger() {
  std::uint8_t buf[8] = {};

  const auto n = Serialize<std::uint16_t>(0x1234, buf);
  TEST_ASSERT_EQUAL(2, n);
  TEST_ASSERT_EQUAL_HEX8(0x34, buf[0]);
  TEST_ASSERT_EQUAL_HEX8(0x12, buf[1]);

  const auto r = Deserialize<std::uint16_t>(buf, n);
  TEST_ASSERT_EQUAL(0x1234, r.value);
  TEST_ASSERT_EQUAL(2, r.bytes_read);
}

void test_TieredInt() {
  using T = TieredInt<std::uint8_t, 254>;

  const T value{123};
  std::uint8_t buf[T::kMaxWireBytes] = {};

  const auto n = Serialize(value, buf);
  TEST_ASSERT(n >= 1);
  TEST_ASSERT(n <= T::kMaxWireBytes);

  const auto r = Deserialize<T>(buf, n);
  TEST_ASSERT_EQUAL(123, r.value.value_);
  TEST_ASSERT_EQUAL(n, r.bytes_read);
}

void test_FixedPointBuiltinRep() {
  using F = FixedPoint<std::uint8_t, 100.0>;

  const auto value = F::FromInteger(10);
  std::uint8_t buf[MaxWireBytes<F>()] = {};

  const auto n = Serialize(value, buf);
  TEST_ASSERT_EQUAL(1, n);

  const auto r = Deserialize<F>(buf, n);
  TEST_ASSERT_EQUAL(value.raw_value(), r.value.raw_value());
}

void test_FixedPointTieredIntRep() {
  using Raw = TieredInt<std::uint8_t, 254>;
  using F = FixedPoint<Raw, 60.0>;

  const auto value = F::FromInteger(30);
  std::uint8_t buf[MaxWireBytes<F>()] = {};

  const auto n = Serialize(value, buf);
  TEST_ASSERT(n >= 1);
  TEST_ASSERT(n <= Raw::kMaxWireBytes);

  const auto r = Deserialize<F>(buf, n);
  TEST_ASSERT_EQUAL(value.raw_value(), r.value.raw_value());
}

void test_Exponential() {
  using Runtime = FixedPoint<std::uint32_t, 60.0>;
  using Wire = TieredInt<std::uint8_t, 254>;
  using E = Exponential<Runtime, Wire, 0.001, 60.0, 254>;

  const auto value = E::from_double(1.0);
  std::uint8_t buf[MaxWireBytes<E>()] = {};

  const auto n = Serialize(value, buf);
  TEST_ASSERT(n >= 1);
  TEST_ASSERT(n <= Wire::kMaxWireBytes);

  const auto r = Deserialize<E>(buf, n);
  TEST_ASSERT_EQUAL(value.code_value(), r.value.code_value());
}

void test_ShortBuffer() {
  using U32 = std::uint32_t;
  const std::uint8_t one_byte[1] = {0};

  bool threw = false;
  try {
    (void)Deserialize<U32>(one_byte, 1);
  } catch (const std::out_of_range&) {
    threw = true;
  }
  TEST_ASSERT(threw);
}

}  // namespace ae::test_wire_io

int test_wire_io() {
  UNITY_BEGIN();
  RUN_TEST(ae::test_wire_io::test_BuiltinInteger);
  RUN_TEST(ae::test_wire_io::test_TieredInt);
  RUN_TEST(ae::test_wire_io::test_FixedPointBuiltinRep);
  RUN_TEST(ae::test_wire_io::test_FixedPointTieredIntRep);
  RUN_TEST(ae::test_wire_io::test_Exponential);
  RUN_TEST(ae::test_wire_io::test_ShortBuffer);
  return UNITY_END();
}
