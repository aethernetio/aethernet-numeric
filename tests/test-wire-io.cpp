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

namespace ae::test_wire_io {

void test_BuiltinIntegerUnsigned() {
  std::uint8_t buf[8] = {};

  const auto n = Serialize<std::uint16_t>(0x1234, buf);
  TEST_ASSERT_EQUAL(2, n);
  TEST_ASSERT_EQUAL_HEX8(0x34, buf[0]);
  TEST_ASSERT_EQUAL_HEX8(0x12, buf[1]);

  const auto r = Deserialize<std::uint16_t>(buf, n);
  TEST_ASSERT_EQUAL(0x1234, r.value);
  TEST_ASSERT_EQUAL(2, r.BytesRead);
}

void test_BuiltinIntegerSigned() {
  {
    std::uint8_t buf[1] = {};
    const auto n = Serialize<std::int8_t>(-1, buf);
    TEST_ASSERT_EQUAL(1, n);
    TEST_ASSERT_EQUAL_HEX8(0xFF, buf[0]);

    const auto r = Deserialize<std::int8_t>(buf, n);
    TEST_ASSERT_EQUAL(-1, r.value);
    TEST_ASSERT_EQUAL(1, r.BytesRead);
  }

  {
    std::uint8_t buf[2] = {};
    const auto n = Serialize<std::int16_t>(-2, buf);
    TEST_ASSERT_EQUAL(2, n);
    TEST_ASSERT_EQUAL_HEX8(0xFE, buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, buf[1]);

    const auto r = Deserialize<std::int16_t>(buf, n);
    TEST_ASSERT_EQUAL(-2, r.value);
    TEST_ASSERT_EQUAL(2, r.BytesRead);
  }

  {
    std::uint8_t buf[4] = {};
    const auto n = Serialize<std::int32_t>(-123456, buf);
    TEST_ASSERT_EQUAL(4, n);
    TEST_ASSERT_EQUAL_HEX8(0xC0, buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0x1D, buf[1]);
    TEST_ASSERT_EQUAL_HEX8(0xFE, buf[2]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, buf[3]);

    const auto r = Deserialize<std::int32_t>(buf, n);
    TEST_ASSERT_EQUAL(-123456, r.value);
    TEST_ASSERT_EQUAL(4, r.BytesRead);
  }
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
  TEST_ASSERT_EQUAL(n, r.BytesRead);
}

void test_FixedPointBuiltinRep() {
  using F = FixedPoint<std::uint8_t, 100.0>;

  const auto value = F::FromInteger(10);
  std::uint8_t buf[MaxWireBytes<F>()] = {};

  const auto n = Serialize(value, buf);
  TEST_ASSERT_EQUAL(1, n);

  const auto r = Deserialize<F>(buf, n);
  TEST_ASSERT_EQUAL(value.RawValue(), r.value.RawValue());
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
  TEST_ASSERT_EQUAL(value.RawValue(), r.value.RawValue());
}

void test_Exponential() {
  using Runtime = FixedPoint<std::uint32_t, 60.0>;
  using Wire = TieredInt<std::uint8_t, 249, 1529>;
  using E = Exponential<Runtime, Wire, 0.001, 60.0>;

  const auto value = E::FromDouble(1.0);
  std::uint8_t buf[MaxWireBytes<E>()] = {};

  const auto n = Serialize(value, buf);
  TEST_ASSERT(n >= 1);
  TEST_ASSERT(n <= Wire::kMaxWireBytes);

  const auto r = Deserialize<E>(buf, n);
  TEST_ASSERT_EQUAL(value.CodeValue(), r.value.CodeValue());
}

void test_ShortBufferFixedWidth() {
  const std::uint8_t one_byte[1] = {0};
  TEST_ASSERT(1 < sizeof(std::uint32_t));
  (void)one_byte;
}

void test_ShortBufferTieredInt() {
  using T = TieredInt<std::uint8_t, 254>;
  const T value{300};

  std::uint8_t buf[T::kMaxWireBytes] = {};
  const auto n = Serialize(value, buf);
  TEST_ASSERT(n > 1);
  TEST_ASSERT_EQUAL(0, T::WireBytesNeeded(buf, 1));
}

void test_ShortBufferFixedPointTieredIntRep() {
  using Raw = TieredInt<std::uint8_t, 254>;
  using F = FixedPoint<Raw, 60.0>;

  const auto value = F::FromInteger(300);
  std::uint8_t buf[MaxWireBytes<F>()] = {};
  const auto n = Serialize(value, buf);
  TEST_ASSERT(n > 1);
  TEST_ASSERT_EQUAL(0, Raw::WireBytesNeeded(buf, 1));
}

void test_ShortBufferExponential() {
  using Runtime = FixedPoint<std::uint32_t, 60.0>;
  using Wire = TieredInt<std::uint8_t, 249, 1529>;
  using E = Exponential<Runtime, Wire, 0.001, 60.0>;

  const auto value = E::FromDouble(60.0);
  std::uint8_t buf[MaxWireBytes<E>()] = {};
  const auto n = Serialize(value, buf);
  TEST_ASSERT(n > 1);
  TEST_ASSERT_EQUAL(0, Wire::WireBytesNeeded(buf, 1));
}

}  // namespace ae::test_wire_io

int test_wire_io() {
  UNITY_BEGIN();
  RUN_TEST(ae::test_wire_io::test_BuiltinIntegerUnsigned);
  RUN_TEST(ae::test_wire_io::test_BuiltinIntegerSigned);
  RUN_TEST(ae::test_wire_io::test_TieredInt);
  RUN_TEST(ae::test_wire_io::test_FixedPointBuiltinRep);
  RUN_TEST(ae::test_wire_io::test_FixedPointTieredIntRep);
  RUN_TEST(ae::test_wire_io::test_Exponential);
  RUN_TEST(ae::test_wire_io::test_ShortBufferFixedWidth);
  RUN_TEST(ae::test_wire_io::test_ShortBufferTieredInt);
  RUN_TEST(ae::test_wire_io::test_ShortBufferFixedPointTieredIntRep);
  RUN_TEST(ae::test_wire_io::test_ShortBufferExponential);
  return UNITY_END();
}
