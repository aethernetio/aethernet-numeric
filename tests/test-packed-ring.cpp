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
#include <type_traits>

#include "numeric/exponential.h"
#include "numeric/exponential_wire_io.h"
#include "numeric/fixed_point.h"
#include "numeric/packed_ring.h"
#include "numeric/tiered_int.h"
#include "numeric/wire_io.h"

namespace ae::test_packed_ring {

template <typename T>
std::size_t WireSize(const T& value) {
  std::uint8_t buf[wire_traits<T>::kMaxWireBytes] = {};
  return Serialize<T>(value, buf);
}

void test_TieredIntBasic() {
  using T = TieredInt<std::uint8_t, 254>;
  PackedRing<T, 64> ring;

  TEST_ASSERT_TRUE(ring.empty());
  TEST_ASSERT_EQUAL(0, ring.size());
  TEST_ASSERT_EQUAL(64, ring.capacity_bytes());

  TEST_ASSERT_TRUE(ring.push(T{10}));
  TEST_ASSERT_TRUE(ring.push(T{254}));
  TEST_ASSERT_TRUE(ring.push(T{255}));
  TEST_ASSERT_TRUE(ring.push(T{510}));

  // 1 + 1 + 2 + 2 bytes.
  TEST_ASSERT_EQUAL(6, ring.used_bytes());
  TEST_ASSERT_EQUAL(64 - 6, ring.free_bytes());
  TEST_ASSERT_EQUAL(4, ring.size());
  TEST_ASSERT_FALSE(ring.empty());

  const int expected[] = {10, 254, 255, 510};
  int i = 0;
  for (const T value : ring) {
    TEST_ASSERT_EQUAL(expected[i], static_cast<int>(value));
    ++i;
  }
  TEST_ASSERT_EQUAL(4, i);

  TEST_ASSERT_EQUAL(10, static_cast<int>(ring.front()));
}

void test_FixedPointLogicalValues() {
  using Raw = TieredInt<std::uint8_t, 254>;
  using F = FixedPoint<Raw, 60.0>;
  PackedRing<F, 64> ring;

  const F a = F{1};
  const F b = F{30};
  const F c = F{60};

  TEST_ASSERT_TRUE(ring.push(a));
  TEST_ASSERT_TRUE(ring.push(b));
  TEST_ASSERT_TRUE(ring.push(c));

  // Used bytes must follow the raw TieredInt wire size of each value.
  const std::size_t expected_used = WireSize(a) + WireSize(b) + WireSize(c);
  TEST_ASSERT_EQUAL(expected_used, ring.used_bytes());
  TEST_ASSERT_EQUAL(3, ring.size());

  const F expected[] = {a, b, c};
  int i = 0;
  for (const F value : ring) {
    TEST_ASSERT_EQUAL(expected[i].raw_value(), value.raw_value());
    ++i;
  }
  TEST_ASSERT_EQUAL(3, i);
}

void test_ExponentialCodes() {
  using Runtime = FixedPoint<std::uint32_t, 60.0>;
  using Wire = TieredInt<std::uint8_t, 254>;
  using E = Exponential<Runtime, Wire, 0.001, 60.0>;
  PackedRing<E, 64> ring;

  TEST_ASSERT_TRUE(ring.push(E::Code(10)));
  TEST_ASSERT_TRUE(ring.push(E::Code(254)));
  TEST_ASSERT_TRUE(ring.push(E::Code(255)));
  TEST_ASSERT_TRUE(ring.push(E::Code(510)));

  // Serialization stores code(); 1 + 1 + 2 + 2 bytes.
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

void test_Eviction() {
  using T = TieredInt<std::uint8_t, 254>;
  PackedRing<T, 4> ring;

  // Sizes 1, 1, 2, 2.
  TEST_ASSERT_TRUE(ring.push(T{10}));   // 1 byte
  TEST_ASSERT_TRUE(ring.push(T{20}));   // 1 byte
  TEST_ASSERT_TRUE(ring.push(T{255}));  // 2 bytes -> ring is full (4 bytes)
  TEST_ASSERT_EQUAL(4, ring.used_bytes());
  TEST_ASSERT_EQUAL(3, ring.size());

  // Pushing the last 2-byte value evicts the two 1-byte values.
  TEST_ASSERT_TRUE(ring.push(T{300}));  // 2 bytes
  TEST_ASSERT_EQUAL(4, ring.used_bytes());
  TEST_ASSERT_EQUAL(2, ring.size());

  const int expected[] = {255, 300};
  int i = 0;
  for (const T value : ring) {
    TEST_ASSERT_EQUAL(expected[i], static_cast<int>(value));
    ++i;
  }
  TEST_ASSERT_EQUAL(2, i);
  TEST_ASSERT_EQUAL(255, static_cast<int>(ring.front()));
}

void test_Wraparound() {
  using T = TieredInt<std::uint8_t, 254>;
  PackedRing<T, 8> ring;

  // Fill with four 2-byte values, then drain the first two to advance head.
  TEST_ASSERT_TRUE(ring.push(T{255}));  // A
  TEST_ASSERT_TRUE(ring.push(T{256}));  // B
  TEST_ASSERT_TRUE(ring.push(T{257}));  // C
  TEST_ASSERT_TRUE(ring.push(T{258}));  // D
  TEST_ASSERT_EQUAL(8, ring.used_bytes());

  TEST_ASSERT_TRUE(ring.pop_front());  // drop A
  TEST_ASSERT_TRUE(ring.pop_front());  // drop B
  TEST_ASSERT_EQUAL(4, ring.used_bytes());

  // These pushes wrap the tail past the physical end of the buffer.
  TEST_ASSERT_TRUE(ring.push(T{10}));   // E, 1 byte
  TEST_ASSERT_TRUE(ring.push(T{20}));   // F, 1 byte
  TEST_ASSERT_TRUE(ring.push(T{300}));  // G, 2 bytes (straddles the end)
  TEST_ASSERT_EQUAL(8, ring.used_bytes());
  TEST_ASSERT_EQUAL(5, ring.size());

  const int expected[] = {257, 258, 10, 20, 300};
  int i = 0;
  for (const T value : ring) {
    TEST_ASSERT_EQUAL(expected[i], static_cast<int>(value));
    ++i;
  }
  TEST_ASSERT_EQUAL(5, i);
}

void test_ClearAndPopEmpty() {
  using T = TieredInt<std::uint8_t, 254>;
  PackedRing<T, 16> ring;

  TEST_ASSERT_FALSE(ring.pop_front());

  TEST_ASSERT_TRUE(ring.push(T{1}));
  TEST_ASSERT_TRUE(ring.push(T{2}));
  TEST_ASSERT_EQUAL(2, ring.size());

  ring.clear();
  TEST_ASSERT_TRUE(ring.empty());
  TEST_ASSERT_EQUAL(0, ring.size());
  TEST_ASSERT_EQUAL(0, ring.used_bytes());
  TEST_ASSERT_EQUAL(16, ring.free_bytes());
}

void test_IndexType() {
  using T = TieredInt<std::uint8_t, 254>;
  static_assert(std::is_same_v<PackedRing<T, 64>::Index, std::uint8_t>);
  static_assert(std::is_same_v<PackedRing<T, 256>::Index, std::uint16_t>);
  TEST_PASS();
}

}  // namespace ae::test_packed_ring

int test_packed_ring() {
  UNITY_BEGIN();
  RUN_TEST(ae::test_packed_ring::test_TieredIntBasic);
  RUN_TEST(ae::test_packed_ring::test_FixedPointLogicalValues);
  RUN_TEST(ae::test_packed_ring::test_ExponentialCodes);
  RUN_TEST(ae::test_packed_ring::test_Eviction);
  RUN_TEST(ae::test_packed_ring::test_Wraparound);
  RUN_TEST(ae::test_packed_ring::test_ClearAndPopEmpty);
  RUN_TEST(ae::test_packed_ring::test_IndexType);
  return UNITY_END();
}
