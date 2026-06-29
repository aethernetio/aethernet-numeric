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

#include "numeric/fixed_point.h"
#include "numeric/packed_ring.h"
#include "numeric/tiered_int.h"
#include "numeric/wire_io.h"

namespace ae::test_composed_types {

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

// ---------------------------------------------------------------------------
// 1. FixedPoint over TieredInt
// ---------------------------------------------------------------------------

using Raw = TieredInt<std::uint8_t, 254>;
using F = FixedPoint<Raw, 60.0>;

static_assert(MaxWireBytes<F>() == MaxWireBytes<Raw>());
static_assert(sizeof(F) == sizeof(Raw));

constexpr F fa{1};
constexpr F fb{30};
constexpr F fc{60};

static_assert(F::FromRaw(fa.raw()).raw_value() == fa.raw_value());
static_assert(F::Raw(fb.raw()).raw_value() == fb.raw_value());

void test_FixedPointOverTieredInt() {
  TEST_ASSERT(WireRoundTripEqual(fa));
  TEST_ASSERT(WireRoundTripEqual(fb));
  TEST_ASSERT(WireRoundTripEqual(fc));

  const F from_raw = F::FromRaw(Raw{500});
  TEST_ASSERT(from_raw.raw_value() == Raw{500});
  TEST_ASSERT(from_raw.raw_value() != fa.raw_value());

  TEST_ASSERT_EQUAL(1, WireSize(fa));
  TEST_ASSERT_EQUAL(WireSize(fa), WireSize(fa.raw()));

  const F two_byte_raw = F::FromRaw(Raw{255});
  TEST_ASSERT_EQUAL(2, WireSize(two_byte_raw));
  TEST_ASSERT_EQUAL(WireSize(two_byte_raw), WireSize(two_byte_raw.raw()));
}

// ---------------------------------------------------------------------------
// 6. PackedRing<TieredInt>
// ---------------------------------------------------------------------------

void test_PackedRingTieredInt() {
  using T = TieredInt<std::uint8_t, 254>;
  PackedRing<T, 64> ring;

  TEST_ASSERT_TRUE(ring.push(T{10}));
  TEST_ASSERT_TRUE(ring.push(T{254}));
  TEST_ASSERT_TRUE(ring.push(T{255}));
  TEST_ASSERT_TRUE(ring.push(T{510}));

  TEST_ASSERT_EQUAL(6, ring.used_bytes());
  TEST_ASSERT_EQUAL(4, ring.size());

  const int expected[] = {10, 254, 255, 510};
  int i = 0;
  for (const T value : ring) {
    TEST_ASSERT_EQUAL(expected[i], static_cast<int>(value));
    ++i;
  }
  TEST_ASSERT_EQUAL(4, i);
}

// ---------------------------------------------------------------------------
// 7. PackedRing<FixedPoint<TieredInt>>
// ---------------------------------------------------------------------------

void test_PackedRingFixedPointTieredInt() {
  PackedRing<F, 64> ring;

  const F a = F{1};
  const F b = F{30};
  const F c = F{60};

  TEST_ASSERT_TRUE(ring.push(a));
  TEST_ASSERT_TRUE(ring.push(b));
  TEST_ASSERT_TRUE(ring.push(c));

  TEST_ASSERT_EQUAL(WireSize(a) + WireSize(b) + WireSize(c), ring.used_bytes());
  TEST_ASSERT_EQUAL(3, ring.size());
  TEST_ASSERT_EQUAL(a.raw_value(), ring.front().raw_value());

  TEST_ASSERT_TRUE(ring.pop_front());
  TEST_ASSERT_EQUAL(b.raw_value(), ring.front().raw_value());
  TEST_ASSERT_EQUAL(2, ring.size());

  const F expected[] = {b, c};
  int i = 0;
  for (const F value : ring) {
    TEST_ASSERT_EQUAL(expected[i].raw_value(), value.raw_value());
    ++i;
  }
  TEST_ASSERT_EQUAL(2, i);
}

// ---------------------------------------------------------------------------
// 11. Ring value too large for capacity
// ---------------------------------------------------------------------------

void test_PackedRingValueTooLarge() {
  using T = TieredInt<std::uint8_t, 254>;
  PackedRing<T, 1> ring;

  TEST_ASSERT_FALSE(ring.push(T{255}));
  TEST_ASSERT_TRUE(ring.empty());
  TEST_ASSERT_EQUAL(0, ring.used_bytes());
  TEST_ASSERT_EQUAL(0, ring.size());
}

// ---------------------------------------------------------------------------
// 12. Empty behavior
// ---------------------------------------------------------------------------

void test_PackedRingEmptyBehavior() {
  using T = TieredInt<std::uint8_t, 254>;
  PackedRing<T, 16> ring;

  // front() requires a non-empty ring; do not call it here.

  TEST_ASSERT_FALSE(ring.pop_front());
  ring.clear();
  TEST_ASSERT_TRUE(ring.empty());
  TEST_ASSERT(ring.begin() == ring.end());

  TEST_ASSERT_TRUE(ring.push(T{1}));
  ring.clear();
  TEST_ASSERT_TRUE(ring.empty());
  TEST_ASSERT(ring.begin() == ring.end());
}

// ---------------------------------------------------------------------------
// 13. No std::size_t state in PackedRing
// ---------------------------------------------------------------------------

using R64 = PackedRing<TieredInt<std::uint8_t, 254>, 64>;
using R256 = PackedRing<TieredInt<std::uint8_t, 254>, 256>;

static_assert(std::is_same_v<typename R64::Index, std::uint8_t>);
static_assert(std::is_same_v<typename R256::Index, std::uint16_t>);

void test_PackedRingIndexType() { TEST_PASS(); }

}  // namespace ae::test_composed_types

int test_composed_types() {
  UNITY_BEGIN();
  RUN_TEST(ae::test_composed_types::test_FixedPointOverTieredInt);
  RUN_TEST(ae::test_composed_types::test_PackedRingTieredInt);
  RUN_TEST(ae::test_composed_types::test_PackedRingFixedPointTieredInt);
  RUN_TEST(ae::test_composed_types::test_PackedRingValueTooLarge);
  RUN_TEST(ae::test_composed_types::test_PackedRingEmptyBehavior);
  RUN_TEST(ae::test_composed_types::test_PackedRingIndexType);
  return UNITY_END();
}
