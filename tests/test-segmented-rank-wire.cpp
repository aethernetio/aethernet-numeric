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
#include <limits>

#include <ae-numeric/details/segmented_rank_wire.h>
#include <ae-numeric/tiered_int.h>

namespace ae::test_segmented_rank_wire {

using U16Wire = std::uint16_t;
using U8Wire = std::uint8_t;
using TierWire = TieredInt<std::uint8_t, 254, 477>;

static_assert(ae::seg::detail::RankFitsWire<U8Wire>(255u));
static_assert(ae::seg::detail::RankFitsWire<U16Wire>(65535u));
static_assert(!ae::seg::detail::RankFitsWire<U8Wire>(256u));
static_assert(!ae::seg::detail::RankFitsWire<U16Wire>(65536u));

void test_Uint8RoundTrip() {
  constexpr auto w = ae::seg::detail::RankToWire<U8Wire>(200u);
  TEST_ASSERT_EQUAL_UINT8(200u, w);
  TEST_ASSERT_EQUAL_UINT32(200u, ae::seg::detail::WireToRank(w));
}

void test_Uint8Max() {
  constexpr auto w = ae::seg::detail::RankToWire<U8Wire>(255u);
  TEST_ASSERT_EQUAL_UINT8(255u, w);
  TEST_ASSERT_EQUAL_UINT32(255u, ae::seg::detail::WireToRank(w));
}

void test_Uint16RoundTrip() {
  constexpr auto w = ae::seg::detail::RankToWire<U16Wire>(40000u);
  TEST_ASSERT_EQUAL_UINT16(40000u, w);
  TEST_ASSERT_EQUAL_UINT32(40000u, ae::seg::detail::WireToRank(w));
}

void test_Uint16Max() {
  constexpr auto w = ae::seg::detail::RankToWire<U16Wire>(65535u);
  TEST_ASSERT_EQUAL_UINT16(65535u, w);
  TEST_ASSERT_EQUAL_UINT32(65535u, ae::seg::detail::WireToRank(w));
}

void test_TieredIntRoundTrip() {
  constexpr auto w = ae::seg::detail::RankToWire<TierWire>(1200u);
  TEST_ASSERT_EQUAL_INT(1200, static_cast<int>(ae::seg::detail::WireToRank(w)));
  TEST_ASSERT_EQUAL_INT(
      1200, static_cast<int>(static_cast<TierWire::ValueType>(w)));
}

}  // namespace ae::test_segmented_rank_wire

int test_segmented_rank_wire() {
  UNITY_BEGIN();
  RUN_TEST(ae::test_segmented_rank_wire::test_Uint8RoundTrip);
  RUN_TEST(ae::test_segmented_rank_wire::test_Uint8Max);
  RUN_TEST(ae::test_segmented_rank_wire::test_Uint16RoundTrip);
  RUN_TEST(ae::test_segmented_rank_wire::test_Uint16Max);
  RUN_TEST(ae::test_segmented_rank_wire::test_TieredIntRoundTrip);
  return UNITY_END();
}
