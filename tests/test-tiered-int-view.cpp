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
#include <span>
#include <type_traits>
#include <vector>

#include "numeric/tiered_int.h"
#include "numeric/tiered_int_view.h"

namespace ae::test_tiered_int_view {

using T2 = TieredInt<std::uint8_t, 249, 1529>;
using S2 = TieredInt<std::int8_t, 10, 20>;
using S4 = TieredInt<std::int8_t, 124, 125, 126>;

template <typename TInt>
std::vector<std::uint8_t> SerializeValues(
    std::initializer_list<typename TInt::ValueType> values) {
  std::vector<std::uint8_t> bytes;
  for (const auto raw : values) {
    const TInt value{raw};
    std::uint8_t buf[TInt::kMaxWireBytes] = {};
    const auto size = value.Serialize(buf);
    bytes.insert(bytes.end(), buf, buf + size);
  }
  return bytes;
}

template <typename TInt>
std::vector<typename TInt::ValueType> DecodeViaView(
    std::span<const std::uint8_t> bytes) {
  std::vector<typename TInt::ValueType> decoded;
  for (const TInt value : MakeTieredIntView<TInt>(bytes)) {
    decoded.push_back(static_cast<typename TInt::ValueType>(value));
  }
  return decoded;
}

template <typename TInt>
void TestViewRoundTrip(std::initializer_list<typename TInt::ValueType> values) {
  const auto bytes = SerializeValues<TInt>(values);
  const auto decoded = DecodeViaView<TInt>(bytes);
  TEST_ASSERT_EQUAL(values.size(), decoded.size());
  std::size_t i = 0;
  for (const auto expected : values) {
    TEST_ASSERT_EQUAL(static_cast<std::int64_t>(expected),
                      static_cast<std::int64_t>(decoded[i++]));
  }
}

void test_EmptyView() {
  using T = T2;
  const std::vector<std::uint8_t> empty;
  const TieredIntView<T> view{empty};

  TEST_ASSERT(view.empty());
  TEST_ASSERT_EQUAL(0, view.ByteSize());
  TEST_ASSERT(view.begin() == std::default_sentinel);

  std::size_t count = 0;
  for ([[maybe_unused]] const T value : view) {
    ++count;
  }
  TEST_ASSERT_EQUAL(0, count);
}

void test_UnsignedView() {
  TestViewRoundTrip<T2>({249, 250, 1529, 1530, 16778745});
}

void test_SignedView() {
  TestViewRoundTrip<S2>({0, 1, -1, 10, -10, 11, -11, S2::kUpper, S2::kLower});
}

void test_MixedWireSizes() {
  TestViewRoundTrip<S4>({124, 125, 126, 127, -124, -125, -126, -127});
}

void test_TruncatedBuffer() {
  const auto bytes = SerializeValues<T2>({1530});
  TEST_ASSERT(bytes.size() > 1);

  std::vector<std::uint8_t> truncated(bytes.begin(), bytes.end() - 1);
  TEST_ASSERT_EQUAL(bytes.size() - 1, truncated.size());
  TEST_ASSERT(truncated.size() >= 1);
}

void test_RangeForCollect() {
  constexpr typename T2::ValueType kValues[] = {249, 1529, 250, 1530};
  const auto bytes = SerializeValues<T2>({249, 1529, 250, 1530});

  std::vector<typename T2::ValueType> decoded;
  for (const T2 value : TieredIntView<T2>{bytes}) {
    decoded.push_back(static_cast<typename T2::ValueType>(value));
  }

  TEST_ASSERT_EQUAL(4, decoded.size());
  for (std::size_t i = 0; i < 4; ++i) {
    TEST_ASSERT_EQUAL(static_cast<std::int64_t>(kValues[i]),
                      static_cast<std::int64_t>(decoded[i]));
  }
}

void test_NoAllocation() {
  static_assert(std::is_nothrow_default_constructible_v<TieredIntView<T2>>);
  static_assert(sizeof(TieredIntView<T2>) ==
                sizeof(std::span<const std::uint8_t>));

  using It = TieredIntView<T2>::iterator;
  static_assert(std::is_default_constructible_v<It>);
  static_assert(
      !std::is_same_v<typename It::reference, typename It::value_type&>);
}

}  // namespace ae::test_tiered_int_view

int test_tiered_int_view() {
  UNITY_BEGIN();
  RUN_TEST(ae::test_tiered_int_view::test_EmptyView);
  RUN_TEST(ae::test_tiered_int_view::test_UnsignedView);
  RUN_TEST(ae::test_tiered_int_view::test_SignedView);
  RUN_TEST(ae::test_tiered_int_view::test_MixedWireSizes);
  RUN_TEST(ae::test_tiered_int_view::test_TruncatedBuffer);
  RUN_TEST(ae::test_tiered_int_view::test_RangeForCollect);
  RUN_TEST(ae::test_tiered_int_view::test_NoAllocation);
  return UNITY_END();
}
