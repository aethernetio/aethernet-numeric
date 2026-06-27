/*
 * Copyright 2024 Aethernet Inc.
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

#include <vector>
#include <random>
#include <cstdint>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <type_traits>

#include "numeric/tiered_int.h"

#include "mstream.h"

namespace ae::test_tiered_int {

using T1 = TieredInt<1, 249>;
using T2 = TieredInt<1, 249, 1529>;
using T3 = TieredInt<1, 249, 1529, 16777215>;
using T4 = TieredInt<2, 1000>;
using T5 = TieredInt<2, 1000, 8000>;
using T6 = TieredInt<4, 10>;

static_assert(std::is_same_v<T1::ValueType, std::uint16_t>);
static_assert(std::is_same_v<T2::ValueType, std::uint32_t>);
static_assert(std::is_same_v<T3::ValueType, std::uint64_t>);
static_assert(std::is_same_v<T4::ValueType, std::uint32_t>);
static_assert(std::is_same_v<T5::ValueType, std::uint64_t>);
static_assert(std::is_same_v<T6::ValueType, std::uint64_t>);

struct ObVector {
  using size_type = std::uint32_t;
  std::vector<std::uint8_t> data_;

  size_t write(std::uint8_t const* data, std::size_t size) {
    data_.insert(std::end(data_), data, data + size);
    return size;
  }
};

struct Omstream {
  ObVector obv_;
  test::Omstream<ObVector> stream_{obv_};

  auto& stream() { return stream_; }
  auto& data() const { return obv_.data_; }
};

struct IbVector {
  using size_type = std::uint32_t;

  std::vector<std::uint8_t> data_;

  explicit IbVector(const std::vector<std::uint8_t>& v) : data_(v) {}

  size_t read(void* data, size_t size) {
    std::memcpy(data, data_.data(), size);
    data_.erase(data_.begin(), data_.begin() + size);
    return size;
  }
};

struct Imstream {
  IbVector ibv_;
  test::Imstream<IbVector> stream_;

  explicit Imstream(std::vector<std::uint8_t> const& v)
      : ibv_{v}, stream_{ibv_} {}

  auto& stream() { return stream_; }
};

#pragma pack(push, 1)
struct S {
  std::uint8_t v8_1;
  std::uint8_t v8_2{0};
  std::uint16_t v16{0};
  std::uint32_t v32{0};
};
#pragma pack(pop)
static_assert(sizeof(S) == 8);

static std::string print(std::vector<std::uint8_t> const& v) {
  std::stringstream ss;
  ss << "[";
  for (auto i : v) {
    ss << std::setfill('0') << std::setw(2) << std::hex
       << static_cast<std::uint32_t>(i);
  }
  ss << "]";
  return ss.str();
}

template <typename T>
T GetRandom(T min, T max) {
  static std::random_device rd;
  static std::mt19937 gen(rd());

  std::uniform_int_distribution<T> dis{min, max};
  return dis(gen);
}

template <typename TInt>
void TestRoundTripBuffer(TInt x) {
  std::uint8_t buf[8] = {};
  auto l0 = x.Serialize(buf);

  TInt y;
  auto l1 = y.Deserialize(buf);
  TEST_ASSERT_EQUAL(static_cast<typename TInt::ValueType>(x.value_),
                    static_cast<typename TInt::ValueType>(y.value_));
  TEST_ASSERT_EQUAL(l0, l1);
}

template <typename TInt>
void TestValueToSize(typename TInt::ValueType value,
                     std::size_t expected_size) {
  auto p = TInt{value};
  std::uint8_t buf[8] = {};
  auto size = p.Serialize(buf);

  auto s = (std::stringstream{}
            << "Test on value " << static_cast<std::uint64_t>(value)
            << ", serialized as hex_arr: " << print({buf, buf + size})
            << " expected size:" << expected_size)
               .str();
  UNITY_SET_DETAIL(s.c_str());

  TEST_ASSERT_EQUAL(expected_size, size);

  TInt des_p;
  auto read_size = des_p.Deserialize(buf);
  TEST_ASSERT_EQUAL(expected_size, read_size);
  TEST_ASSERT_EQUAL(static_cast<typename TInt::ValueType>(p),
                    static_cast<typename TInt::ValueType>(des_p));
}

template <typename TInt>
void TestValueToSizeStream(typename TInt::ValueType value,
                           std::size_t expected_size) {
  auto p = TInt{value};
  Omstream os{};
  p.SerializeTo(os.stream());

  auto s = (std::stringstream{}
            << "Stream test on value " << static_cast<std::uint64_t>(value)
            << ", serialized as hex_arr: " << print(os.data())
            << " expected size:" << expected_size)
               .str();
  UNITY_SET_DETAIL(s.c_str());

  TEST_ASSERT_EQUAL(expected_size, os.data().size());

  Imstream is{os.data()};
  auto des_p = TInt{};
  des_p.DeserializeFrom(is.stream());
  TEST_ASSERT_EQUAL(static_cast<typename TInt::ValueType>(p),
                    static_cast<typename TInt::ValueType>(des_p));
}

template <typename PInt>
void TestRange() {
  auto get_next_step = [&]() {
    constexpr auto step =
        static_cast<typename PInt::ValueType>(PInt::kUpper / 1000);
    if constexpr (step == 0 || step == 1) {
      return typename PInt::ValueType{1};
    } else {
      return GetRandom<typename PInt::ValueType>(1, step);
    }
  };

  for (auto i = get_next_step(); i < PInt::kUpper; i += get_next_step()) {
    TestRoundTripBuffer(PInt{i});
    std::uint8_t buf[8] = {};
    auto size = PInt{i}.Serialize(buf);
    TestValueToSizeStream<PInt>(i, size);
  }
}

void test_TwoTier() {
  TestRoundTripBuffer(T1{249});
  TestRoundTripBuffer(T1{250});
  TestRoundTripBuffer(T1{251});
  TestRoundTripBuffer(T1{1785});

  TestValueToSize<T1>(249, 1);
  TestValueToSize<T1>(250, 2);
  TestValueToSize<T1>(251, 2);
  TestValueToSize<T1>(1785, 2);

  TestValueToSizeStream<T1>(249, 1);
  TestValueToSizeStream<T1>(250, 2);
  TestValueToSizeStream<T1>(1785, 2);

  std::uint64_t v = 0;
  { S b{249}; T1 t; t.Deserialize(reinterpret_cast<std::uint8_t*>(&b)); v = t; }
  TEST_ASSERT_EQUAL(249, v);
  { S b{250, 0}; T1 t; t.Deserialize(reinterpret_cast<std::uint8_t*>(&b)); v = t; }
  TEST_ASSERT_EQUAL(250, v);
  { S b{250, 1}; T1 t; t.Deserialize(reinterpret_cast<std::uint8_t*>(&b)); v = t; }
  TEST_ASSERT_EQUAL(251, v);
  { S b{255, 255}; T1 t; t.Deserialize(reinterpret_cast<std::uint8_t*>(&b)); v = t; }
  TEST_ASSERT_EQUAL(1785, v);

  TEST_ASSERT(T1{249} == T1{249});
  TEST_ASSERT(T1{250} > T1{249});
  TEST_ASSERT(std::numeric_limits<T1>::max() == T1::kUpper);
}

void test_ThreeTier() {
  TestRoundTripBuffer(T2{1529});
  TestRoundTripBuffer(T2{1530});
  TestRoundTripBuffer(T2{16778745});

  TestValueToSize<T2>(1529, 2);
  TestValueToSize<T2>(1530, 4);
  TestValueToSize<T2>(16778745, 4);

  TestValueToSizeStream<T2>(1529, 2);
  TestValueToSizeStream<T2>(1530, 4);
  TestValueToSizeStream<T2>(16778745, 4);

  std::uint64_t v = 0;
  { S b{254, 255}; T2 t; t.Deserialize(reinterpret_cast<std::uint8_t*>(&b)); v = t; }
  TEST_ASSERT_EQUAL(1529, v);
  { S b{255, 0, 0, 0}; T2 t; t.Deserialize(reinterpret_cast<std::uint8_t*>(&b)); v = t; }
  TEST_ASSERT_EQUAL(1530, v);
  { S b{255, 255, 65535}; T2 t; t.Deserialize(reinterpret_cast<std::uint8_t*>(&b)); v = t; }
  TEST_ASSERT_EQUAL(16778745, v);
}

void test_FourTier() {
  TestRoundTripBuffer(T3{16777215});
  TestRoundTripBuffer(T3{16777216});
  TestRoundTripBuffer(T3{16777217});
  TestRoundTripBuffer(T3{6571316740095ull});

  TestValueToSize<T3>(16777215, 4);
  TestValueToSize<T3>(16777216, 8);
  TestValueToSize<T3>(16777217, 8);
  TestValueToSize<T3>(6571316740095ull, 8);

  TestValueToSizeStream<T3>(16777215, 4);
  TestValueToSizeStream<T3>(16777216, 8);

  std::uint64_t v = 0;
  { S b{255, 255, static_cast<std::uint16_t>(65535 - 1530), 0};
    T3 t; t.Deserialize(reinterpret_cast<std::uint8_t*>(&b)); v = t; }
  TEST_ASSERT_EQUAL(16777215, v);
  { S b{255, 255, static_cast<std::uint16_t>(65535 - 1530 + 1), 0};
    T3 t; t.Deserialize(reinterpret_cast<std::uint8_t*>(&b)); v = t; }
  TEST_ASSERT_EQUAL(16777216, v);
  { S b{255, 255, static_cast<std::uint16_t>(65535 - 1530 + 1), 1};
    T3 t; t.Deserialize(reinterpret_cast<std::uint8_t*>(&b)); v = t; }
  TEST_ASSERT_EQUAL(16777217, v);
  { S b{255, 255, 65535, 0xFFFFFFFF};
    T3 t; t.Deserialize(reinterpret_cast<std::uint8_t*>(&b)); v = t; }
  TEST_ASSERT_EQUAL(6571316740095ull, v);
}

void test_StartSizeTwoAndFour() {
  // StartSize selects ValueType; the first wire byte is always uint8_t, so tier
  // max values above 255 only affect range typing, not multi-byte thresholds.
  TestRoundTripBuffer(T4{255});
  TestRoundTripBuffer(T5{255});
  TestRoundTripBuffer(T6{10});
  TestRoundTripBuffer(T6{11});
  TestRoundTripBuffer(T6{255});
}

void test_Range() {
  TestRange<T1>();
  TestRange<T2>();
}

}  // namespace ae::test_tiered_int

int test_tiered_int() {
  UNITY_BEGIN();
  RUN_TEST(ae::test_tiered_int::test_TwoTier);
  RUN_TEST(ae::test_tiered_int::test_ThreeTier);
  RUN_TEST(ae::test_tiered_int::test_FourTier);
  RUN_TEST(ae::test_tiered_int::test_StartSizeTwoAndFour);
  RUN_TEST(ae::test_tiered_int::test_Range);
  return UNITY_END();
}
