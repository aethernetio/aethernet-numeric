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

#include <ae-numeric/tiered_int.h>

#include "mstream.h"

namespace ae::test_tiered_int {

using T1 = TieredInt<std::uint8_t, 249>;
using T2 = TieredInt<std::uint8_t, 249, 1529>;
using T3 = TieredInt<std::uint8_t, 249, 1529, 16777215>;
using T4 = TieredInt<std::uint16_t, 1000>;
using T5 = TieredInt<std::uint16_t, 1000, 8000>;
using T6 = TieredInt<std::uint32_t, 10>;
using T254 = TieredInt<std::uint8_t, 254>;

using S1 = TieredInt<std::int16_t, 1000>;
using S2 = TieredInt<std::int8_t, 10, 20>;
using S3 = TieredInt<std::int16_t, 100>;
using S4 = TieredInt<std::int8_t, 124, 125, 126>;

static_assert(tiered_int_internal::kIsValidTierConfig<std::uint8_t, 254>);
static_assert(tiered_int_internal::kIsValidTierConfig<std::uint8_t, 249>);
static_assert(tiered_int_internal::kIsValidTierConfig<std::uint8_t, 249, 1529>);
static_assert(
    tiered_int_internal::kIsValidTierConfig<std::uint8_t, 249, 1529, 16777215>);
static_assert(!tiered_int_internal::kIsValidTierConfig<std::uint8_t, 255>);
static_assert(
    !tiered_int_internal::kIsValidTierConfig<std::uint8_t, 249, 1000000>);
static_assert(tiered_int_internal::kIsValidTierConfig<std::int8_t, 10, 20>);
static_assert(tiered_int_internal::kIsValidTierConfig<std::int16_t, 1000>);

static_assert(T1::kIsSigned == false);
static_assert(S1::kIsSigned == true);
static_assert(S2::kIsSigned == true);
static_assert(S3::kIsSigned == true);
static_assert(S4::kIsSigned == true);

static_assert(std::is_same_v<T254::ValueType, std::uint16_t>);
static_assert(T254::kBaseBytes == 1);
static_assert(T254::kMaxWireBytes == 2);
static_assert(T254::kUpper == 510);

static_assert(std::is_same_v<T1::ValueType, std::uint16_t>);
static_assert(std::is_same_v<T2::ValueType, std::uint32_t>);
static_assert(std::is_same_v<T3::ValueType, std::uint64_t>);
static_assert(std::is_same_v<T4::ValueType, std::uint32_t>);
static_assert(std::is_same_v<T5::ValueType, std::uint64_t>);
static_assert(std::is_same_v<T6::ValueType, std::uint64_t>);

static_assert(T1::kBaseBytes == 1);
static_assert(T1::kMaxWireBytes == 2);
static_assert(sizeof(T1::ValueType) == 2);
static_assert(T1::kMaxWireBytes == sizeof(T1::ValueType));
static_assert(T1::kUpper == 1785);
static_assert(T1::kUpper > std::numeric_limits<std::uint8_t>::max());
static_assert(T1::kUpper <= std::numeric_limits<std::uint16_t>::max());
static_assert(T1{T1::kUpper}.value_ == T1::kUpper);

static_assert(T2::kBaseBytes == 1);
static_assert(T2::kMaxWireBytes == 4);
static_assert(sizeof(T2::ValueType) == 4);
static_assert(T2::kMaxWireBytes == sizeof(T2::ValueType));
static_assert(T2::kUpper == 16778745);

static_assert(T3::kBaseBytes == 1);
static_assert(T3::kMaxWireBytes == 8);
static_assert(sizeof(T3::ValueType) == 8);
static_assert(T3::kMaxWireBytes == sizeof(T3::ValueType));
static_assert(T3::kUpper == 6571316740095ull);

static_assert(T4::kBaseBytes == 2);
static_assert(T4::kMaxWireBytes == 4);
static_assert(sizeof(T4::ValueType) == 4);
static_assert(T4::kMaxWireBytes == sizeof(T4::ValueType));
static_assert(T4{1000}.value_ == 1000);

static_assert(T5::kBaseBytes == 2);
static_assert(T5::kMaxWireBytes == 8);
static_assert(sizeof(T5::ValueType) == 8);
static_assert(T5::kMaxWireBytes == sizeof(T5::ValueType));

static_assert(T6::kBaseBytes == 4);
static_assert(T6::kMaxWireBytes == 8);
static_assert(sizeof(T6::ValueType) == 8);
static_assert(T6::kMaxWireBytes == sizeof(T6::ValueType));
static_assert(T6::kUpper > static_cast<std::uint64_t>(
                               std::numeric_limits<std::int64_t>::max()));
static_assert(T6{T6::kUpper}.value_ == T6::kUpper);

static_assert(S1::kBaseBytes == 2);
static_assert(S1::kMaxWireBytes == 4);
static_assert(sizeof(S1::ValueType) == 4);
static_assert(S1::kLower == -2081915880);
static_assert(S1::kUpper == 2081915880);
static_assert(S1::kUpper > 2000000000);
static_assert(S1::kLower < -2000000000);
static_assert(ae::tiered_int_internal::ZigZagEncode64(S1::kUpper) >
              static_cast<std::uint64_t>(1) << 31);

static_assert(S2::kBaseBytes == 1);
static_assert(S2::kMaxWireBytes == 4);
static_assert(sizeof(S2::ValueType) == 4);
static_assert(S2::kLower == -1970667540);
static_assert(S2::kUpper == 1970667540);

static_assert(S3::kBaseBytes == 2);
static_assert(S3::kMaxWireBytes == 4);
static_assert(sizeof(S3::ValueType) == 4);
static_assert(S3::kLower == -2140897380);
static_assert(S3::kUpper == 2140897380);
static_assert(S3::kUpper > 2000000000);
static_assert(S3::kUpper < static_cast<std::int32_t>(2147483647));

static_assert(S4::kBaseBytes == 1);
static_assert(S4::kMaxWireBytes == 8);
static_assert(sizeof(S4::ValueType) == 8);
static_assert(S4::kLower == -251920099861069950LL);
static_assert(S4::kUpper == 251920099861069950LL);
static_assert(S4::kUpper > static_cast<std::int64_t>(1) << 31);

template <typename T>
void AssertNumericLimitsMatchBounds() {
  if constexpr (std::numeric_limits<T>::max() != T::kUpper) {
    TEST_FAIL();
  }
  if constexpr (T::kIsSigned) {
    if constexpr (std::numeric_limits<T>::min() != T::kLower) {
      TEST_FAIL();
    }
  }
}

template <typename T, auto Threshold>
void AssertUpperExceeds() {
  if constexpr (!(T::kUpper > Threshold)) {
    TEST_FAIL();
  }
}

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

  auto& stream() {
    return stream_;
  }
  auto& data() const {
    return obv_.data_;
  }
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

  auto& stream() {
    return stream_;
  }
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
  std::uint8_t buf[TInt::kMaxWireBytes] = {};
  auto l0 = x.Serialize(buf);

  TInt y;
  auto l1 = y.Deserialize(buf, l0);
  TEST_ASSERT_EQUAL(static_cast<typename TInt::ValueType>(x.value_),
                    static_cast<typename TInt::ValueType>(y.value_));
  TEST_ASSERT_EQUAL(l0, l1);

  TInt z;
  auto l2 = z.Deserialize(buf);
  TEST_ASSERT_EQUAL(static_cast<typename TInt::ValueType>(x.value_),
                    static_cast<typename TInt::ValueType>(z.value_));
  TEST_ASSERT_EQUAL(l0, l2);
}

template <typename TInt>
void TestValueToSize(typename TInt::ValueType value,
                     std::size_t expected_size) {
  auto p = TInt{value};
  std::uint8_t buf[TInt::kMaxWireBytes] = {};
  auto size = p.Serialize(buf);

  auto s = (std::stringstream{}
            << "Test on value " << static_cast<std::uint64_t>(value)
            << ", serialized as hex_arr: " << print({buf, buf + size})
            << " expected size:" << expected_size)
               .str();
  UNITY_SET_DETAIL(s.c_str());

  TEST_ASSERT_EQUAL(expected_size, size);

  TInt des_p;
  auto read_size = des_p.Deserialize(buf, size);
  TEST_ASSERT_EQUAL(expected_size, read_size);
  TEST_ASSERT_EQUAL(static_cast<typename TInt::ValueType>(p),
                    static_cast<typename TInt::ValueType>(des_p));
}

template <typename TInt>
void TestExactSerializedBytes(typename TInt::ValueType value,
                              std::initializer_list<std::uint8_t> expected) {
  const auto p = TInt{value};
  std::uint8_t buf[TInt::kMaxWireBytes] = {};
  const auto size = p.Serialize(buf);

  TEST_ASSERT_EQUAL(expected.size(), size);
  std::size_t i = 0;
  for (const auto byte : expected) {
    TEST_ASSERT_EQUAL_HEX8(byte, buf[i++]);
  }
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

template <typename TInt>
void TestWireSizeTransition(typename TInt::ValueType last_small,
                            typename TInt::ValueType first_large,
                            std::size_t small_bytes, std::size_t large_bytes) {
  TestValueToSize<TInt>(last_small, small_bytes);
  TestValueToSize<TInt>(first_large, large_bytes);
  TestValueToSizeStream<TInt>(last_small, small_bytes);
  TestValueToSizeStream<TInt>(first_large, large_bytes);
  TestRoundTripBuffer(TInt{last_small});
  TestRoundTripBuffer(TInt{first_large});
}

template <typename TInt>
void TestMaxWireSize() {
  TestValueToSize<TInt>(TInt::kUpper, TInt::kMaxWireBytes);
  TestValueToSizeStream<TInt>(TInt::kUpper, TInt::kMaxWireBytes);
  TestRoundTripBuffer(TInt{TInt::kUpper});
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
    std::uint8_t buf[PInt::kMaxWireBytes] = {};
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
  {
    S b{249};
    T1 t;
    t.Deserialize(reinterpret_cast<std::uint8_t*>(&b));
    v = t;
  }
  TEST_ASSERT_EQUAL(249, v);
  {
    S b{250, 0};
    T1 t;
    t.Deserialize(reinterpret_cast<std::uint8_t*>(&b));
    v = t;
  }
  TEST_ASSERT_EQUAL(250, v);
  {
    S b{250, 1};
    T1 t;
    t.Deserialize(reinterpret_cast<std::uint8_t*>(&b));
    v = t;
  }
  TEST_ASSERT_EQUAL(251, v);
  {
    S b{255, 255};
    T1 t;
    t.Deserialize(reinterpret_cast<std::uint8_t*>(&b));
    v = t;
  }
  TEST_ASSERT_EQUAL(1785, v);

  TEST_ASSERT(T1{249} == T1{249});
  TEST_ASSERT(T1{250} > T1{249});
  AssertNumericLimitsMatchBounds<T1>();
}

void test_ConstexprConstruction() {
  constexpr T1 at_max{T1::kUpper};
  TEST_ASSERT_EQUAL(T1::kUpper, at_max.value_);
  constexpr T4 at_tier_boundary{1000};
  TEST_ASSERT_EQUAL(1000, at_tier_boundary.value_);
  T1 assigned{};
  assigned = 1785;
  TEST_ASSERT_EQUAL(1785, assigned.value_);
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
  {
    S b{254, 255};
    T2 t;
    t.Deserialize(reinterpret_cast<std::uint8_t*>(&b));
    v = t;
  }
  TEST_ASSERT_EQUAL(1529, v);
  {
    S b{255, 0, 0, 0};
    T2 t;
    t.Deserialize(reinterpret_cast<std::uint8_t*>(&b));
    v = t;
  }
  TEST_ASSERT_EQUAL(1530, v);
  {
    S b{255, 255, 65535};
    T2 t;
    t.Deserialize(reinterpret_cast<std::uint8_t*>(&b));
    v = t;
  }
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
  {
    S b{255, 255, static_cast<std::uint16_t>(65535 - 1530), 0};
    T3 t;
    t.Deserialize(reinterpret_cast<std::uint8_t*>(&b));
    v = t;
  }
  TEST_ASSERT_EQUAL(16777215, v);
  {
    S b{255, 255, static_cast<std::uint16_t>(65535 - 1530 + 1), 0};
    T3 t;
    t.Deserialize(reinterpret_cast<std::uint8_t*>(&b));
    v = t;
  }
  TEST_ASSERT_EQUAL(16777216, v);
  {
    S b{255, 255, static_cast<std::uint16_t>(65535 - 1530 + 1), 1};
    T3 t;
    t.Deserialize(reinterpret_cast<std::uint8_t*>(&b));
    v = t;
  }
  TEST_ASSERT_EQUAL(16777217, v);
  {
    S b{255, 255, 65535, 0xFFFFFFFF};
    T3 t;
    t.Deserialize(reinterpret_cast<std::uint8_t*>(&b));
    v = t;
  }
  TEST_ASSERT_EQUAL(6571316740095ull, v);
}

void test_WireCellTwoAndFour() {
  TestRoundTripBuffer(T4{500});
  TestRoundTripBuffer(T4{1000});
  TestRoundTripBuffer(T4{1001});
  TestRoundTripBuffer(T4{70000});

  TestValueToSize<T4>(500, 2);
  TestValueToSize<T4>(1000, 2);
  TestValueToSize<T4>(1001, 4);
  TestValueToSizeStream<T4>(1001, 4);

  TestRoundTripBuffer(T5{8000});
  TestRoundTripBuffer(T5{8001});
  TestValueToSize<T5>(8000, 4);
  TestValueToSize<T5>(8001, 8);

  TestRoundTripBuffer(T6{10});
  TestRoundTripBuffer(T6{11});
  TestValueToSize<T6>(10, 4);
  TestValueToSize<T6>(11, 8);
  TestValueToSizeStream<T6>(11, 8);
}

void test_ExtensionHeaderTier() {
  TestExactSerializedBytes<T254>(254, {0xFE});
  TestExactSerializedBytes<T254>(255, {0xFF, 0x00});
  TestExactSerializedBytes<T254>(510, {0xFF, 0xFF});

  TestRoundTripBuffer(T254{254});
  TestRoundTripBuffer(T254{255});
  TestRoundTripBuffer(T254{510});
  TestValueToSize<T254>(254, 1);
  TestValueToSize<T254>(255, 2);
  TestValueToSize<T254>(510, 2);
  TestMaxWireSize<T254>();
}

void test_RepresentativeWireEncoding() {
  TestExactSerializedBytes<T2>(249, {0xF9});
  TestExactSerializedBytes<T2>(250, {0xFA, 0x00});
  TestExactSerializedBytes<T2>(1529, {0xFE, 0xFF});
  TestExactSerializedBytes<T2>(1530, {0xFF, 0x00, 0x00, 0x00});
}

void test_WireSizeTransitions() {
  TestWireSizeTransition<T1>(249, 250, 1, 2);
  TestMaxWireSize<T1>();

  TestWireSizeTransition<T2>(249, 250, 1, 2);
  TestWireSizeTransition<T2>(1529, 1530, 2, 4);
  TestMaxWireSize<T2>();

  TestWireSizeTransition<T3>(249, 250, 1, 2);
  TestWireSizeTransition<T3>(1529, 1530, 2, 4);
  TestWireSizeTransition<T3>(16777215, 16777216, 4, 8);
  TestMaxWireSize<T3>();

  TestWireSizeTransition<T4>(1000, 1001, 2, 4);
  TestMaxWireSize<T4>();

  TestWireSizeTransition<T5>(1000, 1001, 2, 4);
  TestWireSizeTransition<T5>(8000, 8001, 4, 8);
  TestMaxWireSize<T5>();

  TestWireSizeTransition<T6>(10, 11, 4, 8);
  TestMaxWireSize<T6>();
}

void test_SignedWireCell() {
  TestRoundTripBuffer(S1{0});
  TestRoundTripBuffer(S1{-1000});
  TestRoundTripBuffer(S1{1000});
  TestRoundTripBuffer(S1{-1001});
  TestRoundTripBuffer(S1{1001});
  TestRoundTripBuffer(S1{2000000000});
  TestRoundTripBuffer(S1{-2000000000});
  TestRoundTripBuffer(S1{S1::kUpper});
  TestRoundTripBuffer(S1{S1::kLower});

  TestWireSizeTransition<S1>(1000, 1001, 2, 4);
  TestWireSizeTransition<S1>(-1000, -1001, 2, 4);

  TestValueToSize<S1>(-500, 2);
  TestValueToSize<S1>(500, 2);
  TestValueToSizeStream<S1>(-1001, 4);
  TestValueToSize<S1>(S1::kUpper, S1::kMaxWireBytes);
  TestValueToSize<S1>(S1::kLower, S1::kMaxWireBytes);
  TestValueToSizeStream<S1>(S1::kUpper, S1::kMaxWireBytes);
  TestValueToSizeStream<S1>(S1::kLower, S1::kMaxWireBytes);
  TestMaxWireSize<S1>();

  TEST_ASSERT(S1{-1000} < S1{1000});
  AssertNumericLimitsMatchBounds<S1>();
}

void test_SignedThreeTier() {
  TestRoundTripBuffer(S2{10});
  TestRoundTripBuffer(S2{11});
  TestRoundTripBuffer(S2{20});
  TestRoundTripBuffer(S2{21});
  TestRoundTripBuffer(S2{-10});
  TestRoundTripBuffer(S2{-11});
  TestRoundTripBuffer(S2{-20});
  TestRoundTripBuffer(S2{-21});
  TestRoundTripBuffer(S2{S2::kUpper});
  TestRoundTripBuffer(S2{S2::kLower});

  TestWireSizeTransition<S2>(10, 11, 1, 2);
  TestWireSizeTransition<S2>(-10, -11, 1, 2);
  TestWireSizeTransition<S2>(20, 21, 2, 4);
  TestWireSizeTransition<S2>(-20, -21, 2, 4);
  TestMaxWireSize<S2>();

  TestValueToSize<S2>(10, 1);
  TestValueToSize<S2>(11, 2);
  TestValueToSize<S2>(20, 2);
  TestValueToSize<S2>(21, 4);
  TestValueToSizeStream<S2>(S2::kUpper, S2::kMaxWireBytes);
  TestValueToSizeStream<S2>(S2::kLower, S2::kMaxWireBytes);

  AssertNumericLimitsMatchBounds<S2>();
}

void test_SignedNearInt32Limit() {
  TestRoundTripBuffer(S3{2140897380});
  TestRoundTripBuffer(S3{-2140897380});
  TestRoundTripBuffer(S3{2100000000});
  TestRoundTripBuffer(S3{-2100000000});
  TestRoundTripBuffer(S3{S3::kUpper});
  TestRoundTripBuffer(S3{S3::kLower});

  TestValueToSize<S3>(S3::kUpper, S3::kMaxWireBytes);
  TestValueToSize<S3>(S3::kLower, S3::kMaxWireBytes);
  TestValueToSizeStream<S3>(2100000000, S3::kMaxWireBytes);
  TestMaxWireSize<S3>();

  AssertUpperExceeds<S3, static_cast<std::int32_t>(2000000000)>();
  AssertNumericLimitsMatchBounds<S3>();
}

void test_SignedFourTier() {
  TestRoundTripBuffer(S4{124});
  TestRoundTripBuffer(S4{125});
  TestRoundTripBuffer(S4{126});
  TestRoundTripBuffer(S4{127});
  TestRoundTripBuffer(S4{-124});
  TestRoundTripBuffer(S4{-125});
  TestRoundTripBuffer(S4{-126});
  TestRoundTripBuffer(S4{-127});
  TestRoundTripBuffer(S4{2147483648LL});
  TestRoundTripBuffer(S4{-2147483648LL});
  TestRoundTripBuffer(S4{S4::kUpper});
  TestRoundTripBuffer(S4{S4::kLower});

  TestWireSizeTransition<S4>(124, 125, 1, 2);
  TestWireSizeTransition<S4>(-124, -125, 1, 2);
  TestWireSizeTransition<S4>(125, 126, 2, 4);
  TestWireSizeTransition<S4>(-125, -126, 2, 4);
  TestWireSizeTransition<S4>(126, 127, 4, 8);
  TestWireSizeTransition<S4>(-126, -127, 4, 8);
  TestMaxWireSize<S4>();

  TestValueToSize<S4>(124, 1);
  TestValueToSize<S4>(125, 2);
  TestValueToSize<S4>(126, 4);
  TestValueToSize<S4>(127, 8);
  TestValueToSizeStream<S4>(2147483648LL, 8);
  TestValueToSizeStream<S4>(S4::kUpper, S4::kMaxWireBytes);

  AssertNumericLimitsMatchBounds<S4>();
}

void test_CrossSignedUnsignedCompare() {
  TEST_ASSERT(S1{-1} < T1{1});
  TEST_ASSERT(T1{1} > S1{-1});
  TEST_ASSERT(S1{-1} < T4{100});
  TEST_ASSERT(S1{0} == T1{0});
  TEST_ASSERT(S1{100} == T4{100});
  TEST_ASSERT(S1{100} < T4{101});
  TEST_ASSERT(S4{-1} < T6{1});
  TEST_ASSERT(T6{1} > S4{-1});
  TEST_ASSERT(S4{-1} < T6{1000});
}

void test_Range() {
  TestRange<T1>();
  TestRange<T2>();
  TestRange<T4>();
}

using PackedCompat = TieredInt<std::uint8_t, 250, 1514, 1049834>;

void test_ArithmeticAddAssignAndPlus() {
  // += ValueType within the first tier.
  PackedCompat a{10};
  a += typename PackedCompat::ValueType{5};
  TEST_ASSERT_EQUAL(static_cast<PackedCompat::ValueType>(15),
                    static_cast<PackedCompat::ValueType>(a));

  // Tier boundary transitions via +=.
  PackedCompat b{249};
  b += typename PackedCompat::ValueType{1};
  TEST_ASSERT_EQUAL(static_cast<PackedCompat::ValueType>(250),
                    static_cast<PackedCompat::ValueType>(b));
  b += typename PackedCompat::ValueType{1};
  TEST_ASSERT_EQUAL(static_cast<PackedCompat::ValueType>(251),
                    static_cast<PackedCompat::ValueType>(b));

  PackedCompat c{1513};
  c += typename PackedCompat::ValueType{1};
  TEST_ASSERT_EQUAL(static_cast<PackedCompat::ValueType>(1514),
                    static_cast<PackedCompat::ValueType>(c));
  c += typename PackedCompat::ValueType{1};
  TEST_ASSERT_EQUAL(static_cast<PackedCompat::ValueType>(1515),
                    static_cast<PackedCompat::ValueType>(c));

  PackedCompat d{1049833};
  d += typename PackedCompat::ValueType{1};
  TEST_ASSERT_EQUAL(static_cast<PackedCompat::ValueType>(1049834),
                    static_cast<PackedCompat::ValueType>(d));
  d += typename PackedCompat::ValueType{1};
  TEST_ASSERT_EQUAL(static_cast<PackedCompat::ValueType>(1049835),
                    static_cast<PackedCompat::ValueType>(d));

  // += another TieredInt (converts through ValueType).
  PackedCompat e{100};
  PackedCompat f{25};
  e += f;
  TEST_ASSERT_EQUAL(static_cast<PackedCompat::ValueType>(125),
                    static_cast<PackedCompat::ValueType>(e));
  TEST_ASSERT_EQUAL(static_cast<PackedCompat::ValueType>(25),
                    static_cast<PackedCompat::ValueType>(f));

  // operator+ does not mutate lhs.
  PackedCompat g{40};
  PackedCompat const h = g + typename PackedCompat::ValueType{2};
  TEST_ASSERT_EQUAL(static_cast<PackedCompat::ValueType>(40),
                    static_cast<PackedCompat::ValueType>(g));
  TEST_ASSERT_EQUAL(static_cast<PackedCompat::ValueType>(42),
                    static_cast<PackedCompat::ValueType>(h));

  // Exact bounds.
  PackedCompat upper{PackedCompat::kUpper};
  TEST_ASSERT_EQUAL(PackedCompat::kUpper,
                    static_cast<PackedCompat::ValueType>(
                        upper + typename PackedCompat::ValueType{0}));
  PackedCompat lower{PackedCompat::kLower};
  TEST_ASSERT_EQUAL(PackedCompat::kLower,
                    static_cast<PackedCompat::ValueType>(
                        lower + typename PackedCompat::ValueType{0}));

  // Signed positive and negative addition.
  S2 sp{5};
  sp += typename S2::ValueType{3};
  TEST_ASSERT_EQUAL(static_cast<S2::ValueType>(8),
                    static_cast<S2::ValueType>(sp));
  S2 sn{-5};
  sn += typename S2::ValueType{-3};
  TEST_ASSERT_EQUAL(static_cast<S2::ValueType>(-8),
                    static_cast<S2::ValueType>(sn));
  TEST_ASSERT_EQUAL(static_cast<S2::ValueType>(S2::kUpper),
                    static_cast<S2::ValueType>(S2{S2::kUpper} +
                                              typename S2::ValueType{0}));
  TEST_ASSERT_EQUAL(static_cast<S2::ValueType>(S2::kLower),
                    static_cast<S2::ValueType>(S2{S2::kLower} +
                                              typename S2::ValueType{0}));
}

}  // namespace ae::test_tiered_int

int test_tiered_int() {
  UNITY_BEGIN();
  RUN_TEST(ae::test_tiered_int::test_TwoTier);
  RUN_TEST(ae::test_tiered_int::test_ConstexprConstruction);
  RUN_TEST(ae::test_tiered_int::test_ThreeTier);
  RUN_TEST(ae::test_tiered_int::test_FourTier);
  RUN_TEST(ae::test_tiered_int::test_WireCellTwoAndFour);
  RUN_TEST(ae::test_tiered_int::test_ExtensionHeaderTier);
  RUN_TEST(ae::test_tiered_int::test_RepresentativeWireEncoding);
  RUN_TEST(ae::test_tiered_int::test_WireSizeTransitions);
  RUN_TEST(ae::test_tiered_int::test_SignedWireCell);
  RUN_TEST(ae::test_tiered_int::test_SignedThreeTier);
  RUN_TEST(ae::test_tiered_int::test_SignedNearInt32Limit);
  RUN_TEST(ae::test_tiered_int::test_SignedFourTier);
  RUN_TEST(ae::test_tiered_int::test_CrossSignedUnsignedCompare);
  RUN_TEST(ae::test_tiered_int::test_Range);
  RUN_TEST(ae::test_tiered_int::test_ArithmeticAddAssignAndPlus);
  return UNITY_END();
}
