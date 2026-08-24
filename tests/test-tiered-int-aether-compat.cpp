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

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>

#include <ae-numeric/tiered_int.h>

#include "legacy/aether_tiered_codec.h"

namespace ae::test_tiered_int_aether_compat {

using NewT = TieredInt<std::uint8_t, 250, 1514, 1049834>;
using legacy::kLegacyInclusiveMax;

static_assert(NewT::kMaxWireBytes == 8);
static_assert(sizeof(NewT::ValueType) >= 8);
static_assert(static_cast<std::uint64_t>(NewT::kUpper) >= kLegacyInclusiveMax);

struct Encoded {
  std::array<std::uint8_t, 8> bytes{};
  std::size_t size = 0;
};

Encoded EncodeNew(std::uint64_t value) {
  Encoded e;
  NewT const t{value};
  e.size = t.Serialize(e.bytes.data());
  return e;
}

Encoded EncodeLegacy(std::uint64_t value) {
  Encoded e;
  e.size = legacy::Encode(value, e.bytes.data(), e.bytes.size());
  return e;
}

bool BytesEqual(Encoded const& a, Encoded const& b) {
  return a.size == b.size &&
         std::memcmp(a.bytes.data(), b.bytes.data(), a.size) == 0;
}

bool CompatOk(std::uint64_t value, std::uint64_t& first_bad,
              std::uint64_t& checksum) {
  Encoded const leg = EncodeLegacy(value);
  Encoded const neu = EncodeNew(value);
  checksum = checksum * 1315423911ull + value + leg.size + neu.size +
             leg.bytes[0] + neu.bytes[0];
  if (!BytesEqual(leg, neu)) {
    first_bad = value;
    return false;
  }

  std::uint64_t legacy_decoded = 0;
  std::size_t legacy_consumed = 0;
  if (!legacy::Decode(leg.bytes.data(), leg.size, legacy_decoded,
                      legacy_consumed) ||
      legacy_decoded != value || legacy_consumed != leg.size) {
    first_bad = value;
    return false;
  }

  NewT new_decoded{};
  std::size_t const new_consumed =
      new_decoded.Deserialize(neu.bytes.data(), neu.size);
  if (new_consumed != neu.size ||
      static_cast<std::uint64_t>(new_decoded) != value) {
    first_bad = value;
    return false;
  }

  NewT cross_new{};
  std::size_t const cross_new_n =
      cross_new.Deserialize(leg.bytes.data(), leg.size);
  if (cross_new_n != leg.size ||
      static_cast<std::uint64_t>(cross_new) != value) {
    first_bad = value;
    return false;
  }

  std::uint64_t cross_legacy = 0;
  std::size_t cross_legacy_n = 0;
  if (!legacy::Decode(neu.bytes.data(), neu.size, cross_legacy,
                      cross_legacy_n) ||
      cross_legacy != value || cross_legacy_n != neu.size) {
    first_bad = value;
    return false;
  }
  checksum += legacy_decoded + cross_legacy +
              static_cast<std::uint64_t>(new_decoded) +
              static_cast<std::uint64_t>(cross_new);
  return true;
}

void CheckCompatValue(std::uint64_t value) {
  std::uint64_t bad = 0;
  std::uint64_t checksum = 0;
  if (!CompatOk(value, bad, checksum)) {
    std::cerr << "compat failure at value=" << value << '\n';
    TEST_FAIL_MESSAGE("legacy/new TieredInt wire mismatch");
  }
  // Keep checksum live for optimizers.
  if (checksum == 0xDEADBEEFCAFEull) {
    std::cerr << "unexpected checksum\n";
  }
}

void test_LegacyVsNewMaxDocumented() {
  TEST_ASSERT_EQUAL_UINT64(1099512612074ull, kLegacyInclusiveMax);
  TEST_ASSERT_EQUAL_UINT64(1099512612075ull, legacy::kLegacyExclusiveUpper);
  // New type can represent values above the legacy domain; that is OK.
  TEST_ASSERT(static_cast<std::uint64_t>(NewT::kUpper) > kLegacyInclusiveMax);
  TEST_ASSERT_EQUAL_UINT64(1099512677610ull,
                           static_cast<std::uint64_t>(NewT::kUpper));
  TEST_ASSERT_EQUAL(8, static_cast<int>(NewT::kMaxWireBytes));
}

void test_BoundaryValues() {
  constexpr std::uint64_t kBoundaries[] = {
      0,
      1,
      249,
      250,
      251,
      252,
      1513,
      1514,
      1515,
      1516,
      1049833,
      1049834,
      1049835,
      1049836,
      kLegacyInclusiveMax - 1U,
      kLegacyInclusiveMax,
  };
  for (std::uint64_t v : kBoundaries) {
    CheckCompatValue(v);
  }
}

void test_GoldenVectorsIndependent() {
  struct Vec {
    std::uint64_t value;
    std::uint8_t const* bytes;
    std::size_t size;
  };
  static constexpr std::uint8_t k250[] = {0xfa};
  static constexpr std::uint8_t k251[] = {0xfb, 0x00};
  static constexpr std::uint8_t k1514[] = {0xff, 0xef};
  static constexpr std::uint8_t k1515[] = {0xff, 0xf0, 0x00, 0x00};
  static constexpr std::uint8_t k1049834[] = {0xff, 0xff, 0xff, 0xfe};
  static constexpr std::uint8_t k1049835[] = {0xff, 0xff, 0x00, 0xff,
                                              0x00, 0x00, 0x00, 0x00};
  // Expected bytes are literals (LE), independent of either codec.
  static constexpr Vec kVecs[] = {
      {250, k250, sizeof(k250)},
      {251, k251, sizeof(k251)},
      {1514, k1514, sizeof(k1514)},
      {1515, k1515, sizeof(k1515)},
      {1049834, k1049834, sizeof(k1049834)},
      {1049835, k1049835, sizeof(k1049835)},
  };

  for (Vec const& vec : kVecs) {
    Encoded const leg = EncodeLegacy(vec.value);
    Encoded const neu = EncodeNew(vec.value);
    TEST_ASSERT_EQUAL(vec.size, leg.size);
    TEST_ASSERT_EQUAL(vec.size, neu.size);
    for (std::size_t i = 0; i < vec.size; ++i) {
      TEST_ASSERT_EQUAL_HEX8(vec.bytes[i], leg.bytes[i]);
      TEST_ASSERT_EQUAL_HEX8(vec.bytes[i], neu.bytes[i]);
    }
  }
}

void test_ExhaustiveTwoMillion() {
  constexpr std::uint64_t kLimit = 2000000ull;
  std::uint64_t bad = 0;
  std::uint64_t checksum = 0;
  for (std::uint64_t v = 0; v <= kLimit; ++v) {
    if (!CompatOk(v, bad, checksum)) {
      std::cerr << "exhaustive first failure at " << bad << '\n';
      TEST_FAIL_MESSAGE("exhaustive legacy/new mismatch");
      return;
    }
  }
  // Anti-DCE: exhaustive work must touch checksum.
  TEST_ASSERT(checksum != 0);
  std::cout << "[compat] exhaustive 0..2000000 checksum=" << checksum << '\n';
}

std::uint64_t SplitMix64(std::uint64_t& state) {
  std::uint64_t z = (state += 0x9e3779b97f4a7c15ull);
  z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ull;
  z = (z ^ (z >> 27)) * 0x94d049bb133111ebull;
  return z ^ (z >> 31);
}

std::uint64_t ClampLegacy(std::uint64_t v) {
  return v % (kLegacyInclusiveMax + 1U);
}

void test_RandomOneMillion() {
  constexpr std::size_t kCount = 1000000;
  std::uint64_t rng = 0xAE7E4250ull;  // fixed seed
  std::uint64_t bad = 0;
  std::uint64_t checksum = 0;

  constexpr std::uint64_t kNear[] = {
      0,         250,        251,        1514,       1515,
      1049834,   1049835,    65535ull,   65536ull,   4294967295ull,
      4294967296ull, kLegacyInclusiveMax,
  };

  std::size_t done = 0;
  auto check = [&](std::uint64_t v) {
    if (!CompatOk(v, bad, checksum)) {
      std::cerr << "random/property first failure at " << bad << '\n';
      TEST_FAIL_MESSAGE("random legacy/new mismatch");
      return false;
    }
    ++done;
    return true;
  };

  for (std::uint64_t center : kNear) {
    for (int d = -32; d <= 32; ++d) {
      std::int64_t const signed_v =
          static_cast<std::int64_t>(center) + static_cast<std::int64_t>(d);
      if (signed_v < 0) {
        continue;
      }
      auto const v = static_cast<std::uint64_t>(signed_v);
      if (v > kLegacyInclusiveMax) {
        continue;
      }
      if (!check(v)) {
        return;
      }
    }
  }

  for (unsigned shift = 0; shift < 64; ++shift) {
    std::uint64_t const p = (shift >= 63) ? (1ull << 62) : (1ull << shift);
    for (std::int64_t d = -3; d <= 3; ++d) {
      std::int64_t const signed_v = static_cast<std::int64_t>(p) + d;
      if (signed_v < 0) {
        continue;
      }
      auto const v = static_cast<std::uint64_t>(signed_v);
      if (v > kLegacyInclusiveMax) {
        continue;
      }
      if (!check(v)) {
        return;
      }
    }
  }

  while (done < kCount) {
    if (!check(ClampLegacy(SplitMix64(rng)))) {
      return;
    }
  }
  TEST_ASSERT(checksum != 0);
  std::cout << "[compat] random cases=" << done << " checksum=" << checksum
            << '\n';
}

void test_TruncatedBuffersDetected() {
  constexpr std::uint64_t kSamples[] = {0, 251, 1515, 1049835,
                                        kLegacyInclusiveMax};
  for (std::uint64_t v : kSamples) {
    Encoded const neu = EncodeNew(v);
    for (std::size_t trunc = 0; trunc < neu.size; ++trunc) {
      std::size_t const need =
          NewT::WireBytesNeeded(neu.bytes.data(), trunc);
      TEST_ASSERT_EQUAL(0, need);
    }
    TEST_ASSERT_EQUAL(neu.size,
                      NewT::WireBytesNeeded(neu.bytes.data(), neu.size));
  }
}

void test_NoThrowSerializeDeserialize() {
  std::uint8_t buf[8] = {};
  NewT t{1049835ull};
  std::size_t const n = t.Serialize(buf);
  NewT u{};
  (void)u.Deserialize(buf, n);
  TEST_ASSERT_EQUAL(8, static_cast<int>(n));
}

void BenchOne(char const* label, std::vector<std::uint64_t> const& values) {
  std::uint8_t buf[8] = {};
  auto const t0 = std::chrono::steady_clock::now();
  std::uint64_t sink = 0;
  for (int rep = 0; rep < 5; ++rep) {
    for (std::uint64_t v : values) {
      sink += legacy::Encode(v, buf, 8);
    }
  }
  auto const t1 = std::chrono::steady_clock::now();
  for (int rep = 0; rep < 5; ++rep) {
    for (std::uint64_t v : values) {
      sink += EncodeNew(v).size;
    }
  }
  auto const t2 = std::chrono::steady_clock::now();

  // Decode pass
  std::vector<Encoded> encoded;
  encoded.reserve(values.size());
  for (std::uint64_t v : values) {
    encoded.push_back(EncodeNew(v));
  }
  auto const t3 = std::chrono::steady_clock::now();
  for (int rep = 0; rep < 5; ++rep) {
    for (Encoded const& e : encoded) {
      std::uint64_t out = 0;
      std::size_t n = 0;
      legacy::Decode(e.bytes.data(), e.size, out, n);
      sink += out + n;
    }
  }
  auto const t4 = std::chrono::steady_clock::now();
  for (int rep = 0; rep < 5; ++rep) {
    for (Encoded const& e : encoded) {
      NewT u{};
      sink += u.Deserialize(e.bytes.data(), e.size);
      sink += static_cast<std::uint64_t>(u);
    }
  }
  auto const t5 = std::chrono::steady_clock::now();

  auto ms = [](auto a, auto b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
  };
  double const leg_enc = ms(t0, t1);
  double const new_enc = ms(t1, t2);
  double const leg_dec = ms(t3, t4);
  double const new_dec = ms(t4, t5);
  auto rel = [](double a, double b) {
    return (b <= 0.0) ? 0.0 : ((a - b) / b) * 100.0;
  };

  std::cout << "[bench] " << label << " values=" << values.size()
            << " legacy_enc_ms=" << leg_enc << " new_enc_ms=" << new_enc
            << " enc_rel%_vs_legacy=" << rel(new_enc, leg_enc)
            << " legacy_dec_ms=" << leg_dec << " new_dec_ms=" << new_dec
            << " dec_rel%_vs_legacy=" << rel(new_dec, leg_dec)
            << " sink=" << sink << '\n';
}

void test_BenchmarkLegacyVsNew() {
  auto make_range = [](std::uint64_t lo, std::uint64_t hi, std::size_t n) {
    std::vector<std::uint64_t> out;
    out.reserve(n);
    if (n == 0) {
      return out;
    }
    if (hi <= lo) {
      out.assign(n, lo);
      return out;
    }
    std::uint64_t const span = hi - lo;
    for (std::size_t i = 0; i < n; ++i) {
      out.push_back(lo + (span * i) / (n - 1));
    }
    return out;
  };

  BenchOne("1-byte", make_range(0, 250, 20000));
  BenchOne("2-byte", make_range(251, 1514, 20000));
  BenchOne("4-byte", make_range(1515, 1049834, 20000));
  BenchOne("8-byte", make_range(1049835, kLegacyInclusiveMax, 20000));

  std::vector<std::uint64_t> mixed;
  mixed.reserve(40000);
  auto a = make_range(0, 250, 10000);
  auto b = make_range(251, 1514, 10000);
  auto c = make_range(1515, 1049834, 10000);
  auto d = make_range(1049835, kLegacyInclusiveMax, 10000);
  mixed.insert(mixed.end(), a.begin(), a.end());
  mixed.insert(mixed.end(), b.begin(), b.end());
  mixed.insert(mixed.end(), c.begin(), c.end());
  mixed.insert(mixed.end(), d.begin(), d.end());
  BenchOne("mixed", mixed);
}

}  // namespace ae::test_tiered_int_aether_compat

int test_tiered_int_aether_compat() {
  UNITY_BEGIN();
  RUN_TEST(ae::test_tiered_int_aether_compat::test_LegacyVsNewMaxDocumented);
  RUN_TEST(ae::test_tiered_int_aether_compat::test_BoundaryValues);
  RUN_TEST(ae::test_tiered_int_aether_compat::test_GoldenVectorsIndependent);
  RUN_TEST(ae::test_tiered_int_aether_compat::test_TruncatedBuffersDetected);
  RUN_TEST(ae::test_tiered_int_aether_compat::test_NoThrowSerializeDeserialize);
  RUN_TEST(ae::test_tiered_int_aether_compat::test_ExhaustiveTwoMillion);
  RUN_TEST(ae::test_tiered_int_aether_compat::test_RandomOneMillion);
  RUN_TEST(ae::test_tiered_int_aether_compat::test_BenchmarkLegacyVsNew);
  return UNITY_END();
}
