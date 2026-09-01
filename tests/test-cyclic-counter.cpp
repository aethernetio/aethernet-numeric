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
#include <optional>

#include <ae-numeric/cyclic_counter.h>
#include <ae-numeric/wire_io.h>

namespace ae::test_cyclic_counter {

using U8_16 = CyclicCounter<std::uint8_t, std::uint16_t>;
using U8_32 = CyclicCounter<std::uint8_t, std::uint32_t>;
using U16_32 = CyclicCounter<std::uint16_t, std::uint32_t>;

void test_Size() {
  static_assert(sizeof(U8_16) == sizeof(std::uint16_t));
  static_assert(sizeof(U8_32) == sizeof(std::uint32_t));
  static_assert(sizeof(U16_32) == sizeof(std::uint32_t));
  static_assert(alignof(U8_32) == alignof(std::uint32_t));
  TEST_ASSERT_EQUAL(sizeof(std::uint32_t), sizeof(U8_32));
}

void test_DefaultAndBasic() {
  U8_32 c;
  TEST_ASSERT_EQUAL_UINT32(0u, c.Value());
  TEST_ASSERT_EQUAL_UINT8(0u, c.WireValue());

  c.Set(1000u);
  TEST_ASSERT_EQUAL_UINT32(1000u, c.Value());
  TEST_ASSERT_EQUAL_UINT8(232u, c.WireValue());

  c.Set(1001u);
  TEST_ASSERT_EQUAL_UINT8(233u, c.WireValue());
  c.Set(1005u);
  TEST_ASSERT_EQUAL_UINT8(237u, c.WireValue());
  c.Set(1023u);
  TEST_ASSERT_EQUAL_UINT8(255u, c.WireValue());
  c.Set(1024u);
  TEST_ASSERT_EQUAL_UINT8(0u, c.WireValue());

  U8_32 base{1000u};
  auto const r = base.TryRestore(233u);
  TEST_ASSERT_TRUE(r.has_value());
  TEST_ASSERT_EQUAL_UINT32(1001u, *r);
  TEST_ASSERT_EQUAL_UINT32(1000u, base.Value());
}

void test_LostValues() {
  U8_32 base{1001u};
  auto const r = base.TryRestore(237u);  // wire for 1005
  TEST_ASSERT_TRUE(r.has_value());
  TEST_ASSERT_EQUAL_UINT32(1005u, *r);
  TEST_ASSERT_EQUAL_UINT32(1001u, base.Value());
}

void test_ForwardWrap() {
  U8_32 base{1023u};
  TEST_ASSERT_EQUAL_UINT8(255u, base.WireValue());
  auto r0 = base.TryRestore(0u);
  TEST_ASSERT_TRUE(r0.has_value());
  TEST_ASSERT_EQUAL_UINT32(1024u, *r0);

  base.Set(1024u);
  auto r1 = base.TryRestore(1u);
  TEST_ASSERT_TRUE(r1.has_value());
  TEST_ASSERT_EQUAL_UINT32(1025u, *r1);
}

void test_Backward() {
  U8_32 base{1008u};
  TEST_ASSERT_EQUAL_UINT8(240u, base.WireValue());
  auto const r = base.TryRestore(235u);
  TEST_ASSERT_TRUE(r.has_value());
  TEST_ASSERT_EQUAL_UINT32(1003u, *r);
  TEST_ASSERT_TRUE(*r < base.Value());
}

void test_Advance() {
  U8_32 c{1005u};
  auto a = c.TryAdvance(238u);  // 1006
  TEST_ASSERT_TRUE(a.has_value());
  TEST_ASSERT_EQUAL_UINT32(1006u, *a);
  TEST_ASSERT_EQUAL_UINT32(1006u, c.Value());

  auto b = c.TryAdvance(235u);  // 1003
  TEST_ASSERT_TRUE(b.has_value());
  TEST_ASSERT_EQUAL_UINT32(1003u, *b);
  TEST_ASSERT_EQUAL_UINT32(1006u, c.Value());
}

void test_HalfRangeAmbiguousU8() {
  U8_32 c{0u};
  TEST_ASSERT_FALSE(c.TryRestore(128u).has_value());

  c.Set(128u);
  TEST_ASSERT_FALSE(c.TryRestore(0u).has_value());

  c.Set(1000u);  // wire 232
  std::uint8_t const amb =
      static_cast<std::uint8_t>(c.WireValue() + 128u);
  TEST_ASSERT_FALSE(c.TryRestore(amb).has_value());
}

void test_MaxLegalDistancesU8() {
  U8_32 c{1000u};
  auto fwd = c.TryRestore(static_cast<std::uint8_t>(232u + 127u));
  TEST_ASSERT_TRUE(fwd.has_value());
  TEST_ASSERT_EQUAL_UINT32(1000u + 127u, *fwd);

  auto back = c.TryRestore(static_cast<std::uint8_t>(232u - 127u));
  TEST_ASSERT_TRUE(back.has_value());
  TEST_ASSERT_EQUAL_UINT32(1000u - 127u, *back);
}

void test_MaxLegalDistancesU16() {
  U16_32 c{100000u};
  auto fwd = c.TryRestore(static_cast<std::uint16_t>(
      static_cast<std::uint16_t>(c.WireValue()) + 32767u));
  TEST_ASSERT_TRUE(fwd.has_value());
  TEST_ASSERT_EQUAL_UINT32(100000u + 32767u, *fwd);

  auto back = c.TryRestore(static_cast<std::uint16_t>(
      static_cast<std::uint16_t>(c.WireValue()) - 32767u));
  TEST_ASSERT_TRUE(back.has_value());
  TEST_ASSERT_EQUAL_UINT32(100000u - 32767u, *back);

  TEST_ASSERT_FALSE(c.TryRestore(static_cast<std::uint16_t>(
                                     c.WireValue() + 32768u))
                        .has_value());
}

void test_ValueTypeBoundaries() {
  U8_16 near0{0u};
  TEST_ASSERT_FALSE(near0.TryRestore(255u).has_value());
  TEST_ASSERT_TRUE(near0.TryRestore(0u).has_value());
  TEST_ASSERT_TRUE(near0.TryRestore(1u).has_value());
  TEST_ASSERT_EQUAL_UINT16(1u, *near0.TryRestore(1u));

  U8_16 near_max{std::numeric_limits<std::uint16_t>::max()};
  TEST_ASSERT_FALSE(near_max.TryRestore(static_cast<std::uint8_t>(
                                            near_max.WireValue() + 1u))
                        .has_value());
  auto back = near_max.TryRestore(static_cast<std::uint8_t>(
      near_max.WireValue() - 1u));
  TEST_ASSERT_TRUE(back.has_value());
  TEST_ASSERT_EQUAL_UINT16(
      static_cast<std::uint16_t>(std::numeric_limits<std::uint16_t>::max() - 1u),
      *back);

  U8_32 u32max{std::numeric_limits<std::uint32_t>::max()};
  TEST_ASSERT_FALSE(u32max.TryRestore(static_cast<std::uint8_t>(
                                          u32max.WireValue() + 1u))
                        .has_value());
}

void test_MultipleEpochs() {
  U8_32 c{65534u};
  TEST_ASSERT_EQUAL_UINT8(254u, c.WireValue());
  auto a = c.TryAdvance(255u);
  TEST_ASSERT_EQUAL_UINT32(65535u, *a);
  TEST_ASSERT_EQUAL_UINT32(65535u, c.Value());
  auto b = c.TryAdvance(0u);
  TEST_ASSERT_EQUAL_UINT32(65536u, *b);
  TEST_ASSERT_EQUAL_UINT32(65536u, c.Value());
  auto d = c.TryAdvance(1u);
  TEST_ASSERT_EQUAL_UINT32(65537u, *d);
  TEST_ASSERT_EQUAL_UINT32(65537u, c.Value());
}

void test_OldServer() {
  U8_32 c{65540u};
  auto r = c.TryAdvance(static_cast<std::uint8_t>(65535u & 0xFFu));
  TEST_ASSERT_TRUE(r.has_value());
  TEST_ASSERT_EQUAL_UINT32(65535u, *r);
  TEST_ASSERT_EQUAL_UINT32(65540u, c.Value());
}

void test_ExhaustiveU8() {
  std::uint32_t const bases[] = {
      0u, 1u, 127u, 128u, 255u, 256u, 1000u, 1023u, 1024u,
      65534u, 65535u, 65536u, 0xFFFFFEu, 0xFFFFFFu, 0x1000000u,
      std::numeric_limits<std::uint32_t>::max() - 200u,
      std::numeric_limits<std::uint32_t>::max() - 1u,
      std::numeric_limits<std::uint32_t>::max()};

  for (std::uint32_t base : bases) {
    U8_32 c{base};
    std::uint8_t const cur = c.WireValue();
    for (int w = 0; w < 256; ++w) {
      auto const wire = static_cast<std::uint8_t>(w);
      auto const restored = c.TryRestore(wire);
      std::uint8_t const forward =
          static_cast<std::uint8_t>(wire - cur);
      if (forward == 128u) {
        TEST_ASSERT_FALSE(restored.has_value());
        continue;
      }
      if (forward < 128u) {
        if (base > std::numeric_limits<std::uint32_t>::max() - forward) {
          TEST_ASSERT_FALSE(restored.has_value());
        } else {
          TEST_ASSERT_TRUE(restored.has_value());
          TEST_ASSERT_EQUAL_UINT32(base + forward, *restored);
          TEST_ASSERT_EQUAL_UINT8(
              wire, static_cast<std::uint8_t>(*restored & 0xFFu));
        }
      } else {
        std::uint32_t const backward =
            256u - static_cast<std::uint32_t>(forward);
        if (base < backward) {
          TEST_ASSERT_FALSE(restored.has_value());
        } else {
          TEST_ASSERT_TRUE(restored.has_value());
          TEST_ASSERT_EQUAL_UINT32(base - backward, *restored);
          TEST_ASSERT_EQUAL_UINT8(
              wire, static_cast<std::uint8_t>(*restored & 0xFFu));
        }
      }
    }

    // Identity when in range: Restore(WireValue(value)) == value for
    // neighbors within ±127 of base (same high bits).
    for (int d = -127; d <= 127; ++d) {
      if (d < 0 && base < static_cast<std::uint32_t>(-d)) {
        continue;
      }
      if (d > 0 &&
          base > std::numeric_limits<std::uint32_t>::max() -
                     static_cast<std::uint32_t>(d)) {
        continue;
      }
      std::uint32_t const target =
          d >= 0 ? base + static_cast<std::uint32_t>(d)
                 : base - static_cast<std::uint32_t>(-d);
      U8_32 probe{target};
      auto const back = c.TryRestore(probe.WireValue());
      TEST_ASSERT_TRUE(back.has_value());
      TEST_ASSERT_EQUAL_UINT32(target, *back);
    }
  }
}

void test_WireProjectionAndContextualDecode() {
  static_assert(!WireSerializable<U8_32>,
                "CyclicCounter must not be WireSerializable");
  static_assert(!WireSerializable<U8_16>);
  static_assert(!WireSerializable<U16_32>);

  U8_32 c{1001u};
  std::uint8_t buf[4] = {};
  std::size_t const n =
      wire_traits<std::uint8_t>::Serialize(c.WireValue(), buf);
  TEST_ASSERT_EQUAL(1u, n);
  TEST_ASSERT_EQUAL_UINT8(233u, buf[0]);

  // Same wire again: restore without advancing.
  auto const same = c.TryDeserializeAndRestore(buf, n);
  TEST_ASSERT_TRUE(same.ok());
  TEST_ASSERT_EQUAL_INT(static_cast<int>(CyclicDecodeStatus::Ok),
                        static_cast<int>(same.status));
  TEST_ASSERT_EQUAL_UINT32(1001u, same.value);
  TEST_ASSERT_EQUAL(1u, same.bytes_read);
  TEST_ASSERT_EQUAL_UINT32(1001u, c.Value());

  buf[0] = 237u;
  auto const restored = c.TryDeserializeAndRestore(buf, 1);
  TEST_ASSERT_TRUE(restored.ok());
  TEST_ASSERT_EQUAL_UINT32(1005u, restored.value);
  TEST_ASSERT_EQUAL_UINT32(1001u, c.Value());

  auto const advanced = c.TryDeserializeAndAdvance(buf, 1);
  TEST_ASSERT_TRUE(advanced.ok());
  TEST_ASSERT_EQUAL_UINT32(1005u, advanced.value);
  TEST_ASSERT_EQUAL_UINT32(1005u, c.Value());
}

void test_ContextualRestoreAdvanceOldWrapAmbiguous() {
  U8_32 counter{1001u};
  std::uint8_t wire237 = 237u;
  auto r = counter.TryDeserializeAndRestore(&wire237, 1);
  TEST_ASSERT_TRUE(r.ok());
  TEST_ASSERT_EQUAL_UINT32(1005u, r.value);
  TEST_ASSERT_EQUAL_UINT32(1001u, counter.Value());

  counter.Set(1001u);
  auto a = counter.TryDeserializeAndAdvance(&wire237, 1);
  TEST_ASSERT_EQUAL_UINT32(1005u, a.value);
  TEST_ASSERT_EQUAL_UINT32(1005u, counter.Value());

  counter.Set(1008u);
  std::uint8_t old_wire = 235u;  // 1003
  auto old = counter.TryDeserializeAndRestore(&old_wire, 1);
  TEST_ASSERT_EQUAL_UINT32(1003u, old.value);
  TEST_ASSERT_EQUAL_UINT32(1008u, counter.Value());
  auto old_adv = counter.TryDeserializeAndAdvance(&old_wire, 1);
  TEST_ASSERT_EQUAL_UINT32(1003u, old_adv.value);
  TEST_ASSERT_EQUAL_UINT32(1008u, counter.Value());

  counter.Set(1023u);
  std::uint8_t wrap = 0u;
  auto w = counter.TryDeserializeAndRestore(&wrap, 1);
  TEST_ASSERT_EQUAL_UINT32(1024u, w.value);

  counter.Set(0u);
  std::uint8_t amb = 128u;
  auto bad = counter.TryDeserializeAndRestore(&amb, 1);
  TEST_ASSERT_FALSE(bad.ok());
  TEST_ASSERT_EQUAL_INT(static_cast<int>(CyclicDecodeStatus::Ambiguous),
                        static_cast<int>(bad.status));
  TEST_ASSERT_EQUAL(1u, bad.bytes_read);
  TEST_ASSERT_EQUAL_UINT32(0u, counter.Value());
}

void test_TruncatedInputU16() {
  U16_32 c{1000u};
  std::uint8_t one_byte[1] = {0x01};
  auto r = c.TryDeserializeAndRestore(one_byte, 1);
  TEST_ASSERT_FALSE(r.ok());
  TEST_ASSERT_EQUAL_INT(static_cast<int>(CyclicDecodeStatus::TruncatedInput),
                        static_cast<int>(r.status));
  TEST_ASSERT_EQUAL(0u, r.bytes_read);

  auto a = c.TryDeserializeAndAdvance(one_byte, 1);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(CyclicDecodeStatus::TruncatedInput),
                        static_cast<int>(a.status));
  TEST_ASSERT_EQUAL_UINT32(1000u, c.Value());
}

void test_DecodeOutOfRangeNearBounds() {
  U8_16 near0{0u};
  std::uint8_t back = 255u;
  auto u = near0.TryDeserializeAndRestore(&back, 1);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(CyclicDecodeStatus::OutOfRange),
                        static_cast<int>(u.status));
  TEST_ASSERT_EQUAL(1u, u.bytes_read);

  U8_16 near_max{std::numeric_limits<std::uint16_t>::max()};
  std::uint8_t fwd = static_cast<std::uint8_t>(near_max.WireValue() + 1u);
  auto o = near_max.TryDeserializeAndRestore(&fwd, 1);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(CyclicDecodeStatus::OutOfRange),
                        static_cast<int>(o.status));
}

void test_CompareWire() {
  TEST_ASSERT_TRUE(CompareWire(std::uint8_t{10}, std::uint8_t{10}) ==
                   WireOrder::Same);
  TEST_ASSERT_TRUE(CompareWire(std::uint8_t{10}, std::uint8_t{20}) ==
                   WireOrder::Newer);
  TEST_ASSERT_TRUE(CompareWire(std::uint8_t{20}, std::uint8_t{10}) ==
                   WireOrder::Older);
  TEST_ASSERT_TRUE(CompareWire(std::uint8_t{0}, std::uint8_t{128}) ==
                   WireOrder::Ambiguous);
}

void test_IntegralLikeArithmetic() {
  U8_32 c{100u};
  TEST_ASSERT_EQUAL_UINT32(100u, static_cast<std::uint32_t>(c));
  TEST_ASSERT_EQUAL_UINT8(c.WireValue(), c.WireValue());

  c += 5u;
  TEST_ASSERT_EQUAL_UINT32(105u, c.Value());
  TEST_ASSERT_EQUAL_UINT8(105u & 255u, c.WireValue());

  c -= 3u;
  TEST_ASSERT_EQUAL_UINT32(102u, c.Value());

  U8_32 sum = c + 8u;
  TEST_ASSERT_EQUAL_UINT32(110u, sum.Value());
  TEST_ASSERT_EQUAL_UINT32(102u, c.Value());

  U8_32 sum2 = 10u + U8_32{20u};
  TEST_ASSERT_EQUAL_UINT32(30u, sum2.Value());

  U8_32 diff = c - 2u;
  TEST_ASSERT_EQUAL_UINT32(100u, diff.Value());

  U8_32 dec = c;
  TEST_ASSERT_EQUAL_UINT32(102u, dec.Value());
  auto post_dec = dec--;
  TEST_ASSERT_EQUAL_UINT32(102u, post_dec.Value());
  TEST_ASSERT_EQUAL_UINT32(101u, dec.Value());

  static_assert(sizeof(U8_32) == sizeof(std::uint32_t));
}

void test_IncrementAndCompare() {
  U8_32 a{10u};
  U8_32 b = a;
  ++a;
  TEST_ASSERT_EQUAL_UINT32(11u, a.Value());
  TEST_ASSERT_TRUE(a > b);
  TEST_ASSERT_TRUE(b < a);
  auto post = a++;
  TEST_ASSERT_EQUAL_UINT32(11u, post.Value());
  TEST_ASSERT_EQUAL_UINT32(12u, a.Value());
}

}  // namespace ae::test_cyclic_counter

int test_cyclic_counter() {
  UNITY_BEGIN();
  RUN_TEST(ae::test_cyclic_counter::test_Size);
  RUN_TEST(ae::test_cyclic_counter::test_DefaultAndBasic);
  RUN_TEST(ae::test_cyclic_counter::test_LostValues);
  RUN_TEST(ae::test_cyclic_counter::test_ForwardWrap);
  RUN_TEST(ae::test_cyclic_counter::test_Backward);
  RUN_TEST(ae::test_cyclic_counter::test_Advance);
  RUN_TEST(ae::test_cyclic_counter::test_HalfRangeAmbiguousU8);
  RUN_TEST(ae::test_cyclic_counter::test_MaxLegalDistancesU8);
  RUN_TEST(ae::test_cyclic_counter::test_MaxLegalDistancesU16);
  RUN_TEST(ae::test_cyclic_counter::test_ValueTypeBoundaries);
  RUN_TEST(ae::test_cyclic_counter::test_MultipleEpochs);
  RUN_TEST(ae::test_cyclic_counter::test_OldServer);
  RUN_TEST(ae::test_cyclic_counter::test_ExhaustiveU8);
  RUN_TEST(ae::test_cyclic_counter::test_WireProjectionAndContextualDecode);
  RUN_TEST(
      ae::test_cyclic_counter::test_ContextualRestoreAdvanceOldWrapAmbiguous);
  RUN_TEST(ae::test_cyclic_counter::test_TruncatedInputU16);
  RUN_TEST(ae::test_cyclic_counter::test_DecodeOutOfRangeNearBounds);
  RUN_TEST(ae::test_cyclic_counter::test_CompareWire);
  RUN_TEST(ae::test_cyclic_counter::test_IntegralLikeArithmetic);
  RUN_TEST(ae::test_cyclic_counter::test_IncrementAndCompare);
  return UNITY_END();
}
