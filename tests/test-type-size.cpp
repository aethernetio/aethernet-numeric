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

#include "numeric/exponential.h"
#include "numeric/exponential_wire_io.h"
#include "numeric/fixed_point.h"
#include "numeric/tiered_int.h"
#include "numeric/wire_io.h"

namespace ae::test_type_size {

using F8 = FixedPoint<std::uint8_t, 100.0>;
static_assert(sizeof(F8) == sizeof(std::uint8_t));
static_assert(alignof(F8) == alignof(std::uint8_t));

using F16 = FixedPoint<std::uint16_t, 100.0>;
static_assert(sizeof(F16) == sizeof(std::uint16_t));

using T = TieredInt<std::uint8_t, 254>;
static_assert(sizeof(T) == sizeof(T::ValueType));
static_assert(T::kMaxWireBytes <= sizeof(T::ValueType));

using Raw = TieredInt<std::uint8_t, 254>;
using Compact = FixedPoint<Raw, 60.0>;
static_assert(sizeof(Compact) == sizeof(Raw));
static_assert(alignof(Compact) == alignof(Raw));

using Runtime = FixedPoint<std::uint32_t, 60.0>;
using Wire = TieredInt<std::uint8_t, 254>;
using E = Exponential<Runtime, Wire, 0.001, 60.0>;

static_assert(sizeof(E) == sizeof(Wire));
static_assert(alignof(E) == alignof(Wire));

static_assert(MaxWireBytes<Raw>() == Raw::kMaxWireBytes);
static_assert(MaxWireBytes<Compact>() == MaxWireBytes<Raw>());
static_assert(MaxWireBytes<E>() == MaxWireBytes<Wire>());

void test_CompileTimeSizeChecks() {
  TEST_PASS();
}

}  // namespace ae::test_type_size

int test_type_size() {
  UNITY_BEGIN();
  RUN_TEST(ae::test_type_size::test_CompileTimeSizeChecks);
  return UNITY_END();
}
