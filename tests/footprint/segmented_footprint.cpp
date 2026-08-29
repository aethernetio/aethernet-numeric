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

#include <cstddef>
#include <cstdint>
#include <cstdio>

#include <ae-numeric/segmented_number_wire_io.h>

#include "../segmented_test_formats.h"

namespace {

#if defined(AE_SEG_FOOTPRINT_ALL)
#define AE_SEG_HAS_RSSI 1
#define AE_SEG_HAS_TEMP 1
#define AE_SEG_HAS_HUM 1
#define AE_SEG_HAS_CO2 1
#define AE_SEG_HAS_RX 1
#define AE_SEG_HAS_BAT 1
#define AE_SEG_HAS_CONN 1
#endif

template <typename Num>
void SinkOne(char const* name) {
  volatile std::size_t sink = 0;
  sink += sizeof(Num);
  sink += sizeof(typename Num::runtime_type);
  sink += sizeof(typename Num::wire_type);
  sink += Num::kCodeCount;
  sink += Num::kMaxWireBytes;
  sink += Num::kSegmentCount;
  sink += Num::kFormulaCoefficientBytes;
  auto const n = Num::Saturating(Num::runtime_type::FromInteger(1));
  std::uint8_t buf[8] = {};
  sink += Num::Serialize(n, buf);
  auto const back = Num::Deserialize(buf, sizeof(buf));
  sink += back.bytes_read;
  std::printf(
      "%s sizeof(runtime)=%zu sizeof(number)=%zu sizeof(wire)=%zu "
      "codes=%zu max_bytes=%zu segs=%zu formula_bytes=%zu sink=%zu\n",
      name, sizeof(typename Num::runtime_type), sizeof(Num),
      sizeof(typename Num::wire_type), Num::kCodeCount, Num::kMaxWireBytes,
      Num::kSegmentCount, Num::kFormulaCoefficientBytes,
      static_cast<std::size_t>(sink));
}

}  // namespace

int main() {
#if defined(AE_SEG_HAS_RSSI)
  SinkOne<ae::test_segmented_formats::Rssi>("rssi");
#endif
#if defined(AE_SEG_HAS_TEMP)
  SinkOne<ae::test_segmented_formats::Temperature>("temperature");
#endif
#if defined(AE_SEG_HAS_HUM)
  SinkOne<ae::test_segmented_formats::Humidity>("humidity");
#endif
#if defined(AE_SEG_HAS_CO2)
  SinkOne<ae::test_segmented_formats::Co2>("co2");
#endif
#if defined(AE_SEG_HAS_RX)
  SinkOne<ae::test_segmented_formats::RxWindow>("rx-window");
#endif
#if defined(AE_SEG_HAS_BAT)
  SinkOne<ae::test_segmented_formats::Battery>("battery");
#endif
#if defined(AE_SEG_HAS_CONN)
  SinkOne<ae::test_segmented_formats::ConnectDuration>("connect-duration");
#endif
  return 0;
}
