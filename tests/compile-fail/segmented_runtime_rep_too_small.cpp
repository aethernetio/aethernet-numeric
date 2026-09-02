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

#include <cstdint>

#include <ae-numeric/decimal.h>
#include <ae-numeric/segmented_number.h>

template <std::int64_t M, int E = 0>
using D = ae::Decimal<M, E>;

using Bad = ae::seg::Compile<ae::seg::Format<
    ae::seg::runtime::Fixed<std::uint8_t>,
    ae::seg::wire::AutoTiered<std::uint8_t, ae::seg::wire::MaxBytes<1>>,
    ae::seg::compute::Formula,
    ae::seg::Layout<ae::seg::UniformValues<
        ae::seg::Range<D<0>, D<10>>, ae::seg::Intervals<250>,
        ae::seg::Place<ae::seg::Bytes<1>>>>>>;

Bad value{};

int main() {
  (void)value;
  return 0;
}
