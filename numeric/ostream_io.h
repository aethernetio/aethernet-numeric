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

#ifndef NUMERIC_OSTREAM_IO_H_
#define NUMERIC_OSTREAM_IO_H_

#include <ios>
#include <ostream>
#include <system_error>

#include "numeric/text_io.h"

namespace ae {
namespace ostream_io_internal {

template <typename T>
std::ostream& WriteNumeric(std::ostream& os, const T& value) {
  char buf[MaxTextSize<T>()];
  const auto result = to_chars(buf, buf + sizeof(buf), value);
  if (result.ec == std::errc{}) {
    os.write(buf, static_cast<std::streamsize>(result.ptr - buf));
  } else {
    os.setstate(std::ios_base::badbit);
  }
  return os;
}

}  // namespace ostream_io_internal

template <typename WireCell, std::uint32_t... TierMaxVals>
std::ostream& operator<<(std::ostream& os,
                         const TieredInt<WireCell, TierMaxVals...>& value) {
  return ostream_io_internal::WriteNumeric(os, value);
}

template <typename Rep, auto Max>
  requires IntegralStorage<Rep>
std::ostream& operator<<(std::ostream& os, const FixedPoint<Rep, Max>& value) {
  return ostream_io_internal::WriteNumeric(os, value);
}

}  // namespace ae

#endif  // NUMERIC_OSTREAM_IO_H_
