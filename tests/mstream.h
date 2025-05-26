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

#ifndef TESTS_MSTREAM_H
#define TESTS_MSTREAM_H

#include <cstdint>
#include <cstddef>
#include <type_traits>

namespace ae::test {
template <typename TReader>
class Imstream {
 public:
  void read(std::uint8_t* p, std::size_t size) { reader.read(p, size); }
  TReader& reader;
};

template <typename TWriter>
class Omstream {
 public:
  void write(std::uint8_t const* p, std::size_t size) { writer.write(p, size); }
  TWriter& writer;
};

template <typename T, typename TReader>
std::enable_if_t<std::is_integral_v<T>, Imstream<TReader>&> operator>>(
    Imstream<TReader>& stream, T& value) {
  stream.read(reinterpret_cast<std::uint8_t*>(&value), sizeof(T));
  return stream;
}

template <typename T, typename TWriter>
std::enable_if_t<std::is_integral_v<T>, Omstream<TWriter>&> operator<<(
    Omstream<TWriter>& stream, T const& value) {
  stream.write(reinterpret_cast<std::uint8_t const*>(&value), sizeof(T));
  return stream;
}
}  // namespace ae::test

#endif  // TESTS_MSTREAM_H
