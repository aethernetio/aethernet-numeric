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

#ifndef NUMERIC_TEXT_IO_H_
#define NUMERIC_TEXT_IO_H_

#include <charconv>
#include <cstdint>
#include <limits>
#include <string>
#include <string_view>
#include <system_error>
#include <type_traits>

#include "numeric/exponential.h"
#include "numeric/fixed_point.h"
#include "numeric/numeric_traits.h"
#include "numeric/tiered_int.h"

namespace ae {
namespace text_io_internal {

constexpr std::size_t CountDigits10(std::uint64_t value) {
  std::size_t digits = 1;
  while (value >= 10) {
    value /= 10;
    ++digits;
  }
  return digits;
}

constexpr std::size_t CountDigits10Signed(std::int64_t value) {
  if (value == 0) {
    return 1;
  }
  if (value < 0) {
    return 1 + CountDigits10(static_cast<std::uint64_t>(-(value + 1)) + 1);
  }
  return CountDigits10(static_cast<std::uint64_t>(value));
}

constexpr std::uint64_t Pow2u(unsigned bits) {
  return bits >= 64 ? 0 : (std::uint64_t{1} << bits);
}

struct DecimalParseResult {
  bool ok = false;
  bool negative = false;
  std::int64_t num = 0;
  std::int64_t den = 1;
};

constexpr bool IsDigit(char c) {
  return c >= '0' && c <= '9';
}

constexpr DecimalParseResult ParseDecimal(std::string_view text) {
  DecimalParseResult result;
  if (text.empty()) {
    return result;
  }

  std::size_t index = 0;
  if (text[index] == '+' || text[index] == '-') {
    result.negative = (text[index] == '-');
    ++index;
    if (index >= text.size()) {
      return result;
    }
  }

  std::int64_t integer_part = 0;
  bool has_integer = false;
  while (index < text.size() && IsDigit(text[index])) {
    has_integer = true;
    integer_part = integer_part * 10 + (text[index] - '0');
    if (integer_part > std::numeric_limits<std::int64_t>::max() / 10) {
      return result;
    }
    ++index;
  }

  std::int64_t fraction_num = 0;
  std::int64_t fraction_den = 1;
  if (index < text.size()) {
    if (text[index] != '.') {
      return result;
    }
    ++index;
    if (index >= text.size()) {
      return result;
    }
    std::size_t fraction_digits = 0;
    while (index < text.size() && IsDigit(text[index])) {
      if (fraction_digits >= 19) {
        return result;
      }
      ++fraction_digits;
      if (fraction_den > std::numeric_limits<std::int64_t>::max() / 10) {
        return result;
      }
      fraction_den *= 10;
      fraction_num = fraction_num * 10 + (text[index] - '0');
      if (fraction_num > std::numeric_limits<std::int64_t>::max() / 10) {
        return result;
      }
      ++index;
    }
    if (index != text.size()) {
      return result;
    }
  } else if (!has_integer) {
    return result;
  }

  if (fraction_den == 1) {
    result.num = integer_part;
    result.den = 1;
    result.ok = true;
    return result;
  }

  result.num = integer_part * fraction_den + fraction_num;
  result.den = fraction_den;
  result.ok = true;
  return result;
}

constexpr std::int64_t Gcd64(std::int64_t a, std::int64_t b) {
  a = a < 0 ? -a : a;
  b = b < 0 ? -b : b;
  while (b != 0) {
    auto const t = a % b;
    a = b;
    b = t;
  }
  return a;
}

constexpr DecimalParseResult NormalizeDecimal(DecimalParseResult value) {
  if (!value.ok) {
    return value;
  }
  if (value.num == 0) {
    value.negative = false;
    value.den = 1;
    return value;
  }
  auto const g = Gcd64(value.num, value.den);
  value.num /= g;
  value.den /= g;
  if (value.den < 0) {
    value.num = -value.num;
    value.den = -value.den;
  }
  return value;
}

template <typename RepValue>
constexpr bool RawFromRatioInRange(std::int64_t num, std::int64_t den,
                                   int scale_exp, RepValue raw_min,
                                   RepValue raw_max, RepValue& raw_out) {
  if (den == 0) {
    return false;
  }

  const bool negative = (num < 0) ^ (den < 0);
  num = num < 0 ? -num : num;
  den = den < 0 ? -den : den;

  if constexpr (!std::is_signed_v<RepValue>) {
    if (negative) {
      return false;
    }
  }

#ifdef __SIZEOF_INT128__
  using Wide = __int128_t;
  Wide wnum = num;
  Wide wden = den;

  if (scale_exp < 0) {
    const unsigned shift = static_cast<unsigned>(-scale_exp);
    for (unsigned i = 0; i < shift; ++i) {
      wnum *= Wide{2};
    }
  } else if (scale_exp > 0) {
    const unsigned shift = static_cast<unsigned>(scale_exp);
    for (unsigned i = 0; i < shift; ++i) {
      wden *= Wide{2};
    }
  }

  const Wide rounded = (wnum + wden / Wide{2}) / wden;
  const Wide signed_rounded = negative ? -rounded : rounded;

  if (signed_rounded < static_cast<Wide>(raw_min) ||
      signed_rounded > static_cast<Wide>(raw_max)) {
    return false;
  }

  raw_out = static_cast<RepValue>(signed_rounded);
  return true;
#else
  if (scale_exp < 0) {
    const unsigned shift = static_cast<unsigned>(-scale_exp);
    std::int64_t factor = 1;
    for (unsigned i = 0; i < shift; ++i) {
      if (factor > std::numeric_limits<std::int64_t>::max() / 2) {
        return false;
      }
      factor *= 2;
    }
    if (num > std::numeric_limits<std::int64_t>::max() / factor) {
      return false;
    }
    num *= factor;
  } else if (scale_exp > 0) {
    const unsigned shift = static_cast<unsigned>(scale_exp);
    std::int64_t factor = 1;
    for (unsigned i = 0; i < shift; ++i) {
      if (factor > std::numeric_limits<std::int64_t>::max() / 2) {
        return false;
      }
      factor *= 2;
    }
    if (den > std::numeric_limits<std::int64_t>::max() / factor) {
      return false;
    }
    den *= factor;
  }

  const std::int64_t rounded = ((num + den / 2) / den);
  const std::int64_t signed_rounded = negative ? -rounded : rounded;

  if (signed_rounded < static_cast<std::int64_t>(raw_min) ||
      signed_rounded > static_cast<std::int64_t>(raw_max)) {
    return false;
  }

  raw_out = static_cast<RepValue>(signed_rounded);
  return true;
#endif
}

inline std::to_chars_result WriteChar(char* first, char* last, char c) {
  if (first == last) {
    return {first, std::errc::value_too_large};
  }
  *first = c;
  return {first + 1, std::errc{}};
}

inline std::to_chars_result WriteChars(char* first, char* last,
                                       std::string_view text) {
  if (static_cast<std::size_t>(last - first) < text.size()) {
    return {first, std::errc::value_too_large};
  }
  for (char c : text) {
    *first++ = c;
  }
  return {first, std::errc{}};
}

template <typename Int>
inline std::to_chars_result WriteInteger(char* first, char* last, Int value) {
  return std::to_chars(first, last, value);
}

inline std::to_chars_result WriteUnsignedDecimal(char* first, char* last,
                                                 std::uint64_t value) {
  char buffer[32];
  auto const result = std::to_chars(buffer, buffer + sizeof(buffer), value);
  if (result.ec != std::errc{}) {
    return {first, result.ec};
  }
  return WriteChars(
      first, last,
      std::string_view(buffer, static_cast<std::size_t>(result.ptr - buffer)));
}

inline std::to_chars_result WriteSignedDecimal(char* first, char* last,
                                               std::int64_t value) {
  char buffer[32];
  auto const result = std::to_chars(buffer, buffer + sizeof(buffer), value);
  if (result.ec != std::errc{}) {
    return {first, result.ec};
  }
  return WriteChars(
      first, last,
      std::string_view(buffer, static_cast<std::size_t>(result.ptr - buffer)));
}

inline std::from_chars_result ParseInteger(std::string_view text,
                                           std::int64_t& value) {
  return std::from_chars(text.data(), text.data() + text.size(), value);
}

template <typename RepValue>
inline std::to_chars_result FormatFixedPointScaled(char* first, char* last,
                                                   RepValue raw, int scale_exp,
                                                   bool is_signed) {
  if (scale_exp >= 0) {
    if (is_signed) {
      const std::int64_t logical = static_cast<std::int64_t>(raw)
                                   << static_cast<unsigned>(scale_exp);
      return WriteSignedDecimal(first, last, logical);
    }
    const std::uint64_t logical = static_cast<std::uint64_t>(raw)
                                  << static_cast<unsigned>(scale_exp);
    return WriteUnsignedDecimal(first, last, logical);
  }

  const unsigned fraction_bits = static_cast<unsigned>(-scale_exp);
  const std::uint64_t denominator = Pow2u(fraction_bits);

  bool negative = false;
  std::uint64_t magnitude = 0;
  if (is_signed) {
    auto const signed_raw = static_cast<std::int64_t>(raw);
    negative = signed_raw < 0;
    magnitude = static_cast<std::uint64_t>(negative ? -signed_raw : signed_raw);
  } else {
    magnitude = static_cast<std::uint64_t>(raw);
  }

  const std::uint64_t integer_part = magnitude / denominator;
  std::uint64_t fraction_part = magnitude % denominator;

  char fraction_digits[64]{};
  std::size_t fraction_len = 0;
  constexpr std::size_t kMaxFractionDigits = 18;
  while (fraction_part != 0 && fraction_len < kMaxFractionDigits &&
         fraction_len + 1 < sizeof(fraction_digits)) {
    fraction_part *= 10;
    const std::uint64_t digit = fraction_part / denominator;
    fraction_part %= denominator;
    fraction_digits[fraction_len++] =
        static_cast<char>('0' + static_cast<char>(digit));
  }
  if (fraction_part != 0 && fraction_len > 0) {
    if (fraction_digits[fraction_len - 1] < '9') {
      ++fraction_digits[fraction_len - 1];
    }
  }
  while (fraction_len > 0 && fraction_digits[fraction_len - 1] == '0') {
    --fraction_len;
  }

  char* cursor = first;
  if (negative) {
    auto const sign_result = WriteChar(cursor, last, '-');
    if (sign_result.ec != std::errc{}) {
      return sign_result;
    }
    cursor = sign_result.ptr;
  }

  auto const integer_result = WriteUnsignedDecimal(cursor, last, integer_part);
  if (integer_result.ec != std::errc{}) {
    return integer_result;
  }
  cursor = integer_result.ptr;

  if (fraction_len == 0) {
    return {cursor, std::errc{}};
  }

  auto const dot_result = WriteChar(cursor, last, '.');
  if (dot_result.ec != std::errc{}) {
    return dot_result;
  }
  cursor = dot_result.ptr;

  return WriteChars(cursor, last,
                    std::string_view(fraction_digits, fraction_len));
}

}  // namespace text_io_internal

template <typename T>
struct TextIO;

template <typename T>
constexpr std::size_t MaxTextSize() {
  return TextIO<T>::kMaxTextSize;
}

template <typename T>
std::to_chars_result ToChars(char* first, char* last, T value) {
  return TextIO<T>::ToChars(first, last, value);
}

template <typename T>
std::from_chars_result FromChars(char const* first, char const* last,
                                 T& value) {
  return TextIO<T>::FromChars(first, last, value);
}

template <typename T>
std::string ToString(T value) {
  char buffer[TextIO<T>::kMaxTextSize];
  auto const result = ToChars(buffer, buffer + sizeof(buffer), value);
  if (result.ec != std::errc{}) {
    return {};
  }
  return std::string(buffer, static_cast<std::size_t>(result.ptr - buffer));
}

template <typename T>
bool FromString(std::string_view text, T& value) {
  auto const result = FromChars(text.data(), text.data() + text.size(), value);
  return result.ec == std::errc {} && result.ptr == text.data() + text.size();
}

template <typename WireCell, std::uint32_t... TierMaxVals>
struct TextIO<TieredInt<WireCell, TierMaxVals...>> {
  using T = TieredInt<WireCell, TierMaxVals...>;
  using ValueType = typename T::ValueType;

  static constexpr std::size_t kMaxTextSize =
      text_io_internal::CountDigits10Signed(
          static_cast<std::int64_t>(T::kUpper)) +
      (T::kIsSigned ? 1 : 0);

  static std::to_chars_result ToChars(char* first, char* last, T value) {
    if constexpr (T::kIsSigned) {
      return text_io_internal::WriteSignedDecimal(
          first, last, static_cast<std::int64_t>(value.value_));
    }
    return text_io_internal::WriteUnsignedDecimal(
        first, last, static_cast<std::uint64_t>(value.value_));
  }

  static std::from_chars_result FromChars(char const* first, char const* last,
                                          T& value) {
    std::string_view const text(first, static_cast<std::size_t>(last - first));
    if constexpr (T::kIsSigned) {
      std::int64_t parsed = 0;
      auto const result = text_io_internal::ParseInteger(text, parsed);
      if (result.ec != std::errc {} ||
          result.ptr != text.data() + text.size()) {
        return {first, std::errc::invalid_argument};
      }
      if (parsed < static_cast<std::int64_t>(T::kLower) ||
          parsed > static_cast<std::int64_t>(T::kUpper)) {
        return {first, std::errc::result_out_of_range};
      }
      value = T{static_cast<ValueType>(parsed)};
      return {last, std::errc{}};
    }

    std::int64_t parsed = 0;
    auto const result = text_io_internal::ParseInteger(text, parsed);
    if (result.ec != std::errc {} || result.ptr != text.data() + text.size() ||
        parsed < 0) {
      return {first, std::errc::invalid_argument};
    }
    if (parsed > static_cast<std::int64_t>(T::kUpper)) {
      return {first, std::errc::result_out_of_range};
    }
    value = T{static_cast<ValueType>(parsed)};
    return {last, std::errc{}};
  }
};

template <typename Rep, auto Max>
struct TextIO<FixedPoint<Rep, Max>> {
  using T = FixedPoint<Rep, Max>;
  using RepValue = typename T::rep_value_type;

  static constexpr std::size_t kMaxTextSize =
      text_io_internal::CountDigits10Signed(
          static_cast<std::int64_t>(T::kRawMax)) +
      static_cast<std::size_t>(T::kFractionBits) + 4;

  static std::to_chars_result ToChars(char* first, char* last, T value) {
    return text_io_internal::FormatFixedPointScaled(
        first, last, value.RawValue(), T::kScaleExp, T::kIsSigned);
  }

  static std::from_chars_result FromChars(char const* first, char const* last,
                                          T& value) {
    std::string_view const text(first, static_cast<std::size_t>(last - first));
    auto parsed = text_io_internal::NormalizeDecimal(
        text_io_internal::ParseDecimal(text));
    if (!parsed.ok) {
      return {first, std::errc::invalid_argument};
    }
    if (parsed.negative) {
      parsed.num = -parsed.num;
    }

    RepValue raw = 0;
    if (!text_io_internal::RawFromRatioInRange(parsed.num, parsed.den,
                                               T::kScaleExp, T::kRawMin,
                                               T::kRawMax, raw)) {
      return {first, std::errc::result_out_of_range};
    }

    value = T::FromRaw(raw);
    return {last, std::errc{}};
  }
};

template <typename RuntimeT, typename WireT, auto MinMagnitude,
          auto BoundaryMagnitude, auto BoundaryCode>
struct TextIO<Exponential<RuntimeT, WireT, MinMagnitude, BoundaryMagnitude,
                          BoundaryCode>> {
  using T = Exponential<RuntimeT, WireT, MinMagnitude, BoundaryMagnitude,
                        BoundaryCode>;

  static constexpr std::size_t kMaxTextSize = TextIO<RuntimeT>::kMaxTextSize;

  static std::to_chars_result ToChars(char* first, char* last, T value) {
    return TextIO<RuntimeT>::ToChars(first, last, value.ToRuntime());
  }

  static std::from_chars_result FromChars(char const* first, char const* last,
                                          T& value) {
    RuntimeT runtime = RuntimeT::FromInteger(0);
    auto const runtime_result =
        TextIO<RuntimeT>::FromChars(first, last, runtime);
    if (runtime_result.ec != std::errc{}) {
      return runtime_result;
    }
    value = T::FromRuntime(runtime);
    return {last, std::errc{}};
  }
};

}  // namespace ae

#endif  // NUMERIC_TEXT_IO_H_
