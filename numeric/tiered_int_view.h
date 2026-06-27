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

#ifndef NUMERIC_TIERED_INT_VIEW_H_
#define NUMERIC_TIERED_INT_VIEW_H_

#include <cstddef>
#include <cstdint>
#include <iterator>
#include <ranges>
#include <span>
#include <stdexcept>

#include "numeric/tiered_int.h"

namespace ae {

template <typename TieredIntT>
class TieredIntView : public std::ranges::view_interface<TieredIntView<TieredIntT>> {
 public:
  using value_type = TieredIntT;
  using byte_type = std::uint8_t;

  class iterator {
   public:
    using iterator_concept = std::input_iterator_tag;
    using iterator_category = std::input_iterator_tag;
    using value_type = TieredIntT;
    using difference_type = std::ptrdiff_t;
    using reference = value_type;

    iterator() noexcept = default;

    reference operator*() const {
      ensure_decoded();
      return cached_value_;
    }

    iterator& operator++() {
      ensure_decoded();
      current_ += static_cast<std::ptrdiff_t>(cached_size_);
      decoded_ = false;
      cached_size_ = 0;
      return *this;
    }

    void operator++(int) { ++(*this); }

    friend bool operator==(const iterator& it,
                           std::default_sentinel_t) noexcept {
      return it.current_ == it.end_;
    }

    friend bool operator!=(const iterator& it,
                           std::default_sentinel_t sentinel) noexcept {
      return !(it == sentinel);
    }

   private:
    friend class TieredIntView;

    iterator(const byte_type* current, const byte_type* end) noexcept
        : current_(current), end_(end) {}

    void ensure_decoded() const {
      if (decoded_) {
        return;
      }

      if (current_ == end_) {
        throw std::out_of_range(
            "TieredIntView iterator dereferenced at end");
      }

      value_type value{};
      const auto remaining =
          static_cast<std::size_t>(end_ - current_);
      const auto consumed = value.Deserialize(current_, remaining);

      if (consumed == 0 || consumed > remaining) {
        throw std::out_of_range("Invalid or truncated TieredIntView data");
      }

      cached_value_ = value;
      cached_size_ = consumed;
      decoded_ = true;
    }

    const byte_type* current_ = nullptr;
    const byte_type* end_ = nullptr;
    mutable value_type cached_value_{};
    mutable std::size_t cached_size_ = 0;
    mutable bool decoded_ = false;
  };

  constexpr TieredIntView() noexcept = default;

  constexpr explicit TieredIntView(std::span<const byte_type> bytes) noexcept
      : bytes_(bytes) {}

  constexpr std::span<const byte_type> bytes() const noexcept { return bytes_; }

  constexpr std::size_t byte_size() const noexcept { return bytes_.size(); }

  constexpr bool empty() const noexcept { return bytes_.empty(); }

  constexpr iterator begin() const noexcept {
    return iterator(bytes_.data(), bytes_.data() + bytes_.size());
  }

  constexpr std::default_sentinel_t end() const noexcept {
    return std::default_sentinel;
  }

 private:
  std::span<const byte_type> bytes_{};
};

template <typename TieredIntT>
constexpr TieredIntView<TieredIntT> MakeTieredIntView(
    std::span<const std::uint8_t> bytes) noexcept {
  return TieredIntView<TieredIntT>{bytes};
}

}  // namespace ae

namespace std::ranges {

template <typename TieredIntT>
inline constexpr bool enable_borrowed_range<ae::TieredIntView<TieredIntT>> =
    true;

}  // namespace std::ranges

#endif  // NUMERIC_TIERED_INT_VIEW_H_
