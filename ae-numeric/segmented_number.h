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

#ifndef AE_NUMERIC_SEGMENTED_NUMBER_H_
#define AE_NUMERIC_SEGMENTED_NUMBER_H_

// Purpose: Public runtime type for compile-time defined piecewise
// quantization formats with independent physical and wire layouts.
//
// Motivation: sensor and timing values rarely need one uniform absolute
// precision across their complete range. Common values may need finer
// resolution and fewer wire bytes than rare tails or extreme values.
//
// Analogous concepts: scalar quantization, non-uniform quantization,
// logarithmic quantization, companding, and piecewise transfer curves.
// SegmentedNumber additionally separates the physical curve from the
// placement of ranks into 1/2/4/8-byte wire tiers.
//
// Usage: describe a seg::Format with runtime, wire, compute, and Layout
// policies, then instantiate seg::Compile<Spec>. Convert with
// TryFromRuntime()/TryEncode(), and serialize through wire_io.
//
// How it works: analytical segment encode maps a physical value to a
// rank; the compiled wire policy maps that rank to uint8_t or TieredInt.
// Decode performs the inverse path. Each instance stores only
// runtime_type: no packed rank, segment table, lookup table, or heap
// state. Production fractional math uses FixedPoint with Rep <= 32 bits.


#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <type_traits>

#include "ae-numeric/details/segmented_compiler.h"
#include "ae-numeric/details/segmented_curves.h"
#include "ae-numeric/details/segmented_format.h"
#include "ae-numeric/details/segmented_formula_backend.h"
#include "ae-numeric/details/segmented_rank_wire.h"
#include "ae-numeric/fixed_point.h"
#include "ae-numeric/numeric_traits.h"
#include "ae-numeric/runtime_numeric_traits.h"
#include "ae-numeric/wire_io.h"

namespace ae::seg {

template <typename T>
inline constexpr bool kFloatingRuntimeEnabled = false;

namespace detail {

template <typename Logical, typename RT>
struct RuntimeRawMap {
  using raw_type = typename Logical::rep_value_type;

  static constexpr raw_type ToRaw(RT const& v) {
    static_assert(numeric_traits<RT>::kIsFixedPoint,
                  "include ae-numeric/segmented_number_floating_runtime.h "
                  "to use floating runtime");
    return v.RawValue();
  }

  static constexpr RT FromRaw(raw_type raw) {
    static_assert(numeric_traits<RT>::kIsFixedPoint,
                  "include ae-numeric/segmented_number_floating_runtime.h "
                  "to use floating runtime");
    auto const clamped =
        raw < RT::kRawMin ? RT::kRawMin
                          : (raw > RT::kRawMax ? RT::kRawMax : raw);
    return RT::FromRaw(
        fixed_point_internal::RepFromRawValue<typename RT::rep_type>(
            RT::ClampRaw(clamped)));
  }
};

template <typename Spec>
using RuntimeType = std::conditional_t<
    kIsFloatingRuntimePolicy<typename Spec::runtime_policy>,
    typename Spec::runtime_policy::rep,
    LogicalTypeOf<Spec>>;

}  // namespace detail

template <SegmentedFormatSpec Spec>
class SegmentedNumber {
 public:
  using spec_type = Spec;
  using runtime_type = detail::RuntimeType<Spec>;
  using logical_type = detail::LogicalTypeOf<Spec>;
  using wire_type = detail::WireTypeOf<Spec>;
  using runtime_raw_type = typename logical_type::rep_value_type;
  static_assert(sizeof(runtime_raw_type) <= 4,
                "The Formula backend supports FixedPoint Rep values up to "
                "32 bits. A wider representation requires a separate backend "
                "because current MulDiv, root, coefficient, and conversion "
                "paths are 32-bit-specific.");

  static_assert(std::is_same_v<typename Spec::compute_policy, compute::Formula>,
                "SegmentedNumber supports compute::Formula only");

  static_assert(detail::kIsSegmentedMathRep32<runtime_raw_type>,
                "The Formula backend supports FixedPoint Rep values up to "
                "32 bits. A wider representation requires a separate backend "
                "because current MulDiv, root, coefficient, and conversion "
                "paths are 32-bit-specific.");
  static constexpr auto kPlan =
      detail::PlanHolder<Spec>::kPlan;
  static constexpr std::size_t kSegmentCount =
      static_cast<std::size_t>(kPlan.count);
  static constexpr std::size_t kCodeCount =
      static_cast<std::size_t>(kPlan.code_count);
  static constexpr std::size_t kMaxWireBytes =
      kPlan.n8 > 0 ? 8U : (kPlan.n4 > 0 ? 4U : (kPlan.n2 > 0 ? 2U : 1U));
  // Compile-time schema identity (FNV-1a). Not used by encode/decode math.
  static constexpr std::uint64_t kSchemaHash = kPlan.schema_hash;
  static constexpr std::uint32_t kOneByteCount = kPlan.n1;
  static constexpr std::uint32_t kTwoByteCount = kPlan.n2;
  static constexpr std::uint32_t kFourByteCount = kPlan.n4;
  static constexpr std::size_t kFormulaCoefficientBytes =
      detail::FormulaCoefficientBytes<Spec>();

  static_assert(!kIsFloatingRuntimePolicy<typename Spec::runtime_policy> ||
                    kFloatingRuntimeEnabled<typename Spec::runtime_policy::rep>,
                "include ae-numeric/segmented_number_floating_runtime.h "
                "to use floating runtime");
  static_assert(kCodeCount >= 1U, "format without values");

  static constexpr auto kSegments =
      detail::MakeExactCompiledSegments<
          Spec, logical_type, kSegmentCount>();

  constexpr SegmentedNumber() : value_(DecodeUnchecked(wire_type{})) {}

  runtime_type Value() const { return value_; }

  static std::optional<runtime_type> TryDecode(wire_type wire) {
    std::uint32_t const rank =
        detail::WireToRank<wire_type>(wire);
    if (rank >= kCodeCount) {
      return std::nullopt;
    }
    return DecodeUnchecked(wire);
  }

  static constexpr runtime_type Decode(wire_type wire) {
    std::uint32_t const rank =
        detail::WireToRank<wire_type>(wire);
    if (rank >= kCodeCount) {
      assert(rank < kCodeCount);
      if (!std::is_constant_evaluated()) {
        std::abort();
      }
    }
    return DecodeUnchecked(wire);
  }

  static std::optional<wire_type> TryEncode(runtime_type value) {
    runtime_raw_type const raw =
        detail::RuntimeRawMap<logical_type,
                                                 runtime_type>::ToRaw(value);
    if (raw < kDeclaredRawMin || raw > kDeclaredRawMax) {
      return std::nullopt;
    }
    return detail::RankToWire<wire_type>(EncodeRaw(raw));
  }

  static std::optional<SegmentedNumber> TryFromRuntime(runtime_type value) {
    auto const w = TryEncode(value);
    if (!w.has_value()) {
      return std::nullopt;
    }
    return FromWire(*w);
  }

  static SegmentedNumber Saturating(runtime_type value) {
    runtime_raw_type raw =
        detail::RuntimeRawMap<logical_type,
                                                 runtime_type>::ToRaw(value);
    if (raw < kDeclaredRawMin) {
      raw = kDeclaredRawMin;
    }
    if (raw > kDeclaredRawMax) {
      raw = kDeclaredRawMax;
    }
    return FromWire(detail::RankToWire<wire_type>(
        EncodeRaw(raw)));
  }

  static SegmentedNumber FromRuntimeUnchecked(runtime_type value) {
    return FromWire(detail::RankToWire<wire_type>(
        EncodeRaw(detail::RuntimeRawMap<
                  logical_type, runtime_type>::ToRaw(value))));
  }

  static SegmentedNumber FromWire(wire_type wire) {
    SegmentedNumber n;
    n.value_ = DecodeUnchecked(wire);
    return n;
  }

  constexpr bool operator==(SegmentedNumber const& o) const {
    return value_ == o.value_;
  }
  constexpr bool operator!=(SegmentedNumber const& o) const {
    return value_ != o.value_;
  }
  constexpr bool operator<(SegmentedNumber const& o) const {
    return value_ < o.value_;
  }
  constexpr bool operator>(SegmentedNumber const& o) const {
    return value_ > o.value_;
  }
  constexpr bool operator<=(SegmentedNumber const& o) const {
    return value_ <= o.value_;
  }
  constexpr bool operator>=(SegmentedNumber const& o) const {
    return value_ >= o.value_;
  }

  static std::size_t Serialize(SegmentedNumber const& value,
                               std::uint8_t* out) {
    auto const enc = TryEncode(value.Value());
    if (!enc.has_value()) {
      return 0;
    }
    return wire_traits<wire_type>::Serialize(*enc, out);
  }

  static DeserializeResult<SegmentedNumber> Deserialize(std::uint8_t const* in,
                                                        std::size_t len) {
    if (in == nullptr || len == 0) {
      return {SegmentedNumber{}, 0};
    }
    if constexpr (!std::is_same_v<wire_type, std::uint8_t>) {
      if (wire_type::WireBytesNeeded(in, len) == 0) {
        return {SegmentedNumber{}, 0};
      }
    } else if (len < 1) {
      return {SegmentedNumber{}, 0};
    }
    auto const wr = wire_traits<wire_type>::Deserialize(in, len);
    if (wr.bytes_read == 0 || wr.bytes_read > len) {
      return {SegmentedNumber{}, 0};
    }
    std::uint32_t const rank =
        detail::WireToRank<wire_type>(wr.value);
    if (rank >= kCodeCount) {
      return {SegmentedNumber{}, 0};
    }
    return {FromWire(wr.value), wr.bytes_read};
  }

  static constexpr std::size_t MaxWireBytes() { return kMaxWireBytes; }

  static constexpr auto const& Logical() { return kPlan; }

  static constexpr runtime_raw_type kDeclaredRawMin =
      logical_type::FromRatio(kPlan.declared_min.num, kPlan.declared_min.den)
          .RawValue();
  static constexpr runtime_raw_type kDeclaredRawMax =
      logical_type::FromRatio(kPlan.declared_max.num, kPlan.declared_max.den)
          .RawValue();
  static constexpr runtime_raw_type kRepresentableRawMin =
      detail::ScanRepresentable<Spec, logical_type>(true);
  static constexpr runtime_raw_type kRepresentableRawMax =
      detail::ScanRepresentable<Spec, logical_type>(
          false);

 private:
  static constexpr runtime_type DecodeUnchecked(wire_type wire) {
    std::uint32_t const rank =
        detail::WireToRank<wire_type>(wire);
    runtime_raw_type const raw =
        detail::DecodeRankRaw<Spec, logical_type>(rank);
    return detail::RuntimeRawMap<logical_type,
                                                    runtime_type>::FromRaw(raw);
  }

  static constexpr std::uint32_t EncodeRaw(runtime_raw_type raw) {
    return detail::EncodeRaw<Spec, logical_type>(
        static_cast<std::uint32_t>(kCodeCount), raw);
  }

  runtime_type value_{};
};

template <typename Spec>
using Compile = SegmentedNumber<Spec>;

}  // namespace ae::seg

namespace ae::seg {
namespace detail {
template <typename Spec>
constexpr bool SizeMatches() {
  return sizeof(SegmentedNumber<Spec>) ==
         sizeof(typename SegmentedNumber<Spec>::runtime_type);
}
}  // namespace detail
}  // namespace ae::seg

namespace ae {

using seg::SegmentedNumber;

template <typename Spec>
struct numeric_traits<seg::SegmentedNumber<Spec>> {
  using value_type = seg::SegmentedNumber<Spec>;
  using rep_type = typename value_type::wire_type;
  using rep_value_type = typename numeric_traits<rep_type>::rep_value_type;

  static constexpr bool kIsIntegerLike = false;
  static constexpr bool kIsFixedPoint = false;
  static constexpr bool kIsExponential = false;
  static constexpr bool kIsSegmented = true;
  static constexpr bool kIsSigned =
      numeric_traits<typename value_type::runtime_type>::kIsSigned;
};

template <typename Spec>
struct runtime_numeric_traits<seg::SegmentedNumber<Spec>> {
  using value_type = seg::SegmentedNumber<Spec>;
  using RT = typename value_type::runtime_type;

  static constexpr bool kIsSupported = true;
  static constexpr bool kIsSigned = runtime_numeric_traits<RT>::kIsSigned;

  static constexpr value_type FromInteger(std::int64_t value) {
    return value_type::Saturating(
        runtime_numeric_traits<RT>::FromInteger(value));
  }
};

}  // namespace ae

#endif  // AE_NUMERIC_SEGMENTED_NUMBER_H_
