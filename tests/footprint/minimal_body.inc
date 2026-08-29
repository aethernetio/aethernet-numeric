// Instantiated by each minimal_*.cpp after defining AE_FP_NUM.
// Object-only: no main, no printf, no Unity.

#include "ae_fp_common.h"

#include <type_traits>

using Num = AE_FP_NUM;
using RT = typename Num::runtime_type;

template <typename Wire>
AE_FP_NOINLINE std::uint32_t AeFpWireToU32(Wire const& w) {
  if constexpr (std::is_same_v<Wire, std::uint8_t>) {
    return static_cast<std::uint32_t>(w);
  } else {
    return static_cast<std::uint32_t>(static_cast<typename Wire::ValueType>(w));
  }
}

template <typename Wire>
AE_FP_NOINLINE Wire AeFpU32ToWire(std::uint32_t code) {
  if constexpr (std::is_same_v<Wire, std::uint8_t>) {
    return static_cast<std::uint8_t>(code);
  } else {
    return Wire{code};
  }
}

extern "C" AE_FP_NOINLINE AE_FP_USED std::uint32_t TestEncode(
    std::uint32_t raw) {
  volatile std::uint32_t in = raw;
  auto const v = RT::FromRaw(
      RT::ClampRaw(static_cast<typename RT::rep_value_type>(
          static_cast<std::int64_t>(in))));
  auto const w = Num::TryEncode(v);
  if (!w.has_value()) {
    return 0xFFFFFFFFu;
  }
  return AeFpWireToU32<typename Num::wire_type>(*w);
}

extern "C" AE_FP_NOINLINE AE_FP_USED std::uint32_t TestDecode(
    std::uint32_t code) {
  volatile std::uint32_t in = code;
  auto const w = AeFpU32ToWire<typename Num::wire_type>(in);
  auto const d = Num::TryDecode(w);
  if (!d.has_value()) {
    return 0xFFFFFFFFu;
  }
  return static_cast<std::uint32_t>(d->RawValue());
}

extern "C" AE_FP_NOINLINE AE_FP_USED std::size_t TestSerialize(
    std::uint32_t raw, std::uint8_t* out) {
  volatile std::uint32_t in = raw;
  auto const v = RT::FromRaw(
      RT::ClampRaw(static_cast<typename RT::rep_value_type>(
          static_cast<std::int64_t>(in))));
  auto const n = Num::Saturating(v);
  return Num::Serialize(n, out);
}

extern "C" AE_FP_NOINLINE AE_FP_USED std::uint32_t TestMeta() {
  volatile std::size_t sink = 0;
  sink += Num::kCodeCount;
  sink += Num::kSegmentCount;
  sink += Num::kFormulaCoefficientBytes;
  sink += sizeof(Num::kSegments);
  return static_cast<std::uint32_t>(sink);
}
