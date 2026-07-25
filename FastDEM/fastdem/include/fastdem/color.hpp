// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>
//
// PCL-convention packed RGB float utilities.

#pragma once

#include <cstdint>
#include <cstring>

namespace fastdem::color {

/// Pack RGB (0-255) into a single float (PCL convention).
inline float pack(uint8_t r, uint8_t g, uint8_t b) {
  unsigned long lc =
      (static_cast<unsigned long>(r) << 16) |
      (static_cast<unsigned long>(g) << 8) | b;
  float packed;
  std::memcpy(&packed, &lc, sizeof(float));
  return packed;
}

/// Unpack a PCL packed-float into RGB (0-255).
inline void unpack(float packed, uint8_t& r, uint8_t& g, uint8_t& b) {
  unsigned long lc = 0;
  std::memcpy(&lc, &packed, sizeof(float));
  r = static_cast<uint8_t>((lc >> 16) & 0xFF);
  g = static_cast<uint8_t>((lc >> 8) & 0xFF);
  b = static_cast<uint8_t>(lc & 0xFF);
}

}  // namespace fastdem::color
