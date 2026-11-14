#pragma once

#include <array>
#include <cstdint>

constexpr std::array<uint8_t, 15> makePsuWriteRequest(
    uint8_t address, bool outputOn, bool ocpOn, bool remoteControl,
    uint16_t voltage, uint16_t current, bool bigEndian)
{
  return {{address, 0x10, 0x00, 0x00, 0x00, 0x03, 0x06,
           static_cast<uint8_t>((outputOn ? 1 : 0) | (ocpOn ? 2 : 0) |
                                (remoteControl ? 4 : 0)),
           0x00, static_cast<uint8_t>(bigEndian ? voltage >> 8 : voltage),
           static_cast<uint8_t>(bigEndian ? voltage : voltage >> 8),
           static_cast<uint8_t>(bigEndian ? current >> 8 : current),
           static_cast<uint8_t>(bigEndian ? current : current >> 8),
           0x00, 0x00}};
}
