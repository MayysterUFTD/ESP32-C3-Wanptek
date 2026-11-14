#include "WanptekProtocol.h"

constexpr auto remote = makePsuWriteRequest(1, false, false, true, 1234, 5678, false);
static_assert(remote[7] == 0x04 && remote[8] == 0x00,
              "Remote control belongs in the high byte of the status register");
static_assert(remote[9] == 0xD2 && remote[10] == 0x04 &&
                  remote[11] == 0x2E && remote[12] == 0x16,
              "Setpoints must be encoded little-endian within each register");

constexpr auto outputAndOcp = makePsuWriteRequest(1, true, true, false, 0, 0, false);
static_assert(outputAndOcp[7] == 0x03 && outputAndOcp[8] == 0x00,
              "Output and OCP flags must not enable remote control");

constexpr auto bigEndian = makePsuWriteRequest(0, true, false, true, 1500, 1000, true);
static_assert(bigEndian[7] == 0x05 && bigEndian[8] == 0x00 &&
                  bigEndian[9] == 0x05 && bigEndian[10] == 0xDC &&
                  bigEndian[11] == 0x03 && bigEndian[12] == 0xE8,
              "Big-endian setpoints must match the PSU byte order");
