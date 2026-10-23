"""Shared ESP wire layout. Keep synchronized with esp32/include/robot_link.h."""
import struct

TELEM_VER = 4
MAGIC = b"\xaa\x55"
FMT = "<BBfffffffBHHHHffBBBHH4H4b4BH"
BODY_SIZE = struct.calcsize(FMT)
PKT_SIZE = len(MAGIC) + BODY_SIZE
