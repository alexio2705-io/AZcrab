"""battery_probe: напряжение из пакетов v4 и проверка, что порт ESP занят."""
import binascii
import os
import struct
import tempfile
import unittest

import fakes  # noqa: F401  (кладёт pi/ в sys.path)
import battery_probe as bp
from protocol import FMT, MAGIC


def packet(dv):
    body = list(struct.unpack(FMT, bytes(struct.calcsize(FMT))))
    body[0] = bp.TELEM_VER
    body[21:25] = dv
    raw = struct.pack(FMT, *body)[:-2]
    crc = binascii.crc_hqx(raw, 0xFFFF)
    return MAGIC + raw + struct.pack("<H", crc)


class VoltsTest(unittest.TestCase):
    def test_lowest_fresh_motor_voltage(self):
        buf = b"\x13\x37" + packet((161, 163, 0, 162)) + packet((160, 0, 0, 0))[:-5]
        self.assertEqual(bp.volts_from_bytes(buf), 16.1)

    def test_no_motor_reply_or_bad_crc(self):
        self.assertIsNone(bp.volts_from_bytes(packet((0, 0, 0, 0))))
        bad = bytearray(packet((161, 161, 161, 161)))
        bad[-1] ^= 0xFF
        self.assertIsNone(bp.volts_from_bytes(bytes(bad)))


class PortInUseTest(unittest.TestCase):
    def test_other_process_holding_port(self):
        with tempfile.TemporaryDirectory() as d:
            port = os.path.join(d, "ttyAMA0")
            open(port, "w").close()
            os.makedirs(os.path.join(d, "proc", "42", "fd"))
            self.assertFalse(bp.port_in_use(port, os.path.join(d, "proc")))
            os.symlink(port, os.path.join(d, "proc", "42", "fd", "3"))
            self.assertTrue(bp.port_in_use(port, os.path.join(d, "proc")))


if __name__ == "__main__":
    unittest.main()
