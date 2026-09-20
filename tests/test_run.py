"""
Tests for run.py's CLI argument handling (--battery, --port).

These only exercise argument parsing and the resulting Drone/config wiring
-- they don't start the Flask dev server or the background sim thread.
"""

import unittest

from run import parse_args


class TestParseArgs(unittest.TestCase):
    def test_defaults(self):
        args = parse_args([])
        self.assertEqual(args.battery, 100.0)
        self.assertEqual(args.port, 5000)

    def test_battery_flag_is_parsed(self):
        args = parse_args(["--battery", "15"])
        self.assertEqual(args.battery, 15.0)

    def test_port_flag_is_parsed(self):
        args = parse_args(["--port", "8080"])
        self.assertEqual(args.port, 8080)

    def test_battery_out_of_range_is_rejected(self):
        with self.assertRaises(SystemExit):
            parse_args(["--battery", "150"])
        with self.assertRaises(SystemExit):
            parse_args(["--battery", "-5"])


if __name__ == "__main__":
    unittest.main()
