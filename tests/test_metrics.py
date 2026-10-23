import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'bricks/ros2_jazzy/app'))
from metrics import Reception, decode_packet, sustainable


class MetricsTests(unittest.TestCase):
    def test_original_size_dummy_packet(self):
        self.assertEqual(decode_packet([1, 0, .1, .2, 9.81, .01, .02, .03]),
                         [.1, .2, 9.81, .01, .02, .03])

    def test_wrong_sketch_and_bad_packets_rejected(self):
        for packet in ([1, 100, 0, 0, 0, 0, 0, 0], [True, 0, 0, 0, 0, 0, 0, 0],
                       [1, 0, float('nan'), 0, 0, 0, 0, 0], [1, 0], 'packet'):
            with self.subTest(packet=packet), self.assertRaises(ValueError):
                decode_packet(packet)

    def test_loss_duplicates_reordering_and_drain(self):
        r = Reception(first=10, start=100, end=110)
        r.record(9, 101)    # Stale packet from warmup/previous stage.
        r.record(10, 101)
        r.record(12, 102)
        r.record(10, 103)  # Duplicate is not extra throughput.
        r.record(11, 104)  # Reordered, but delivered.
        r.record(14, 111)  # Drained packet counts as delivery, not steady rate.
        result = r.result(sent=6)
        self.assertEqual(result['received'], 4)
        self.assertEqual(result['received_in_window'], 3)
        self.assertEqual(result['lost'], 2)
        self.assertEqual(result['duplicates'], 1)
        self.assertEqual(result['reordered'], 1)
        self.assertAlmostEqual(result['received_hz'], .3)
        self.assertAlmostEqual(result['loss_fraction'], 2 / 6)

    def test_no_data_cannot_pass(self):
        r = Reception(first=0, start=0, end=10).result(sent=0)
        self.assertEqual(r['loss_fraction'], 1)
        self.assertEqual(r['received_hz'], 0)

    def test_target_pass_requires_send_receive_loss_and_error_criteria(self):
        row = dict(target_hz=100, sent=1000, sent_hz=99, received_hz=98,
                   loss_fraction=.005, errors=0)
        self.assertTrue(sustainable(row, .95, .01))
        for key, value in [('target_hz', 0), ('sent', 0), ('sent_hz', 90),
                           ('received_hz', 90), ('loss_fraction', .02), ('errors', 1)]:
            with self.subTest(key=key):
                self.assertFalse(sustainable(dict(row, **{key: value}), .95, .01))


if __name__ == '__main__':
    unittest.main()
