"""Раннее распознавание чёрного края по образцам с поля."""
import unittest

from test_reliability import load


class EarlyBlackTests(unittest.TestCase):
    def setUp(self):
        self.tc = load('tile_classify', {})
        self.samples = {
            'white': [dict(r=113, g=158, b=165, c=433)],
            'black': [dict(r=10, g=13, b=14, c=38)],
            'silver': [dict(r=17, g=20, b=19, c=55.5)],
            'red': [dict(r=21, g=6, b=6, c=30)],
            'blue': [dict(r=17, g=62, b=105, c=180)],
            'air': [dict(r=77, g=61, b=40, c=175)],
        }

    def test_dark_edge_precedes_nearest_sample_verdict(self):
        white = self.samples['white'][0]
        black = self.samples['black'][0]
        for fraction in (0.8, 0.9, 0.95):
            with self.subTest(fraction=fraction):
                values = [white[k] * (1 - fraction) + black[k] * fraction
                          for k in ('r', 'g', 'b', 'c')]
                self.assertNotEqual(self.tc.classify(self.samples, *values)[0], 'black')
                self.assertTrue(self.tc.early_black(self.samples, *values))

    def test_calibrated_other_tiles_are_not_black(self):
        for color in ('white', 'silver', 'red', 'blue', 'air'):
            with self.subTest(color=color):
                sample = self.samples[color][0]
                values = [sample[k] for k in ('r', 'g', 'b', 'c')]
                self.assertFalse(self.tc.early_black(self.samples, *values))


if __name__ == '__main__':
    unittest.main()
