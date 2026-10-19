"""watch_logic.guess_rings: решение по нескольким неполным чтениям мишени."""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import watch_logic as wl  # noqa: E402


def reading(rings, ok, agree):
    return {'rings': rings, 'ring_ok': ok, 'ring_agree': agree}


class TestGuessRings(unittest.TestCase):
    def test_reliable_ring_wins_over_later_misread(self):
        # заезд 18.10: первое чтение верное (чёрное ненадёжно, но 0.42 за
        # чёрное), потом мишень мелкая и чёрное то зелёное, то синее
        rs = [reading(['yellow', 'red', 'green', 'red', 'black'],
                      [True] * 4 + [False], [1, 1, 1, 1, 0.42]),
              reading(['yellow', 'red', 'green', 'red', 'black'],
                      [True] * 5, [1, 1, 1, 1, 0.7]),
              reading(['yellow', 'red', 'green', 'red', 'green'],
                      [True] * 4 + [False], [1, 1, 1, 1, 0.5])]
        self.assertEqual(wl.guess_rings(rs), ['yellow', 'red', 'green', 'red', 'black'])

    def test_no_reliable_reading_votes_by_agreement(self):
        rs = [reading(['yellow', 'red', 'green', 'red', 'black'], [True] * 4 + [False],
                      [1, 1, 1, 1, 0.55]),
              reading(['yellow', 'red', 'green', 'red', 'green'], [True] * 4 + [False],
                      [1, 1, 1, 1, 0.3])]
        self.assertEqual(wl.guess_rings(rs)[4], 'black')

    def test_unseen_ring_counts_as_zero(self):
        rs = [reading(['yellow', 'red', 'green', 'red', '?'], [True] * 4 + [False],
                      [1, 1, 1, 1, 0.0])]
        self.assertEqual(wl.guess_rings(rs)[4], 'yellow')

    def test_old_readings_without_reliability(self):
        self.assertEqual(wl.guess_rings([{'rings': ['black', 'blue', 'yellow', 'yellow', 'blue']}]),
                         ['black', 'blue', 'yellow', 'yellow', 'blue'])


if __name__ == '__main__':
    unittest.main()
