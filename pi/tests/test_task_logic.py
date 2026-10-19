"""task_logic: очередь плиток по цветам и порядок по кольцам когнитивных меток."""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import task_logic as tl  # noqa: E402


class ColorQueueTest(unittest.TestCase):
    def test_order_and_out_of_order_tiles(self):
        q = tl.ColorQueue(["blue", "red", "black"])
        tiles = {(1, 0): "red", (2, 0): "black", (0, 1): "white"}
        self.assertEqual(q.next_color(), "blue")
        self.assertEqual(q.targets(tiles), [])           # красная раньше синей - только запомнить
        self.assertFalse(q.is_turn((1, 0), "red"))
        tiles[(3, 3)] = "blue"
        self.assertEqual(q.targets(tiles), [(3, 3)])
        q.mark((3, 3))
        self.assertEqual(q.next_color(), "red")
        self.assertEqual(q.targets(tiles), [(1, 0)])
        q.mark((1, 0))
        q.mark((2, 0))
        self.assertTrue(q.finished())
        self.assertEqual(q.targets(tiles), [])

    def test_same_colour_twice_needs_two_tiles(self):
        q = tl.ColorQueue(["red", "blue", "red"])
        tiles = {(1, 0): "red", (2, 0): "blue", (4, 4): "red"}
        q.mark((1, 0))
        q.mark((2, 0))
        self.assertFalse(q.is_turn((1, 0), "red"))       # эту уже отработали
        self.assertEqual(q.targets(tiles), [(4, 4)])


class RingOrderTest(unittest.TestCase):
    def test_two_marks_agree(self):
        r = ["red", "blue", "black", "red", "blue"]
        self.assertEqual(tl.ring_order([r, list(r)]), r)

    def test_wrong_colours_go_to_nearest(self):
        self.assertEqual(tl.ring_order([["yellow", "green", "black"]]), ["red", "blue", "black"])

    def test_tie_goes_to_first_mark(self):
        self.assertEqual(tl.ring_order([["red", "blue"], ["black", "blue"]]), ["red", "blue"])

    def test_vote_fixes_one_bad_ring(self):
        a = ["red", "yellow", "black"]       # жёлтое - это красное (ближайшее)
        b = ["red", "blue", "black"]
        c = ["red", "blue", "black"]
        self.assertEqual(tl.ring_order([a, b, c]), ["red", "blue", "black"])

    def test_unread_ring_and_shorter_read(self):
        self.assertEqual(tl.ring_order([["red", None, "blue"], ["red", "black"]]),
                         ["red", "black", "blue"])
        self.assertEqual(tl.ring_order([]), [])


if __name__ == "__main__":
    unittest.main()
