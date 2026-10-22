import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('uwb_rates', Path(__file__).resolve().parents[1] / 'scripts/measure_uwb_forwarding.py')
rates = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rates)


class RateCounterTests(unittest.TestCase):
    def test_diagnostic_refreshes_not_http_polls_determine_rate(self):
        counter = rates.CounterWindow()
        for index in range(301):
            counter.update(100 + (index // 100) * 40, index / 100)
        self.assertEqual(counter.hz(), 40)

    def test_missing_counter_is_unavailable_and_restart_starts_a_new_window(self):
        counter = rates.CounterWindow()
        counter.update(None, 0)
        self.assertIsNone(counter.hz())
        counter.update(1000, 1)
        counter.update(1040, 2)
        counter.update(10, 3)
        self.assertIsNone(counter.hz())
        counter.update(50, 4)
        self.assertEqual(counter.hz(), 40)
        self.assertEqual(counter.resets, 1)


if __name__ == '__main__':
    unittest.main()
