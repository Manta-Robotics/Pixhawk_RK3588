import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class BridgeConfigEnvironmentTests(unittest.TestCase):
    def test_config_precedence_and_isolated_log_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            for variables, expected in (
                ({'MANTA_CONFIG_FILE': 'simulation.json', 'MANTA_CONFIG_PATH': 'legacy.json'}, 'simulation.json'),
                ({'MANTA_CONFIG_PATH': 'legacy.json'}, 'legacy.json'),
                ({}, str(ROOT / 'config' / 'system.config.json')),
            ):
                with self.subTest(variables=variables):
                    environment = dict(os.environ, MANTA_LOGS_DIR=directory)
                    for name in ('MANTA_CONFIG_FILE', 'MANTA_CONFIG_PATH'):
                        environment.pop(name, None)
                    environment.update(variables)
                    result = subprocess.run(
                        [sys.executable, '-c',
                         'import json, sys; sys.path.insert(0, "backend"); '
                         'import mavlink_bridge as bridge; '
                         'print(json.dumps([str(bridge.CONFIG_PATH), str(bridge.LOGS_DIR)]))'],
                        cwd=ROOT, env=environment, check=True, capture_output=True, text=True,
                        timeout=20,
                    )
                    self.assertEqual(json.loads(result.stdout), [expected, directory])


if __name__ == '__main__':
    unittest.main()
