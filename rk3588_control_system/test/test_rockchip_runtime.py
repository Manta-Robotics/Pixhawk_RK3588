import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from rockchip_runtime import compatible_rknn_model, detect_soc, npu_core_mask
from validate_board_profile import validate_overlays


class RockchipRuntimeTests(unittest.TestCase):
    def test_device_tree_identifies_rk3576(self):
        with tempfile.TemporaryDirectory() as temporary:
            tree = Path(temporary) / 'compatible'
            tree.write_bytes(b'vendor,board\0rockchip,rk3576\0')
            self.assertEqual(detect_soc(tree), 'rk3576')

    def test_rk3588_weights_cannot_be_used_silently_on_rk3576(self):
        with tempfile.TemporaryDirectory() as temporary:
            old = Path(temporary) / 'face-rk3588-fp16.rknn'
            old.touch()
            with self.assertRaisesRegex(RuntimeError, 'rk3576 requires a matching RKNN model'):
                compatible_rknn_model(old, 'rk3576')
            new = old.with_name('face-rk3576-fp16.rknn')
            new.touch()
            self.assertEqual(compatible_rknn_model(old, 'rk3576'), new)

    def test_core_selection_handles_two_and_three_core_boards(self):
        class OldRuntime:
            NPU_CORE_AUTO, NPU_CORE_0_1, NPU_CORE_0_1_2 = 0, 3, 7
        self.assertEqual(npu_core_mask(OldRuntime, 'rk3576'), 3)
        self.assertEqual(npu_core_mask(OldRuntime, 'rk3588'), 7)
        self.assertEqual(npu_core_mask(OldRuntime, 'rk3566'), 0)
        class NewRuntime(OldRuntime):
            NPU_CORE_ALL = 0xffff
        self.assertEqual(npu_core_mask(NewRuntime, 'rk3576'), NewRuntime.NPU_CORE_ALL)

    def test_rk3576_rejects_legacy_uart_overlay_but_usb_and_network_need_no_overlay(self):
        config = {'serial_port': '/dev/ttyS1', 'pixhawk': {'uart_overlay': 'rk3588-uart1'},
                  'camera': {'enabled': False}, 'gimbal': {'control_transport': 'udp'}}
        self.assertIn('running board is rk3576', validate_overlays(config, 'rk3576')[0])
        config['serial_port'] = '/dev/serial/by-id/usb-ArduPilot'
        self.assertEqual(validate_overlays(config, 'rk3576'), [])


if __name__ == '__main__':
    unittest.main()
