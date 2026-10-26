"""Execute the deployed Lua source with simulated serial ports (optional lupa).

Run with `python -m unittest discover -s test -p test_pixhawk_uwb_lua.py -v`.
Install lupa in the test environment; no hardware or flight commands are used.
"""
import pathlib
import struct
import unittest

try:
    from lupa.lua53 import LuaRuntime
except ImportError:
    LuaRuntime = None

SCRIPT = pathlib.Path(__file__).resolve().parents[1] / 'scripts/pixhawk/uwb_aoa.lua'
HARNESS = '''
now_ms = 0
messages = {}
ports = {}
for id = 1, 2 do
    local port = {queue = {}, head = 1, reads = 0}
    function port:begin(baud) assert(baud == 115200) end
    function port:set_flow_control(value) assert(value == 0) end
    function port:available()
        local value = #self.queue - self.head + 1
        return {toint = function() return value end}
    end
    function port:read()
        local value = self.queue[self.head]
        if not value then return -1 end
        self.head = self.head + 1
        self.reads = self.reads + 1
        return value
    end
    ports[id] = port
end
serial = {find_serial = function(_, instance) return ports[instance + 1] end}
gcs = {}
function gcs:send_text(severity, text) end
function gcs:send_named_float(name, value)
    messages[#messages + 1] = {name = name, value = value, at = now_ms}
end
function millis() return {toint = function() return now_ms end} end
heartbeat_pending = true
channel = 1
send_clock_step = 0
drop_field = nil
mavlink = {}
function mavlink:init(depth, count) assert(depth == 4 and count == 1) end
function mavlink:register_rx_msgid(id) assert(id == 0) end
function mavlink:receive_chan()
    if heartbeat_pending then heartbeat_pending = false; return 'heartbeat', channel end
end
function mavlink:send_chan(chan, id, payload)
    assert(id == 251)
    local at, value, name = string.unpack('<I4fc10', payload)
    name = string.gsub(name, '%z', '')
    now_ms = now_ms + send_clock_step
    if name == drop_field then return false end
    messages[#messages + 1] = {name = name, value = value, at = at, channel = chan, payload = payload}
    return true
end
function feed(id, bytes)
    local queue = ports[id].queue
    for index = 1, #bytes do queue[#queue + 1] = string.byte(bytes, index) end
end
max_instructions = 0
function run_tick(at)
    now_ms = at
    if at % 1000 == 0 then heartbeat_pending = true end
    local instructions = 0
    debug.sethook(function() instructions = instructions + 100 end, '', 100)
    local next_callback, delay = callback()
    debug.sethook()
    assert(delay == 10)
    callback = next_callback
    max_instructions = math.max(max_instructions, instructions)
end
'''


def frame(seq=1, cm=125, az=3, el=4):
    data = bytearray(37)
    data[:4] = b'\xff' * 4
    struct.pack_into('>HHH', data, 4, 37, seq, 0x2001)
    struct.pack_into('>IIIhhH', data, 12, 5313, 5313, cm, az, el, 0x1234)
    for value in data[:-1]:
        data[-1] ^= value
    return bytes(data)


@unittest.skipUnless(LuaRuntime, 'Lua execution requires optional test dependency lupa')
class PixhawkUwbLuaTests(unittest.TestCase):
    def make_runtime(self, script=SCRIPT):
        lua = LuaRuntime(unpack_returned_tuples=True, encoding=None)
        lua.execute(HARNESS)
        callback, _ = lua.execute(script.read_bytes())
        lua.globals().callback = callback
        return lua

    def messages(self, lua, name):
        return [value for value in lua.globals().messages.values() if value[b'name'] == name]

    def test_two_40_hz_sources_forward_every_position_with_bounded_work(self):
        lua = self.make_runtime()
        for at in range(5, 10006, 5):
            if at % 25 == 0 and at <= 10000:
                for receiver in (1, 2):
                    lua.globals().feed(receiver, frame(at // 25, 125 + receiver))
            if at % 10 == 0:
                lua.globals().run_tick(at)
        # Drain the last input from both alternating visits.
        lua.globals().run_tick(10010)
        lua.globals().run_tick(10020)
        for receiver in (1, 2):
            prefix = f'UWB{receiver}_'.encode()
            samples = self.messages(lua, prefix + b'AZ')
            self.assertEqual(len(samples), 400)
            self.assertEqual(len({row[b'at'] for row in samples}), 400)
            self.assertEqual(lua.globals().ports[receiver].reads, 400 * 37)
            self.assertEqual(len(self.messages(lua, prefix + b'DIST')), 400)
            self.assertEqual(len(self.messages(lua, prefix + b'EL')), 400)
            self.assertLessEqual(len(self.messages(lua, prefix + b'GOOD')), 11)
            validity = self.messages(lua, prefix + b'VALID')
            self.assertLessEqual(len(validity), 12)
            self.assertTrue(all(row[b'value'] == 1 for row in validity))
        # Includes Lua mocks standing in for native C bindings. This is a VM
        # work bound, not an actual flight-controller CPU-time measurement.
        self.assertLess(lua.globals().max_instructions, 10000)

    def test_crc_and_range_rejection_then_first_good_sample_recovers(self):
        lua = self.make_runtime()
        bad = bytearray(frame())
        bad[-1] ^= 1
        for at, data in ((20, bytes(bad)), (60, frame(cm=0)), (100, frame(seq=3))):
            lua.globals().feed(1, data)
            for tick in range(at - 10, at + 30, 10):
                lua.globals().run_tick(tick)
        self.assertEqual(len(self.messages(lua, b'UWB1_AZ')), 1)
        self.assertEqual([m[b'value'] for m in self.messages(lua, b'UWB1_VALID')], [0, 1])
        names = [m[b'name'] for m in lua.globals().messages.values()]
        self.assertEqual(names[-5:], [b'UWB1_VALID', b'UWB1_PROTO', b'UWB1_DIST', b'UWB1_EL', b'UWB1_AZ'])

    def test_position_fields_share_source_clock_even_across_send_ticks(self):
        lua = self.make_runtime()
        lua.globals().send_clock_step = 1
        lua.globals().feed(1, frame())
        for at in range(10, 51, 10):
            lua.globals().run_tick(at)
        fields = [self.messages(lua, b'UWB1_' + field)[0] for field in (b'DIST', b'EL', b'AZ')]
        self.assertEqual(len({m[b'at'] for m in fields}), 1)
        self.assertTrue(all(m[b'channel'] == 1 for m in fields))
        # Check the exact payload layout independently of the Lua mock decoder.
        at, value, name = struct.unpack('<If10s', fields[0][b'payload'])
        self.assertEqual(at, fields[0][b'at'])
        self.assertAlmostEqual(value, 1.25)
        self.assertEqual(name.rstrip(b'\0'), b'UWB1_DIST')

    def test_out_of_range_angle_is_not_forwarded_as_a_position(self):
        lua = self.make_runtime()
        for at, data in ((20, frame(az=181)), (60, frame(el=-91)), (100, frame(seq=3))):
            lua.globals().feed(1, data)
            for tick in range(at - 10, at + 30, 10):
                lua.globals().run_tick(tick)
        self.assertEqual(len(self.messages(lua, b'UWB1_AZ')), 1)
        self.assertEqual([m[b'value'] for m in self.messages(lua, b'UWB1_VALID')], [0, 1])

    def test_partial_send_is_not_completed_and_next_observation_recovers(self):
        lua = self.make_runtime()
        lua.globals().drop_field = b'UWB1_EL'
        lua.globals().feed(1, frame())
        for at in range(10, 51, 10):
            lua.globals().run_tick(at)
        self.assertEqual(len(self.messages(lua, b'UWB1_DIST')), 1)
        self.assertEqual(len(self.messages(lua, b'UWB1_AZ')), 0)
        lua.globals().drop_field = None
        lua.globals().feed(1, frame(seq=2))
        for at in range(60, 101, 10):
            lua.globals().run_tick(at)
        self.assertEqual(len(self.messages(lua, b'UWB1_AZ')), 1)
        self.assertEqual(self.messages(lua, b'UWB1_AZ')[0][b'at'], self.messages(lua, b'UWB1_DIST')[-1][b'at'])

    def test_protocol_announcement_precedes_positions_and_repeats_after_heartbeat(self):
        lua = self.make_runtime()
        lua.globals().drop_field = b'UWB1_PROTO'
        lua.globals().feed(1, frame())
        for at in (10, 20):
            lua.globals().run_tick(at)
        self.assertFalse(self.messages(lua, b'UWB1_DIST'))
        lua.globals().drop_field = None
        for at in (30, 50, 1010):
            if at == 1010:
                lua.globals().heartbeat_pending = True
            lua.globals().feed(1, frame())
            lua.globals().run_tick(at)
            lua.globals().run_tick(at + 10)
        self.assertEqual(len(self.messages(lua, b'UWB1_PROTO')), 2)
        names = [m[b'name'] for m in lua.globals().messages.values()]
        self.assertLess(names.index(b'UWB1_PROTO'), names.index(b'UWB1_DIST'))
        self.assertTrue(all(m[b'value'] == 2 for m in self.messages(lua, b'UWB1_PROTO')))

    def test_inactive_channel_expires_and_new_heartbeat_restores_forwarding(self):
        lua = self.make_runtime()
        lua.globals().feed(1, frame())
        for at in (4010, 4020):
            lua.globals().run_tick(at)
        self.assertEqual(len(self.messages(lua, b'UWB1_AZ')), 0)
        lua.globals().channel = 3
        lua.globals().heartbeat_pending = True
        lua.globals().feed(1, frame(seq=2))
        for at in (4030, 4040):
            lua.globals().run_tick(at)
        samples = self.messages(lua, b'UWB1_AZ')
        self.assertEqual(len(samples), 1)
        self.assertEqual(samples[0][b'channel'], 3)

    def test_source_timestamp_preserves_uint32_precision_and_wrap(self):
        lua = self.make_runtime()
        lua.globals().feed(1, frame(seq=1) + frame(seq=2))
        for at in (0xFFFFFFF0, 0xFFFFFFFA, 4, 14):
            lua.globals().heartbeat_pending = True
            lua.globals().run_tick(at)
        self.assertEqual([m[b'at'] for m in self.messages(lua, b'UWB1_AZ')], [0xFFFFFFFA, 14])

    def test_burst_is_drained_without_giving_two_samples_one_timestamp(self):
        lua = self.make_runtime()
        lua.globals().feed(1, frame(seq=1) + frame(seq=2))
        for at in range(10, 61, 10):
            lua.globals().run_tick(at)
        samples = self.messages(lua, b'UWB1_AZ')
        self.assertEqual(len(samples), 2)
        self.assertNotEqual(samples[0][b'at'], samples[1][b'at'])


if __name__ == '__main__':
    unittest.main()
