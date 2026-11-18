"""Storage/recovery and causal media tests with injected clocks and data."""
import csv
import io
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts' / 'experiment_logger'))

from history_query import HistoryError, export_video, list_events, list_sessions, load_session, locate, read_clips, session_manifest
from journal_store import SessionStore, json_line, replace_file


class JournalTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.config = {'directory': str(self.root / 'logs'), 'rotate_seconds': .03, 'min_free_gb': 0}
        self.now = 1000000000
        self.root = Path(self.config['directory'])
        self.store = SessionStore(self.config, boot='fixture-boot', now=lambda: self.now)
        self.epoch = 'fixture-node'

    def envelope(self, kind, seq, payload):
        return {'v': 2, 'kind': kind, 'seq': seq, 'epoch': self.epoch, 'session_id': self.store.session, 'payload': payload}

    def ready(self, seq=1):
        return {'kind': 'ready', 'epoch': self.epoch, 'first_seq': seq, 'session_first_seq': seq}

    def sample(self, seq, source, name, mt, at, values, **meta):
        return self.envelope('sample', seq, {'source': source, 'sample_id': name, 'obs_ns': str(mt), 'avail_ns': str(at),
            'time_kind': 'host_receive', 'quality': 'measured', 'values': values, 'meta': meta})

    def event(self, seq, id, at, event_type, module='mode', detail=None):
        return self.envelope('event', seq, {'event_id': id, 'mono_ns': str(at), 'type': event_type,
            'module': module, 'level': 'info', 'message': id, 'detail_json': detail})

    def test_recovery_keeps_offsets_ids_and_one_logical_session(self):
        row = self.envelope('overview', 8, {'row_no': 0, 'mono_ns': str(self.now), 'armed': False, 'uwb_bearing_deg': 0})
        self.store.flush([self.ready(8), row, self.event(9, 'evt-a', self.now, 'manual_mark'),
            self.sample(10, 'uwb', 'uwb-a', self.now, self.now, {'uwb_range_m': 4.2})])
        self.assertEqual(self.store.ledger['ack_seq'], 10)
        path = self.store.directory / 'state_overview.csv'
        with path.open('ab') as target:
            target.write(b'outside-last-commit\n')
        self.now += 50_000_000
        old = self.store
        self.store = SessionStore(self.config, boot='fixture-boot', now=lambda: self.now)
        self.assertEqual(self.store.session, old.session)
        more = self.envelope('overview', 13, {'row_no': 1, 'mono_ns': str(self.now)})
        self.store.flush([self.ready(8), row, self.event(9, 'evt-a', self.now, 'manual_mark'),
            {'kind': 'gap', 'epoch': self.epoch, 'first_missing': 11, 'last_missing': 12, 'reason': 'buffer_expired'}, more,
            self.sample(14, 'uwb', 'uwb-b', self.now, self.now, {'uwb_range_m': 4.3})])
        self.assertEqual(self.store.ledger['ack_seq'], 14)
        self.assertEqual(self.store.ledger['drop_total'], 2)
        self.assertEqual(len(list(self.store.directory.glob('diagnostics/samples-*'))), 2)
        self.assertNotIn(b'outside-last-commit', path.read_bytes())
        with path.open(newline='') as file:
            rows = list(csv.DictReader(file))
        self.assertEqual([row['row_no'] for row in rows], ['0', '1'])
        self.assertEqual((rows[0]['armed'], rows[0]['uwb_bearing_deg']), ('false', '0'))
        items = list_events(load_session(self.root, self.store.session), {'limit': 1})
        self.assertEqual(items['items'][0]['event_id'], 'evt-a')
        self.assertTrue(items['has_more'])
        page = list_events(load_session(self.root, self.store.session), {'after': items['next_after']})
        self.assertEqual(page['items'][0]['type'], 'data_gap')
        self.assertEqual(page['items'][0]['detail_json']['first_missing'], 11)
        self.store.finish('closed')
        self.assertNotEqual(SessionStore(self.config, boot='fixture-boot', now=lambda: self.now).session, self.store.session)

    def test_failed_commit_is_invisible_and_replay_recovers(self):
        first = self.envelope('overview', 1, {'row_no': 0, 'mono_ns': str(self.now)})
        self.store.flush([self.ready(), first])
        retry = self.event(2, 'recoverable', self.now, 'manual_mark')

        def replace_except_commit(path, data):
            if path.name == 'commit.json':
                raise OSError(28, 'fixture ENOSPC')
            return replace_file(path, data)

        with patch('journal_store.replace_file', replace_except_commit):
            with self.assertRaises(OSError):
                self.store.flush([retry])
        self.assertEqual(self.store.ledger['ack_seq'], 1)
        session = load_session(self.root, self.store.session)
        self.assertEqual(list_events(session, {})['items'], [])
        self.store = SessionStore(self.config, boot='fixture-boot', now=lambda: self.now)
        self.store.flush([self.ready(), retry])
        self.assertEqual(self.store.ledger['ack_seq'], 2)
        page = list_events(load_session(self.root, self.store.session), {})
        self.assertEqual([(row['event_no'], row['event_id']) for row in page['items']], [(1, 'recoverable')])

    def test_node_epoch_change_keeps_event_and_overview_primary_keys(self):
        old = self.envelope('overview', 1, {'row_no': 0, 'mono_ns': str(self.now)})
        self.store.flush([self.ready(), old])
        self.epoch = 'fixture-node-restarted'
        self.now += 50_000_000
        new = self.envelope('overview', 1, {'row_no': 1, 'mono_ns': str(self.now)})
        gap = {'kind': 'gap', 'epoch': self.epoch, 'first_missing': None, 'last_missing': None, 'reason': 'node_restart'}
        self.store.flush([self.ready(), gap, new])
        self.assertEqual(self.store.ledger['row_no'], 1)
        self.assertEqual(self.store.ledger['ack_seq'], 1)
        self.assertEqual(self.store.ledger['gaps'][0]['reason'], 'node_restart')

    def test_exact_frame_reconstruction_uses_observation_and_availability_clocks(self):
        stamp = self.now + 10_000_000_000
        self.now = stamp + 400_000_000
        samples = [self.ready(),
            self.sample(1, 'mode', 'mode-a', stamp-5_000_000, stamp-3_000_000,
                        {'follow_active': False, 'follow_side': 'left', 'gimbal_mode': 'swimmer'}, mode_generation=1),
            self.sample(2, 'uwb', 'uwb-a', stamp-10_000_000, stamp-7_000_000,
                        {'uwb_range_m': 4.2, 'uwb_bearing_deg': -179}, calibration_id='receiver'),
            self.sample(3, 'gimbal', 'gimbal-a', stamp, stamp, {'gimbal_yaw_deg': 179}, calibration_id='mount'),
            self.sample(4, 'vision', 'known-before', stamp-20_000_000, stamp-7_000_000,
                        {'vision_state': 'no_target', 'detected': False, 'target_count': 0},
                        frame_id='stream/f41', detector='swimmer', mode_generation=1),
            self.sample(5, 'vision', 'returned-later', stamp, stamp+180_000_000,
                        {'vision_state': 'detected', 'detected': True, 'target_count': 1},
                        frame_id='stream/f42', detector='swimmer', mode_generation=1),
            self.sample(6, 'vision', 'old-generation', stamp, stamp+190_000_000,
                        {'vision_state': 'no_target', 'detected': False, 'target_count': 0},
                        frame_id='stream/f42', detector='face', mode_generation=0, primary=False),
            self.sample(7, 'uwb', 'too-late', stamp-1, stamp+200_000_000, {'uwb_range_m': 10}),
            self.event(8, 'applied', stamp + 10_000_000, 'mode_applied', detail={'axis': 'follow_active', 'actual': True})]
        self.store.flush(samples)
        folder = self.root / 'clip_indexes'
        folder.mkdir()
        recordings = self.root / 'recordings'
        recordings.mkdir()
        (recordings / 'clip.mp4').touch()
        frame = {'display_no': 50, 'frame_id': 'stream/f42', 'play_us': 2000000, 'play_end_us': 2100000,
                 'frame_mono_ns': str(stamp), 'frame_mono_end_ns': str(stamp+100_000_000), 'identity': 'exact',
                 'file_pts': '1600', 'tb_num': 1, 'tb_den': 800, 'time_kind': 'host_receive'}
        clip = {'clip_id': 'c2', 'boot_id': self.store.boot, 'recording_id': 'r1', 'file_name': 'clip.mp4',
                'index_state': 'ready', 'frames': [frame], 'mono_start_ns': str(stamp),
                'mono_end_ns': str(stamp+100_000_000)}
        replace_file(folder / 'c2.json', json_line(clip))
        self.store.finish('closed')
        session = load_session(self.root, self.store.session)
        clips = read_clips(session)
        result = locate(session, clips, {'clip_id': 'c2', 'display_no': '50'}, recordings)
        self.assertEqual(result['match'], 'matched')
        self.assertEqual(result['frame']['t_ms'], 10000)
        self.assertEqual(result['state']['follow_active'], False)
        self.assertEqual(result['state']['uwb_range_m'], 4.2)
        self.assertEqual(result['state']['vision_sample_id'], 'known-before')
        self.assertEqual(result['state']['bearing_error_deg'], 2)
        row = list(csv.DictReader(io.StringIO(export_video(session).decode('utf-8'))))[0]
        self.assertEqual((row['frame_detected'], row['vision_result_ns']), ('true', str(stamp+180_000_000)))
        self.assertEqual(row['vision_sample_id'], 'known-before')
        self.assertEqual(locate(session, clips, {'t_ms': '10099.9'}, recordings)['match'], 'matched')
        self.assertEqual(locate(session, clips, {'t_ms': '10100'}, recordings)['match'], 'no_video')
        self.assertEqual(locate(session, clips, {'t_ms': '10100'}, recordings)['state']['follow_active'], True)
        (recordings / 'clip.mp4').unlink()
        self.assertEqual(locate(session, clips, {'clip_id': 'c2', 'play_us': '2000000'}, recordings)['match'], 'media_missing')
        with self.assertRaises(HistoryError) as error:
            locate(session, clips, {'play_us': '1', 'display_no': '2', 'clip_id': 'c2'}, recordings)
        self.assertEqual(error.exception.status, 400)

    def write_clip(self, clip_id, rows, status='ready'):
        index = self.root / 'clip_indexes'
        index.mkdir(exist_ok=True)
        recordings = self.root / 'recordings'
        recordings.mkdir(exist_ok=True)
        name = f'{clip_id}.mp4'
        (recordings / name).touch()
        stamps = [int(row['frame_mono_ns']) for row in rows if row.get('frame_mono_ns')]
        clip = {'clip_id': clip_id, 'recording_id': 'recording-a', 'boot_id': self.store.boot,
                'session_id': self.store.session, 'file_name': name, 'index_state': status, 'frames': rows,
                'mono_start_ns': str(min(stamps)) if stamps else None,
                'mono_end_ns': str(max(stamps) + 50_000_000) if stamps else None}
        replace_file(index / f'{clip_id}.json', json_line(clip))
        return recordings

    def test_locator_returns_ambiguous_candidates_and_reports_pending_index(self):
        self.store.flush([self.ready()])
        self.now += 100_000_000
        frame = {'frame_mono_ns': str(self.now), 'frame_mono_end_ns': str(self.now + 25_000_000),
                 'play_us': 5000, 'play_end_us': 30000, 'frame_id': None, 'identity': 'unknown'}
        recordings = self.write_clip('dups', [{**frame, 'display_no': 1}, {**frame, 'display_no': 2}])
        self.write_clip('pending', [], 'building')
        self.write_clip('failed', [], 'failed')
        session = load_session(self.root, self.store.session)
        clips = read_clips(session)
        result = locate(session, clips, {'t_ms': '100'}, recordings)
        self.assertEqual((result['match'], result['clip_id'], result['candidate_display_nos']), ('ambiguous', 'dups', [1, 2]))
        result = locate(session, clips, {'clip_id': 'dups', 'play_us': '30000'}, recordings)
        self.assertEqual(result['match'], 'no_video')
        for name, code in [('pending', 'INDEX_BUILDING'), ('failed', 'INDEX_FAILED')]:
            with self.assertRaises(HistoryError) as error:
                locate(session, clips, {'clip_id': name, 'play_us': '0'}, recordings)
            self.assertEqual((error.exception.status, error.exception.code), (409, code))
        self.write_clip('overlap', [{**frame, 'display_no': 1}])
        result = locate(session, read_clips(session), {'t_ms': '100'}, recordings)
        self.assertEqual(result['candidate_clip_ids'], ['dups', 'overlap'])
        self.assertNotIn('clip_id', result)

    def test_cross_session_index_keeps_unassigned_frames_and_rebuilds_late_result(self):
        start, first_sid = self.now, self.store.session
        first_at, second_at = start + 100_000_000, start + 600_000_000
        cutoff = start + 300_000_000
        self.now = start + 500_000_000
        self.store.flush([self.ready(), self.sample(1, 'mode', 'mode-a', start, start,
            {'gimbal_mode': 'swimmer'}, mode_generation=7)])
        recordings = self.write_clip('cross', [
            {'display_no': 0, 'frame_id': 'clip/F0', 'frame_mono_ns': str(first_at),
             'frame_mono_end_ns': str(first_at + 250_000_000), 'play_us': 0, 'identity': 'exact'},
            {'display_no': 1, 'frame_id': 'clip/F1', 'frame_mono_ns': str(second_at),
             'play_us': 1200, 'identity': 'exact'},
            {'display_no': 2, 'frame_id': None, 'frame_mono_ns': None, 'play_us': 2400, 'identity': 'unknown'},
        ])
        self.store.finish('closed', ended_mono_ns=str(cutoff))
        first = load_session(self.root, first_sid)
        self.assertEqual(first['manifest']['ended_mono_ns'], str(cutoff))
        clips = read_clips(first)
        self.assertEqual([r['display_no'] for r in clips[0]['frames']], [0])
        self.assertEqual(clips[0]['unassigned_frame_count'], 1)
        self.assertEqual(locate(first, clips, {'t_ms': '310'}, recordings)['match'], 'no_video')
        self.assertEqual(list(csv.DictReader(io.StringIO(export_video(first).decode())))[0]['frame_detected'], '')
        self.store = SessionStore(self.config, boot='fixture-boot', now=lambda: self.now)
        self.now = start + 900_000_000
        late = self.sample(1, 'vision', 'late-f0', first_at, second_at,
            {'vision_state': 'detected', 'detected': True, 'target_count': 1}, frame_id='clip/F0',
            detector='swimmer', mode_generation=7)
        self.store.flush([self.ready(), late])
        self.store.finish('closed')
        second = load_session(self.root, self.store.session)
        self.assertEqual([r['display_no'] for r in read_clips(second)[0]['frames']], [1])
        rebuilt = list(csv.DictReader(io.StringIO(export_video(first, persist=True).decode())))[0]
        self.assertEqual((rebuilt['frame_detected'], rebuilt['vision_result_ns']), ('true', str(second_at)))
        self.assertEqual(rebuilt['vision_sample_id'], '')

    def test_old_manifest_and_pagination_are_safe(self):
        root = self.root
        old = root / 'session_old'
        old.mkdir(parents=True)
        (old / 'manifest.json').write_text('{"schema":1,"created_wall_ns":1000,"running":false}')
        (old / 'samples.csv').touch()
        new = self.store.session
        self.store.flush([self.ready()])
        first = list_sessions(root, {'limit': 1})
        self.assertEqual(first['next_cursor'], new)
        second = list_sessions(root, {'before': new})
        self.assertEqual(second['items'][0]['session_id'], 'session_old')
        info = session_manifest(load_session(root, 'session_old'), [])
        self.assertEqual(info['schema_version'], 1)
        self.assertIn('samples.csv', info['legacy_files'])
        with self.assertRaises(HistoryError):
            load_session(root, '../secrets')


if __name__ == '__main__':
    unittest.main()
