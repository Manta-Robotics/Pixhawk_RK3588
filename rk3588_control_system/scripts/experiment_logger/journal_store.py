"""Durable, append-only storage for the v2 experiment journal."""
import csv
from datetime import datetime, timezone
import io
import json
import os
import re
from pathlib import Path
import shutil
import time
import uuid


SCHEMA_PATH = Path(__file__).resolve().parents[2] / 'config' / 'experiment_schema.json'
FILES = {'overview': 'state_overview.csv', 'events': 'events.csv', 'video': 'video_aligned.csv'}
DIAGNOSTIC_COLUMNS = ('sample_id', 'obs_ns', 'avail_ns', 'time_kind', 'quality', 'meta_json')
FIELDS = {
    'fc': ('fc_mode', 'armed'),
    'mode': ('control_owner', 'follow_active', 'follow_side', 'follow_method', 'follow_control_mode', 'gimbal_mode'),
    'att': ('heading_deg', 'yaw_rate_deg_s'), 'nav': ('ground_speed_m_s',),
    'imu': ('ax_m_s2', 'ay_m_s2', 'az_m_s2', 'acc_norm_m_s2', 'gx_rad_s', 'gy_rad_s', 'gz_rad_s'),
    'uwb': ('uwb_range_m', 'uwb_bearing_deg', 'uwb_elevation_deg'),
    'uwb_pred': ('uwb_pred_range_m', 'uwb_pred_bearing_deg'),
    'gimbal': ('gimbal_yaw_deg', 'gimbal_pitch_deg'),
    'gimbal_cmd': ('gimbal_cmd_yaw_deg', 'gimbal_cmd_pitch_deg', 'gimbal_cmd_yaw_rate_deg_s', 'gimbal_cmd_pitch_rate_deg_s'),
    'vision': ('vision_state', 'vision_frame_id', 'vision_last_measured_frame_id', 'detected', 'target_count', 'target_id', 'confidence', 'cx_norm', 'cy_norm'),
    'motor_cmd': ('cmd_left_pwm_us', 'cmd_right_pwm_us'), 'pwm': ('left_pwm_us', 'right_pwm_us'),
    'esc': ('left_erpm', 'right_erpm', 'left_rpm', 'right_rpm'),
    'video': ('record_phase', 'recording_id', 'clip_id', 'frame_id'),
}


def read_json(path, default=None):
    try:
        return json.loads(Path(path).read_text(encoding='utf-8'))
    except FileNotFoundError:
        return default


def sync_directory(path):
    fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def replace_file(path, data):
    path = Path(path)
    temp = path.with_name(f'.{path.name}.{uuid.uuid4().hex}.tmp')
    try:
        with temp.open('wb') as file:
            file.write(data)
            file.flush()
            os.fsync(file.fileno())
        os.replace(temp, path)
        sync_directory(path.parent)
    finally:
        temp.unlink(missing_ok=True)


def json_line(value):
    return (json.dumps(value, ensure_ascii=False, separators=(',', ':'), allow_nan=False) + '\n').encode('utf-8')


def csv_value(value):
    if isinstance(value, (bool, dict, list)):
        return json_line(value).decode('utf-8').rstrip('\n')
    return value


def csv_bytes(fields, rows, header=False):
    out = io.StringIO(newline='')
    writer = csv.DictWriter(out, fields, extrasaction='ignore')
    if header:
        writer.writeheader()
    for row in rows:
        writer.writerow({k: csv_value(v) for k, v in row.items()})
    return out.getvalue().encode('utf-8')


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec='milliseconds').replace('+00:00', 'Z')


def boot_id():
    return Path('/proc/sys/kernel/random/boot_id').read_text(encoding='ascii').strip()


def committed_lines(path, limit):
    if not limit:
        return
    with Path(path).open('rb') as file:
        while file.tell() < limit:
            yield file.readline(limit - file.tell()).decode('utf-8')


class SessionStore:
    def __init__(self, config, *, boot=None, now=time.monotonic_ns):
        self.config, self.now = config, now
        self.root = Path(config['directory'])
        self.boot = boot or boot_id()
        self.schema = read_json(SCHEMA_PATH)
        self.rotate_ns = round(max(.01, float(config.get('rotate_seconds', 900))) * 1e9)
        self.directory = None
        self.events = set()
        self.samples = set()
        self.marker_ns = None
        self.ros_status = {}
        self.queue_drops = 0
        self.identify_session()

    def identify_session(self):
        current = read_json(self.root / 'current.json') or {}
        directory = Path(current.get('session') or self.root)
        if not directory.is_absolute():
            directory = self.root / directory
        if directory.parent.resolve() == self.root.resolve() and directory != self.root:
            manifest = read_json(directory / 'manifest.json') or {}
            if manifest.get('schema_version') == 2 and manifest.get('status') == 'running':
                if manifest.get('boot_id') == self.boot:
                    self.directory, self.manifest = directory, manifest
                else:
                    manifest.update(status='partial', end_reason='host_restarted')
                    replace_file(directory / 'manifest.json', json_line(manifest))
        if self.directory is None:
            session_id = 'session_' + time.strftime('%Y%m%d_%H%M%S') + '_' + uuid.uuid4().hex[:8]
            self.directory = self.root / session_id
            self.manifest = {'schema_version': 2, 'session_id': session_id, 'boot_id': self.boot,
                'session_start_mono_ns': str(self.now()), 'started_utc': utc_now(), 'ended_mono_ns': None,
                'status': 'running', 'diagnostic_config': self.schema['diagnostic'],
                'calibration': {}, 'channel_mapping': {'motor_channels': self.config.get('motor_channels', [1, 3])},
                'rpm_conversion': self.config.get('rpm_conversion'), 'clips': [], 'gaps': []}
        self.session = self.manifest['session_id']
        self.start = int(self.manifest['session_start_mono_ns'])
        self.ledger = read_json(self.directory / 'commit.json') or {
            'epoch': None, 'ack_seq': 0, 'row_no': None, 'event_no': 0, 'offsets': {}, 'gaps': [], 'drop_total': 0,
            'marker_ns': None, 'last_flush_ns': None}
        self.opened = False

    def storage_guard(self):
        self.root.mkdir(parents=True, exist_ok=True)
        free = shutil.disk_usage(self.root).free
        if free < self.config.get('min_free_gb', 2) * 1024**3:
            raise OSError('磁盘剩余空间低于实验日志限额')
        used = sum(p.stat().st_size for p in self.root.glob('session_*/**/*') if p.is_file())
        if used >= self.config.get('max_total_gb', 20) * 1024**3:
            raise OSError('实验日志已达到存储上限')

    def open(self):
        if self.opened:
            return
        self.storage_guard()
        self.directory.mkdir(exist_ok=True)
        (self.directory / 'diagnostics').mkdir(exist_ok=True)
        for relative, limit in self.ledger['offsets'].items():
            with (self.directory / relative).open('r+b') as file:
                if file.seek(0, os.SEEK_END) < limit:
                    raise OSError(f'已提交实验文件不完整：{relative}')
                file.truncate(limit)
        own_shards = '|'.join(('samples', 'mavlink', 'state', 'ros_topics', 'raw_events', 'imu-raw', 'vision-targets', *self.schema['sources']))
        for shard in (self.directory / 'diagnostics').iterdir():
            relative = str(shard.relative_to(self.directory))
            if (relative not in self.ledger['offsets'] and not shard.is_symlink() and shard.is_file()
                    and re.fullmatch(rf'(?:{own_shards})-[0-9]+\.(?:csv|jsonl)', shard.name)):
                with shard.open('wb'):
                    pass
        for row in self.read_events():
            self.events.add(row['event_id'])
        for relative, size in self.ledger['offsets'].items():
            if relative.startswith('diagnostics/samples-'):
                for line in committed_lines(self.directory / relative, size):
                    data = json.loads(line)
                    self.samples.add((data['source'], data['sample_id']))
        self.manifest['gaps'] = self.ledger['gaps']
        self.write_manifest()
        replace_file(self.root / 'current.json', json_line({'session': str(self.directory), 'session_id': self.session, 'pid': os.getpid()}))
        self.opened = True

    def write_manifest(self):
        replace_file(self.directory / 'manifest.json', json_line(self.manifest))

    def read_events(self):
        return csv.DictReader(committed_lines(self.directory / FILES['events'], self.ledger['offsets'].get(FILES['events'], 0)))

    def elapsed_ms(self, ns):
        return (int(ns) - self.start) / 1e6 if ns is not None else None

    def flush(self, messages, raw=(), extra_drops=0, *, normalize_imu=None):
        self.open()
        old = self.ledger
        ledger = {**old, 'offsets': dict(old['offsets']), 'gaps': list(old['gaps'])}
        ledger['drop_total'] += extra_drops
        needs_replay = False
        changes = {}
        written_events, written_samples = set(), set()
        now = self.now()
        shard = max(0, (now - self.start) // self.rotate_ns)

        def append(relative, payload):
            changes.setdefault(relative, bytearray()).extend(payload)

        def row(kind, record):
            relative = FILES[kind]
            append(relative, csv_bytes(self.schema[kind], [record], not ledger['offsets'].get(relative) and relative not in changes))

        def event(payload):
            event_id = payload['event_id']
            if not isinstance(event_id, str) or not event_id:
                raise ValueError('invalid event identity')
            if event_id in self.events or event_id in written_events:
                return
            written_events.add(event_id)
            ledger['event_no'] += 1
            ns = payload.get('mono_ns')
            row('events', {**payload, 'session_id': self.session, 'event_no': ledger['event_no'],
                't_ms': self.elapsed_ms(ns), 'detail_json': payload.get('detail_json') or {}})

        def gap(epoch, first, last, reason, **extra):
            ns = str(now)
            detail = {'epoch': epoch, 'first_missing': first, 'last_missing': last, 'reason': reason, **extra}
            ledger['gaps'].append({'mono_ns': ns, **detail})
            count = last - first + 1 if isinstance(first, int) and isinstance(last, int) else 0
            ledger['drop_total'] += max(0, count)
            event({'event_id': f'{self.session}/gap/{len(ledger["gaps"])}', 'mono_ns': ns, 'utc': utc_now(),
                'module': 'logger', 'type': 'data_gap', 'level': 'warn', 'code': 'DATA_GAP',
                'message': '日志数据存在缺口', 'detail_json': detail})

        if extra_drops:
            gap(ledger['epoch'], None, None, 'diagnostic_input_dropped', count=extra_drops)
        for message in messages:
            kind, epoch = message['kind'], message.get('epoch')
            if kind == 'ready':
                if epoch != ledger['epoch']:
                    first = message.get('session_first_seq', message.get('first_seq', 1))
                    # A new logical session can start after earlier global sequence numbers.
                    # For a restart of the same session the stream is a new Node epoch.
                    baseline = first - 1 if ledger['epoch'] is None and type(first) is int and first > 0 else 0
                    ledger.update(epoch=epoch, ack_seq=baseline)
                for key in ('diagnostic_config', 'calibration', 'channel_mapping', 'rpm_conversion'):
                    if key in message:
                        self.manifest[key] = message[key]
                continue
            if epoch != ledger['epoch']:
                continue
            if kind == 'gap':
                first, last = message.get('first_missing'), message.get('last_missing')
                if first is None or last is None:
                    gap(epoch, None, None, message.get('reason', 'unknown_gap'))
                elif first <= ledger['ack_seq'] + 1 and last > ledger['ack_seq']:
                    gap(epoch, max(first, ledger['ack_seq'] + 1), last, message.get('reason', 'journal_gap'))
                    ledger['ack_seq'] = last
                continue
            seq = message['seq']
            if seq <= ledger['ack_seq']:
                continue
            if seq > ledger['ack_seq'] + 1:
                # Reconnect from the durable cursor; a missing envelope is
                # never treated as received without an explicit gap.
                needs_replay = True
                break
            try:
                payload = message['payload']
                if kind == 'overview':
                    row_no = payload['row_no']
                    if ledger['row_no'] is None or row_no > ledger['row_no']:
                        mono_ns = str(int(payload['mono_ns']))
                        if ledger['row_no'] is not None and row_no > ledger['row_no'] + 1:
                            gap(epoch, None, None, 'overview_ticks_missed', first_row_no=ledger['row_no'] + 1, last_row_no=row_no - 1)
                        row('overview', {**payload, 'session_id': self.session, 'mono_ns': mono_ns,
                            't_ms': self.elapsed_ms(mono_ns)})
                        ledger['row_no'] = row_no
                elif kind == 'event':
                    event(payload)
                elif kind == 'sample':
                    source, sample_id = payload['source'], payload['sample_id']
                    key = (source, sample_id)
                    if source not in self.schema['sources'] or not isinstance(sample_id, str) or not sample_id:
                        raise ValueError('invalid sample identity')
                    if key not in self.samples and key not in written_samples:
                        values, meta = payload.get('values'), payload.get('meta')
                        if not isinstance(values, dict) or not isinstance(meta, dict):
                            raise ValueError('invalid sample values or metadata')
                        sample_record = json_line(payload)
                        written_samples.add(key)
                        append(f'diagnostics/samples-{shard:04d}.jsonl', sample_record)
                        source_fields = FIELDS.get(source, ())
                        columns = list(DIAGNOSTIC_COLUMNS) + list(source_fields)
                        relative = f'diagnostics/{source}-{shard:04d}.csv'
                        append(relative, csv_bytes(columns, [{**values, **payload, 'meta_json': meta}],
                            not ledger['offsets'].get(relative) and relative not in changes))
                        targets = meta.get('targets')
                        if source == 'vision' and isinstance(targets, list):
                            for target in targets:
                                if isinstance(target, dict):
                                    append(f'diagnostics/vision-targets-{shard:04d}.jsonl', json_line({
                                        'sample_id': sample_id, 'detector': meta.get('detector'),
                                        'frame_id': meta.get('frame_id'), 'target': target}))
            except (KeyError, TypeError, ValueError):
                gap(epoch, seq, seq, 'invalid_journal_payload')
            ledger['ack_seq'] = seq

        marker = read_json(self.root / 'marker.json') or {}
        marker_ns = marker.get('mono_ns')
        if isinstance(marker_ns, int) and str(marker_ns) != ledger['marker_ns'] and self.start <= marker_ns <= now:
            event({'event_id': f'{self.session}/mark/{marker_ns}', 'mono_ns': str(marker_ns), 'utc': utc_now(),
                'module': 'logger', 'type': 'manual_mark', 'level': 'info', 'message': marker.get('experiment_label', ''),
                'detail_json': {k: marker.get(k) for k in ('water_context', 'hull_stationary_observer')}})
            ledger['marker_ns'] = str(marker_ns)
        for packet in raw:
            kind = packet.get('kind')
            name = {'mavlink': 'mavlink', 'snapshot': 'state', 'ros': 'ros_topics'}.get(kind, 'raw_events')
            # Legacy raw packets preserve their content, including unverified source values.
            append(f'diagnostics/{name}-{shard:04d}.jsonl',
                (json.dumps(packet, ensure_ascii=False, separators=(',', ':'), default=str) + '\n').encode('utf-8'))
            original = packet.get('message') or {}
            message_type = original.get('mavpackettype', '') if isinstance(original, dict) else ''
            if kind == 'mavlink' and normalize_imu is not None and (message_type.endswith('_IMU') or message_type.startswith('SCALED_IMU')):
                fields = ('source', 'system_id', 'component_id', 'sensor_id', 'source_mono_ns', 'source_wall_ns',
                          'ax_m_s2', 'ay_m_s2', 'az_m_s2', 'gx_rad_s', 'gy_rad_s', 'gz_rad_s')
                item = {**normalize_imu(original), 'source': message_type, 'system_id': packet.get('system_id'),
                    'component_id': packet.get('component_id'), 'sensor_id': original.get('id'),
                    'source_mono_ns': packet.get('mono_ns'), 'source_wall_ns': packet.get('wall_ns')}
                relative = f'diagnostics/imu-raw-{shard:04d}.csv'
                append(relative, csv_bytes(fields, [item], not ledger['offsets'].get(relative) and relative not in changes))
        for kind in FILES:
            relative = FILES[kind]
            if relative not in ledger['offsets'] and relative not in changes:
                append(relative, csv_bytes(self.schema[kind], [], header=True))
        if not changes and ledger == old:
            return needs_replay
        for relative, payload in changes.items():
            path = self.directory / relative
            mode = 'r+b' if path.exists() else 'w+b'
            with path.open(mode) as file:
                offset = old['offsets'].get(relative, 0)
                file.seek(offset)
                file.write(payload)
                file.truncate()
                file.flush()
                os.fsync(file.fileno())
                ledger['offsets'][relative] = file.tell()
        if any(relative.startswith('diagnostics/') for relative in changes):
            sync_directory(self.directory / 'diagnostics')
        ledger['last_flush_ns'] = str(self.now())
        replace_file(self.directory / 'commit.json', json_line(ledger))
        self.ledger = ledger
        self.events.update(written_events)
        self.samples.update(written_samples)
        self.manifest['gaps'] = ledger['gaps']
        self.write_manifest()
        return needs_replay

    def finish(self, status, reason=None, ended_mono_ns=None):
        if self.opened:
            now = self.now()
            cutoff = int(ended_mono_ns) if isinstance(ended_mono_ns, str) and ended_mono_ns.isdigit() else now
            if not self.start <= cutoff <= now:
                cutoff = now
            self.manifest.update(status=status, ended_mono_ns=str(cutoff), end_reason=reason)
            self.write_manifest()
