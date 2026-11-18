"""Committed history queries and causal, per-frame CSV reconstruction."""
import argparse
from bisect import bisect_right
from collections import defaultdict
import csv
from datetime import datetime, timezone
from decimal import Decimal, InvalidOperation
import io
import json
import math
from pathlib import Path
import re
import sys
from urllib.parse import quote

from journal_store import FIELDS, FILES, SCHEMA_PATH, committed_lines, csv_bytes, json_line, read_json, replace_file


FRAME_VISION_FIELDS = {
    'vision_state': 'frame_vision_state', 'detected': 'frame_detected', 'target_count': 'frame_target_count',
    'target_id': 'frame_target_id', 'confidence': 'frame_confidence', 'cx_norm': 'frame_cx_norm', 'cy_norm': 'frame_cy_norm',
}
FRAME_FIELDS = ('recording_id', 'clip_id', 'display_no', 'frame_id', 'file_pts', 'tb_num', 'tb_den', 'play_us',
                'frame_mono_ns', 't_ms', 'time_kind', 'time_error_ms', 'identity')
SAFE_ID = re.compile(r'[A-Za-z0-9_-]{1,160}\Z')


class HistoryError(Exception):
    def __init__(self, status, code, message):
        super().__init__(message)
        self.status, self.code = status, code


def invalid(message='查询参数无效'):
    return HistoryError(400, 'INVALID_ARGUMENT', message)


def ns(value):
    if type(value) is int and value >= 0 or isinstance(value, str) and re.fullmatch(r'[0-9]+', value):
        return int(value)
    return None


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def safe(name):
    return isinstance(name, str) and bool(SAFE_ID.fullmatch(name))


def number(value, *, integer=False):
    if isinstance(value, bool) or not isinstance(value, (str, int, float)):
        raise invalid()
    try:
        parsed = Decimal(str(value))
    except InvalidOperation as exc:
        raise invalid() from exc
    if not parsed.is_finite() or parsed < 0 or parsed > Decimal('9007199254740991'):
        raise invalid()
    if integer and parsed != parsed.to_integral_value():
        raise invalid()
    return int(parsed) if integer else parsed


def page_length(query, default, maximum):
    value = number(query.get('limit', default), integer=True)
    if not 1 <= value <= maximum:
        raise invalid('分页大小超出范围')
    return value


def file_name(clip):
    value = clip.get('file_name')
    return value if isinstance(value, str) and value not in ('', '.', '..') and '/' not in value and '\\' not in value else None


def stream_url(clip):
    name = file_name(clip)
    return f'/api/gimbal/recordings/{quote(name, safe="")}/stream' if name else None


def safe_json(path, default=None):
    try:
        return read_json(path, default)
    except (OSError, ValueError):
        return default


def load_session(root, session):
    if not safe(session):
        raise HistoryError(404, 'SESSION_NOT_FOUND', '没有找到实验会话')
    root = Path(root).resolve()
    directory = root / session
    if directory.resolve().parent != root or directory.is_symlink():
        raise HistoryError(404, 'SESSION_NOT_FOUND', '没有找到实验会话')
    manifest = safe_json(directory / 'manifest.json')
    if not isinstance(manifest, dict):
        raise HistoryError(404, 'SESSION_NOT_FOUND', '没有找到实验会话')
    if manifest.get('schema_version') != 2:
        wall = manifest.get('created_wall_ns')
        started = datetime.fromtimestamp(wall/1e9, timezone.utc).isoformat().replace('+00:00', 'Z') if finite(wall) else None
        manifest = {**manifest, 'schema_version': manifest.get('schema', 1), 'session_id': session,
            'started_utc': started, 'status': 'partial' if manifest.get('running') else 'closed',
            'capabilities': {'causal_state': False, 'alignment': False}, 'legacy_files':
                [name for name in ('samples.csv', 'imu.csv', 'events.jsonl', 'mavlink.jsonl', 'state.jsonl') if (directory / name).is_file()]}
    ledger = safe_json(directory / 'commit.json', {})
    return {'root': root, 'directory': directory, 'manifest': manifest, 'offsets': ledger.get('offsets', {}),
        'gaps': ledger.get('gaps', manifest.get('gaps', [])), 'schema': read_json(SCHEMA_PATH)}


def list_sessions(root, query):
    limit = page_length(query, 20, 100)
    items = []
    for path in Path(root).glob('session_*'):
        try:
            manifest = load_session(root, path.name)['manifest']
        except HistoryError:
            continue
        items.append({key: manifest.get(key) for key in ('session_id', 'started_utc', 'status')})
    items.sort(key=lambda item: (item['started_utc'] or '', item['session_id']), reverse=True)
    if 'before' in query:
        cursor = query['before']
        index = next((i for i, item in enumerate(items) if item['session_id'] == cursor), None)
        if index is None:
            raise invalid('分页游标无效')
        items = items[index + 1:]
    return {'items': items[:limit], 'next_cursor': items[limit - 1]['session_id'] if len(items) > limit else None}


def table_rows(session, kind):
    name = FILES[kind]
    return csv.DictReader(committed_lines(session['directory'] / name, session['offsets'].get(name, 0)))


def event_rows(session):
    for row in table_rows(session, 'events'):
        for key in row:
            if row[key] == '':
                row[key] = None
        row['event_no'] = int(row['event_no'])
        row['t_ms'] = float(row['t_ms']) if row['t_ms'] is not None else None
        row['detail_json'] = json.loads(row['detail_json'] or '{}')
        yield row


def list_events(session, query):
    after = number(query.get('after', 0), integer=True)
    limit = page_length(query, 100, 500)
    for field in ('module', 'level'):
        if field in query and not safe(query[field]):
            raise invalid()
    items = []
    for row in event_rows(session):
        if row['event_no'] <= after or any(field in query and row.get(field) != query[field] for field in ('module', 'level')):
            continue
        items.append(row)
        if len(items) > limit:
            break
    return {'items': items[:limit], 'next_after': items[min(len(items), limit) - 1]['event_no'] if items else after,
            'has_more': len(items) > limit}


def session_times(session):
    manifest = session['manifest']
    return ns(manifest.get('session_start_mono_ns')), ns(manifest.get('ended_mono_ns'))


def within_frame_time(stamp, start, end):
    return stamp is not None and start is not None and stamp >= start and (end is None or stamp < end)


def read_clips(session):
    manifest = session['manifest']
    if manifest['schema_version'] != 2:
        return []
    clips = {}
    for clip in manifest.get('clips', []) or []:
        if isinstance(clip, dict) and safe(clip.get('clip_id')):
            clips[clip['clip_id']] = clip
    # Upcoming indexes have an identity as soon as the corresponding
    # recording-layer event is committed, even before a frame is available.
    for event in event_rows(session):
        clip_id = event.get('clip_id')
        if event.get('type') != 'record_phase' or not safe(clip_id):
            continue
        phase = event['detail_json'].get('to')
        clip = clips.setdefault(clip_id, {'clip_id': clip_id, 'recording_id': event['recording_id'],
            'boot_id': manifest.get('boot_id'), 'frames': []})
        clip['phase'] = phase
        clip['index_state'] = 'failed' if phase == 'failed' else 'building'
        if phase in ('starting', 'recording'):
            clip.setdefault('mono_start_ns', event['mono_ns'])
        if phase == 'finalizing':
            clip.setdefault('mono_end_ns', event['mono_ns'])
    session_clip_ids = set(clips)
    for path in (session['root'] / 'clip_indexes').glob('*.json'):
        if path.is_symlink() or not safe(path.stem):
            continue
        clip = safe_json(path)
        if isinstance(clip, dict) and safe(clip.get('clip_id')):
            clips[clip['clip_id']] = {**clips.get(clip['clip_id'], {}), **clip}
    start, end = session_times(session)
    result = []
    for clip in clips.values():
        related = clip['clip_id'] in session_clip_ids
        explicitly_owned = clip.get('session_id') == manifest['session_id']
        comparable = clip.get('boot_id') == manifest.get('boot_id') and bool(manifest.get('boot_id'))
        if not comparable and not related and not explicitly_owned:
            continue
        clip_start, clip_end = ns(clip.get('mono_start_ns')), ns(clip.get('mono_end_ns'))
        overlaps = comparable and start is not None and clip_start is not None and (end is None or clip_start < end) and (clip_end is None or clip_end > start)
        contained = comparable and within_frame_time(clip_start, start, end) and clip_end is not None and (end is None or clip_end <= end)
        rows = [frame for frame in clip.get('frames', []) or [] if isinstance(frame, dict) and type(frame.get('display_no')) is int]
        known = [frame for frame in rows if comparable and within_frame_time(ns(frame.get('frame_mono_ns')), start, end)]
        unknown = [frame for frame in rows if not comparable or ns(frame.get('frame_mono_ns')) is None]
        if not (known or contained or overlaps or related or explicitly_owned):
            continue
        include_unknown = contained or explicitly_owned and not (comparable and clip_start is not None and not contained)
        frames = known + (unknown if include_unknown else [])
        if not comparable:
            frames = [{**row, 'frame_mono_ns': None, 'frame_mono_end_ns': None, 't_ms': None,
                'time_kind': 'unknown', 'time_error_ms': None} for row in frames]
        result.append({**clip, 'frames': sorted(frames, key=lambda f: f['display_no']),
                       'unassigned_frame_count': 0 if include_unknown else len(unknown)})
    return sorted(result, key=lambda clip: (ns(clip.get('mono_start_ns')) or 0, clip['clip_id']))


def samples_in(session):
    for relative, size in sorted(session['offsets'].items()):
        if relative.startswith('diagnostics/samples-') and relative.endswith('.jsonl'):
            for line in committed_lines(session['directory'] / relative, size):
                yield json.loads(line)


class Timeline:
    def __init__(self, session):
        self.session = session
        self.by_source = defaultdict(list)
        self.vision_by_frame = defaultdict(list)
        self.alerts = []
        self.mode_events = {}
        self.start, _ = session_times(session)
        boot = session['manifest'].get('boot_id')
        for sample in samples_in(session):
            source = sample['source']
            obs, avail = ns(sample.get('obs_ns')), ns(sample.get('avail_ns'))
            meta = sample.get('meta') or {}
            if sample.get('boot_id') not in (None, boot):
                continue
            if source == 'vision' and avail is not None:
                frame = meta.get('frame_id')
                if frame:
                    self.vision_by_frame[frame].append((avail, sample))
            quality = sample.get('quality')
            if (obs is not None and avail is not None and sample.get('time_kind') != 'unknown'
                    and meta.get('primary') is not False and (quality == 'measured' or
                        quality == 'predicted' and meta.get('based_on_sample_id'))
                    and not (source == 'vision' and sample.get('values', {}).get('vision_state') == 'error')):
                self.by_source[source].append((obs, avail, sample))
        for source, points in tuple(self.by_source.items()):
            points.sort(key=lambda row: row[:2])
            self.by_source[source] = ([row[0] for row in points], points)
        for row in sorted(event_rows(session), key=lambda row: (ns(row['mono_ns']) or 0, row['event_no'])):
            at, kind = ns(row['mono_ns']), row['type']
            if at is None:
                continue
            if kind in ('alert_open', 'alert_level', 'alert_close') and row['alert_id']:
                self.alerts.append((at, row['alert_id'], row['code'], kind != 'alert_close'))
            if kind == 'mode_applied':
                detail = row['detail_json']
                if detail.get('axis') in FIELDS['mode']:
                    self.mode_events.setdefault(detail['axis'], []).append((at, detail.get('actual')))
        options = session['manifest'].get('diagnostic_config') or {}
        self.times = {**session['schema']['diagnostic'], **options}
        self.state_keys = tuple(key for key in session['schema']['overview'] if key not in ('session_id', 'row_no', 'mono_ns', 't_ms', 'utc'))

    def admit_late_frame_results(self, clips):
        start, end = session_times(self.session)
        if end is None:
            return
        frames = {frame['frame_id'] for clip in clips for frame in clip['frames']
                  if frame.get('identity') == 'exact' and frame.get('frame_id')}
        if not frames:
            return
        boot = self.session['manifest'].get('boot_id')
        for sibling in self.session['root'].glob('session_*'):
            if sibling == self.session['directory']:
                continue
            try:
                other = load_session(self.session['root'], sibling.name)
            except HistoryError:
                continue
            other_start, _ = session_times(other)
            if other_start is None or other_start < start or other['manifest'].get('boot_id') != boot:
                continue
            for sample in samples_in(other):
                if sample.get('source') != 'vision' or sample.get('boot_id') not in (None, boot):
                    continue
                avail = ns(sample.get('avail_ns'))
                frame_id = (sample.get('meta') or {}).get('frame_id')
                if avail is not None and avail >= end and frame_id in frames:
                    # This can only affect exact per-frame results, never as-of state.
                    self.vision_by_frame[frame_id].append((avail, sample))

    def sample(self, source, at):
        keys, points = self.by_source.get(source, ([], []))
        index = bisect_right(keys, at) - 1
        while index >= 0:
            obs, avail, sample = points[index]
            if avail <= at:
                return obs, sample
            index -= 1
        return None, None

    def state(self, at):
        if at is None or self.start is None:
            return None
        row = dict.fromkeys(self.state_keys)
        active = {}
        for event_at, alert, code, opened in self.alerts:
            if event_at > at:
                break
            if opened:
                active[alert] = code
            else:
                active.pop(alert, None)
        row['active_alert_ids'] = sorted(active)
        has_evidence = bool(active)
        uwb_age, uwb_current = None, False
        gimbal_cal, uwb_cal = None, None
        mode_obs = None
        for source, fields in FIELDS.items():
            obs, sample = self.sample(source, at)
            if sample is None:
                continue
            has_evidence = True
            age = (at - obs) / 1e6
            row[f'{source}_sample_id'] = sample['sample_id']
            row[f'{source}_age_ms'] = age
            meta, values = sample.get('meta') or {}, sample.get('values') or {}
            expiry = None if source == 'mode' else meta.get('valid_for_ms', self.times.get(source, 350))
            fresh = (not finite(expiry) or age <= expiry) and not (source == 'fc' and 'FC_LINK_DOWN' in active.values())
            if source == 'uwb':
                uwb_age = age
                uwb_current = fresh and sample['quality'] == 'measured'
                uwb_cal = meta.get('calibration_id')
                fresh = uwb_current
            elif source == 'uwb_pred':
                fresh = fresh and sample['quality'] == 'predicted'
            elif source == 'gimbal':
                gimbal_cal = meta.get('calibration_id')
            elif source == 'mode':
                mode_obs = obs
            if fresh:
                row.update({field: values[field] for field in fields if field in values})
        for axis, transitions in self.mode_events.items():
            for stamp, actual in reversed(transitions):
                if stamp <= at:
                    if mode_obs is None or stamp >= mode_obs:
                        row[axis] = actual
                        has_evidence = True
                    break
        row['uwb_quality'] = ('measured' if uwb_current else 'predicted' if row['uwb_pred_range_m'] is not None
                              or row['uwb_pred_bearing_deg'] is not None else 'stale' if uwb_age is not None else 'unknown')
        if uwb_cal and gimbal_cal and finite(row['uwb_bearing_deg']) and finite(row['gimbal_yaw_deg']):
            row['bearing_error_deg'] = (row['uwb_bearing_deg'] - row['gimbal_yaw_deg'] + 180) % 360 - 180
        accelerations = [row[f'a{axis}_m_s2'] for axis in 'xyz']
        if row['acc_norm_m_s2'] is None and all(finite(value) for value in accelerations):
            row['acc_norm_m_s2'] = math.hypot(*accelerations)
        return row if has_evidence else None

    def exact_vision(self, frame, state, at):
        if frame.get('identity') != 'exact' or not frame.get('frame_id'):
            return {}
        _, mode_sample = self.sample('mode', at) if at is not None else (None, None)
        mode_generation = frame.get('mode_generation', (mode_sample or {}).get('meta', {}).get('mode_generation'))
        mode = frame.get('gimbal_mode', (state or {}).get('gimbal_mode'))
        expected = 'swimmer' if isinstance(mode, str) and 'swim' in mode else 'face' if isinstance(mode, str) and 'face' in mode else None
        if mode in ('idle', 'click', 'beacon', 'none'):
            return {}
        candidates = []
        for avail, sample in self.vision_by_frame.get(frame['frame_id'], []):
            meta = sample.get('meta') or {}
            detector = str(meta.get('detector') or '').lower()
            if mode_generation is not None and meta.get('mode_generation') != mode_generation:
                continue
            if mode_generation is None and meta.get('primary') is False:
                continue
            if expected is not None and expected not in detector:
                continue
            candidates.append((avail, sample))
        if not candidates:
            return {}
        signatures = {(sample.get('meta', {}).get('detector'), sample.get('meta', {}).get('mode_generation')) for _, sample in candidates}
        if len(signatures) != 1:
            return {}
        priority = {'detected': 3, 'no_target': 3, 'predicted': 2, 'not_run': 1, 'error': 0}
        avail, sample = max(candidates, key=lambda pair: (
            priority.get(pair[1].get('values', {}).get('vision_state'), -1), pair[0]))
        values = sample.get('values') or {}
        return {**{column: values.get(key) for key, column in FRAME_VISION_FIELDS.items()}, 'vision_result_ns': str(avail)}


def frame_summary(session, clip, frame):
    start, _ = session_times(session)
    at = ns(frame.get('frame_mono_ns'))
    result = {field: frame.get(field) for field in FRAME_FIELDS}
    result.update(recording_id=clip.get('recording_id'), clip_id=clip['clip_id'],
                  t_ms=(at - start) / 1e6 if at is not None and start is not None else None,
                  stream_url=stream_url(clip))
    return result


def session_manifest(session, clips):
    schema = session['manifest']['schema_version']
    result = {**session['manifest'], 'gaps': session['gaps'], 'clips':
        [{**{key: val for key, val in clip.items() if key not in ('frames', 'gaps')}, 'stream_url': stream_url(clip)} for clip in clips]}
    sid = result['session_id']
    result['files'] = [{'kind': kind, 'available': bool(session['offsets'].get(name) if kind != 'video' else schema == 2),
                       'url': f'/api/experiments/sessions/{quote(sid)}/files/{kind}'} for kind, name in FILES.items()]
    return result


def locating_rows(clip, point, key, end_field, gap_lower, gap_upper):
    gaps = clip.get('gaps') or []
    if any(isinstance(gap, dict) and (lo := ns(gap.get(gap_lower))) is not None and
           (hi := ns(gap.get(gap_upper))) is not None and lo <= point < hi for gap in gaps):
        return []
    frames = clip['frames']
    hits = []
    for row in frames:
        at, end = ns(row.get(key)), ns(row.get(end_field))
        if at is not None and (point == at or end is not None and at <= point < end):
            if end is not None and end < at:
                continue
            hits.append(row)
    if key == 'play_us' and hits:
        stamps = {row[key] for row in hits}
        hits = [row for row in frames if row.get(key) in stamps]
    return hits


def locate(session, clips, query, recordings_dir):
    selectors = [key for key in ('t_ms', 'play_us', 'display_no') if key in query]
    if len(selectors) != 1 or set(query) - {'t_ms', 'play_us', 'display_no', 'clip_id'}:
        raise invalid('一次只能使用一种录像定位方式')
    selector = selectors[0]
    clip_id = query.get('clip_id')
    if clip_id is not None and not safe(clip_id) or selector != 't_ms' and not clip_id:
        raise invalid('播放器定位需指定录像段')
    if clip_id is not None:
        clips = [clip for clip in clips if clip['clip_id'] == clip_id]
        if not clips:
            raise HistoryError(404, 'CLIP_NOT_FOUND', '没有找到录像段')
    point = number(query[selector], integer=selector != 't_ms')
    start, session_end = session_times(session)
    timeline = Timeline(session)
    at = start + int(point * 1_000_000) if selector == 't_ms' and start is not None else None
    found = []
    for clip in clips:
        if selector == 'display_no':
            rows = [row for row in clip['frames'] if row['display_no'] == point]
        elif selector == 'play_us':
            rows = locating_rows(clip, point, 'play_us', 'play_end_us', 'play_start_us', 'play_end_us')
        else:
            rows = [] if not within_frame_time(at, start, session_end) else locating_rows(clip, at, 'frame_mono_ns', 'frame_mono_end_ns', 'mono_start_ns', 'mono_end_ns')
        found.extend((clip, row) for row in rows)
    if not found:
        pending = [clip for clip in clips if selector != 't_ms' or at is None or
                   ((ns(clip.get('mono_start_ns')) is None or at >= ns(clip.get('mono_start_ns'))) and
                    (ns(clip.get('mono_end_ns')) is None or at < ns(clip.get('mono_end_ns'))))]
        if any(clip.get('index_state') in ('building', 'none') and not clip['frames'] for clip in pending):
            raise HistoryError(409, 'INDEX_BUILDING', '录像索引尚未提交')
        if clip_id and any(clip.get('index_state') == 'failed' and not clip['frames'] for clip in clips):
            raise HistoryError(409, 'INDEX_FAILED', '录像对齐索引生成失败')
        state = timeline.state(at) if within_frame_time(at, start, session_end) else None
        unknown = selector == 't_ms' and any(clip['frames'] and all(ns(frame.get('frame_mono_ns')) is None for frame in clip['frames']) for clip in clips)
        return {'match': 'time_unknown' if unknown else 'no_video', 'frame': None, 'state': state}
    if len(found) > 1:
        candidates = sorted({clip['clip_id'] for clip, _ in found})
        result = {'match': 'ambiguous', 'frame': None, 'state': None}
        if len(candidates) > 1:
            result['candidate_clip_ids'] = candidates
        else:
            result['clip_id'] = candidates[0]
            result['candidate_display_nos'] = sorted(row['display_no'] for _, row in found)
        return result
    clip, frame = found[0]
    summary = frame_summary(session, clip, frame)
    frame_at = ns(frame.get('frame_mono_ns'))
    state = timeline.state(frame_at)
    name = file_name(clip)
    available = bool(name and recordings_dir and not clip.get('media_missing') and (Path(recordings_dir) / name).is_file())
    if not available:
        match = 'media_missing'
        summary['stream_url'] = None
    else:
        match = 'time_unknown' if frame_at is None else 'matched'
    return {'match': match, 'frame': summary, 'state': state}


def video_rows(session, clips):
    timeline = Timeline(session)
    timeline.admit_late_frame_results(clips)
    for clip in clips:
        for frame in clip['frames']:
            at = ns(frame.get('frame_mono_ns'))
            state = timeline.state(at)
            base = frame_summary(session, clip, frame)
            exact = timeline.exact_vision(frame, state, at)
            yield {**(state or {}), **base, **exact, 'session_id': session['manifest']['session_id']}


def export_video(session, clips=None, *, persist=False):
    if session['manifest']['schema_version'] != 2:
        raise HistoryError(404, 'FILE_NOT_FOUND', '旧实验没有录像对齐表')
    clips = read_clips(session) if clips is None else clips
    if not any(clip['frames'] for clip in clips):
        if any(clip.get('index_state') in ('building', 'none') for clip in clips):
            raise HistoryError(409, 'INDEX_BUILDING', '录像索引尚未提交')
        if any(clip.get('index_state') == 'failed' for clip in clips):
            raise HistoryError(409, 'INDEX_FAILED', '录像对齐索引生成失败')
    content = csv_bytes(session['schema']['video'], video_rows(session, clips), header=True)
    if persist:
        replace_file(session['directory'] / FILES['video'], content)
        manifest = {**session['manifest'], 'clips': [
            {key: val for key, val in clip.items() if key not in ('frames', 'gaps')} for clip in clips]}
        replace_file(session['directory'] / 'manifest.json', json_line(manifest))
    return content


def download(session, clips, kind):
    if kind == 'video':
        return export_video(session, clips)
    if kind not in FILES or not session['offsets'].get(FILES[kind], 0):
        raise HistoryError(404, 'FILE_NOT_FOUND', '此会话没有该表')
    name = FILES[kind]
    with (session['directory'] / name).open('rb') as file:
        return file.read(session['offsets'][name])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('root')
    parser.add_argument('recordings_dir')
    parser.add_argument('query_json')
    args = parser.parse_args()
    command = json.loads(args.query_json)
    query = command.get('query') or {}
    body = None
    try:
        action = command['action']
        if action == 'sessions':
            result = list_sessions(args.root, query)
        else:
            session = load_session(args.root, command.get('session'))
            clips = read_clips(session) if action in ('manifest', 'download', 'locate') else []
            if action == 'manifest':
                result = session_manifest(session, clips)
            elif action == 'events':
                result = list_events(session, query)
            elif action == 'locate':
                result = locate(session, clips, query, args.recordings_dir)
            elif action == 'download':
                kind = command['kind']
                body = download(session, clips, kind)
                result = {'csv': FILES[kind]}
            else:
                raise invalid()
        sys.stdout.buffer.write(json_line({'success': True, 'data': result}))
        if body is not None:
            sys.stdout.buffer.write(body)
    except HistoryError as exc:
        sys.stdout.buffer.write(json_line({'success': False, 'status': exc.status, 'code': exc.code, 'message': str(exc)}))
    except (OSError, ValueError, TypeError):
        sys.stdout.buffer.write(json_line({'success': False, 'status': 500, 'code': 'HISTORY_UNAVAILABLE', 'message': '无法读取该实验数据'}))


if __name__ == '__main__':
    main()
