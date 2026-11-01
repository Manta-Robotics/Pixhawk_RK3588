#!/usr/bin/env python3
"""Passive Manta + ROS2 experiment recorder. Never sends actuator commands."""
import argparse
import collections
import csv
import datetime
import json
import math
import os
from pathlib import Path
import queue
import re
import shutil
import signal
import socket
import threading
import time
import uuid

from motion_labels import MotorLabels, esc_power, number

IMU_TYPES = {'HIGHRES_IMU', 'RAW_IMU', 'SCALED_IMU', 'SCALED_IMU2', 'SCALED_IMU3'}
CSV_FIELDS = ['wall_ns', 'mono_ns', 'elapsed_s', 'snapshot_age_s', 'snapshot_sequence',
              'motor_state', 'motor_neutral_duration_s', 'rotation_feedback', 'segment_id',
              'experiment_label', 'water_context', 'hull_stationary_observer',
              'left_pwm', 'right_pwm', 'left_power_w', 'right_power_w', 'esc_age_s',
              'imu_source', 'imu_age_s', 'ax_m_s2', 'ay_m_s2', 'az_m_s2',
              'gx_rad_s', 'gy_rad_s', 'gz_rad_s', 'imu_fields_updated',
              'roll_deg', 'pitch_deg', 'yaw_deg', 'attitude_age_s', 'armed',
              'uwb_fresh', 'uwb_age_s', 'uwb_distance_m', 'uwb_azimuth_deg', 'uwb_elevation_deg',
              'uwb_x_forward_m', 'uwb_y_right_m', 'vision_active', 'vision_age_s',
              'vision_status', 'vision_detections', 'gimbal_connected', 'gimbal_age_s',
              'gimbal_roll_deg', 'gimbal_pitch_deg', 'gimbal_yaw_deg',
              'follow_active', 'follow_status', 'command_throttle_percent', 'command_steering_deg']
IMU_FIELDS = ['wall_ns', 'mono_ns', 'source_wall_ns', 'source_mono_ns', 'source',
              'system_id', 'component_id', 'sensor_id', 'source_time_usec', 'source_time_boot_ms',
              'ax_m_s2', 'ay_m_s2', 'az_m_s2', 'gx_rad_s', 'gy_rad_s', 'gz_rad_s',
              'raw_xacc', 'raw_yacc', 'raw_zacc', 'raw_xgyro', 'raw_ygyro', 'raw_zgyro',
              'fields_updated', 'motor_state', 'segment_id', 'motor_feedback_age_s', 'experiment_label']


def json_text(value):
    return json.dumps(value, ensure_ascii=False, separators=(',', ':'), default=str)


def atomic_json(path, value):
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json_text(value) + '\n', encoding='utf-8')
    os.replace(tmp, path)


def age_ms(value, wall_ns):
    value = number(value)
    if value is None or value <= 0:
        return None
    age = wall_ns / 1e9 - value / 1000
    return age if age >= 0 else None


def normalize_imu(message):
    kind = message['mavpackettype']
    if kind == 'HIGHRES_IMU':
        acc_scale, gyro_scale = 1.0, 1.0
    elif kind.startswith('SCALED_IMU'):
        acc_scale, gyro_scale = 9.80665 / 1000, .001
    else:
        # RAW_IMU units are dialect dependent; retain counts without inventing SI.
        return {}
    result = {}
    flags = message.get('fields_updated') if kind == 'HIGHRES_IMU' else None
    for index, (axis, field) in enumerate(zip('xyz', ['ax_m_s2', 'ay_m_s2', 'az_m_s2'])):
        value = number(message.get(axis + 'acc'))
        result[field] = value * acc_scale if value is not None and (flags is None or flags & (1 << index)) else None
    for index, (axis, field) in enumerate(zip('xyz', ['gx_rad_s', 'gy_rad_s', 'gz_rad_s'])):
        value = number(message.get(axis + 'gyro'))
        result[field] = value * gyro_scale if value is not None and (flags is None or flags & (1 << (index + 3))) else None
    return result


class Recorder:
    def __init__(self, directory, config):
        self.root = Path(directory)
        self.root.mkdir(parents=True, exist_ok=True)
        self.config = config
        existing_bytes = sum(p.stat().st_size for p in self.root.glob('session_*/*') if p.is_file())
        if (shutil.disk_usage(self.root).free < config.get('min_free_gb', 2)*1024**3
                or existing_bytes >= config.get('max_total_gb', 20)*1024**3):
            raise RuntimeError('Storage guard: no recording space; existing sessions retained')
        self.started_ns = time.monotonic_ns()
        self.boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip() if Path('/proc/sys/kernel/random/boot_id').exists() else None
        self.session = self.root / ('session_' + time.strftime('%Y%m%d_%H%M%S') + '_' + uuid.uuid4().hex[:8])
        self.session.mkdir()
        self.files = {}
        self.writers = {}
        self.counts = collections.Counter()
        self.last = {}
        self.last_seq = {}
        self.gaps = collections.Counter()
        self.state = {}
        self.snapshot_mono_ns = None
        self.motors = MotorLabels(center=config.get('neutral_pwm', 1500),
                                  deadband=config.get('neutral_deadband_pwm', 25))
        self.segment = 0
        self.previous_motor = None
        self.annotations = {'experiment_label': '', 'water_context': 'unknown', 'hull_stationary_observer': 'unknown'}
        self.marker_stamp = None
        self.ros_status = {}
        self.queue_drops = 0
        self.udp_drops = 0
        self.sender_drops = {}
        self.row_start_ns = None
        self.row_last_ns = None
        self.max_row_gap_s = 0
        self.storage_bytes = existing_bytes
        self.last_disk_check = 0
        self.manifest = {'schema': 1, 'boot_id': self.boot_id, 'created_wall_ns': time.time_ns(),
                         'created_mono_ns': self.started_ns, 'config': config,
                         'clock_reference': config.get('clock_reference'),
                         'labels': {'motor_state': 'driven=actual FC PWM outside neutral; neutral after 2s; unknown if stale/invalid',
                                    'rotation_feedback': 'ESC eRPM only; PWM does not prove physical rotation',
                                    'hull_stationary_observer': 'manual observation; motor neutral does not prove hull stationary'},
                         'power': 'ESC electrical input V*I, not mechanical shaft power; missing feedback is null',
                         'coordinates': 'UWB x=range*cos(elevation)*cos(azimuth), y=range*cos(elevation)*sin(azimuth); receiver frame, x forward/y right',
                         'source_docs': ['https://mavlink.io/en/messages/common.html#HIGHRES_IMU',
                                         'https://mavlink.io/en/messages/common.html#SCALED_IMU',
                                         'https://mavlink.io/en/messages/ardupilotmega.html#ESC_TELEMETRY_1_TO_4'],
                         'running': True}
        atomic_json(self.session / 'manifest.json', self.manifest)
        atomic_json(self.root / 'current.json', {'session': str(self.session), 'pid': os.getpid()})

    def file(self, name):
        if name not in self.files:
            self.files[name] = (self.session / name).open('a', encoding='utf-8', newline='')
        return self.files[name]

    def event(self, name, event):
        self.file(name).write(json_text(event) + '\n')

    def row(self, name, fields, value):
        if name not in self.writers:
            self.writers[name] = csv.DictWriter(self.file(name), fieldnames=fields, extrasaction='ignore')
            self.writers[name].writeheader()
        self.writers[name].writerow({k: '' if v is None else v for k, v in value.items()})

    def motor_state(self, now_ns):
        packet = self.last.get('SERVO_OUTPUT_RAW')
        channels = self.config.get('motor_channels', [1, 3])
        outputs = [packet['message'].get('servo%d_raw' % channel) if packet else None for channel in channels]
        # Ports 1+ represent outputs 9+; this logger explicitly uses configured channels 1..8.
        if packet and packet['message'].get('port', 0) != 0:
            outputs = [None, None]
        source_ns = packet.get('mono_ns') if packet else None
        label, duration = self.motors.classify(outputs, source_ns, now_ns)
        if label != self.previous_motor:
            self.segment += 1
            self.event('events.jsonl', {'kind': 'motor_transition', 'wall_ns': time.time_ns(), 'mono_ns': now_ns,
                                      'previous': self.previous_motor, 'state': label, 'segment_id': self.segment})
            self.previous_motor = label
        age = (now_ns-source_ns)/1e9 if source_ns is not None else None
        return label, duration, outputs, age

    def ingest(self, packet, now_ns=None):
        now_ns = now_ns or time.monotonic_ns()
        wall_ns = time.time_ns()
        kind = packet.get('kind')
        if kind == 'snapshot':
            self.state = packet['state']
            self.snapshot_mono_ns = int(packet['mono_ns'])
            self.snapshot_sequence = packet['sequence']
            source = 'snapshot'
            self.event('state.jsonl', packet)
        elif kind == 'mavlink':
            message = packet['message']
            source = 'mavlink'
            name = message['mavpackettype']
            # All source messages are preserved; control labels use only the autopilot component.
            if packet.get('autopilot'):
                if name != 'SERVO_OUTPUT_RAW' or message.get('port', 0) == 0:
                    self.last[name] = packet
            self.counts[name] += 1
            if name in IMU_TYPES:
                label, _, _, age = self.motor_state(now_ns)
                normalized = normalize_imu(message)
                row = dict(wall_ns=wall_ns, mono_ns=now_ns, source_wall_ns=packet['wall_ns'],
                           source_mono_ns=packet['mono_ns'], source=name, system_id=packet.get('system_id'),
                           component_id=packet.get('component_id'), sensor_id=message.get('id'),
                           source_time_usec=message.get('time_usec'), source_time_boot_ms=message.get('time_boot_ms'),
                           fields_updated=message.get('fields_updated'), motor_state=label, segment_id=self.segment,
                           motor_feedback_age_s=age, experiment_label=self.annotations['experiment_label'], **normalized)
                for axis in 'xyz':
                    for field in ('acc', 'gyro'):
                        row['raw_' + axis + field] = message.get(axis + field)
                self.row('imu.csv', IMU_FIELDS, row)
            self.event('mavlink.jsonl', packet)
        elif kind == 'ros':
            source = 'ros:' + packet['topic']
            self.event('ros_topics.jsonl', packet)
        else:
            self.event('events.jsonl', packet)
            return
        self.counts[source] += 1
        self.sender_drops[source] = packet.get('sender_drops', 0)
        seq = packet.get('sequence')
        if seq is not None:
            seq = int(seq)
            prior = self.last_seq.get(source)
            if prior and seq > prior + 1:
                self.gaps[source] += seq-prior-1
            self.last_seq[source] = seq

    def fresh_packet(self, name, mono_ns, timeout=0.7):
        packet = self.last.get(name)
        if packet and 0 <= (mono_ns-int(packet['mono_ns']))/1e9 <= timeout:
            return packet
        return None

    def sample(self):
        ns, wall = time.monotonic_ns(), time.time_ns()
        self.read_marker()
        label, duration, outputs, _ = self.motor_state(ns)
        age = (ns-self.snapshot_mono_ns)/1e9 if self.snapshot_mono_ns else None
        state = self.state if age is not None and 0 <= age <= .7 else {}
        telemetry = state.get('telemetry', {})
        uwb = telemetry.get('uwb', {})
        uwb_age = age_ms(uwb.get('updatedAt'), wall)
        uwb_fresh = bool(uwb.get('fresh') and uwb.get('online') and uwb_age is not None and uwb_age <= .7)
        attitude = telemetry.get('attitude', {})
        gimbal = state.get('gimbal', {})
        feedback = gimbal.get('feedback') or {}
        gimbal_age = age_ms(feedback.get('updatedAt'), wall)
        vision = state.get('vision', {})
        track = gimbal.get('trackStatus') or {}
        det = vision.get('detections') or {}
        vision_age = age_ms(track.get('updatedAt'), wall)
        follow = state.get('follow', {})
        command = state.get('roverControl', {})
        imu = None
        for name in ('HIGHRES_IMU', 'SCALED_IMU', 'SCALED_IMU2', 'SCALED_IMU3'):
            candidate = self.fresh_packet(name, ns)
            if candidate and any(v is not None for v in normalize_imu(candidate['message']).values()):
                imu = candidate
                break
        esc_packets = [p for name in self.last if name.startswith('ESC_TELEMETRY_')
                       for p in [self.fresh_packet(name, ns, 1.0)] if p]
        channels = self.config.get('motor_channels', [1, 3])
        powers, rpms, esc_ages = [], [], []
        for channel in channels:
            power, rpm = None, None
            for packet in esc_packets:
                message = packet['message']
                first = int(message['mavpackettype'].split('_')[2])
                i = channel-first
                if 0 <= i < 4:
                    power = esc_power(message, channel)
                    # A zero-filled unsupported ESC packet must never prove stopped.
                    if power is not None and len(message.get('rpm', [])) > i:
                        rpm = number(message['rpm'][i])
                    esc_ages.append((ns-int(packet['mono_ns']))/1e9)
            powers.append(power)
            rpms.append(rpm)
        rotation = 'unknown' if any(v is None for v in rpms) else ('rotating' if any(v > 0 for v in rpms) else 'stopped')
        values = dict(wall_ns=wall, mono_ns=ns, elapsed_s=(ns-self.started_ns)/1e9,
                      snapshot_age_s=age, snapshot_sequence=getattr(self, 'snapshot_sequence', None),
                      motor_state=label, motor_neutral_duration_s=duration, rotation_feedback=rotation,
                      segment_id=self.segment, **self.annotations, left_pwm=outputs[0], right_pwm=outputs[1],
                      left_power_w=powers[0], right_power_w=powers[1], esc_age_s=max(esc_ages) if esc_ages else None,
                      imu_source=imu['message']['mavpackettype'] if imu else None,
                      imu_age_s=(ns-int(imu['mono_ns']))/1e9 if imu else None,
                      imu_fields_updated=imu['message'].get('fields_updated') if imu else None,
                      roll_deg=attitude.get('roll'), pitch_deg=attitude.get('pitch'), yaw_deg=attitude.get('yaw'),
                      attitude_age_s=age_ms(attitude.get('updatedAt'), wall), armed=telemetry.get('armed'),
                      uwb_fresh=uwb_fresh, uwb_age_s=uwb_age,
                      uwb_distance_m=uwb.get('distanceM') if uwb_fresh else None,
                      uwb_azimuth_deg=uwb.get('azimuthDeg') if uwb_fresh else None,
                      uwb_elevation_deg=uwb.get('elevationDeg') if uwb_fresh else None,
                      vision_active=vision.get('active'), vision_age_s=vision_age,
                      vision_status=track.get('status'), vision_detections=track.get('detections', len(det.get('rects', []))),
                      gimbal_connected=gimbal.get('connected'), gimbal_age_s=gimbal_age,
                      follow_active=follow.get('active'), follow_status=follow.get('status'),
                      command_throttle_percent=command.get('throttle'), command_steering_deg=command.get('steering'))
        if imu:
            values.update(normalize_imu(imu['message']))
        if gimbal.get('connected') and gimbal_age is not None and gimbal_age <= .7:
            for axis in ('roll', 'pitch', 'yaw'):
                values['gimbal_' + axis + '_deg'] = feedback.get(axis + 'Deg', feedback.get(axis))
        distance, azimuth, elevation = [number(values.get(k)) for k in ('uwb_distance_m', 'uwb_azimuth_deg', 'uwb_elevation_deg')]
        if all(v is not None for v in (distance, azimuth, elevation)):
            r = distance*math.cos(math.radians(elevation))
            values['uwb_x_forward_m'] = r*math.cos(math.radians(azimuth))
            values['uwb_y_right_m'] = r*math.sin(math.radians(azimuth))
        self.row('samples.csv', CSV_FIELDS, values)
        self.counts['samples'] += 1
        if self.row_start_ns is None:
            self.row_start_ns = ns
        if self.row_last_ns is not None:
            self.max_row_gap_s = max(self.max_row_gap_s, (ns-self.row_last_ns)/1e9)
        self.row_last_ns = ns
        self.latest_values = values

    def read_marker(self):
        path = self.root / 'marker.json'
        try:
            stamp = path.stat().st_mtime_ns
            if stamp != self.marker_stamp:
                marker = json.loads(path.read_text())
                for key in self.annotations:
                    if key in marker:
                        self.annotations[key] = marker[key]
                self.marker_stamp = stamp
                self.event('events.jsonl', {'kind': 'manual_marker', 'wall_ns': time.time_ns(),
                                          'mono_ns': time.monotonic_ns(), 'marker': self.annotations.copy()})
        except FileNotFoundError:
            pass
        except Exception as exc:
            self.event('events.jsonl', {'kind': 'marker_error', 'error': str(exc)})

    def flush(self):
        for f in self.files.values():
            f.flush()
            os.fsync(f.fileno())
        elapsed = ((self.row_last_ns-self.row_start_ns)/1e9) if self.row_start_ns is not None and self.row_last_ns else 0
        ns = time.monotonic_ns()
        source_ages = {name: max(0, (ns-int(packet['mono_ns']))/1e9) for name, packet in self.last.items()}
        status = {'session': str(self.session), 'wall_ns': time.time_ns(), 'mono_ns': ns,
                  'counts': dict(self.counts), 'sequence_gaps': dict(self.gaps), 'sender_drops': self.sender_drops,
                  'queue_drops': self.queue_drops, 'udp_socket_drops': self.udp_drops,
                  'sample_rate_hz': (self.counts['samples']-1)/elapsed if elapsed else 0,
                  'max_sample_gap_s': self.max_row_gap_s, 'source_ages_s': source_ages,
                  'latest': getattr(self, 'latest_values', {}), 'ros': self.ros_status}
        atomic_json(self.root / 'status.json', status)
        self.manifest.update(counts=dict(self.counts), sequence_gaps=dict(self.gaps))
        atomic_json(self.session / 'manifest.json', self.manifest)

    def check_storage(self):
        free = shutil.disk_usage(self.root).free
        current = sum(p.stat().st_size for p in self.session.iterdir() if p.is_file())
        if free < self.config.get('min_free_gb', 2)*1024**3 or self.storage_bytes+current > self.config.get('max_total_gb', 20)*1024**3:
            raise RuntimeError('Storage guard: recording stopped, existing sessions retained')

    def close(self):
        self.manifest.update(running=False, closed_wall_ns=time.time_ns(), closed_mono_ns=time.monotonic_ns())
        self.flush()
        for f in self.files.values():
            f.close()


def ros_worker(out_queue, stop, recorder):
    try:
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from rosidl_runtime_py.utilities import get_message
        from rosidl_runtime_py.convert import message_to_ordereddict
        rclpy.init()
        node = rclpy.create_node('manta_experiment_logger')
        subscriptions = {}
        selected = {'sensor_msgs/msg/Imu', 'nav_msgs/msg/Path', 'nav_msgs/msg/Odometry',
                    'geometry_msgs/msg/Twist', 'geometry_msgs/msg/TwistStamped',
                    'geometry_msgs/msg/PoseStamped', 'vision_msgs/msg/Detection2DArray',
                    'vision_msgs/msg/Detection3DArray'}
        last_scan = 0
        while not stop.is_set():
            if time.monotonic()-last_scan > 3:
                last_scan = time.monotonic()
                discovered = node.get_topic_names_and_types()
                recorder.ros_status = {'available': True, 'domain_id': os.environ.get('ROS_DOMAIN_ID', '0'),
                                       'discovered': dict(discovered), 'subscribed': list(subscriptions)}
                for topic, types in discovered:
                    wanted = types and (types[0] in selected or topic in ('/usv/control', '/usv/status')
                                        or re.search(r'uwb|gimbal|follow.*path|plan|motor.*(power|output)|detections', topic, re.I))
                    if not wanted or topic in subscriptions:
                        continue
                    try:
                        msgtype = types[0]
                        cls = get_message(msgtype)
                        publishers = node.get_publishers_info_by_topic(topic)
                        durable = any(p.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for p in publishers)
                        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                                         durability=DurabilityPolicy.TRANSIENT_LOCAL if durable else DurabilityPolicy.VOLATILE)
                        def callback(msg, name=topic, schema=msgtype):
                            event = {'kind': 'ros', 'topic': name, 'message_type': schema,
                                     'wall_ns': time.time_ns(), 'mono_ns': time.monotonic_ns(),
                                     'message': message_to_ordereddict(msg)}
                            try:
                                out_queue.put_nowait(event)
                            except queue.Full:
                                recorder.queue_drops += 1
                        subscriptions[topic] = node.create_subscription(cls, topic, callback, qos)
                        out_queue.put_nowait({'kind': 'ros_subscription', 'topic': topic, 'message_type': msgtype,
                                              'wall_ns': time.time_ns(), 'mono_ns': time.monotonic_ns()})
                    except Exception as exc:
                        recorder.ros_status.setdefault('errors', {})[topic] = str(exc)
            rclpy.spin_once(node, timeout_sec=.05)
        node.destroy_node()
        rclpy.shutdown()
    except Exception as exc:
        recorder.ros_status = {'available': False, 'error': str(exc)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    args = parser.parse_args()
    config = json.loads(Path(args.config).read_text())
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4*1024*1024)
    sock.bind(('127.0.0.1', 14660))
    sock.setblocking(False)
    if hasattr(socket, 'SO_RXQ_OVFL'):
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RXQ_OVFL, 1)
    recorder = Recorder(config['directory'], config)
    inbox = queue.Queue(maxsize=4000)
    thread = threading.Thread(target=ros_worker, args=(inbox, stop, recorder), daemon=True)
    thread.start()
    period = 1 / max(5, float(config.get('rate_hz', 10)))
    next_sample = time.monotonic()
    next_flush = next_sample+1
    next_disk = next_sample+30
    rotate = next_sample+config.get('rotate_seconds', 900)
    print(json_text({'started': str(recorder.session)}), flush=True)
    try:
        while not stop.is_set():
            # Bounded bursts leave time for periodic samples, disk flush and shutdown.
            for _ in range(300):
                try:
                    raw, ancillary, _, _ = sock.recvmsg(65535, 32)
                except BlockingIOError:
                    break
                for level, opt, data in ancillary:
                    if level == socket.SOL_SOCKET and opt == getattr(socket, 'SO_RXQ_OVFL', 40):
                        recorder.udp_drops = int.from_bytes(data[:4], 'little')
                try:
                    recorder.ingest(json.loads(raw))
                except (ValueError, KeyError, TypeError) as exc:
                    recorder.counts['invalid_packets'] += 1
                    recorder.event('events.jsonl', {'kind': 'invalid_packet', 'error': str(exc)})
            for _ in range(300):
                try:
                    recorder.ingest(inbox.get_nowait())
                except queue.Empty:
                    break
            now = time.monotonic()
            if now >= next_sample:
                recorder.sample()
                next_sample += period
                if next_sample < now:
                    next_sample = now+period
            if now >= next_flush:
                recorder.flush()
                next_flush = now+1
            if now >= next_disk:
                recorder.check_storage()
                next_disk = now+30
            if now >= rotate:
                # Restarting this process gives each part a clean manifest and counters.
                stop.set()
            stop.wait(min(.005, max(0, next_sample-time.monotonic())))
    finally:
        stop.set()
        thread.join(timeout=2)
        recorder.close()
        sock.close()


if __name__ == '__main__':
    main()
