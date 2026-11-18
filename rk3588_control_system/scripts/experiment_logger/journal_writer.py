"""Unix journal client and passive raw datagram capture."""
from collections import deque
import fcntl
import json
from pathlib import Path
import queue
import socket
import threading
import time

from journal_store import SessionStore, json_line, replace_file


MAX_LINE = 1024 * 1024
MAX_PENDING = 32768


def read_datagrams(sock, data, limit=300):
    invalid = 0
    for _ in range(limit):
        try:
            body = sock.recv(65535)
        except BlockingIOError:
            break
        try:
            packet = json.loads(body)
            if isinstance(packet, dict):
                data.append(packet)
            else:
                invalid += 1
        except (ValueError, UnicodeError):
            invalid += 1
    return invalid


class JournalClient:
    def __init__(self, store, socket_path):
        self.store = store
        self.socket_path = socket_path
        self.socket = None
        self.pending = []
        self.buffer = bytearray()
        self.bad_lines = 0
        self.epoch = store.ledger['epoch']
        self.error = None
        self.last_status = 0
        self.ready = False
        self.requested_finish = False
        self.finish_sent = False
        self.end_epoch = self.end_seq = self.end_mono_ns = None

    def send(self, payload):
        self.socket.sendall(json_line({'v': 2, 'session_id': self.store.session, **payload}))

    def connect(self):
        if self.socket is not None:
            return
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.socket.settimeout(.05)
        try:
            self.socket.connect(self.socket_path)
            ledger = self.store.ledger
            self.send({'kind': 'hello', 'boot_id': self.store.boot, 'session_start_mono_ns': str(self.store.start),
                'last_epoch': ledger['epoch'], 'last_ack': ledger['ack_seq'], 'last_row_no': ledger['row_no']})
            self.last_status = 0
        except OSError:
            self.disconnect()

    def disconnect(self):
        if self.socket is not None:
            self.socket.close()
        self.socket = None
        self.pending.clear()
        self.buffer.clear()
        self.ready = False
        self.finish_sent = False

    def receive(self):
        if self.socket is None:
            return
        try:
            body = self.socket.recv(65536)
        except socket.timeout:
            return
        except OSError:
            self.disconnect()
            return
        if not body:
            self.disconnect()
            return
        self.buffer.extend(body)
        while (end := self.buffer.find(b'\n')) >= 0:
            line = bytes(self.buffer[:end])
            del self.buffer[:end + 1]
            try:
                if len(line) > MAX_LINE:
                    raise ValueError('line exceeds 1 MiB')
                message = json.loads(line)
                if not isinstance(message, dict) or message.get('v') != 2:
                    raise ValueError('unsupported journal envelope')
                if message.get('session_id', self.store.session) == self.store.session:
                    self.consume(message)
            except (ValueError, TypeError, KeyError, UnicodeError):
                self.bad_lines += 1
        if len(self.buffer) > MAX_LINE or len(self.pending) >= MAX_PENDING:
            # An uncommitted queue is replayed from the last on-disk cursor.
            self.disconnect()

    def consume(self, message):
        kind = message.get('kind')
        if kind == 'ready':
            if not isinstance(message.get('epoch'), str):
                raise ValueError('invalid epoch')
            self.ready = True
            self.epoch = message['epoch']
            if self.end_epoch != self.epoch:
                self.end_epoch = self.end_seq = self.end_mono_ns = None
            self.pending.append(message)
        elif kind == 'request_finish':
            self.requested_finish = True
        elif kind == 'end':
            self.end_epoch, self.end_seq = message['epoch'], message['last_seq']
            self.end_mono_ns = message.get('ended_mono_ns')
        elif kind == 'gap':
            first, last = message.get('first_missing'), message.get('last_missing')
            if first is not None and (type(first) is not int or type(last) is not int or not 0 < first <= last):
                raise ValueError('invalid gap')
            if self.ready:
                self.pending.append(message)
        elif kind in ('sample', 'event', 'overview'):
            if type(message.get('seq')) is not int or message['seq'] < 1:
                raise ValueError('invalid sequence')
            if self.ready:
                self.pending.append(message)

    def status(self, state=None):
        ledger = self.store.ledger
        return {'kind': 'writer_status', 'epoch': self.epoch,
            'ack_seq': ledger['ack_seq'] if ledger['epoch'] == self.epoch else 0,
            'at_ns': str(self.store.now()), 'last_flush_ns': ledger['last_flush_ns'],
            'state': state or ('failed' if self.error else 'writing' if self.store.opened else 'starting'),
            'drop_total': ledger['drop_total'], 'error': self.error}

    def heartbeat(self, state=None):
        status = self.status(state)
        if self.socket is not None:
            try:
                self.send(status)
            except OSError:
                self.disconnect()
        if self.store.opened:
            try:
                replace_file(self.store.root / 'status.json', json_line({'session': str(self.store.directory),
                    'session_id': self.store.session, 'schema_version': 2, 'ros': self.store.ros_status, **status}))
            except OSError:
                self.error = self.error or {'code': 'LOG_WRITE_FAILED', 'message': '无法写入记录器状态'}
        self.last_status = time.monotonic()

    def request_stop(self):
        if self.socket is None or self.finish_sent:
            return
        try:
            self.send({'kind': 'finish'})
            self.finish_sent = True
        except OSError:
            self.disconnect()

    def finished(self):
        return (type(self.end_seq) is int and self.end_epoch == self.store.ledger['epoch']
                and self.store.ledger['ack_seq'] >= self.end_seq)


def run_journal(config, stop, *, normalize_imu=None, ros_worker=None, raw_port=14660):
    socket_path = Path(config['journal_socket'])
    socket_path.parent.mkdir(parents=True, exist_ok=True)
    with socket_path.with_name(socket_path.name + '.writer.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError('已有活动会话') from exc
        store = SessionStore(config)
        client = JournalClient(store, str(socket_path))
        data = deque(maxlen=MAX_PENDING)
        inbox = queue.Queue(maxsize=4000)
        ros_stop = threading.Event()
        thread = threading.Thread(target=ros_worker, args=(inbox, ros_stop, store), daemon=True) if ros_worker else None
        if thread:
            thread.start()
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as raw:
            raw.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
            raw.bind(('127.0.0.1', raw_port))
            raw.setblocking(False)
            next_connect = next_flush = next_guard = 0
            deadline = None
            lost_raw = 0
            print(json_line({'session_id': store.session, 'journal_socket': str(socket_path)}).decode().strip(), flush=True)
            try:
                while True:
                    now = time.monotonic()
                    if client.socket is None and now >= next_connect:
                        client.connect()
                        next_connect = now + .25
                    client.receive()
                    lost_raw += read_datagrams(raw, data)
                    while not inbox.empty() and len(data) < MAX_PENDING:
                        data.append(inbox.get_nowait())
                    if len(data) == MAX_PENDING:
                        lost_raw += len(data)
                        data.clear()
                    if (stop.is_set() or client.requested_finish) and deadline is None:
                        grace = store.manifest['diagnostic_config'].get('finalize', 5000) / 1000
                        # Leave at least one more heartbeat for the final ACK and fsync.
                        deadline = now + config.get('finalize_seconds', grace) + 2
                    if deadline is not None:
                        client.request_stop()
                    if client.ready and (now >= next_flush or client.end_seq is not None):
                        old_error = client.error
                        try:
                            if now >= next_guard:
                                store.storage_guard()
                                next_guard = now + 5
                            replay = store.flush(client.pending, data, client.bad_lines + lost_raw, normalize_imu=normalize_imu)
                            client.pending.clear()
                            if replay:
                                client.disconnect()
                            data.clear()
                            client.bad_lines = lost_raw = 0
                            client.error = None
                        except OSError as exc:
                            code = 'STORAGE_FULL' if getattr(exc, 'errno', None) == 28 else 'LOG_WRITE_FAILED'
                            client.error = {'code': code, 'message': str(exc)}
                        next_flush = now + .25
                        if client.error != old_error:
                            client.heartbeat()
                    if client.finished() or deadline is not None and now >= deadline:
                        break
                    if now - client.last_status >= 1:
                        client.heartbeat()
                    if client.socket is None:
                        time.sleep(.025)
                complete = client.finished() and client.error is None
                try:
                    store.finish('closed' if complete else 'partial', None if complete else 'journal_finish_timeout',
                        ended_mono_ns=client.end_mono_ns)
                except OSError as exc:
                    client.error = {'code': 'LOG_WRITE_FAILED', 'message': str(exc)}
                client.heartbeat('closed' if complete and client.error is None else 'failed')
                if store.opened:
                    from history_query import export_video, load_session
                    try:
                        export_video(load_session(store.root, store.session), persist=True)
                    except (OSError, ValueError):
                        # The committed log remains usable. A later history export may retry.
                        pass
            finally:
                ros_stop.set()
                if thread:
                    thread.join(timeout=2)
                client.disconnect()
