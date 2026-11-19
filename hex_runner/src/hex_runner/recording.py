"""Bounded asynchronous HDF5 log. Only the writer thread touches the file."""
from copy import deepcopy
from dataclasses import asdict
from datetime import datetime
import json
from pathlib import Path
from queue import Empty, Full, Queue
from threading import Event, Lock, Thread
import time
from uuid import uuid4

import numpy as np

from .console import banner


def json_text(value):
    return json.dumps(value, ensure_ascii=False, default=lambda v:
                      v.tolist() if isinstance(v, np.ndarray) else str(v))


# (dtype, shape per record). Every dataset has an unlimited first dimension.
SCHEMAS = {
    'feedback/pose': dict(stamp_ns=('i8', ()), received_ns=('i8', ()),
                          position=('f8', (3,)), quaternion_xyzw=('f8', (4,)),
                          valid=('?', ()), frame_id=('str', ())),
    'feedback/motor': dict(stamp_ns=('i8', ()), received_ns=('i8', ()),
                           position=('f8', (6, 3)), velocity=('f8', (6, 3)),
                           torque=('f8', (6, 3)), valid=('?', (6,))),
    'feedback/servo': dict(stamp_ns=('i8', ()), received_ns=('i8', ()),
                           position=('f8', (6,)), valid=('?', (6,))),
    'feedback/magnetic': dict(stamp_ns=('i8', ()), received_ns=('i8', ()),
                              attached=('f8', (6,)), enabled=('?', (6,)), valid=('?', (6,))),
    'runner': dict(received_ns=('i8', ()), ros_time_ns=('i8', ()), phase=('str', ()),
                   initialization_stage=('str', ()), reset_count=('i8', ()),
                   compute_count=('i8', ()), failure_reason=('str', ()),
                   motor_servo_disabled=('?', ()), magnetic_forced_off=('?', ())),
    'planning': dict(received_ns=('i8', ()), landing_leg=('f8', (6, 3)),
                     landing_world=('f8', (6, 3)), stance=('?', (6,)),
                     trajectory_index=('i8', (6,)), trajectory_id=('i8', (6,)),
                     body_trajectory_id=('i8', ()), follow_path_index=('i8', ()),
                     swing_failures=('str', ())),
    'trajectories/feet': dict(id=('i8', ()), leg=('i8', ()), offset=('i8', ()),
                              length=('i8', ()), received_ns=('i8', ())),
    'trajectories/body': dict(id=('i8', ()), offset=('i8', ()), length=('i8', ()),
                              received_ns=('i8', ())),
}


class H5Recorder:
    def __init__(self, config, capacity=4096):
        self.queue = Queue(maxsize=capacity)
        self.stopping = Event()
        self.ready = Event()
        self.lock = Lock()
        self.error = ''
        self.closed = False
        self.last_trajectories = {}
        self.trajectory_ids = {}
        self.next_id = 0
        self.config = config
        directory = Path(config.data_root) / 'h5_log'
        self.path = directory / ('complex_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f')
                                 + '_' + uuid4().hex[:8] + '.h5')
        self.thread = Thread(target=self._write, name='hex-h5-writer', daemon=True)
        self.thread.start()
        self.ready.wait()
        if self.error:
            self.thread.join()
            raise RuntimeError(self.error)
        banner('RECORDING', 'START', str(self.path), 'green')

    @property
    def active(self):
        return not self.stopping.is_set()

    def fail(self, error):
        with self.lock:
            if not self.error:
                self.error = str(error)
                banner('RECORDING', 'ERROR', '记录已停止：' + self.error, 'red')
            self.stopping.set()

    def submit(self, records):
        """Copy producer-owned data; never block callbacks on disk or a full queue."""
        with self.lock:
            if not self.active:
                return False
            try:
                self.queue.put_nowait(deepcopy(records))
                return True
            except Full:
                pass
        self.fail('H5 recording queue overflow (capacity=%d)' % self.queue.maxsize)
        return False

    def feedback(self, name, values):
        try:
            self.submit([('feedback/' + name, values)])
        except Exception as exc:
            self.fail('feedback capture: ' + str(exc))

    def tick(self, runner, snapshot, ros_time_ns):
        if not self.active:
            return
        try:
            now = time.monotonic_ns()
            records = [('runner', dict(received_ns=now, ros_time_ns=ros_time_ns,
                phase=runner.phase.value, initialization_stage=(runner.initialization.stage.value
                    if runner.initialization is not None else ''), reset_count=runner.reset_count,
                compute_count=runner.compute_count, failure_reason=runner.failure_reason,
                motor_servo_disabled=runner.motor_servo_disabled,
                magnetic_forced_off=runner.magnetic_forced_off))]
            ids = []
            for key in list(range(6)) + ['body']:
                raw = snapshot['body'] if key == 'body' else (
                    snapshot['feet'][key].T if len(snapshot['feet']) > key else [])
                shape = (-1, 4, 4) if key == 'body' else (-1, 3)
                points = np.asarray(raw, dtype=float).reshape(shape)
                previous = self.last_trajectories.get(key)
                if previous is None or not np.array_equal(previous, points, equal_nan=True):
                    self.next_id += 1
                    self.trajectory_ids[key] = self.next_id
                    self.last_trajectories[key] = points.copy()
                    records.append(('trajectory', dict(id=self.next_id, leg=key,
                                                       points=points, received_ns=now)))
                ids.append(self.trajectory_ids[key])
            records.append(('planning', dict(received_ns=now,
                landing_leg=snapshot['landing_leg'], landing_world=snapshot['landing_world'],
                stance=snapshot['stance'], trajectory_index=snapshot['trajectory_index'],
                trajectory_id=ids[:6], body_trajectory_id=ids[6],
                follow_path_index=snapshot['follow_path_index'],
                swing_failures=json_text(snapshot['swing_failures']))))
            self.submit(records)
        except Exception as exc:
            self.fail('planning capture: ' + str(exc))

    def close(self):
        with self.lock:
            if self.closed:
                return
            self.closed = True
            self.stopping.set()
        self.thread.join()
        banner('RECORDING', 'CLOSED', str(self.path), 'yellow' if self.error else 'green')

    def _write(self):
        handle = None
        try:
            import h5py  # Optional for other experts and when recording is disabled.
            self.path.parent.mkdir(parents=True, exist_ok=True)
            handle = h5py.File(str(self.path), 'x')
            handle.attrs.update(schema_version=1, normal_close=False, incomplete=True,
                error='', config_json=json_text(asdict(self.config)),
                leg_names=json_text(list(self.config.leg_names)),
                units='position=m; joint angles=rad; motor velocity=rad/s; torque=N*m; timestamps=ns',
                time_basis='stamp_ns=source ROS clock; received_ns=host monotonic clock',
                coordinate_frames='pose,landing_world,body=world; landing_leg,feet=per-leg B frame',
                trajectory_dt=self.config.control_dt,
                measurements='feedback only; no differentiated body velocity; planning is not measurement')
            for path, fields in SCHEMAS.items():
                group = handle.require_group(path)
                for name, (dtype, shape) in fields.items():
                    group.create_dataset(name, shape=(0,) + shape, maxshape=(None,) + shape,
                        chunks=(128,) + shape, dtype=h5py.string_dtype('utf-8') if dtype == 'str' else dtype)
            for name, shape in [('feet_points', (3,)), ('body_points', (4, 4))]:
                handle['trajectories'].create_dataset(name, shape=(0,) + shape,
                    maxshape=(None,) + shape, chunks=(256,) + shape, dtype='f8')
            handle.flush()
            self.ready.set()
            last_flush = time.monotonic()
            while not self.stopping.is_set() or not self.queue.empty():
                batches = []
                try:
                    batches.append(self.queue.get(timeout=.1))
                except Empty:
                    pass
                for _ in range(255):
                    try:
                        batches.append(self.queue.get_nowait())
                    except Empty:
                        break
                rows = {}
                for batch in batches:
                    for path, value in batch:
                        if path == 'trajectory':
                            leg = value['leg']
                            kind = 'body' if value['leg'] == 'body' else 'feet'
                            ds = handle['trajectories/' + kind + '_points']
                            points = value['points']
                            offset = len(ds)
                            ds.resize(offset + len(points), axis=0)
                            ds[offset:] = points
                            path = 'trajectories/' + kind
                            value = dict(id=value['id'], offset=offset, length=len(points),
                                         received_ns=value['received_ns'])
                            if kind == 'feet':
                                value['leg'] = leg
                        rows.setdefault(path, []).append(value)
                for path, values in rows.items():
                    for field in SCHEMAS[path]:
                        ds = handle[path + '/' + field]
                        offset = len(ds)
                        ds.resize(offset + len(values), axis=0)
                        ds[offset:] = [row[field] for row in values]
                if time.monotonic() - last_flush >= 1:
                    handle.flush()
                    last_flush = time.monotonic()
            handle.attrs['normal_close'] = True
            handle.attrs['incomplete'] = bool(self.error)
            handle.attrs['error'] = self.error
            handle.flush()
        except Exception as exc:
            self.fail('H5 writer: ' + str(exc))
            if handle is not None:
                try:
                    handle.attrs['normal_close'] = False
                    handle.attrs['incomplete'] = True
                    handle.attrs['error'] = self.error
                    handle.flush()
                except Exception:
                    pass
        finally:
            if handle is not None:
                try:
                    handle.close()
                except Exception as exc:
                    self.fail('H5 close: ' + str(exc))
            self.ready.set()
