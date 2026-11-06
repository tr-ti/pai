#!/usr/bin/env python3
"""rosbag -> LeRobotDataset。environment_state に立方体位置(x,y)を入れる版"""
import os, sys, traceback
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, torch, yaml
from rosbag2lerobot import RosbagToLeRobot
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr

STATE_ORDER = ['crane_plus_joint1', 'crane_plus_joint2', 'crane_plus_joint4',
               'crane_plus_joint3', 'crane_plus_joint_hand']      # infer.py と同じ順
ARM_ORDER = STATE_ORDER[:4]
J = '/joint_states'
A = '/crane_plus_arm_controller/joint_trajectory'
G = '/crane_plus_gripper_controller/joint_trajectory'
C = '/cube_pose'

def js_vec(m):
    d = dict(zip(m.name, m.position))
    return [d.get(n, 0.0) for n in STATE_ORDER]

def arm_vec(m):
    if not m.points:
        return None
    d = dict(zip(m.joint_names, m.points[0].positions))
    return [d[n] for n in ARM_ORDER] if all(n in d for n in ARM_ORDER) else None

def grip_vec(m):
    return [m.points[0].positions[0]] if m.points and len(m.points[0].positions) > 0 else None

def cube_vec(m):
    return [m.pose.position.x, m.pose.position.y]

def series(msgs, fn):
    ts, vs = [], []
    for m, t in msgs:
        v = fn(m)
        if v is not None:
            ts.append(t); vs.append(v)
    return np.array(ts), np.array(vs, dtype=np.float64)

def hold(times, values, target):
    idx = np.clip(np.searchsorted(times, target, side='right') - 1, 0, len(times) - 1)
    return values[idx]

class PushConverter(RosbagToLeRobot):
    def _get_messages(self, episode_dir):
        topics = {J: [], A: [], G: [], C: []}
        with Reader(episode_dir) as reader:
            conns = [x for x in reader.connections if x.topic in topics]
            for conn, ts, raw in reader.messages(connections=conns):
                topics[conn.topic].append((deserialize_cdr(raw, conn.msgtype), ts))
        return topics

    def convert(self):
        freq = self.target_freq or 20
        eps = []
        for d in self.episode_dirs:
            try:
                m = self._get_messages(d)
                if not m[C]:
                    raise KeyError(f'{C} が bag にありません')
                jt, jv = series(m[J], js_vec)
                at, av = series(m[A], arm_vec)
                gt, gv = series(m[G], grip_vec)
                ct, cv = series(m[C], cube_vec)
                t0 = max(jt[0], at[0], gt[0], ct[0])
                t1 = min(jt[-1], at[-1], gt[-1], ct[-1])
                n = int((t1 - t0) * freq / 1e9)
                if n < 2:
                    raise ValueError('エピソードが短すぎます')
                eps.append(dict(d=d, jt=jt, jv=jv, at=at, av=av, gt=gt, gv=gv,
                                ct=ct, cv=cv, t0=t0, t1=t1, n=n))
                print(f'{d}: {n} frames')
            except Exception as e:
                print(f'SKIP {d}: {e}')
        if not eps:
            raise RuntimeError('変換できるエピソードがありません')
        max_frames = max(e['n'] for e in eps)
        print(f'longest: {max_frames} frames, episodes: {len(eps)}')
        for k, e in enumerate(eps):
            tt = np.linspace(e['t0'], e['t1'], e['n'])
            state = hold(e['jt'], e['jv'], tt)
            action = np.hstack([hold(e['at'], e['av'], tt), hold(e['gt'], e['gv'], tt)])
            cube = hold(e['ct'], e['cv'], tt)
            try:
                for i in range(max_frames):
                    j = min(i, e['n'] - 1)
                    self.dataset.add_frame({
                        'observation.state': torch.tensor(state[j], dtype=torch.float32),
                        'observation.environment_state': torch.tensor(cube[j], dtype=torch.float32),
                        'action': torch.tensor(action[j], dtype=torch.float32),
                        'task': self.task_name,
                    })
                self.dataset.save_episode()
                print(f'episode {k} saved')
            except Exception:
                traceback.print_exc()
        return str(self.dataset.root)

def main():
    with open('configs/conversion_config.yaml') as f:
        c = yaml.safe_load(f)
    conv = PushConverter(c['bag_dir'], c['output_dir'], c['config'], c.get('target_freq', 20))
    print(conv.convert())

if __name__ == '__main__':
    main()

