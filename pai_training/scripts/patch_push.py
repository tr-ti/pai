p = '/home/ubuntu/pai_training/scripts/rosbag2lerobot_push.py'
s = open(p).read().replace('\r\n', '\n')

s = s.replace(
    "from rosbag2lerobot import RosbagToLeRobot",
    "from rosbag2lerobot import RosbagToLeRobot\n"
    "from rosbags.rosbag2 import Reader\nfrom rosbags.serde import deserialize_cdr", 1)

s = s.replace(
    "class PushConverter(RosbagToLeRobot):\n",
    "class PushConverter(RosbagToLeRobot):\n"
    "    def _get_messages(self, episode_dir):\n"
    "        topics = {J: [], A: [], G: [], C: []}\n"
    "        with Reader(episode_dir) as reader:\n"
    "            conns = [x for x in reader.connections if x.topic in topics]\n"
    "            for conn, ts, raw in reader.messages(connections=conns):\n"
    "                topics[conn.topic].append((deserialize_cdr(raw, conn.msgtype), ts))\n"
    "        return topics\n\n", 1)

s = s.replace(
    "if C not in m:\n                    raise KeyError(f'{C} が bag にありません')",
    "if not m[C]:\n                    raise KeyError(f'{C} が bag にありません')", 1)

open(p, 'w').write(s)
print('patched')
