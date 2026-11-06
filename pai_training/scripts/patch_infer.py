
# python3 ~/pai_training/scripts/patch_infer.py
# python3 -m py_compile ~/pai_training/scripts/infer.py && echo OK

import re, shutil
p = '/home/ubuntu/pai_training/scripts/infer.py'
shutil.copy(p, p + '.bak')
s = open(p).read().replace('\r\n', '\n')

s = s.replace('from sensor_msgs.msg import JointState',
              'from sensor_msgs.msg import JointState\nfrom geometry_msgs.msg import PoseStamped', 1)
s, k = re.subn(r'^([ \t]*)self\.latest_joint_state = None[ \t]*$',
               r"\1self.latest_joint_state = None\n\1self.latest_cube = None\n"
               r"\1self.create_subscription(PoseStamped, '/cube_pose', self.cube_callback, 10)",
               s, count=1, flags=re.M)
assert k == 1, 'init'
s, k = re.subn(r'(    def joint_state_callback\(self, msg\):)',
               "    def cube_callback(self, msg):\n"
               "        self.latest_cube = np.array([msg.pose.position.x, msg.pose.position.y], dtype=np.float32)\n\n\\1",
               s, count=1)
assert k == 1, 'callback'
s, k = re.subn(r'if self\.latest_joint_state is None:',
               'if self.latest_joint_state is None or self.latest_cube is None:', s, count=1)
assert k == 1, 'guard'
s, k = re.subn(r"('observation\.environment_state': torch\.tensor\()self\.latest_joint_state",
               r"\1self.latest_cube", s, count=1)
assert k == 1, 'obs'
open(p, 'w').write(s)
print('patched')

