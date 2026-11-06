#!/bin/bash
ign topic -e -t /world/default/pose/info -n 1 | python3 -c "
import sys,re
t=sys.stdin.read()
i=t.find('name: \"wood_cube_5cm\"')
m=re.search(r'position \{(.*?)\}',t[i:i+600],re.S)
p=m.group(1) if m else ''
g=lambda k:(re.search(k+r': ([-0-9.e]+)',p) or [0,0])[1]
print('x=%s y=%s z=%s'%(g('x'),g('y'),g('z')))"
