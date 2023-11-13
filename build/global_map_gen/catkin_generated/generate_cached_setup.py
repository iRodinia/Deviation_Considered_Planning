# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/noetic/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/noetic/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in '/home/cz_linux/Documents/Deviation_Considered_Planning/devel;/home/cz_linux/catkin_overlay_ws/devel;/opt/ros/noetic;/home/cz_linux/Documents/catkin_ws_build/devel;/home/cz_linux/Documents/catkin_ws_make/devel;/home/cz_linux/Documents/ego-planner-swarm/devel;/home/cz_linux/Documents/crazyswarm/ros_ws/devel'.split(';'):
        python_path = os.path.join(workspace, 'lib/python3/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/cz_linux/Documents/Deviation_Considered_Planning/devel/.private/global_map_gen/env.sh')

output_filename = '/home/cz_linux/Documents/Deviation_Considered_Planning/build/global_map_gen/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    # print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
