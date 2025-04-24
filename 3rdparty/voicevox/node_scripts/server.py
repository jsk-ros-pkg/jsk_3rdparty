#!/usr/bin/env python3

import os.path as osp
import rospkg
import rospy
import subprocess

PKG_NAME="voicevox"
rospack = rospkg.RosPack()
voicevox_dir = rospack.get_path(PKG_NAME)
voicevox_lib_dir = osp.join(voicevox_dir, 'lib')
print(voicevox_lib_dir)

def main():
    rospy.init_node("voicevox_server")
    host = rospy.get_param("~host", "127.0.0.1")
    port = rospy.get_param("~port", 50021)
    cpu_num_threads = rospy.get_param('~cpu_num_threads', None)
    if cpu_num_threads is None:
        cpu_num_threads = multiprocessing.cpu_count()

    cmd = ["rosrun", PKG_NAME, "run.py",
           "--host", host,
           "--port", str(port),
           "--cpu_num_threads", str(cpu_num_threads),
           "--voicelib_dir", voicevox_lib_dir]
    result = subprocess.run(cmd, capture_output=False, text=True)

if __name__ == "__main__":
    main()
