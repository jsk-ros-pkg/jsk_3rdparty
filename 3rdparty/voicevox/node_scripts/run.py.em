#!/usr/bin/env python3
from __future__ import annotations

import imp
import os.path as osp
import os

import rospkg

PKG_NAME = 'voicevox'
abs_path = osp.dirname(osp.abspath(__file__))
voicevox_engine = imp.load_package(
    'voicevox_engine', osp.join(abs_path, 'voicevox_engine/voicevox_engine'))
rospack = rospkg.RosPack()
voicevox_dir = rospack.get_path(PKG_NAME)
voicevox_lib_dir = osp.join(voicevox_dir, 'lib')
os.environ['OPEN_JTALK_DICT_DIR'] = osp.join(
    voicevox_dir, 'dict', 'open_jtalk_dic_utf_8-1.11')

@{
with open(VOICEVOX_ENGINE_RUN_PY, "r") as f:
    empy.write(f.read())
}@
