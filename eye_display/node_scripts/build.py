#!/usr/bin/env python

import rospkg
import shutil
import os
import yaml
import argparse
from typing import Dict


def get_target_directory():
    return rospkg.RosPack().get_path("eye_display")


def get_target_data_directory():
    return os.path.join(get_target_directory(), "data")


def clear_directory(target_directory):
    for filepath in os.listdir(target_directory):
        if filepath[0] == ".":
            continue
        fullpath = os.path.join(target_directory, filepath)
        os.remove(fullpath)


def load_param(filepath):
    file_directory = os.path.dirname(filepath)
    target_directory = get_target_data_directory()
    with open(filepath, "r") as f:
        data = yaml.load(f)

    if not isinstance(data, dict):
        raise ValueError()

    ans = {}
    for key, value in data.items():
        if os.path.exists(value):
            source = value
        else:
            source = os.path.join(file_directory, value)
        target = os.path.join(target_directory, key + ".jpg")
        ans[source] = target

    return ans


def copy_layer_image(source, target):
    ret = os.system("convert {} -background white -resize 139x139 -flatten {}".format(source, target))
    if ret != 0:
        raise SystemExit("Error: could not run convert command correctly (try apt install imagemagick), please check 'convert' command is installed or {source} and {target} file exists".format(source, target))


def run_platformio_project(env, port):
    current_directory = os.getcwd()
    os.chdir(get_target_directory())
    os.system("pio run -e {} -t uploadfs --upload-port {}".format(env, port))
    os.system("pio run -e {} -t upload --upload-port {}".format(env, port))
    os.chdir(current_directory)


def main(target_param_filename, env, port):
    params = load_param(target_param_filename)
    print("params: {}".format(params))
    clear_directory(get_target_data_directory())
    for source, target in params.items():
        print("copying {} to {}".format(source, target))
        copy_layer_image(source, target)
    run_platformio_project(env, port)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("param_filepath")
    parser.add_argument("--env", required=True)
    parser.add_argument("--port", required=True)
    args = parser.parse_args()

    main(args.param_filepath, args.env, args.port)
