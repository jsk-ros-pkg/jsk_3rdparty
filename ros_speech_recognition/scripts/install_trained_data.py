#!/usr/bin/env python

import argparse
import multiprocessing
import os.path as osp

import jsk_data


def download_data(*args, **kwargs):
    p = multiprocessing.Process(
            target=jsk_data.download_data,
            args=args,
            kwargs=kwargs)
    p.start()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-v', '--verbose', dest='quiet', action='store_false')
    # relative to the package, or absolute (ROS2 install, see CMakeLists.txt)
    parser.add_argument('--data-dir', default='trained_data')
    args = parser.parse_args()
    quiet = args.quiet

    PKG = 'ros_speech_recognition'

    download_data(
        pkg_name=PKG,
        path=osp.join(args.data_dir, 'vosk-model-small-ja-0.22.zip'),
        url='https://alphacephei.com/vosk/models/vosk-model-small-ja-0.22.zip',  # NOQA
        md5='0e3163dd62dfb0d823353718ac3cbf79',
        extract=True,
        quiet=quiet,
    )

    download_data(
        pkg_name=PKG,
        path=osp.join(args.data_dir, 'vosk-model-small-en-us-0.15.zip'),
        url='https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip',  # NOQA
        md5='09ab50ccd62b674cbaa231b825f9c1cb',
        extract=True,
        quiet=quiet,
    )

if __name__ == '__main__':
    main()
