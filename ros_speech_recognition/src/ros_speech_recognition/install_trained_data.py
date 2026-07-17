#!/usr/bin/env python3

import argparse
import hashlib
import os
from pathlib import Path
import shutil
from tempfile import NamedTemporaryFile
from urllib.request import urlopen
import zipfile


MODELS = {
    'vosk-model-small-ja-0.22': {
        'url': 'https://alphacephei.com/vosk/models/'
               'vosk-model-small-ja-0.22.zip',
        'md5': '0e3163dd62dfb0d823353718ac3cbf79',
    },
    'vosk-model-small-en-us-0.15': {
        'url': 'https://alphacephei.com/vosk/models/'
               'vosk-model-small-en-us-0.15.zip',
        'md5': '09ab50ccd62b674cbaa231b825f9c1cb',
    },
}


def default_data_directory():
    root = os.environ.get(
        'XDG_DATA_HOME', os.path.expanduser('~/.local/share'))
    return Path(root) / 'ros_speech_recognition'


def download_model(name, output_directory):
    model = MODELS[name]
    destination = output_directory / name
    if destination.is_dir():
        print('{} already exists'.format(destination))
        return

    output_directory.mkdir(parents=True, exist_ok=True)
    print('Downloading {}'.format(model['url']))
    with urlopen(model['url']) as response, \
            NamedTemporaryFile(suffix='.zip') as archive:
        shutil.copyfileobj(response, archive)
        archive.flush()
        archive.seek(0)
        digest = hashlib.md5(archive.read()).hexdigest()
        if digest != model['md5']:
            raise RuntimeError(
                'Checksum mismatch for {}: expected {}, got {}'.format(
                    name, model['md5'], digest))
        archive.seek(0)
        with zipfile.ZipFile(archive) as zip_file:
            zip_file.extractall(output_directory)
    print('Installed {}'.format(destination))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--output-dir',
        type=Path,
        default=default_data_directory(),
        help='Directory in which Vosk models are installed',
    )
    parser.add_argument(
        'models',
        nargs='*',
        choices=sorted(MODELS),
        default=sorted(MODELS),
    )
    arguments = parser.parse_args()
    for model_name in arguments.models:
        download_model(model_name, arguments.output_dir)


if __name__ == '__main__':
    main()
