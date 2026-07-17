import importlib.util
from pathlib import Path

import pytest


def load_request_synthesis():
    path = Path(__file__).parents[1] / 'node_scripts' / 'request_synthesis.py'
    spec = importlib.util.spec_from_file_location('request_synthesis', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_convert_to_str():
    module = load_request_synthesis()

    assert module.convert_to_str('こんにちは') == 'こんにちは'
    assert module.convert_to_str('こんにちは'.encode()) == 'こんにちは'
    with pytest.raises(ValueError):
        module.convert_to_str(1)


def test_speakers_cache_round_trip(tmp_path, monkeypatch):
    module = load_request_synthesis()
    monkeypatch.setenv('ROS_HOME', str(tmp_path))

    speakers = {'2': '四国めたん-ノーマル'}
    module.save_speakers_cache(speakers)

    assert module.get_voicevox_cache_dir() == str(tmp_path / 'voicevox')
    assert module.load_speakers_cache() == speakers
