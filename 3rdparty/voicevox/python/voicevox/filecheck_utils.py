import hashlib
import os


def get_cache_dir():
    """Return cache dir.

    Returns
    -------
    cache_dir : str
        cache directory.
    """
    ros_home = os.getenv('ROS_HOME', os.path.expanduser('~/.ros'))
    pkg_ros_home = os.path.join(ros_home, 'voicevox_texttospeech')
    default_cache_dir = os.path.join(pkg_ros_home, 'cache')
    cache_dir = os.environ.get(
        'ROS_VOICEVOX_TEXTTOSPEECH_CACHE_DIR',
        default_cache_dir)
    if not os.path.exists(cache_dir):
        os.makedirs(cache_dir)
    return cache_dir


def checksum_md5(filename, blocksize=8192):
    """Calculate md5sum.

    Parameters
    ----------
    filename : str or pathlib.Path
        input filename.
    blocksize : int
        MD5 has 128-byte digest blocks (default: 8192 is 128x64).
    Returns
    -------
    md5 : str
        calculated md5sum.
    """
    filename = str(filename)
    hash_factory = hashlib.md5()
    with open(filename, 'rb') as f:
        for chunk in iter(lambda: f.read(blocksize), b''):
            hash_factory.update(chunk)
    return hash_factory.hexdigest()
