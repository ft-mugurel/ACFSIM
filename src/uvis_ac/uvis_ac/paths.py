"""Locations of the Steam install and the bridge files.

Override them when the game is not in the default Steam library:

- ``AC_STEAM_HOME`` — directory that contains ``.local/share/Steam`` (default: ``$HOME``)
- ``AC_APP_ID`` — Steam app id (default: ``244210``, original Assetto Corsa)
- ``AC_BRIDGE_DIR`` — folder where the copier writes ``physics.bin``
- ``AC_TRACK_KN5`` — track model used to raycast the lidar
"""

import os
from pathlib import Path


def _has_game(home: Path) -> bool:
    return (home / '.local/share/Steam/steamapps/common/assettocorsa').is_dir()


def steam_home() -> Path:
    override = os.environ.get('AC_STEAM_HOME')
    if override:
        return Path(override)
    home = Path.home()
    if _has_game(home):
        return home
    # Distrobox sets HOME under ~/.distrobox while Steam stays on the host.
    host = os.environ.get('DISTROBOX_HOST_HOME')
    if host and _has_game(Path(host)):
        return Path(host)
    return home


def bridge_dir() -> Path:
    override = os.environ.get('AC_BRIDGE_DIR')
    if override:
        return Path(override)
    app_id = os.environ.get('AC_APP_ID', '244210')
    return (
        steam_home()
        / '.local/share/Steam/steamapps/compatdata'
        / app_id
        / 'pfx/drive_c/uvis')


def track_kn5() -> Path:
    override = os.environ.get('AC_TRACK_KN5')
    if override:
        return Path(override)
    return (
        steam_home()
        / '.local/share/Steam/steamapps/common/assettocorsa'
        / 'content/tracks/fs_uk_sprint/fs_uk_sprint.kn5')
