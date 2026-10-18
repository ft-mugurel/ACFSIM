"""Locations of the Steam install and the bridge files.

Override them when the game is not in the default Steam library:

- ``AC_STEAM_HOME`` — directory that contains ``.local/share/Steam`` (default: ``$HOME``)
- ``AC_APP_ID`` — Steam app id (default: ``244210``, original Assetto Corsa)
- ``AC_BRIDGE_DIR`` — folder where the copier writes ``physics.bin``
- ``AC_TRACK_KN5`` — one track model used to raycast the lidar. When unset,
  the lidar uses the track selected in the game's ``race.ini``.
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


def _ini_values(path: Path):
    section = ''
    for raw in path.read_text(encoding='utf-8', errors='replace').splitlines():
        line = raw.strip()
        if not line or line.startswith((';', '#')):
            continue
        if line.startswith('[') and line.endswith(']'):
            section = line[1:-1].strip().upper()
            continue
        if '=' not in line:
            continue
        key, value = line.split('=', 1)
        yield section, key.strip().upper(), value.strip()


def _tracks_root() -> Path:
    return (
        steam_home()
        / '.local/share/Steam/steamapps/common/assettocorsa/content/tracks')


def _race_track():
    """Track folder name and layout from the last Content Manager session."""
    app_id = os.environ.get('AC_APP_ID', '244210')
    ini = (
        steam_home()
        / '.local/share/Steam/steamapps/compatdata'
        / app_id
        / 'pfx/drive_c/users/steamuser/Documents/Assetto Corsa/cfg/race.ini')
    if not ini.is_file():
        return None
    track = ''
    config = ''
    for section, key, value in _ini_values(ini):
        if section != 'RACE':
            continue
        if key == 'TRACK':
            track = value
        elif key == 'CONFIG_TRACK':
            config = value
    if not track:
        return None
    return track, config


def _models_ini(folder: Path, config: str):
    if config:
        for path in (folder / config / 'models.ini', folder / ('models_%s.ini' % config)):
            if path.is_file():
                return path
    direct = folder / 'models.ini'
    if direct.is_file():
        return direct
    named = sorted(folder.glob('models_*.ini'))
    if len(named) == 1:
        return named[0]
    return None


def _kn5s_from_models(models: Path, track_folder: Path):
    files = []
    for _section, key, value in _ini_values(models):
        if key != 'FILE' or not value:
            continue
        relative = Path(value.replace('\\', '/'))
        for base in (models.parent, track_folder):
            path = base / relative
            if path.is_file():
                files.append(path)
                break
    return files


def track_kn5_files() -> list:
    """KN5 files the lidar raycasts for the session that is loaded in the game.

    ``AC_TRACK_KN5`` forces a single file. Otherwise the files are the
    ``FILE=`` entries of that track's ``models.ini`` (layout included). The
    fallback, when the game has no ``race.ini``, is FS UK Sprint.
    """
    override = os.environ.get('AC_TRACK_KN5')
    if override:
        return [Path(override)]
    root = _tracks_root()
    chosen = _race_track()
    track = chosen[0] if chosen else 'fs_uk_sprint'
    config = chosen[1] if chosen else ''
    folder = root / track
    models = _models_ini(folder, config)
    if models is not None:
        files = _kn5s_from_models(models, folder)
        if files:
            return files
    direct = folder / ('%s.kn5' % track)
    if direct.is_file():
        return [direct]
    return [root / 'fs_uk_sprint/fs_uk_sprint.kn5']


def track_key() -> str:
    """Folder name of the track in the current game session, plus the layout."""
    chosen = _race_track()
    if not chosen:
        return ''
    track, config = chosen
    if config:
        return '%s/%s' % (track, config)
    return track


def track_kn5() -> Path:
    override = os.environ.get('AC_TRACK_KN5')
    if override:
        return Path(override)
    files = track_kn5_files()
    for path in files:
        if path.stem == path.parent.name:
            return path
    return files[0]
