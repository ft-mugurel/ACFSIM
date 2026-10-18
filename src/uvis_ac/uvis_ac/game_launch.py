"""Launch Assetto Corsa on a virtual screen.

Content Manager is not used. The panel picks a ``uvis_*`` track, this
process writes that track into ``race.ini``, and Proton starts ``acs.exe``.
The game window is moved onto a headless Hyprland output named ``uvis-ac``
so it never covers the laptop. The camera grabs that output.

One output is one view. A later second camera would be a second line in
``capture_output.txt`` and a second output, the same way a headset has one
panel per eye. This launch writes only the first line.
"""

import os
import signal
import subprocess
import sys
import time
from pathlib import Path

from uvis_ac.paths import steam_home

OUTPUT = 'uvis-ac'
# One ZED X sensor at SVGA, 16:10. The bonnet camera's vertical field
# of view is 78.327 deg, which spans 105 deg across this frame.
WIDTH = 960
HEIGHT = 600
POSITION = '8000x8000'
WORKSPACE_ID = 91
CAR = 'madformulateam_mftc3'
SKIN = '1_lava'

_HOST_LD = Path('/run/host/usr/lib/ld-linux-x86-64.so.2')
_HOST_LIB = '/run/host/usr/lib:/run/host/usr/lib64'


def _host(name):
    binary = Path('/run/host/usr/bin') / name
    if binary.is_file() and _HOST_LD.is_file():
        return [str(_HOST_LD), '--library-path', _HOST_LIB, str(binary)]
    return [name]


def _hypr(*args, check=True):
    try:
        done = subprocess.run(
            _host('hyprctl') + list(args),
            check=False,
            capture_output=True,
            text=True,
            timeout=3,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        raise RuntimeError('hyprctl %s failed: %s' % (args[0], exc)) from exc
    text = (done.stdout or '') + (done.stderr or '')
    if check and done.returncode != 0:
        raise RuntimeError(text.strip() or 'hyprctl %s failed' % args[0])
    return text


class MonitorMissing(RuntimeError):
    """The compositor was asked to use a monitor that is not in the layout."""


def _is_monitor_missing(text):
    lowered = (text or '').lower()
    return (
        'invalid monitor' in lowered
        or "monitor doesn't exist" in lowered
        or 'monitor not found' in lowered)


def _eval(code):
    text = _hypr('eval', code)
    if 'error' in text.lower() and not text.strip().startswith('ok'):
        message = text.strip()
        if _is_monitor_missing(message):
            raise MonitorMissing(message)
        raise RuntimeError(message)
    return text


def _monitors():
    import json
    # `all` includes a disabled headless output. The plain list hides it,
    # and `output remove` then says it is not there.
    return json.loads(_hypr('monitors', 'all', '-j'))


def _clients():
    import json
    return json.loads(_hypr('clients', '-j'))


def _workspaces():
    import json
    return json.loads(_hypr('workspaces', '-j'))


def _monitor_name(monitors, index):
    for monitor in monitors:
        if monitor.get('id') == index:
            return monitor.get('name')
    return None


def _focused_name(monitors):
    for monitor in monitors:
        if monitor.get('focused'):
            return monitor.get('name')
    return None


def _restore_focus(previous):
    focused = _focused_name(_monitors())
    if previous and focused == OUTPUT and previous != OUTPUT:
        _eval('hl.dispatch(hl.dsp.focus({ monitor = "%s" }))' % previous)


def _without_cursor_warp(action):
    """Hyprland warps the pointer to the center of a window it focuses or moves.

    cursor:no_warps is left as it was found. The change is only for this call.
    """
    text = _hypr('getoption', 'cursor:no_warps', check=False)
    line = (text.splitlines() or [''])[0].strip()
    already = line.endswith('true') or line.endswith('1')
    if not already:
        _hypr('keyword', 'cursor:no_warps', 'true', check=False)
    try:
        return action()
    finally:
        if not already:
            _hypr('keyword', 'cursor:no_warps', 'false', check=False)


def _apply_monitor():
    # disabled = false wakes an output that an earlier remove left behind.
    _eval(
        'hl.monitor({ output = "%s", mode = "%dx%d@60", position = "%s", scale = 1, disabled = false })'
        % (OUTPUT, WIDTH, HEIGHT, POSITION))


def _output_ready():
    for monitor in _monitors():
        if monitor.get('name') != OUTPUT:
            continue
        workspace = (monitor.get('activeWorkspace') or {}).get('id')
        return (
            not monitor.get('disabled')
            and monitor.get('width') == WIDTH
            and monitor.get('height') == HEIGHT
            and int(monitor.get('x') or 0) == 8000
            and int(monitor.get('y') or 0) == 8000
            and workspace not in (None, 0))
    return False


def _wait_until_parked():
    """The headless output appears at the right edge first. Do not launch until it has moved."""
    for _ in range(15):
        if _output_ready():
            return True
        time.sleep(0.2)
    _apply_monitor()
    time.sleep(0.4)
    return _output_ready()


# Reinstalling this rule applies fullscreen to the live game window again,
# which destroys it. Assetto Corsa then opens a new session.
_rule_installed = False


def _install_rule():
    global _rule_installed
    if _rule_installed:
        return
    # Title, not class: Proton's own launch is not always steam_app_244210.
    # no_initial_focus matters: no_focus alone still warps the pointer on Hyprland 0.56.
    _eval(
        'hl.window_rule({ name = "uvis-ac-game", match = { title = "^Assetto Corsa$" }, '
        'float = true, no_focus = true, no_initial_focus = true, '
        'focus_on_activate = false, stay_focused = false, '
        'monitor = "%s", fullscreen = true, border_size = 0 })'
        % OUTPUT)
    _rule_installed = True


def ensure_output():
    """Create the virtual screen and keep it off to the side of the real ones."""
    before = {}
    monitors = _monitors()
    focused_before = _focused_name(monitors)
    for client in _clients():
        before[client.get('address')] = _monitor_name(monitors, client.get('monitor'))

    def _create():
        names = {monitor.get('name') for monitor in _monitors()}
        if OUTPUT not in names:
            _hypr('output', 'create', 'headless', OUTPUT)
        _apply_monitor()
        _install_rule()
        monitors_now = _monitors()
        for client in _clients():
            now = _monitor_name(monitors_now, client.get('monitor'))
            previous = before.get(client.get('address'))
            if now == OUTPUT and previous and previous != OUTPUT:
                _eval(
                    'hl.dispatch(hl.dsp.window.move({ window = "address:%s", monitor = "%s", follow = false }))'
                    % (client.get('address'), previous))
        _park_workspace()
        _restore_focus(focused_before)

    _without_cursor_warp(_create)
    if not _wait_until_parked():
        raise RuntimeError('virtual screen %s did not park at %s' % (OUTPUT, POSITION))


def _park_workspace():
    """Rename the empty workspace the virtual screen took so it is not workspace 3."""
    current = None
    for monitor in _monitors():
        if monitor.get('name') == OUTPUT:
            current = (monitor.get('activeWorkspace') or {}).get('id')
    if current in (None, 0, WORKSPACE_ID):
        return
    windows = 0
    for workspace in _workspaces():
        if workspace.get('id') == WORKSPACE_ID:
            return
        if workspace.get('id') == current:
            windows = workspace.get('windows') or 0
    if windows:
        return
    try:
        _eval('hl.dispatch(hl.dsp.workspace.change_id({ workspace = "%s", id = %d }))' % (current, WORKSPACE_ID))
    except RuntimeError:
        return


def remove_output():
    def _remove():
        text = _hypr('output', 'remove', OUTPUT, check=False)
        if 'not found' not in text.lower():
            return
        # A disabled headless output stays reserved but is invisible to remove.
        if not any(monitor.get('name') == OUTPUT for monitor in _monitors()):
            return
        focused = _focused_name(_monitors())
        try:
            _apply_monitor()
        except RuntimeError:
            return
        monitors = _monitors()
        output_id = None
        for monitor in monitors:
            if monitor.get('name') == OUTPUT:
                output_id = monitor.get('id')
        home = focused if focused and focused != OUTPUT else 'eDP-1'
        for client in _clients():
            address = client.get('address')
            if address and client.get('monitor') == output_id:
                try:
                    _eval(
                        'hl.dispatch(hl.dsp.window.move({ window = "address:%s", monitor = "%s", follow = false }))'
                        % (address, home))
                except RuntimeError:
                    pass
        _hypr('output', 'remove', OUTPUT, check=False)
        _restore_focus(focused)

    _without_cursor_warp(_remove)


# address -> next time it may be moved. Once it is on the virtual screen this
# stays far ahead, so a 0 fullscreen flag cannot move it again every 0.4 s.
_place_next = {}
_PLACE_WAIT = 1.5


def _window_moves(clients, output_id, focused, now):
    """Dispatches that still need to happen. A settled game window adds none."""
    codes = []
    placed = False
    for client in clients:
        address = client.get('address')
        if not address:
            continue
        title = client.get('title') or ''
        on_virtual = client.get('monitor') == output_id
        if title != 'Assetto Corsa':
            if on_virtual and now >= _place_next.get(address, 0):
                home = focused if focused and focused != OUTPUT else 'eDP-1'
                codes.append(
                    'hl.dispatch(hl.dsp.window.move({ window = "address:%s", monitor = "%s", follow = false }))'
                    % (address, home))
                _place_next[address] = now + _PLACE_WAIT
            continue
        placed = True
        if on_virtual:
            # Already on the virtual screen. Fullscreen again recreates the
            # window, and Assetto Corsa opens a new session.
            _place_next[address] = now + 3600
            continue
        # It was settled on the virtual screen and then left. Moving it back
        # recreates the window, so the session opens again.
        if _place_next.get(address, 0) > now + _PLACE_WAIT:
            continue
        if now < _place_next.get(address, 0):
            continue
        _place_next[address] = now + _PLACE_WAIT
        codes.append(
            'hl.dispatch(hl.dsp.window.move({ window = "address:%s", monitor = "%s", follow = false }))'
            % (address, OUTPUT))
        codes.append(
            'hl.dispatch(hl.dsp.window.fullscreen({ window = "address:%s", mode = "fullscreen", action = "set" }))'
            % address)
    return placed, codes


def _game_open(clients):
    return any((client.get('title') or '') == 'Assetto Corsa' for client in clients)


def _usable_output(monitor):
    """Enabled and in the layout. A disabled headless output is not a move target."""
    if not monitor or monitor.get('disabled'):
        return False
    try:
        width = int(monitor.get('width') or 0)
        height = int(monitor.get('height') or 0)
    except (TypeError, ValueError):
        return False
    return width > 0 and height > 0


def maintenance_action(game_open, usable):
    """What to do on a 0.4 s tick.

    The game window already exists and the virtual screen is gone: do nothing.
    Creating the screen again, or moving the window onto a name Hyprland has
    dropped, destroys that window and Assetto Corsa opens a new session.
    """
    if game_open and not usable:
        return 'leave'
    if not usable:
        return 'create'
    return 'place'


_warned = set()


def _warn_once(key, message):
    if key in _warned:
        return
    _warned.add(key)
    print(message, flush=True)


def _output_named(monitors):
    for monitor in monitors:
        if monitor.get('name') == OUTPUT:
            return monitor
    return None


def place_game():
    """Move the game onto the virtual screen. Leave the user's focus where it was."""
    monitors = _monitors()
    clients = _clients()
    monitor = _output_named(monitors)
    action = maintenance_action(_game_open(clients), _usable_output(monitor))
    if action == 'leave':
        _warn_once(
            'leave',
            'virtual screen %s is not available; leaving the game window alone' % OUTPUT)
        return False
    if action == 'create':
        ensure_output()
        monitors = _monitors()
        monitor = _output_named(monitors)
        if not _usable_output(monitor):
            return False
    elif not _game_open(clients):
        # The game is not mapped yet, so parking the screen cannot restart a session.
        # Once the window exists, a modeset removes the headless output and the
        # next move reports that the monitor does not exist.
        drifted = (
            int(monitor.get('width') or 0) != WIDTH
            or int(monitor.get('height') or 0) != HEIGHT
            or int(monitor.get('x') or 0) != 8000
            or int(monitor.get('y') or 0) != 8000)
        if drifted and time.monotonic() >= _place_next.get('monitor', 0):
            _place_next['monitor'] = time.monotonic() + 5
            try:
                _without_cursor_warp(_apply_monitor)
            except MonitorMissing as exc:
                _warn_once('park', 'virtual screen %s could not be parked: %s' % (OUTPUT, exc))
    focused = _focused_name(monitors)
    placed, codes = _window_moves(clients, monitor.get('id'), focused, time.monotonic())
    if not codes:
        return placed

    def _apply():
        for code in codes:
            try:
                _eval(code)
            except MonitorMissing as exc:
                _warn_once('move', 'left the game window alone: %s' % exc)
                return
        _restore_focus(focused)

    _without_cursor_warp(_apply)
    return placed


def _game_dir():
    return steam_home() / '.local/share/Steam/steamapps/common/assettocorsa'


def _compat():
    app = os.environ.get('AC_APP_ID', '244210')
    return steam_home() / '.local/share/Steam/steamapps/compatdata' / app


def _cfg_dir():
    return _compat() / 'pfx/drive_c/users/steamuser/Documents/Assetto Corsa/cfg'


def _bridge():
    override = os.environ.get('AC_BRIDGE_DIR')
    if override:
        return Path(override)
    return _compat() / 'pfx/drive_c/uvis'


def _proton():
    override = os.environ.get('AC_PROTON')
    if override:
        return Path(override)
    return steam_home() / '.local/share/Steam/compatibilitytools.d/GE-Proton9-20/proton'


def track_name(text):
    name = (text or '').strip()
    if not name:
        raise SystemExit('Pick a track.')
    if not name.startswith('uvis_'):
        name = 'uvis_' + name
    folder = _game_dir() / 'content/tracks' / name
    marker = folder / 'uvis_eufs_track.txt'
    if not marker.is_file():
        raise SystemExit('No generated track at %s' % folder)
    return name


def _update_ini(path, sections):
    """Set keys inside an Assetto ini section. Other lines stay as they are."""
    # read_text() turns CRLF into LF, and Assetto writes race.ini with CRLF.
    blob = path.read_bytes() if path.is_file() else b''
    newline = '\r\n' if b'\r\n' in blob or not blob else '\n'
    raw = blob.decode('utf-8', errors='replace').replace('\r\n', '\n').replace('\r', '\n')
    lines = raw.split('\n')
    if lines and lines[-1] == '':
        lines.pop()
    blocks = []
    current = ['', []]
    for line in lines:
        stripped = line.strip()
        if stripped.startswith('[') and stripped.endswith(']') and not stripped.startswith((';', '#')):
            blocks.append(current)
            current = [stripped[1:-1].strip().upper(), [line]]
        else:
            current[1].append(line)
    blocks.append(current)
    found = set()
    for name, body in blocks:
        if name not in sections:
            continue
        found.add(name)
        seen = set()
        updated = []
        for line in body:
            stripped = line.strip()
            if '=' in line and not stripped.startswith((';', '#')) and not stripped.startswith('['):
                key = line.split('=', 1)[0].strip().upper()
                if key in sections[name]:
                    updated.append('%s=%s' % (line.split('=', 1)[0].strip(), sections[name][key]))
                    seen.add(key)
                    continue
            updated.append(line)
        for key, value in sections[name].items():
            if key not in seen:
                insert_at = len(updated)
                while insert_at > 0 and updated[insert_at - 1].strip() == '':
                    insert_at -= 1
                updated.insert(insert_at, '%s=%s' % (key, value))
        body[:] = updated
    for name, keys in sections.items():
        if name in found:
            continue
        blocks.append([name, ['[%s]' % name] + ['%s=%s' % item for item in keys.items()]])
    out = []
    for _name, body in blocks:
        out.extend(body)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes((newline.join(out) + newline).encode('utf-8'))


def write_session(track):
    cfg = _cfg_dir()
    _update_ini(cfg / 'race.ini', {
        'RACE': {
            'TRACK': track,
            'CONFIG_TRACK': '',
            'MODEL': CAR,
            'SKIN': SKIN,
            'CARS': '1',
        },
        'SESSION_0': {
            'NAME': 'Practice',
            'TYPE': '1',
            'SPAWN_SET': 'START',
        },
        'CAR_0': {
            'SKIN': SKIN,
            'MODEL': '-',
        },
    })
    _update_ini(cfg / 'video.ini', {
        'VIDEO': {
            'FULLSCREEN': '0',
            'WIDTH': str(WIDTH),
            'HEIGHT': str(HEIGHT),
            'INDEX': '0',
            'REFRESH': '60',
        },
    })


def install_lua():
    source = Path(__file__).resolve().parents[1] / 'csp' / 'uvis_camera' / 'uvis_camera.lua'
    if not source.is_file():
        raise SystemExit('Missing %s' % source)
    dest = _game_dir() / 'apps/lua/uvis_camera/uvis_camera.lua'
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_bytes(source.read_bytes())


def _already_running():
    proc = Path('/proc')
    for entry in proc.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            cmd = (entry / 'cmdline').read_bytes().replace(b'\0', b' ')
        except OSError:
            continue
        if cmd.strip().endswith(b'acs.exe') or b'/acs.exe' in cmd or b'\\acs.exe' in cmd:
            return True
    return False


# Host user unit. The panel runs in Distrobox; this unit does not.
_UNIT = 'uvis-ac-game'


def _systemctl(*args, timeout=8):
    try:
        return subprocess.run(
            ['systemctl', '--user', *args],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=timeout,
        )
    except (OSError, subprocess.SubprocessError):
        return None


def _stop_unit():
    # --no-block so a panel SIGKILL during cleanup still leaves systemd
    # stopping the game. The unit is only this launch, not the Steam client.
    _systemctl('stop', '--no-block', _UNIT, timeout=3)


def _steam_appid(game, app_id):
    """Keep Steam from quitting acs.exe and relaunching it through Content Manager."""
    path = game / 'steam_appid.txt'
    text = app_id + '\n'
    try:
        current = path.read_text(encoding='utf-8', errors='replace')
    except OSError:
        current = ''
    if current.strip() != app_id:
        path.write_text(text, encoding='utf-8')


def _spawn(track):
    proton = _proton()
    game = _game_dir()
    acs = game / 'acs.exe'
    if not proton.is_file():
        raise SystemExit('Proton not found at %s' % proton)
    if not acs.is_file():
        raise SystemExit('Missing %s' % acs)
    app_id = os.environ.get('AC_APP_ID', '244210')
    steam = steam_home() / '.local/share/Steam'
    _steam_appid(game, app_id)
    runtime = steam / 'steamapps/common/SteamLinuxRuntime_sniper/_v2-entry-point'
    if not runtime.is_file():
        raise SystemExit('Steam Linux Runtime is missing at %s' % runtime)
    # The Distrobox user namespace cannot see the Steam client's /proc entry,
    # and its HOME is not ~/.steam. acs.exe then exits from SteamAPI_Init.
    # A host user service has the real home, DISPLAY, and the same uid as Steam.
    # UMU_ID is set empty in that session; Proton treats any value as "set"
    # and runs `wine start /unix`, which returns before the game does.
    # `proton run` joins a wineserver that is already up instead of waiting
    # for the AC bridge copier to quit. cwd must be the game directory.
    # A cwd under /run/host makes pressure-vessel refuse to start.
    wine_overrides = os.environ.get('WINEDLLOVERRIDES') or 'dwrite=n,b'
    env_pairs = {
        'STEAM_COMPAT_CLIENT_INSTALL_PATH': str(steam),
        'STEAM_COMPAT_DATA_PATH': str(_compat()),
        'STEAM_COMPAT_APP_ID': app_id,
        'SteamAppId': app_id,
        'SteamGameId': app_id,
        'SteamOverlayGameId': app_id,
        '__NV_PRIME_RENDER_OFFLOAD': '1',
        '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
        'WINEDLLOVERRIDES': wine_overrides,
    }
    _systemctl('stop', _UNIT, timeout=8)
    _systemctl('reset-failed', _UNIT, timeout=3)
    command = [
        'systemd-run', '--user', '--quiet', '--pipe', '--wait', '--collect',
        '--unit', _UNIT,
        '--working-directory', str(game),
        '--property', 'TimeoutStopSec=5',
        '--property', 'UnsetEnvironment=UMU_ID',
    ]
    for key, value in env_pairs.items():
        command.append('--setenv=%s=%s' % (key, value))
    command += [
        '--',
        str(runtime), '--verb=waitforexitandrun', '--',
        str(proton), 'run', str(acs),
    ]
    print('launching %s on %s' % (acs.name, track), flush=True)
    try:
        return subprocess.Popen(command)
    except OSError as exc:
        raise SystemExit('Could not start the host game session: %s' % exc) from exc


def main() -> None:
    track = track_name(sys.argv[1] if len(sys.argv) > 1 else '')
    if _already_running():
        raise SystemExit('Assetto Corsa is already running. Stop that game before starting another.')
    stop = {'now': False}

    def _stop(_signum, _frame):
        stop['now'] = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    capture = _bridge() / 'capture_output.txt'
    proc = None
    try:
        ensure_output()
        write_session(track)
        install_lua()
        capture.parent.mkdir(parents=True, exist_ok=True)
        # One name per view. Only the forward view is written for now.
        capture.write_text(OUTPUT + '\n')
        print('virtual screen %s %dx%d' % (OUTPUT, WIDTH, HEIGHT), flush=True)
        print('track %s' % track, flush=True)
        proc = _spawn(track)
        announced = False
        while not stop['now']:
            if proc.poll() is not None:
                print('game exited %s' % proc.returncode, flush=True)
                break
            try:
                if place_game() and not announced:
                    print('game is on the virtual screen', flush=True)
                    announced = True
            except MonitorMissing as exc:
                _warn_once('monitor', 'left the game window alone: %s' % exc)
            except RuntimeError as exc:
                _warn_once('hypr', str(exc))
            time.sleep(0.4)
    finally:
        capture.unlink(missing_ok=True)
        remove_output()
        _stop_unit()
        if proc is not None and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()


def self_check():
    assert maintenance_action(True, False) == 'leave'
    assert maintenance_action(True, True) == 'place'
    assert maintenance_action(False, False) == 'create'
    assert maintenance_action(False, True) == 'place'
    assert _usable_output(None) is False
    assert _usable_output({'disabled': True, 'width': 800, 'height': 600}) is False
    assert _usable_output({'disabled': False, 'width': 0, 'height': 600}) is False
    assert _usable_output({'disabled': False, 'width': 800, 'height': 600}) is True
    assert _is_monitor_missing("error: =[C]:-1: Invalid monitor / monitor doesn't exist")
    _place_next.clear()
    placed, codes = _window_moves(
        [{
            'address': '0xsettled',
            'title': 'Assetto Corsa',
            'monitor': 3,
            'size': [1920, 1080],
            'fullscreen': 0,
        }],
        3,
        'HDMI-A-1',
        0,
    )
    assert placed and codes == []
    placed, codes = _window_moves(
        [{
            'address': '0xsettled',
            'title': 'Assetto Corsa',
            'monitor': 1,
            'size': [2560, 1418],
            'fullscreen': 0,
        }],
        3,
        'HDMI-A-1',
        10,
    )
    assert placed and codes == []
    placed, codes = _window_moves(
        [{
            'address': '0xfresh',
            'title': 'Assetto Corsa',
            'monitor': 1,
            'size': [800, 600],
            'fullscreen': 0,
        }],
        3,
        'HDMI-A-1',
        0,
    )
    assert placed
    assert any(OUTPUT in code and 'window.move' in code for code in codes)
    assert any('fullscreen' in code for code in codes)
    print('game launch ok')


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '--check':
        self_check()
    else:
        main()
