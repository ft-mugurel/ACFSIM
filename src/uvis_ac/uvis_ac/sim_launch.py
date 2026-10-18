"""Open the game, then the AC bridge.

The bridge copier has to join the wineserver the game already started, and
the lidar reads ``race.ini`` once when the bridge starts. Starting the
bridge first leaves the two unable to see each other. This process owns
both, in that order.

The panel kills this process group two seconds after Stop. The game and
the bridge are therefore their own sessions. Stop signals the bridge
first, then the game. It never signals this process group.
"""

import os
import signal
import subprocess
import sys
import threading
import time

from uvis_ac.game_launch import remove_output, track_name
from uvis_ac.paths import bridge_dir

_READY = 'game is on the virtual screen'
_ALREADY = 'already running'
_PLACE_WAIT_S = 120.0
_CONNECT_WAIT_S = 20.0
_UNIT = 'uvis-ac-game'


class _Jobs:
    """Pids the signal handler is allowed to signal. 0 means none."""

    def __init__(self):
        self.bridge = 0
        self.game = 0
        self.extra = []
        self.started_game = False
        self.user_stop = False


_jobs = _Jobs()


def is_bridge_argv(argv):
    """True for the AC bridge launch or its shared-memory copier, not a mention in a script."""
    if not argv:
        return False
    for arg in argv:
        name = arg.replace('\\', '/').rsplit('/', 1)[-1]
        if name in ('ac_shm_bridge.sh', 'ac_shm_copy.exe'):
            return True
    return 'ac.launch.py' in argv and ('launch' in argv or any(
        part == 'ros2' or part.endswith('/ros2') for part in argv))


def is_acs_argv(argv):
    """True when this process is acs.exe. A shell that only mentions the path is not."""
    if not argv:
        return False
    last = argv[-1]
    if any(char.isspace() for char in last):
        return False
    name = last.replace('\\', '/').rsplit('/', 1)[-1]
    return name == 'acs.exe'


def stop_plan(bridge_pgid, extra, game_pgid, mine):
    """Bridge groups first, then the game group. Never this process group."""
    bridges = []
    for group in (bridge_pgid, *tuple(extra)):
        if not group or group == mine or group == game_pgid:
            continue
        if group not in bridges:
            bridges.append(group)
    game = game_pgid if game_pgid and game_pgid != mine else 0
    return bridges, game


def should_stop_game_unit(own_launcher_live, user_stop, acs_running):
    """The host unit is stopped here only when no game launcher is left to do it."""
    if own_launcher_live:
        return False
    return bool(user_stop) and bool(acs_running)


def note_game_line(line, state):
    if _READY in line:
        state['placed'] = True
    if _ALREADY in line:
        state['already'] = True


def _popen_kwargs(env):
    return {
        'stdout': subprocess.PIPE,
        'stderr': subprocess.STDOUT,
        'stdin': subprocess.DEVNULL,
        'start_new_session': True,
        'text': True,
        'bufsize': 1,
        'env': env,
    }


def _argv(pid):
    try:
        raw = open('/proc/%s/cmdline' % pid, 'rb').read().split(b'\0')
    except OSError:
        return []
    return [part.decode('utf-8', 'replace') for part in raw if part]


def _processes():
    found = []
    proc = '/proc'
    try:
        names = os.listdir(proc)
    except OSError:
        return found
    for name in names:
        if not name.isdigit():
            continue
        argv = _argv(name)
        if argv:
            found.append((int(name), argv))
    return found


def _bridge_groups():
    """Process groups of a bridge that is not the game and not this launch."""
    mine = os.getpgrp()
    groups = set()
    owners = {}
    for pid, argv in _processes():
        try:
            group = os.getpgid(pid)
        except ProcessLookupError:
            continue
        owners.setdefault(group, []).append(argv)
        if group == mine or not is_bridge_argv(argv):
            continue
        groups.add(group)
    safe = set()
    for group in groups:
        if any(is_acs_argv(argv) for argv in owners.get(group, ())):
            continue
        safe.add(group)
    return safe


def _live_bridges(groups):
    """Groups that still contain a bridge and still do not contain the game."""
    if not groups:
        return []
    wanted = set(groups)
    mine = os.getpgrp()
    owners = {}
    for pid, argv in _processes():
        try:
            group = os.getpgid(pid)
        except ProcessLookupError:
            continue
        if group in wanted and group != mine:
            owners.setdefault(group, []).append(argv)
    alive = []
    for group, argvs in owners.items():
        if any(is_acs_argv(argv) for argv in argvs):
            continue
        if any(is_bridge_argv(argv) for argv in argvs):
            alive.append(group)
    return alive


def _acs_running():
    return any(is_acs_argv(argv) for _pid, argv in _processes())


def _signal_group(group, sig):
    if not group or group == os.getpgrp():
        return False
    try:
        os.killpg(group, sig)
    except (ProcessLookupError, PermissionError, OSError):
        return False
    return True


def _remember(groups):
    for group in groups:
        if group and group not in _jobs.extra:
            _jobs.extra.append(group)


def _on_signal(_signum, _frame):
    # killpg only. The panel SIGKILLs this process two seconds later, so the
    # children have to be signaled here, before finally gets a turn.
    _jobs.user_stop = True
    game_pgid = _jobs.game if _jobs.started_game else 0
    bridges, game = stop_plan(_jobs.bridge, tuple(_jobs.extra), game_pgid, os.getpgrp())
    for group in bridges:
        _signal_group(group, signal.SIGINT)
    if game:
        _signal_group(game, signal.SIGINT)


def _stop_leftover(groups):
    """Drop a bridge that is holding the wineserver before the game starts."""
    bridges, _game = stop_plan(0, groups, 0, os.getpgrp())
    if not bridges:
        return False
    _remember(bridges)
    for group in bridges:
        _signal_group(group, signal.SIGINT)
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline and not _jobs.user_stop:
        if not _live_bridges(bridges):
            break
        time.sleep(0.2)
    if not _jobs.user_stop:
        for group in _live_bridges(bridges):
            _signal_group(group, signal.SIGKILL)
    _jobs.extra = [group for group in _jobs.extra if group not in bridges]
    return True


def _stop_game_unit():
    try:
        subprocess.run(
            ['systemctl', '--user', 'stop', '--no-block', _UNIT],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=1,
        )
    except (OSError, subprocess.SubprocessError):
        pass
    try:
        remove_output()
    except (OSError, RuntimeError):
        return


def _physics_mtime():
    path = bridge_dir() / 'physics.bin'
    try:
        return path.stat().st_mtime
    except OSError:
        return None


def _pump(proc, prefix, state):
    if proc.stdout is None:
        return
    for line in proc.stdout:
        text = line.rstrip('\n')
        if prefix:
            print('%s: %s' % (prefix, text), flush=True)
        else:
            print(text, flush=True)
        if state is not None:
            note_game_line(text, state)


def _spawn(args, env):
    return subprocess.Popen(args, **_popen_kwargs(env))


def _child_env():
    env = os.environ.copy()
    env['PYTHONUNBUFFERED'] = '1'
    return env


def _start_bridge():
    print('Starting the AC bridge.', flush=True)
    try:
        proc = _spawn(['ros2', 'launch', 'uvis_ac', 'ac.launch.py'], _child_env())
    except OSError as exc:
        print('Could not start the AC bridge: %s' % exc, flush=True)
        return None
    _jobs.bridge = proc.pid
    threading.Thread(target=_pump, args=(proc, 'AC bridge', None), daemon=True).start()
    return proc


def _watch_connection(baseline, deadline):
    """Report once when physics.bin starts changing, which means the copier sees the game."""
    while not _jobs.user_stop and time.monotonic() < deadline:
        mtime = _physics_mtime()
        if mtime is not None and (baseline is None or mtime > baseline + 0.05):
            print('AC bridge is reading the game.', flush=True)
            return True
        time.sleep(0.4)
    if not _jobs.user_stop:
        print('AC bridge has not seen the game shared memory yet.', flush=True)
    return False


def _wait_ready(proc, state):
    deadline = time.monotonic() + _PLACE_WAIT_S
    while not _jobs.user_stop:
        if state['placed'] or state['already']:
            return True
        if proc.poll() is not None:
            return False
        if time.monotonic() >= deadline and _acs_running():
            print('The game is running. Starting the AC bridge now.', flush=True)
            state['placed'] = True
            return True
        time.sleep(0.2)
    return False


def _shutdown(game):
    mine = os.getpgrp()
    extra = list(_jobs.extra)
    for group in _bridge_groups():
        if group not in extra:
            extra.append(group)
    game_pgid = _jobs.game if _jobs.started_game else 0
    bridges, game_group = stop_plan(_jobs.bridge, extra, game_pgid, mine)
    for group in bridges:
        _signal_group(group, signal.SIGINT)
    own_live = (
        _jobs.started_game
        and game is not None
        and game.poll() is None
        and game_group)
    if own_live:
        _signal_group(game_group, signal.SIGINT)
    if not _jobs.user_stop:
        deadline = time.monotonic() + 0.4
        while time.monotonic() < deadline and _live_bridges(bridges):
            try:
                time.sleep(0.1)
            except InterruptedError:
                break
    for group in _live_bridges(bridges):
        _signal_group(group, signal.SIGKILL)
    if should_stop_game_unit(own_live, _jobs.user_stop, _acs_running()):
        _stop_game_unit()


def _install_signals():
    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)


def main() -> None:
    track = track_name(sys.argv[1] if len(sys.argv) > 1 else '')
    _install_signals()
    game = None
    bridge = None
    try:
        game_up = _acs_running()
        bridges = _bridge_groups()
        if game_up and bridges:
            _remember(bridges)
            print(
                'The game and the AC bridge are already running. '
                'Stop to close them.',
                flush=True)
        else:
            if not game_up and bridges and _stop_leftover(bridges):
                print('Stopped an AC bridge that was already running.', flush=True)
            if _jobs.user_stop:
                return
            if _acs_running():
                print(
                    'The game is already open. Starting the AC bridge onto it. '
                    'Stop and start Game to load %s.' % track,
                    flush=True)
            else:
                print('Starting the game on %s.' % track, flush=True)
                try:
                    game = _spawn(
                        [sys.executable, '-m', 'uvis_ac.game_launch', track],
                        _child_env())
                except OSError as exc:
                    print('Could not start the game: %s' % exc, flush=True)
                    return
                _jobs.game = game.pid
                _jobs.started_game = True
                state = {'placed': False, 'already': False}
                threading.Thread(
                    target=_pump, args=(game, None, state), daemon=True).start()
                ready = _wait_ready(game, state)
                if _jobs.user_stop:
                    return
                if state['already'] or not ready:
                    if state['already'] and _acs_running():
                        _jobs.started_game = False
                        _jobs.game = 0
                        print(
                            'The game is already open. Starting the AC bridge onto it. '
                            'Stop and start Game to load %s.' % track,
                            flush=True)
                    else:
                        print('Game exited before the bridge could start.', flush=True)
                        return
            if _jobs.user_stop:
                return
            existing = _bridge_groups()
            if existing:
                _remember(existing)
                print('The AC bridge is already running.', flush=True)
            else:
                baseline = _physics_mtime()
                bridge = _start_bridge()
                if bridge is not None:
                    _watch_connection(baseline, time.monotonic() + _CONNECT_WAIT_S)

        misses = 0
        while not _jobs.user_stop:
            if _jobs.started_game and game is not None and game.poll() is not None:
                print('Game exited %s. Stopping the AC bridge.' % game.returncode, flush=True)
                break
            if not _jobs.started_game:
                if _acs_running():
                    misses = 0
                else:
                    misses += 1
                    if misses >= 3:
                        print('Game is gone. Stopping the AC bridge.', flush=True)
                        break
            if bridge is not None and bridge.poll() is not None:
                print(
                    'AC bridge exited %s. Stop and start Game to connect them again.'
                    % bridge.returncode,
                    flush=True)
                bridge = None
                _jobs.bridge = 0
                while not _jobs.user_stop:
                    if _jobs.started_game:
                        if game is None or game.poll() is not None:
                            break
                    elif not _acs_running():
                        break
                    time.sleep(0.4)
                break
            time.sleep(0.4)
    finally:
        _shutdown(game)


def self_check():
    assert is_bridge_argv(['ros2', 'launch', 'uvis_ac', 'ac.launch.py'])
    assert is_bridge_argv([
        '/usr/bin/python3', '/opt/ros/jazzy/bin/ros2', 'launch', 'uvis_ac', 'ac.launch.py'])
    assert is_bridge_argv(['bash', '/opt/uvis/ac_shm_bridge.sh'])
    assert is_bridge_argv(['proton', 'run', r'c:\uvis\ac_shm_copy.exe'])
    assert not is_bridge_argv(['bash', '-c', 'echo ac.launch.py'])
    assert not is_bridge_argv(['python3', '-m', 'uvis_ac.sim_launch', 'small_track'])
    assert not is_bridge_argv(['/usr/bin/steam'])
    assert not is_bridge_argv(['/opt/ros/jazzy/lib/uvis_perception/cone_visualizer'])
    assert is_acs_argv(['proton', 'run', '/home/mtu/steamapps/common/assettocorsa/acs.exe'])
    assert not is_acs_argv(['bash', '-c', 'echo /acs.exe'])
    bridges, game = stop_plan(10, [10, 11, 99], 12, 99)
    assert bridges == [10, 11]
    assert game == 12
    bridges, game = stop_plan(0, [], 5, 5)
    assert bridges == [] and game == 0
    assert _popen_kwargs(None)['start_new_session'] is True
    assert _signal_group(os.getpgrp(), signal.SIGKILL) is False
    assert _signal_group(0, signal.SIGKILL) is False
    assert not should_stop_game_unit(True, True, True)
    assert should_stop_game_unit(False, True, True)
    assert not should_stop_game_unit(False, False, True)
    assert not should_stop_game_unit(False, True, False)
    state = {'placed': False, 'already': False}
    note_game_line('game is on the virtual screen', state)
    assert state['placed'] and not state['already']
    note_game_line('Assetto Corsa is already running. Stop that game before starting another.', state)
    assert state['already']
    print('sim launch ok')


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '--check':
        self_check()
    else:
        main()
