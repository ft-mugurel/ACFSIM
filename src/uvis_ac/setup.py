import subprocess
from glob import glob
from os.path import join
from pathlib import Path

from setuptools import setup

def _compile_cast():
    root = Path(__file__).resolve().parent
    src = root / 'uvis_ac' / 'lidar_cast.cpp'
    out = root / 'uvis_ac' / '_lidar_cast.so'
    if not src.is_file():
        return
    if out.is_file() and out.stat().st_mtime >= src.stat().st_mtime:
        return
    subprocess.check_call([
        'g++', '-O3', '-march=native', '-fPIC', '-shared', '-std=c++17',
        '-o', str(out), str(src),
    ])


_compile_cast()

package_name = 'uvis_ac'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (join('share', package_name, 'config'), glob('config/*')),
        (join('share', package_name, 'launch'), glob('launch/*.py')),
        (join('share', package_name, 'rviz'), glob('rviz/*')),
        (join('share', package_name, 'scripts'), ['scripts/ac_shm_bridge.sh']),
        (join('share', package_name, 'tools'), ['tools/ac_shm_copy.exe']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'ac_telemetry = uvis_ac.telemetry_node:main',
            'ac_lidar = uvis_ac.lidar_node:main',
            'ac_drive = uvis_ac.drive_node:main',
        ],
    },
    maintainer='UVIS',
    maintainer_email='uvis@example.com',
    description='Assetto Corsa bridge pieces that do not need the game running.',
    license='MIT',
)
