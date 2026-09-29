"""Print live Assetto Corsa telemetry once the bridge is copying pages."""

import time

from uvis_ac.convert import specific_force, world_to_map
from uvis_ac.pages import graphics_from_bytes, physics_from_bytes
from uvis_ac.paths import bridge_dir

PREFIX = bridge_dir()
PHYSICS = PREFIX / "physics.bin"
GRAPHICS = PREFIX / "graphics.bin"


def main():
    print("Waiting for", PHYSICS)
    while not PHYSICS.exists() or not GRAPHICS.exists():
        time.sleep(0.2)
    last = None
    while True:
        physics = physics_from_bytes(PHYSICS.read_bytes())
        graphics = graphics_from_bytes(GRAPHICS.read_bytes())
        if physics is None or graphics is None:
            time.sleep(0.05)
            continue
        if physics.packet_id != last:
            force = specific_force(physics.acc_g)
            position = world_to_map(graphics.car_coordinates)
            print(
                "speed %.1f km/h  pos (%.1f, %.1f, %.1f)  accel (%.2f, %.2f, %.2f)"
                % (physics.speed_kmh, position[0], position[1], position[2], *force)
            )
            last = physics.packet_id
        time.sleep(0.2)


if __name__ == "__main__":
    main()
