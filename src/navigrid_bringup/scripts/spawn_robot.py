#!/usr/bin/env python3
"""
NaviGrid Robot Spawner
Spawns the robot model into Gazebo Harmonic via native gz service entity factory.
"""

import argparse
import os
import subprocess
import sys
import time


def main():
    parser = argparse.ArgumentParser(description="Spawn robot into Gazebo Harmonic")
    parser.add_argument("-world", "--world", default="navigrid_arena", help="World name")
    parser.add_argument("-name", "--name", default="navigrid_robot", help="Robot entity name")
    parser.add_argument("-x", type=float, default=-11.0, help="Initial X position")
    parser.add_argument("-y", type=float, default=0.0, help="Initial Y position")
    parser.add_argument("-z", type=float, default=0.15, help="Initial Z position")
    parser.add_argument("-Y", "--yaw", type=float, default=0.0, help="Initial Yaw angle")
    parser.add_argument("-xacro", "--xacro", required=True, help="Path to xacro file")
    args, _ = parser.parse_known_args()

    # Ensure environment has loopback discovery set
    env = os.environ.copy()
    env["GZ_IP"] = "127.0.0.1"
    env["GZ_TRANSPORT_DISCOVERY_INTERFACE"] = "lo"

    # Convert xacro -> URDF -> SDF
    tmp_urdf = "/tmp/spawn_navigrid_robot.urdf"
    tmp_sdf = "/tmp/spawn_navigrid_robot.sdf"

    subprocess.run(f"xacro '{args.xacro}' > '{tmp_urdf}'", shell=True, check=True)
    subprocess.run(f"gz sdf -p '{tmp_urdf}' > '{tmp_sdf}'", shell=True, check=True)

    # Prepare gz service request
    req = f'''sdf_filename: "{tmp_sdf}"
name: "{args.name}"
pose {{
  position {{ x: {args.x} y: {args.y} z: {args.z} }}
}}'''

    # Wait for /world/<world>/create service to become available
    service_name = f"/world/{args.world}/create"
    max_retries = 30
    spawned = False

    for attempt in range(max_retries):
        cmd = [
            "gz", "service", "-s", service_name,
            "--reqtype", "gz.msgs.EntityFactory",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "3000",
            "--req", req
        ]
        res = subprocess.run(cmd, env=env, capture_output=True, text=True)
        if "data: true" in res.stdout:
            print(f"[spawn_robot] Successfully spawned {args.name} into {args.world} at ({args.x}, {args.y}, {args.z})")
            spawned = True
            break
        time.sleep(0.5)

    if not spawned:
        print(f"[spawn_robot] Failed to spawn {args.name} after {max_retries} attempts", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
