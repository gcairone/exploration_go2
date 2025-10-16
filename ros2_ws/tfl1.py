#!/usr/bin/env python3

import argparse
import subprocess
import math

def main():
    parser = argparse.ArgumentParser(description="Launch a ROS2 static_transform_publisher with configurable parameters")
    parser.add_argument("--x", type=float, default=0.3, help="Translation X")
    parser.add_argument("--y", type=float, default=0.0, help="Translation Y")
    parser.add_argument("--z", type=float, default=0.0, help="Translation Z")
    parser.add_argument("--alpha", type=float, default=20.0, help="Rotation angle around Y axis in degrees")
    parser.add_argument("--frame_id", type=str, default="robot_center", help="Parent frame")
    parser.add_argument("--child_frame_id", type=str, default="utlidar_lidar", help="Child frame")

    args = parser.parse_args()

    # Converti alpha da gradi a radianti
    alpha_rad = math.radians(args.alpha)

    # Calcola il quaternione per rotazione attorno all'asse Y
    qx = 0.0
    qy = math.sin(alpha_rad / 2.0)
    qz = 0.0
    qw = math.cos(alpha_rad / 2.0)

    # Costruisci il comando
    cmd = [
        "ros2", "run", "tf2_ros", "static_transform_publisher",
        str(args.x), str(args.y), str(args.z),
        str(qx), str(qy), str(qz), str(qw),
        args.frame_id, args.child_frame_id
    ]

    print("Launching:", " ".join(cmd))
    subprocess.run(cmd)

if __name__ == "__main__":
    main()
