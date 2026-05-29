#!/usr/bin/env python3
"""
teach_setpoints.py

Walks through color -> zone -> position, records live SICK sensor values,
and saves to yaml immediately after every recording.

Usage:
    python3 teach_setpoints.py --yaml setpoints.yaml
    python3 teach_setpoints.py --yaml setpoints.yaml --color red
    python3 teach_setpoints.py --yaml setpoints.yaml --color red --zone entrance
    python3 teach_setpoints.py --yaml setpoints.yaml --color red --zone entrance --redo closest_straight
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import yaml
import threading
import argparse
import os
import sys


SICK_TOPIC = "sick_data"


class SickListener(Node):
    def __init__(self):
        super().__init__("teach_setpoints_node")
        self.latest = None
        self.create_subscription(
            Float32MultiArray,
            SICK_TOPIC,
            lambda msg: setattr(self, "latest", list(msg.data)),
            10
        )


def load_yaml(path: str) -> dict:
    if not os.path.exists(path):
        print(f"ERROR: {path} not found.")
        sys.exit(1)
    with open(path) as f:
        data = yaml.safe_load(f)
    if data is None:
        print(f"ERROR: {path} is empty.")
        sys.exit(1)
    if "red" not in data or "blue" not in data:
        print("ERROR: yaml must have 'red' and 'blue' top-level keys.")
        sys.exit(1)
    return data


def save_yaml(path: str, data: dict):
    # preserve null as null (not ~)
    def null_representer(dumper, value):
        return dumper.represent_scalar('tag:yaml.org,2002:null', 'null')
    yaml.add_representer(type(None), null_representer)

    with open(path, "w") as f:
        yaml.dump(data, f, default_flow_style=None, sort_keys=False, allow_unicode=True)
    print(f"  [saved -> {path}]")


def wait_for_sick(node: SickListener):
    while node.latest is None:
        pass


def record_position(node: SickListener, name: str) -> list:
    current = node.latest
    current_str = f"{[round(v, 1) for v in current]}" if current else "(no data)"
    print(f"\n  Position : {name}")
    print(f"  Live     : {current_str}")
    input(f"  Move robot to [{name}], press Enter to record...")

    if node.latest is None:
        print("  WARNING: No SICK data. Skipping.")
        return None

    scaled = [round(v * 10.0) for v in node.latest]
    print(f"  Recorded : {scaled}")
    return scaled


def teach_zone(node: SickListener, positions: dict, zone_name: str,
               yaml_path: str, data: dict, redo: str = None):
    """Record all positions within a single zone dict."""

    names = list(positions.keys())
    recorded = [n for n in names if positions[n] is not None]
    missing  = [n for n in names if positions[n] is None]

    print(f"\n  --- Zone: {zone_name} ---")
    print(f"  Recorded : {recorded if recorded else 'none'}")
    print(f"  Missing  : {missing if missing else 'none'}")

    if redo:
        if redo not in positions:
            print(f"  ERROR: '{redo}' not found in zone '{zone_name}'.")
            return
        vals = record_position(node, redo)
        if vals is not None:
            positions[redo] = vals
            save_yaml(yaml_path, data)
        return

    for name in names:
        already = positions[name]

        if already is not None:
            ans = input(
                f"\n  [{name}] already recorded as {already}. "
                f"Re-record? (y/N): "
            ).strip().lower()
            if ans != "y":
                print(f"  Keeping: {already}")
                continue

        vals = record_position(node, name)
        if vals is not None:
            positions[name] = vals
            save_yaml(yaml_path, data)


def teach_color(node: SickListener, data: dict, color: str,
                yaml_path: str, zone_filter: str = None, redo: str = None):

    color_data = data[color]
    zones = list(color_data.keys())

    print(f"\n{'='*50}")
    print(f"  Teaching [{color.upper()}]  —  zones: {zones}")
    print(f"{'='*50}")

    for zone_name in zones:
        if zone_filter and zone_name != zone_filter:
            continue
        teach_zone(node, color_data[zone_name], zone_name,
                   yaml_path, data, redo=redo)


def print_summary(data: dict, color: str):
    print(f"\n--- {color.upper()} summary ---")
    for zone, positions in data[color].items():
        print(f"  [{zone}]")
        for name, val in positions.items():
            status = str(val) if val is not None else "NOT RECORDED"
            print(f"    {name:<30} {status}")


def main():
    parser = argparse.ArgumentParser(description="Teach SICK setpoints.")
    parser.add_argument("--yaml",  default="/home/maomao/ROBOCON_26/2026_ros2_ws/src/martial_tree/include/yaml_files/setpoints.yaml")
    parser.add_argument("--color", choices=["red", "blue", "both"], default="both")
    parser.add_argument("--zone",  default=None, help="Only teach this zone")
    parser.add_argument("--redo",  default=None,
                        help="Re-record one position (requires --color and --zone)")
    args = parser.parse_args()

    if args.redo and (args.color == "both" or args.zone is None):
        print("ERROR: --redo requires both --color (red/blue) and --zone.")
        sys.exit(1)

    data = load_yaml(args.yaml)

    rclpy.init()
    node = SickListener()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    print(f"\nWaiting for SICK data on {SICK_TOPIC}...")
    wait_for_sick(node)
    print(f"SICK data received: {[round(v, 1) for v in node.latest]}")

    colors = ["red", "blue"] if args.color == "both" else [args.color]
    for color in colors:
        teach_color(node, data, color, args.yaml,
                    zone_filter=args.zone, redo=args.redo)
        print_summary(data, color)

    rclpy.shutdown()
    print("\nDone.")


if __name__ == "__main__":
    main()