import argparse
import math
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def analyze_bag(bag_path: Path, topic_name: str) -> None:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='mcap'),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        ),
    )

    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    if topic_name not in topic_types:
        available_topics = ', '.join(sorted(topic_types)) or '(keine)'
        raise ValueError(
            f'Topic {topic_name!r} nicht in der Bag gefunden. '
            f'Vorhandene Topics: {available_topics}'
        )

    message_type = get_message(topic_types[topic_name])
    scan_count = 0
    total_measurements = 0
    min_measurements = None
    max_measurements = 0
    first_scan = None

    while reader.has_next():
        topic, serialized_message, _ = reader.read_next()
        if topic != topic_name:
            continue

        scan = deserialize_message(serialized_message, message_type)
        measurement_count = len(scan.ranges)
        scan_count += 1
        total_measurements += measurement_count
        min_measurements = (
            measurement_count
            if min_measurements is None
            else min(min_measurements, measurement_count)
        )
        max_measurements = max(max_measurements, measurement_count)
        if first_scan is None:
            first_scan = scan

    if first_scan is None:
        raise ValueError(f'Keine Nachrichten auf {topic_name!r} gefunden.')

    fov_rad = first_scan.angle_max - first_scan.angle_min
    resolution_rad = first_scan.angle_increment

    print(f'Bag: {bag_path}')
    print(f'Topic: {topic_name} ({topic_types[topic_name]})')
    print(f'Anzahl Scans: {scan_count}')
    print(
        f'Messwerte pro Scan: {min_measurements} bis {max_measurements}'
    )
    print(f'Messwerte insgesamt: {total_measurements}')
    print(
        f'Field of View (FOV): {math.degrees(fov_rad):.2f} Grad '
        f'({fov_rad:.4f} rad)'
    )
    print(
        f'Winkelaufloesung: {math.degrees(resolution_rad):.4f} Grad '
        f'({resolution_rad:.6f} rad)'
    )


def main() -> None:
    default_bag = Path(__file__).resolve().parent / 'subset_5minSTILL'
    parser = argparse.ArgumentParser(
        description='Zaehlt alle LaserScan-Messwerte in einer ROS-2-Bag.'
    )
    parser.add_argument(
        'bag',
        nargs='?',
        type=Path,
        default=default_bag,
        help=f'Bag-Verzeichnis (Standard: {default_bag})',
    )
    parser.add_argument('--topic', default='/scan', help='Topic (Standard: /scan)')
    args = parser.parse_args()

    if not args.bag.exists():
        parser.error(f'Bag-Verzeichnis nicht gefunden: {args.bag}')

    analyze_bag(args.bag, args.topic)


if __name__ == '__main__':
    main()