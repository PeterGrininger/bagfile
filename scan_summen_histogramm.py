import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def percentile(sorted_values: list[float], fraction: float) -> float:
    position = (len(sorted_values) - 1) * fraction
    lower_index = math.floor(position)
    upper_index = math.ceil(position)
    weight = position - lower_index
    return (
        sorted_values[lower_index] * (1 - weight)
        + sorted_values[upper_index] * weight
    )


def choose_bin_count(values: list[float]) -> int:
    sorted_values = sorted(values)
    value_range = sorted_values[-1] - sorted_values[0]
    if value_range == 0:
        return 1

    interquartile_range = percentile(sorted_values, 0.75) - percentile(
        sorted_values, 0.25
    )
    if interquartile_range > 0:
        bin_width = 2 * interquartile_range / len(values) ** (1 / 3)
        bin_count = math.ceil(value_range / bin_width)
    else:
        bin_count = math.ceil(math.log2(len(values)) + 1)

    return max(1, min(bin_count, 100))


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
    scan_sums = []
    valid_counts = []
    scan_number = 0

    while reader.has_next():
        topic, serialized_message, _ = reader.read_next()
        if topic != topic_name:
            continue

        scan = deserialize_message(serialized_message, message_type)
        scan_number += 1
        valid_ranges = [
            distance
            for distance in scan.ranges
            if math.isfinite(distance)
            and scan.range_min <= distance <= scan.range_max
        ]
        valid_counts.append(len(valid_ranges))
        if valid_ranges:
            scan_sums.append((scan_number, len(valid_ranges), sum(valid_ranges)))

    if scan_number == 0:
        raise ValueError(f'Keine Nachrichten auf {topic_name!r} gefunden.')
    if not scan_sums:
        raise ValueError(f'Keine gueltigen Entfernungswerte auf {topic_name!r}.')

    sums = [item[2] for item in scan_sums]
    sorted_sums = sorted(sums)
    bin_count = choose_bin_count(sums)
    bin_width = (sorted_sums[-1] - sorted_sums[0]) / bin_count

    csv_path = Path.cwd() / 'scan_summen.csv'
    with csv_path.open('w', newline='', encoding='utf-8') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                'scan_number',
                'valid_measurement_count',
                'sum_distance_m',
                'sum_distance_mm',
            ]
        )
        writer.writerows(
            [scan_number, count, distance_sum, distance_sum * 1000]
            for scan_number, count, distance_sum in scan_sums
        )

    figure, axis = plt.subplots(figsize=(11, 6))
    sums_mm = [distance_sum * 1000 for distance_sum in sums]
    axis.hist(sums_mm, bins=bin_count, edgecolor='white', linewidth=0.7)
    axis.set_title('Verteilung der Entfernungssummen pro Laserscan')
    axis.set_xlabel('Summe gueltiger Entfernungswerte pro Scan [mm]')
    axis.set_ylabel('Anzahl Scans')
    axis.grid(axis='y', alpha=0.3)
    figure.tight_layout()

    plot_path = Path.cwd() / 'scan_summen_histogramm.png'
    figure.savefig(plot_path, dpi=150)

    mean_sum = sum(sums) / len(sums)
    variance = sum((value - mean_sum) ** 2 for value in sums) / len(sums)
    standard_deviation = math.sqrt(variance)
    valid_scan_count = len(scan_sums)
    mean_valid_count = sum(valid_counts) / len(valid_counts)

    print(f'Bag: {bag_path}')
    print(f'Topic: {topic_name} ({topic_types[topic_name]})')
    print(f'Anzahl Scans: {scan_number}')
    print(f'Scans mit gueltigen Messwerten: {valid_scan_count}')
    print(f'Mittlere Anzahl gueltiger Werte pro Scan: {mean_valid_count:.2f}')
    print(
        f'Mittelwert der Entfernungssummen: {mean_sum * 1000:.1f} mm '
        f'({mean_sum:.3f} m)'
    )
    median_sum = percentile(sorted_sums, 0.5)
    print(
        f'Median der Entfernungssummen: {median_sum * 1000:.1f} mm '
        f'({median_sum:.3f} m)'
    )
    print(
        f'Standardabweichung der Summen: {standard_deviation * 1000:.1f} mm '
        f'({standard_deviation:.3f} m)'
    )
    print(
        f'Bereich der Summen: {sorted_sums[0] * 1000:.1f} bis '
        f'{sorted_sums[-1] * 1000:.1f} mm '
        f'({sorted_sums[0]:.3f} bis {sorted_sums[-1]:.3f} m)'
    )
    print(
        f'Histogramm: {bin_count} Klassen, Klassenbreite etwa '
        f'{bin_width * 1000:.1f} mm ({bin_width:.3f} m, '
        'Freedman-Diaconis-Regel)'
    )
    print(f'CSV gespeichert: {csv_path}')
    print(f'Histogramm gespeichert: {plot_path}')
    print('Das interaktive Fenster bleibt offen, bis es geschlossen wird.')
    plt.show()


def main() -> None:
    default_bag = Path(__file__).resolve().parent / 'subset_5minSTILL'
    parser = argparse.ArgumentParser(
        description=(
            'Summiert die gueltigen Laserentfernungen pro Scan und erstellt '
            'ein Histogramm ueber die Summen.'
        )
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
