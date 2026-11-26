import argparse
import csv
import math
import subprocess
import time
from pathlib import Path

import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

# Repo-Ordnerstruktur: Scripts/ (dieses Skript), Messaufnahmen/, Results/, Images/
REPO_DIR = Path(__file__).resolve().parent.parent
BAG_DIR = REPO_DIR / 'Messaufnahmen'
RESULTS_DIR = REPO_DIR / 'Results'
IMAGES_DIR = REPO_DIR / 'Images'


class ScanStatisticsNode(Node):

    def __init__(self):
        super().__init__('scan_statistics_node')
        self.topic_name = '/scan'
        self.subscription = self.create_subscription(
            LaserScan,
            self.topic_name,
            self.callback,
            qos_profile_sensor_data,
        )
        self.scan_count = 0
        self.valid_measurement_count = 0
        self.counts = []
        self.means = []
        self.sum_squared_differences = []
        self.angle_min = None
        self.angle_increment = None
        self.layout_warning_logged = False
        self.results_saved = False
        self.figure = None

    def callback(self, msg: LaserScan) -> None:
        if self.angle_min is None:
            self.angle_min = msg.angle_min
            self.angle_increment = msg.angle_increment
            measurement_count = len(msg.ranges)
            self.counts = [0] * measurement_count
            self.means = [0.0] * measurement_count
            self.sum_squared_differences = [0.0] * measurement_count
        elif (
            len(msg.ranges) != len(self.counts)
            or not math.isclose(msg.angle_min, self.angle_min, abs_tol=1e-6)
            or not math.isclose(
                msg.angle_increment,
                self.angle_increment,
                abs_tol=1e-9,
            )
        ):
            if not self.layout_warning_logged:
                self.get_logger().warning(
                    'Scan-Geometrie hat sich geaendert; abweichende Scans '
                    'werden ignoriert.'
                )
                self.layout_warning_logged = True
            return

        self.scan_count += 1
        for index, distance in enumerate(msg.ranges):
            if (
                not math.isfinite(distance)
                or distance < msg.range_min
                or distance > msg.range_max
            ):
                continue

            self.counts[index] += 1
            count = self.counts[index]
            delta = distance - self.means[index]
            self.means[index] += delta / count
            delta_after_update = distance - self.means[index]
            self.sum_squared_differences[index] += delta * delta_after_update
            self.valid_measurement_count += 1

    def save_results(self) -> None:
        if self.results_saved:
            return
        self.results_saved = True

        if self.scan_count == 0:
            print('Keine LaserScan-Nachrichten empfangen.')
            return

        angles_deg = [
            math.degrees(self.angle_min + index * self.angle_increment)
            for index in range(len(self.counts))
        ]
        variances = [
            self.sum_squared_differences[index] / count
            if count
            else math.nan
            for index, count in enumerate(self.counts)
        ]
        standard_deviations = [
            math.sqrt(variance) if math.isfinite(variance) else math.nan
            for variance in variances
        ]
        means = [
            self.means[index] if count else math.nan
            for index, count in enumerate(self.counts)
        ]

        csv_path = RESULTS_DIR / 'scan_statistik.csv'
        with csv_path.open('w', newline='', encoding='utf-8') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(
                [
                    'angle_deg',
                    'mean_m',
                    'variance_m2',
                    'standard_deviation_m',
                    'valid_sample_count',
                ]
            )
            for index, angle_deg in enumerate(angles_deg):
                writer.writerow(
                    [
                        angle_deg,
                        means[index],
                        variances[index],
                        standard_deviations[index],
                        self.counts[index],
                    ]
                )

        figure, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
        axes[0].plot(angles_deg, means)
        axes[0].set_ylabel('Mittelwert [m]')
        axes[0].set_title('LaserScan-Statistik pro Messrichtung')
        axes[1].plot(angles_deg, variances)
        axes[1].set_ylabel('Varianz [m^2]')
        axes[2].plot(angles_deg, standard_deviations)
        axes[2].set_ylabel('Standardabweichung [m]')
        axes[2].set_xlabel('Winkel [Grad]')
        for axis in axes:
            axis.grid(True, alpha=0.3)
        figure.tight_layout()
        plot_path = IMAGES_DIR / 'scan_statistik.png'
        figure.savefig(plot_path, dpi=150)
        self.figure = figure

        print(f'Topic: {self.topic_name}')
        print(f'Ausgewertete Scans: {self.scan_count}')
        print(f'Gueltige Einzelmessungen: {self.valid_measurement_count}')
        print(
            'Varianz ist als Populationsvarianz berechnet (Summe der '
            'quadrierten Abweichungen / Anzahl gueltiger Werte).'
        )
        print(f'CSV gespeichert: {csv_path}')
        print(f'Plot gespeichert: {plot_path}')


def main(args=None) -> None:
    default_bag = BAG_DIR / 'subset_5minSTILL'
    parser = argparse.ArgumentParser(
        description=(
            'Spielt eine ROS-2-Bag ab, berechnet Scan-Statistiken und '
            'oeffnet anschliessend einen interaktiven Plot.'
        )
    )
    parser.add_argument(
        'bag',
        nargs='?',
        type=Path,
        default=default_bag,
        help=f'Bag-Verzeichnis (Standard: {default_bag})',
    )
    parser.add_argument('--rate', type=float, default=20.0)
    options = parser.parse_args(args)
    if not options.bag.exists() and (BAG_DIR / options.bag).exists():
        options.bag = BAG_DIR / options.bag  # nur der Bag-Name angegeben
    if not options.bag.exists():
        parser.error(f'Bag-Verzeichnis nicht gefunden: {options.bag}')
    if options.rate <= 0:
        parser.error('--rate muss groesser als 0 sein')

    rclpy.init(args=None)
    node = ScanStatisticsNode()
    player = None
    try:
        discovery_deadline = time.monotonic() + 1.0
        while time.monotonic() < discovery_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        player = subprocess.Popen(
            [
                'ros2',
                'bag',
                'play',
                str(options.bag.resolve()),
                '--rate',
                str(options.rate),
            ]
        )
        print(
            f'Spiele {options.bag} mit Rate {options.rate:g}x ab. '
            'Die Auswertung startet automatisch.'
        )
        while player.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.1)

        drain_deadline = time.monotonic() + 0.5
        while time.monotonic() < drain_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

        if player.returncode != 0:
            print(f'ros2 bag play wurde mit Status {player.returncode} beendet.')
    except KeyboardInterrupt:
        print('Auswertung durch Benutzer abgebrochen.')
    finally:
        if player is not None and player.poll() is None:
            player.terminate()
            player.wait()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        node.save_results()

    if node.figure is not None:
        print('Das Plot-Fenster bleibt offen, bis es geschlossen wird.')
        plt.show()


if __name__ == '__main__':
    main()