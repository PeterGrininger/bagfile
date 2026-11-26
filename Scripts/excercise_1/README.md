# excercise_1 – Odometrie und LiDAR (ARS3IL, Übung 1)

ROS-2-Paket (Jazzy, TurtleBot3 Burger) mit den Nodes für Übung 1. Es liegt im Repo unter
`Scripts/excercise_1/`. **Alle Befehle werden im Repo-Ordner `bagfile` ausgeführt**, weil
die Pfade relativ dazu angegeben sind: Bags in `Messaufnahmen/`, CSVs nach `Results/`, Bilder
nach `Images/`.

## Bauen

```bash
cd <pfad>/bagfile
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install      # findet das Paket unter Scripts/excercise_1 automatisch
source install/setup.bash           # build/, install/, log/ sind in .gitignore
```

Alternativ in einen eigenen Workspace verlinken:
`ln -s "$(pwd)/Scripts/excercise_1" ~/ars26_ws/src/excercise_1`

Benötigte apt-Pakete: `ros-jazzy-desktop ros-jazzy-rosbag2-storage-mcap
ros-jazzy-turtlebot3-msgs`, optional `ros-jazzy-turtlebot3-description`
(3D-Modell des Roboters in RViz).

## Nodes

| Node | Aufgabe | Ein / Aus |
|---|---|---|
| `my_odometry` | eigene Odometrie aus den Encoder-Ticks (Odometrie 2) | `/sensor_state`, `/imu` → `/my_odom`, `/my_odom_imu`, `*/path`, TF |
| `trajectory_logger` | schreibt die Odometrie-Varianten in **eine** CSV (Odometrie 3) | `/odom`, `/my_odom`, `/my_odom_imu` → `trajectories.csv` |
| `scan_statistics` | Mittelwert, Varianz, σ je Messrichtung (Laser 4) | `/scan` → `scan_statistics.csv` |
| `scan_sum` | Summe aller Distanzen je Scan (Laser 5) | `/scan` → `scan_sums.csv` |
| `tf_filter` | entfernt das defekte `odom→base_footprint` aus dem Bag-TF | `/tf_bag` → `/tf` |

Die Formeln stehen ohne ROS-Abhängigkeit in `excercise_1/odometry_core.py` und
`excercise_1/laser_core.py`.

## Bags abspielen mit RViz

```bash
# Laserscanner (Roboter steht) – Screenshot für Laser 3, schreibt Results/scan_statistics.csv
# und Results/scan_sums.csv
ros2 launch excercise_1 laser_replay.launch.py \
    bags:="Messaufnahmen/rosbag2_2026_11_12-10_55_22 Messaufnahmen/rosbag2_2026_11_12-10_56_01"

# Fahrt – eigene Odometrie in RViz, schreibt die Trajektorien-CSV
ros2 launch excercise_1 odometry_replay.launch.py bag:=Messaufnahmen/rosbag2_2026_11_12-11_10_48 \
    output_file:=Results/trajectories_11_10_48.csv
```

Optionen: `rviz:=false` (ohne GUI), `rate:=4` (schneller), `analyze:=false`,
`loop:=true` (nur ohne `analyze`), `output_dir:=...` (Laser, Standard `Results`),
`output_file:=...` (Odometrie, Standard `Results/trajectories.csv`). Nach dem Ende des Bags
mit **Strg+C** beenden, erst dann werden die Dateien endgültig geschrieben.

Ohne Launch-Datei geht es auch manuell, z. B. `ros2 run excercise_1 scan_statistics`
in einem Terminal und `ros2 bag play Messaufnahmen/<bag>` in einem zweiten.

RViz-Configs: `rviz/laser.rviz` (Fixed Frame `base_footprint`) und
`rviz/odometry.rviz` (Fixed Frame `odom`, grün = Encoder, orange = Encoder + IMU,
Laser mit 20 s Decay → man sieht, wie gut die Odometrie die Wände deckungsgleich hält).
Wenn RViz unter WSL schwarz bleibt oder abstürzt: `export LIBGL_ALWAYS_SOFTWARE=1`.

Die Plot- und Auswerteskripte liegen eine Ebene höher in `Scripts/`, siehe `README.md` im
Repo-Ordner.

## Hinweis zu den Aufnahmen vom 12.11.

Das `/odom` des `turtlebot3_node` ist in **allen** Bags defekt (x ≈ −6.7·10²⁴⁸ m,
Gierwinkel konstant, Spitzen von −160 rad/s in `twist.angular.z`), ebenso der TF
`odom→base_footprint` und dadurch auch Cartographers `map→odom`. Deshalb blenden die
Launch-Dateien das Bag-TF über `tf_filter` aus.
