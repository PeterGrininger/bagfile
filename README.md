# ARS LABOR: ROS-2-Bags, Odometrie und Laserscanner

## Ordnerstruktur

| Ordner | Inhalt |
|---|---|
| `Messaufnahmen/` | alle aufgenommenen Bags (siehe Tabelle unten) |
| `Scripts/` | alle Python-Skripte fuer Auswertung und Plots |
| `Scripts/excercise_1/` | ROS-2-Paket mit den Nodes und Launch-Dateien fuer Uebung 1 (eigene README) |
| `Results/` | Ergebnisse als CSV sowie `Auswertung_Laserscanner_4_5.md` |
| `Images/` | alle Plots, RViz-Screenshots und das Foto des Testfelds |
| `Exercise_1.pdf`, `Exercise_1_Zusatz.pdf` | Angabe |

Die Skripte bestimmen ihre Pfade relativ zum Repo-Ordner. Bags koennen deshalb nur mit
Namen angegeben werden (z. B. `subset_5minSTILL` statt `Messaufnahmen/subset_5minSTILL`).
CSV-Dateien landen in `Results/`, Bilder in `Images/`, egal aus welchem Ordner man das Skript
startet.

## Voraussetzungen

- ROS 2 Jazzy ist installiert.
- Das Workspace wurde gebaut und `install/setup.bash` ist vorhanden (siehe unten).
- Fuer `scan_statistik.py` wird eine grafische Linux-Sitzung benoetigt, damit sich das Plot-Fenster oeffnen kann.

Alle Befehle in diesem Dokument werden im Ordner `bagfile` ausgefuehrt:

```bash
cd <pfad>/bagfile
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install   # nur einmal bzw. nach Aenderungen am Paket
source install/setup.bash
```

`colcon build` findet das Paket unter `Scripts/excercise_1` automatisch. Die Ordner
`build/`, `install/` und `log/` sind in `.gitignore` eingetragen. Wenn die Shell neu
gestartet wird, muss `source install/setup.bash` erneut ausgefuehrt werden.

## Vorhandene Bags (`Messaufnahmen/`)

| Bag | Dauer | Inhalt |
|---|---|---|
| `rosbag2_2026_11_12-09_56_59` | 70 s | Fahrt (Quadrat + Diagonale), enthaelt 1 verspaetete `/sensor_state`-Nachricht |
| `rosbag2_2026_11_12-10_55_22`, `rosbag2_2026_11_12-10_56_01` | je 34 s | Roboter steht, Laser-Statistik |
| `rosbag2_2026_11_12-11_04_08`, `rosbag2_2026_11_12-11_10_48` | 76 / 66 s | Fahrt (Quadrat + Diagonale) |
| `rosbag2_2026_11_19-12_28_12` | 123 s | Fahrt, Odometrie-Vergleich (`odometry_calculations.py`) |
| `subset` | 103 s | Ausschnitt mit `/odom`, `/imu`, `/cmd_vel` (12.11.) |
| `subset_5minSTILL` | 310 s | nur `/scan`, Roboter steht (Laser 4/5, Foto `Images/Turtlebot3_TopView.jpeg`) |
| `subset_5min_90d` | 65 s | nur `/scan`, Roboter steht, um ca. 90° gedreht |

Hinweis: In den Bags vom 12.11. ist das `/odom` des `turtlebot3_node` defekt
(x ca. -6.7e248 m). Die Launch-Dateien blenden deshalb das Bag-TF ueber `tf_filter` aus,
Details in `Scripts/excercise_1/README.md`.

Bag-Informationen anzeigen:

```bash
ros2 bag info Messaufnahmen/subset_5minSTILL
ros2 bag info Messaufnahmen/subset_5min_90d
```

## Bag abspielen

Eine Bag normal abspielen:

```bash
ros2 bag play Messaufnahmen/subset_5minSTILL
```

Die zweite Bag abspielen:

```bash
ros2 bag play Messaufnahmen/subset_5min_90d
```

Schneller abspielen, zum Beispiel mit 20-facher Geschwindigkeit:

```bash
ros2 bag play Messaufnahmen/subset_5minSTILL --rate 20
```

Mit `Ctrl+C` kann die Wiedergabe abgebrochen werden. Ein bereits laufender Player sollte beendet sein, bevor dieselbe Bag fuer eine weitere Auswertung gestartet wird.

---

# Exercise 1 – Odometrie

| Schritt | Befehl | Ergebnis |
|---|---|---|
| Odometrie aus Encoder-Ticks, Approximation vs. exakte Gleichung vs. `/odom` | `python3 Scripts/odometry_calculations.py` (Standard-Bag `rosbag2_2026_11_19-12_28_12`) | `Results/odometrie_trajektorien.csv`, `Images/odometrie_vergleich_plot.png` |
| Eigene Odometrie als ROS-Node, Bag mit RViz abspielen | `ros2 launch excercise_1 odometry_replay.launch.py bag:=Messaufnahmen/rosbag2_2026_11_12-11_10_48 output_file:=Results/trajectories_11_10_48.csv` | `Results/trajectories_11_10_48.csv` |
| Trajektorien plotten, signifikante Punkte | `python3 Scripts/plot_trajectories.py Results/trajectories_11_10_48.csv` | `Images/trajectories_11_10_48_plot.png`, `Results/trajectories_11_10_48_points.csv` |

`odometry_calculations.py` rechnet zuerst die Aufgaben aus der Vorlesung (Teil 1–4) und
danach die Odometrie aus den echten Encoder-Daten. Ein anderer Bag kann als Argument
angegeben werden (`python3 Scripts/odometry_calculations.py rosbag2_2026_11_12-09_56_59`).
Vorhandene Ergebnisse: `trajectories_09_56_59`, `trajectories_11_04_08`,
`trajectories_11_10_48` (jeweils CSV, Punkte-CSV und Plot). `trajectories.csv` ist ein
weiterer Lauf mit `rosbag2_2026_11_12-11_10_48`. `Images/Odometrie_19_11_12_28.png` ist ein
Bildschirmfoto des Odometrie-Vergleichsplots (`rosbag2_2026_11_19-12_28_12`).

# Exercise 1 – Laserscanner

Schriftliche Auswertung zu Punkt 4 und 5: **`Results/Auswertung_Laserscanner_4_5.md`**.

| Schritt | Befehl | Ergebnis |
|---|---|---|
| Statistik und Summen direkt aus dem Bag (gleiche Berechnung wie die Nodes) | `python3 Scripts/analyze_scan_bag.py subset_5minSTILL` | `Results/5minSTILL_scan_statistics.csv`, `Results/5minSTILL_scan_sums.csv` |
| Plots Statistik je Messrichtung (Punkt 4) | `python3 Scripts/plot_scan_statistics.py Results/5minSTILL_scan_statistics.csv` | `Images/5minSTILL_scan_statistics_{plot,polar,std_vs_dist}.png` |
| Histogramm der Distanzsummen (Punkt 5) | `python3 Scripts/plot_scan_histogram.py Results/5minSTILL_scan_sums.csv` | `Images/5minSTILL_scan_sums_histogram.png` |
| Vergleich Ausgangslage / um 90° gedreht | `python3 Scripts/compare_scan_bags.py subset_5minSTILL subset_5min_90d --labels "Ausgangslage (5minSTILL)" "um 90° gedreht (5min_90d)"` | `Images/vergleich_5min_{ausrichtung,sigma_intensitaet}.png` |
| x/y-Darstellung und Ueberlagerung mit dem Foto | `python3 Scripts/plot_scan_overlay.py subset_5minSTILL subset_5min_90d` | `Images/scan_xy_{sensor,foto,foto_sigma}.png` |
| Dasselbe als ROS-Nodes mit RViz (Bags vom 12.11.) | `ros2 launch excercise_1 laser_replay.launch.py bags:="Messaufnahmen/rosbag2_2026_11_12-10_55_22 Messaufnahmen/rosbag2_2026_11_12-10_56_01"` | `Results/scan_statistics.csv`, `Results/scan_sums.csv` |

Fuer `subset_5min_90d` gelten dieselben Befehle mit `subset_5min_90d` bzw. Praefix
`5min_90d_`. Das Histogramm-Skript waehlt die Klassenbreite standardmaessig nach
Freedman-Diaconis (`--bin-width fd|scott|sturges|<m>`). Fuer das Foto ist die Lage des Scans
im Bild voreingestellt, `--fit` passt sie automatisch neu an. RViz-Screenshots:
`Images/RVIZ_Bild5minSTILL.png`, `Images/RVIZ_Bild90degree.png`.

Die folgenden drei Skripte sind eine zweite, unabhaengige Auswertung derselben Bags.

## Skript: Scan-Aufloesung und Messwertzahl

`messwert_aufloesung.py` liest die Bag direkt ein. Eine separate Wiedergabe mit `ros2 bag play` ist nicht erforderlich. Ohne Argument wird `subset_5minSTILL` ausgewertet:

```bash
python3 Scripts/messwert_aufloesung.py
```

Eine andere Bag angeben:

```bash
python3 Scripts/messwert_aufloesung.py subset_5min_90d
```

Optional kann das Topic angegeben werden. Standard ist `/scan`:

```bash
python3 Scripts/messwert_aufloesung.py subset_5min_90d --topic /scan
```

Das Skript gibt die Anzahl der Scans, Messwerte pro Scan und insgesamt, das Field of View sowie die Winkelaufloesung aus.

## Skript: Statistik und interaktiver Plot

`scan_statistik.py` startet die Bag automatisch, berechnet pro Messrichtung Mittelwert, Populationsvarianz und Standardabweichung und oeffnet anschliessend ein interaktives Plot-Fenster. Standardmaessig wird `subset_5minSTILL` mit 20-facher Geschwindigkeit abgespielt:

```bash
python3 Scripts/scan_statistik.py
```

Dafuer muss kein separates `ros2 bag play` laufen. Das Fenster schliessen, um das Skript zu beenden.

Eine andere Bag auswerten:

```bash
python3 Scripts/scan_statistik.py subset_5min_90d
```

Wiedergaberate veraendern:

```bash
python3 Scripts/scan_statistik.py --rate 5
```

Bag und Wiedergaberate zusammen angeben:

```bash
python3 Scripts/scan_statistik.py subset_5min_90d --rate 10
```

Die Wiedergaberate muss groesser als null sein. Das Skript erwartet standardmaessig das Topic `/scan`.

Nach der Auswertung werden gespeichert:

- `Results/scan_statistik.csv`: Werte und Anzahl gueltiger Stichproben je Messrichtung.
- `Images/scan_statistik.png`: statischer Export des Diagramms; zusaetzlich bleibt das interaktive Fenster geoeffnet.

`NaN`/`Inf`-Werte und Distanzen ausserhalb der in der Nachricht angegebenen Reichweite werden bei den Statistiken ausgelassen. Die Varianz wird als Populationsvarianz berechnet.

## Skript: Histogramm der Entfernungssummen

`scan_summen_histogramm.py` liest die Bag direkt ein und bildet fuer jeden `/scan` die Summe aller gueltigen Entfernungsmesswerte. Anschliessend zeigt es ein interaktives Histogramm und speichert zusaetzlich die Einzelsummen:

```bash
python3 Scripts/scan_summen_histogramm.py
```

Standardmaessig wird `subset_5minSTILL` ausgewertet. Eine andere Bag oder ein anderes Topic kann so angegeben werden:

```bash
python3 Scripts/scan_summen_histogramm.py subset_5min_90d
python3 Scripts/scan_summen_histogramm.py subset_5min_90d --topic /scan
```

Die Anzahl der Histogrammklassen wird mit der Freedman-Diaconis-Regel aus der Streuung der Summen bestimmt und auf maximal 100 Klassen begrenzt. Das Programm gibt ausserdem Anzahl der Scans, Mittelwert, Median, Standardabweichung und Wertebereich der Summen aus.

Ausgaben:

- `Results/scan_summen.csv`: Summe und Anzahl gueltiger Entfernungsmessungen pro Scan.
- `Images/scan_summen_histogramm.png`: statischer Export des Histogramms; zusaetzlich bleibt das interaktive Fenster geoeffnet.

Die Histogramm-Achse und die Hauptausgabe zeigen Summen in Millimetern; die Ausgabe nennt den umgerechneten Meterwert in Klammern. Die CSV enthaelt die Summe sowohl in Metern (`sum_distance_m`) als auch in Millimetern (`sum_distance_mm`). ROS `LaserScan` speichert Einzelentfernungen in Metern; die Millimeterwerte sind nur eine Umrechnung zur leichteren Lesbarkeit.

Ungueltige Werte (`NaN`/`Inf`) sowie Entfernungen ausserhalb des im Scan angegebenen Messbereichs werden nicht summiert. Eine Entfernungssumme ist die Summe vieler radialer Messwerte in unterschiedlichen Richtungen und daher keine einzelne Distanz zum Objekt. Sie beschreibt vor allem die Gesamtstruktur der erfassten Umgebung und kann sich auch durch fehlende/ungueltige Strahlen veraendern.

Bei `subset_5minSTILL` wurden 1.542 Scans ausgewertet. Die Summen liegen zwischen etwa 233,8 m und 240,1 m, bei einem Mittelwert von 238,6 m und einer Standardabweichung von 0,77 m. Der Grossteil liegt zwischen etwa 238,5 m und 239,7 m; wenige deutlich kleinere Summen bilden einen linken Auslaeufer. Das kann auf einzelne Scans mit anderer Sicht auf die Umgebung oder mehr ungueltigen Messstrahlen hindeuten. Die Summen allein zeigen nicht, welche Winkel oder Objekte dafuer verantwortlich sind; dafuer ist die richtungsbezogene Statistik aus `scan_statistik.py` hilfreicher.

## Typischer Ablauf

```bash
cd <pfad>/bagfile
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 Scripts/messwert_aufloesung.py
python3 Scripts/scan_statistik.py
python3 Scripts/scan_summen_histogramm.py
```

Das erste Skript liest die Bag direkt und gibt die Scanner-Parameter aus. Das zweite startet die Bag automatisch, sammelt die Statistik und zeigt anschliessend den Plot. Das dritte liest die Bag direkt und zeigt das Histogramm der Entfernungssummen.
