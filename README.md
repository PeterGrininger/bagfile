# ARS LABOR: ROS-2-Bags und Laserscanner

## Voraussetzungen

- ROS 2 Jazzy ist installiert.
- Das Workspace wurde gebaut und `install/setup.bash` ist vorhanden.
- Fuer `scan_statistik.py` wird eine grafische Linux-Sitzung benoetigt, damit sich das Plot-Fenster oeffnen kann.

Alle Befehle in diesem Dokument werden im Ordner `bagfile` ausgefuehrt:

```bash
cd "$HOME/ARS_LABOR 1/ARS_LABOR/bagfile"
source install/setup.bash
```

Wenn die Shell neu gestartet wird, muss `source install/setup.bash` erneut ausgefuehrt werden.

## Vorhandene Bags

- `subset_5minSTILL/`: kurze Aufnahme, etwa 5 Minuten; Topic `/scan`.
- `subset_5min_90d/`: weitere Aufnahme. Vor Verwendung Topic und Dauer mit `ros2 bag info` kontrollieren.

Bag-Informationen anzeigen:

```bash
ros2 bag info subset_5minSTILL
ros2 bag info subset_5min_90d
```

## Bag abspielen

Eine Bag normal abspielen:

```bash
ros2 bag play subset_5minSTILL
```

Die zweite Bag abspielen:

```bash
ros2 bag play subset_5min_90d
```

Schneller abspielen, zum Beispiel mit 20-facher Geschwindigkeit:

```bash
ros2 bag play subset_5minSTILL --rate 20
```

Mit `Ctrl+C` kann die Wiedergabe abgebrochen werden. Ein bereits laufender Player sollte beendet sein, bevor dieselbe Bag fuer eine weitere Auswertung gestartet wird.

## Skript: Scan-Aufloesung und Messwertzahl

`messwert_aufloesung.py` liest die Bag direkt ein. Eine separate Wiedergabe mit `ros2 bag play` ist nicht erforderlich. Ohne Argument wird `subset_5minSTILL` ausgewertet:

```bash
python3 messwert_aufloesung.py
```

Eine andere Bag angeben:

```bash
python3 messwert_aufloesung.py subset_5min_90d
```

Optional kann das Topic angegeben werden. Standard ist `/scan`:

```bash
python3 messwert_aufloesung.py subset_5min_90d --topic /scan
```

Das Skript gibt die Anzahl der Scans, Messwerte pro Scan und insgesamt, das Field of View sowie die Winkelaufloesung aus.

## Skript: Statistik und interaktiver Plot

`scan_statistik.py` startet die Bag automatisch, berechnet pro Messrichtung Mittelwert, Populationsvarianz und Standardabweichung und oeffnet anschliessend ein interaktives Plot-Fenster. Standardmaessig wird `subset_5minSTILL` mit 20-facher Geschwindigkeit abgespielt:

```bash
python3 scan_statistik.py
```

Dafuer muss kein separates `ros2 bag play` laufen. Das Fenster schliessen, um das Skript zu beenden.

Eine andere Bag auswerten:

```bash
python3 scan_statistik.py subset_5min_90d
```

Wiedergaberate veraendern:

```bash
python3 scan_statistik.py --rate 5
```

Bag und Wiedergaberate zusammen angeben:

```bash
python3 scan_statistik.py subset_5min_90d --rate 10
```

Die Wiedergaberate muss groesser als null sein. Das Skript erwartet standardmaessig das Topic `/scan`.

Nach der Auswertung werden im aktuellen Arbeitsordner gespeichert:

- `scan_statistik.csv`: Werte und Anzahl gueltiger Stichproben je Messrichtung.
- `scan_statistik.png`: statischer Export des Diagramms; zusaetzlich bleibt das interaktive Fenster geoeffnet.

`NaN`/`Inf`-Werte und Distanzen ausserhalb der in der Nachricht angegebenen Reichweite werden bei den Statistiken ausgelassen. Die Varianz wird als Populationsvarianz berechnet.

## Typischer Ablauf

```bash
cd "$HOME/ARS_LABOR 1/ARS_LABOR/bagfile"
source install/setup.bash
python3 messwert_aufloesung.py
python3 scan_statistik.py
```

Der erste Befehl liest die Bag direkt und gibt die Scanner-Parameter aus. Der zweite startet die Bag automatisch, sammelt die Statistik und zeigt anschliessend den Plot.
