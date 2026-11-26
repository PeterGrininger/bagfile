import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

# Parameters in cm and cm/s (Rechenaufgabe aus der Vorlesung, Teil 1 bis 4)
b = 25
v_l = 75
v_r = 80
# start pose of the robot
x0, y0, theta0 = 0, 0, 0

# Echter Roboter: TurtleBot3 Burger (fuer die Auswertung der Bag-Daten)
WHEEL_RADIUS = 3.3          # cm
WHEEL_SEPARATION = 16.0     # cm (= b des echten Roboters)
TICKS_PER_REVOLUTION = 4096  # Encoder-Ticks pro Radumdrehung
CM_PER_TICK = 2 * np.pi * WHEEL_RADIUS / TICKS_PER_REVOLUTION


# Calculation functions
# v_l, v_r und b sind Parameter: Standardwerte = Vorlesungsaufgabe,
# bei den echten Daten werden pro Zeitschritt die gemessenen Werte uebergeben.
def calculate_approximation(x, y, theta, delta_t, v_l=v_l, v_r=v_r, b=b):
    v = (v_l + v_r) / 2
    delta_theta = (v_r - v_l) / b
    x_n = x + v * delta_t * np.cos(theta + ((delta_theta * delta_t) / 2))
    y_n = y + v * delta_t * np.sin(theta + ((delta_theta * delta_t) / 2))
    theta_n = theta + delta_theta * delta_t
    return x_n, y_n, theta_n


def calculate_exacte_equation(x, y, theta, delta_t, v_l=v_l, v_r=v_r, b=b):
    v = (v_l + v_r) / 2
    delta_theta = (v_r - v_l) / b
    theta_t = theta + delta_theta * delta_t
    # Schutz vor Division durch 0, falls v_r == v_l (Geradeausfahrt)
    if abs(v_r - v_l) < 1e-6:
        x_t = x + v * delta_t * np.cos(theta)
        y_t = y + v * delta_t * np.sin(theta)
    else:
        x_t = x + (b / 2) * ((v_l + v_r) / (v_r - v_l)) * (np.sin(theta_t) - np.sin(theta))
        y_t = y - (b / 2) * ((v_l + v_r) / (v_r - v_l)) * (np.cos(theta_t) - np.cos(theta))
    return x_t, y_t, theta_t


# --- Ursprüngliche Parts (Teil 1 bis 4) ---
# Part 1
x1, y1, theta_1 = calculate_approximation(x0, y0, theta0, delta_t=1)
print(f"Part1 \nPose after 1s= x(t): {x1} cm, y(t): {y1} cm, theta(t): {theta_1} rad\n")

# Part 2
x2, y2, theta_2 = calculate_approximation(x1, y1, theta_1, delta_t=1)
print(f"Part2 \nPose after 1s using initial conditions from part 1 = x(t): {x2} cm, y(t): {y2} cm, theta(t): {theta_2} rad\n")

# Part 3
x3, y3, theta_3 = calculate_approximation(x0, y0, theta0, delta_t=2)
print(f"Part3 \nPose after 2s= x(t): {x3} cm, y(t): {y3} cm, theta(t): {theta_3} rad\n")

# Part 4
x4, y4, theta_4 = calculate_exacte_equation(x0, y0, theta0, delta_t=2)
print(f"Part4 \nPose after 2s with the exact equation = x(t): {x4} cm, y(t): {y4} cm, theta(t): {theta_4} rad\n")


# --- Odometrie aus den ECHTEN Encoder-Daten (Übungsaufgaben 2, 3 & 5) ---
# Statt konstanter Radgeschwindigkeiten werden v_l und v_r fuer jeden Zeitschritt
# aus den Encoder-Ticks (/sensor_state, ~20 Hz) des aufgenommenen Bags berechnet:
#   v = (Ticks_k - Ticks_k-1) * cm_pro_Tick / (t_k - t_k-1)
# Als dritte Bahn wird die vom turtlebot3_node gemessene Odometrie (/odom) geplottet.
# Aufruf (ROS muss gesourct sein):
#   python3 odometry_calculations.py [pfad/zum/bag]
def read_bag(bag_path):
    """Liest /sensor_state (Encoder-Ticks) und /odom (turtlebot3_node) aus dem Bag."""
    try:
        import rosbag2_py
        from nav_msgs.msg import Odometry
        from rclpy.serialization import deserialize_message
        from turtlebot3_msgs.msg import SensorState
    except ImportError:
        sys.exit('ROS 2 nicht gefunden -> zuerst: source /opt/ros/jazzy/setup.bash')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/sensor_state', '/odom']))
    ticks, odom = [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/sensor_state':
            msg = deserialize_message(data, SensorState)
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            ticks.append((t, msg.left_encoder, msg.right_encoder))
        else:
            msg = deserialize_message(data, Odometry)
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
            odom.append((t, p.x, p.y, yaw))
    return ticks, np.array(odom)


def odom_in_start_frame(odom):
    """/odom [m, rad] relativ zur ersten Pose und in cm -> alle Bahnen starten bei (0, 0, 0)."""
    t, x, y = odom[:, 0], odom[:, 1], odom[:, 2]
    theta = np.unwrap(odom[:, 3])
    dx, dy = x - x[0], y - y[0]
    c, s = np.cos(theta[0]), np.sin(theta[0])
    return t, (c * dx + s * dy) * 100, (-s * dx + c * dy) * 100, theta - theta[0]


# Repo-Ordnerstruktur: Scripts/ (dieses Skript), Messaufnahmen/, Results/, Images/
REPO = Path(__file__).resolve().parent.parent
default_bag = REPO / 'Messaufnahmen' / 'rosbag2_2026_11_19-12_28_12'
bag_path = Path(sys.argv[1]) if len(sys.argv) > 1 else default_bag
if not bag_path.exists():  # nur der Name angegeben -> in Messaufnahmen/ suchen
    bag_path = REPO / 'Messaufnahmen' / bag_path
ticks, odom = read_bag(bag_path)
print(f"Lese '{bag_path.name}': {len(ticks)} Encoder-Messungen, {len(odom)} /odom-Nachrichten")

x_app, y_app, theta_app = x0, y0, theta0
x_ex, y_ex, theta_ex = x0, y0, theta0
t_start = ticks[0][0]
rows = [(0.0, 0.0, 0.0, x_app, y_app, theta_app, x_ex, y_ex, theta_ex)]

t_prev, l_prev, r_prev = ticks[0]
for t, left, right in ticks[1:]:
    delta_t = t - t_prev
    if delta_t <= 0:  # verspaetet angekommene Nachricht (WLAN) -> ueberspringen
        continue
    # gemessene Radgeschwindigkeiten in diesem Zeitschritt [cm/s]
    v_l_k = (left - l_prev) * CM_PER_TICK / delta_t
    v_r_k = (right - r_prev) * CM_PER_TICK / delta_t
    x_app, y_app, theta_app = calculate_approximation(
        x_app, y_app, theta_app, delta_t, v_l_k, v_r_k, WHEEL_SEPARATION)
    x_ex, y_ex, theta_ex = calculate_exacte_equation(
        x_ex, y_ex, theta_ex, delta_t, v_l_k, v_r_k, WHEEL_SEPARATION)
    rows.append((t - t_start, v_l_k, v_r_k, x_app, y_app, theta_app, x_ex, y_ex, theta_ex))
    t_prev, l_prev, r_prev = t, left, right

data = np.array(rows)
time_data = data[:, 0]
diff = np.hypot(data[:, 3] - data[:, 6], data[:, 4] - data[:, 7])

# Gemessene Odometrie (/odom) auf die Zeitpunkte der Encoder-Messungen interpolieren
odom_ok = (len(odom) > 1 and np.all(np.isfinite(odom[:, 1:]))
           and np.max(np.abs(odom[:, 1:3])) < 1e3)  # > 1 km -> Odometrie divergiert
if odom_ok:
    t_o, x_o, y_o, th_o = odom_in_start_frame(odom)
    x_odom = np.interp(time_data + t_start, t_o, x_o)
    y_odom = np.interp(time_data + t_start, t_o, y_o)
    theta_odom = np.interp(time_data + t_start, t_o, th_o)
    diff_odom = np.hypot(data[:, 6] - x_odom, data[:, 7] - y_odom)
else:
    x_odom = y_odom = theta_odom = diff_odom = np.full(len(data), np.nan)
    if len(odom):
        print(f"WARNUNG: /odom ist defekt (x bis {np.max(np.abs(odom[:, 1])):.3g} m, "
              f"theta konstant: {len(np.unique(odom[:, 3])) == 1}) -> wird nicht geplottet")

# 1. Trajektorien in eine CSV-Datei schreiben (für Aufgabe 3)
csv_filename = REPO / 'Results' / 'odometrie_trajektorien.csv'
with open(csv_filename, "w") as f:
    f.write("time,v_l,v_r,x_approx,y_approx,theta_approx,x_exact,y_exact,theta_exact,"
            "x_odom,y_odom,theta_odom\n")
    for row, xo, yo, tho in zip(rows, x_odom, y_odom, theta_odom):
        f.write(",".join(f"{value:.6f}" for value in (*row, xo, yo, tho)) + "\n")
print(f"-> Trajektorien erfolgreich in '{csv_filename}' gespeichert.")
print(f"Endpose Approximation: x={x_app:.2f} cm, y={y_app:.2f} cm, "
      f"theta={np.degrees(theta_app):.1f} deg")
print(f"Endpose exakt:         x={x_ex:.2f} cm, y={y_ex:.2f} cm, "
      f"theta={np.degrees(theta_ex):.1f} deg")
if odom_ok:
    print(f"Endpose /odom:         x={x_odom[-1]:.2f} cm, y={y_odom[-1]:.2f} cm, "
          f"theta={np.degrees(theta_odom[-1]):.1f} deg")
    print(f"Abstand exakt <-> /odom: Ende {diff_odom[-1]:.1f} cm, max {diff_odom.max():.1f} cm, "
          f"Winkel am Ende {np.degrees(theta_ex - theta_odom[-1]):.1f} deg")
print(f"Max. Abweichung Approximation <-> exakt: {diff.max() * 1e4:.4f} um")

# 2. Die Odometrie-Varianten plotten (für Aufgabe 5)
fig = plt.figure(figsize=(16, 9))
gs = fig.add_gridspec(3, 2, width_ratios=[1.3, 1])
ax1 = fig.add_subplot(gs[:, 0])
ax_th = fig.add_subplot(gs[0, 1])
ax_od = fig.add_subplot(gs[1, 1], sharex=ax_th)
ax2 = fig.add_subplot(gs[2, 1], sharex=ax_th)

ax1.plot(data[:, 3], data[:, 4], label="Approximation (Encoder)", linestyle="--", color="blue",
         linewidth=2)
ax1.plot(data[:, 6], data[:, 7], label="Exakte Gleichung (Encoder)", color="red", linewidth=1)
ax1.plot(x_ex, y_ex, "s", color="red")
if odom_ok:
    ax1.plot(x_odom, y_odom, label="/odom gemessen (turtlebot3_node)", color="green",
             linewidth=2)
    ax1.plot(x_odom[-1], y_odom[-1], "s", color="green")
else:
    ax1.text(0.01, 0.01, "/odom defekt -> nicht dargestellt", transform=ax1.transAxes,
             color="green")
ax1.plot(0, 0, "ko", label="Start (■ = Ende)")
ax1.set_title(f"Odometrie-Vergleich ({bag_path.name})")
ax1.set_xlabel("X Position [cm]")
ax1.set_ylabel("Y Position [cm]")
ax1.grid(True)
ax1.legend()
ax1.axis("equal")  # Wichtig für korrekte räumliche Darstellung

ax_th.plot(time_data, np.degrees(data[:, 5]), "--", color="blue", label="Approximation")
ax_th.plot(time_data, np.degrees(data[:, 8]), color="red", linewidth=1, label="Exakt")
ax_th.plot(time_data, np.degrees(theta_odom), color="green", label="/odom")
ax_th.set_title("Orientierung θ")
ax_th.set_ylabel("θ [°]")
ax_th.grid(True)
ax_th.legend(fontsize=8)

ax_od.plot(time_data, diff_odom, color="green")
ax_od.set_title("Abstand exakte Gleichung <-> /odom")
ax_od.set_ylabel("Abweichung [cm]")
ax_od.grid(True)

ax2.plot(time_data, diff * 1e4, color="purple")  # cm -> µm
ax2.set_title("Abstand Approximation <-> exakte Gleichung")
ax2.set_xlabel("t [s]")
ax2.set_ylabel("Abweichung [µm]")
ax2.grid(True)
fig.tight_layout()

plot_filename = REPO / 'Images' / 'odometrie_vergleich_plot.png'
fig.savefig(plot_filename, dpi=300)
print(f"-> Plot erfolgreich als '{plot_filename}' gespeichert.")
plt.show()
