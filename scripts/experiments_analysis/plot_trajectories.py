import sqlite3
import sys

try:
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
except ImportError:
    print("Could not import rclpy or nav_msgs. Ensure ROS 2 is sourced.")
    sys.exit(1)

def extract_trajectory(db3_path):
    conn = sqlite3.connect(db3_path)
    c = conn.cursor()
    c.execute("SELECT id, name FROM topics WHERE name='/odometry/filtered'")
    row = c.fetchone()
    if not row:
        return [], []
    topic_id = row[0]
    c.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", (topic_id,))
    
    xs, ys = [], []
    for ts, data in c.fetchall():
        msg = deserialize_message(data, Odometry)
        xs.append(msg.pose.pose.position.x)
        ys.append(msg.pose.pose.position.y)
    return xs, ys

try:
    import matplotlib.pyplot as plt
except ImportError:
    print("Could not import matplotlib.")
    sys.exit(1)

best_bag = "/home/chan/Downloads/experiments/bags/mppi_s1_05/mppi_s1_05_0.db3"
worst_bag = "/home/chan/Downloads/experiments/bags/mppi_s1_02/mppi_s1_02_0.db3"

best_x, best_y = extract_trajectory(best_bag)
worst_x, worst_y = extract_trajectory(worst_bag)

output_dir = "/home/chan/.gemini/antigravity/brain/bf341c6f-08d2-4891-bdda-a9d0b16ad2b8"

plt.figure(figsize=(10, 8))
plt.plot(best_x, best_y, color='blue', linewidth=2, label='Best Run (Trial 05: 28.98s)')
plt.title('Best Run Trajectory (Trial 05)', fontsize=16)
plt.xlabel('X (m)', fontsize=14)
plt.ylabel('Y (m)', fontsize=14)
plt.axis('equal')
plt.grid(True, linestyle='--', alpha=0.7)
plt.legend(fontsize=12)
plt.savefig(f"{output_dir}/best_trajectory.png", dpi=150, bbox_inches='tight')
plt.close()

plt.figure(figsize=(10, 8))
plt.plot(worst_x, worst_y, color='red', linewidth=2, label='Worst Run (Trial 02: 39.07s)')
plt.title('Worst Run Trajectory (Trial 02)', fontsize=16)
plt.xlabel('X (m)', fontsize=14)
plt.ylabel('Y (m)', fontsize=14)
plt.axis('equal')
plt.grid(True, linestyle='--', alpha=0.7)
plt.legend(fontsize=12)
plt.savefig(f"{output_dir}/worst_trajectory.png", dpi=150, bbox_inches='tight')
plt.close()

plt.figure(figsize=(10, 8))
plt.plot(best_x, best_y, color='blue', linewidth=2, label='Best (Trial 05)')
plt.plot(worst_x, worst_y, color='red', linewidth=2, alpha=0.6, label='Worst (Trial 02)')
plt.title('Trajectory Comparison: Best vs Worst', fontsize=16)
plt.xlabel('X (m)', fontsize=14)
plt.ylabel('Y (m)', fontsize=14)
plt.axis('equal')
plt.grid(True, linestyle='--', alpha=0.7)
plt.legend(fontsize=12)
plt.savefig(f"{output_dir}/comparison_trajectory.png", dpi=150, bbox_inches='tight')
plt.close()

print("PNG images successfully generated.")
