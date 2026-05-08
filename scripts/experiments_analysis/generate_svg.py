import sqlite3
import os
import sys

# Append ROS 2 paths manually if needed, or rely on the environment
try:
    from rclpy.serialization import deserialize_message
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
except ImportError:
    print("Error: Could not import rclpy or geometry_msgs. Run within ROS 2 environment.")
    sys.exit(1)

def extract_bag_data(db3_path):
    conn = sqlite3.connect(db3_path)
    c = conn.cursor()
    
    # Get topic IDs
    c.execute("SELECT id, name FROM topics")
    topics = c.fetchall()
    topic_map = {name: tid for tid, name in topics}
    
    cmd_vel_id = topic_map.get('/cmd_vel')
    odom_id = topic_map.get('/odometry/filtered')
    
    cmd_vel_data = []
    if cmd_vel_id is not None:
        c.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", (cmd_vel_id,))
        for ts, data in c.fetchall():
            msg = deserialize_message(data, Twist)
            cmd_vel_data.append((ts, msg.linear.x, msg.angular.z))
            
    odom_data = []
    if odom_id is not None:
        c.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", (odom_id,))
        for ts, data in c.fetchall():
            msg = deserialize_message(data, Odometry)
            odom_data.append((ts, msg.pose.pose.position.x, msg.pose.pose.position.y))
            
    return cmd_vel_data, odom_data

def generate_svg_line_chart(data, title, y_label, output_file, colors, labels):
    if not data:
        return
    
    width = 800
    height = 400
    margin = 50
    
    t0 = data[0][0]
    # data format: [(ts, val1, val2)]
    points = []
    for row in data:
        t = (row[0] - t0) / 1e9
        points.append((t, row[1], row[2]))
        
    t_max = points[-1][0]
    
    val1_max = max(p[1] for p in points)
    val1_min = min(p[1] for p in points)
    val2_max = max(p[2] for p in points)
    val2_min = min(p[2] for p in points)
    
    y_max = max(val1_max, val2_max) + 0.1
    y_min = min(val1_min, val2_min) - 0.1
    
    if y_max == y_min:
        y_max += 1
        y_min -= 1
        
    def get_x(t):
        return margin + (t / t_max) * (width - 2 * margin)
        
    def get_y(v):
        return height - margin - ((v - y_min) / (y_max - y_min)) * (height - 2 * margin)

    svg = [
        f'<svg width="{width}" height="{height}" xmlns="http://www.w3.org/2000/svg">',
        f'<rect width="100%" height="100%" fill="white"/>',
        f'<text x="{width/2}" y="30" font-family="Arial" font-size="20" text-anchor="middle">{title}</text>',
        f'<text x="{width/2}" y="{height-10}" font-family="Arial" font-size="14" text-anchor="middle">Time (s)</text>',
        f'<text x="20" y="{height/2}" font-family="Arial" font-size="14" text-anchor="middle" transform="rotate(-90 20 {height/2})">{y_label}</text>',
        # Grid lines
        f'<line x1="{margin}" y1="{get_y(0)}" x2="{width-margin}" y2="{get_y(0)}" stroke="gray" stroke-width="1" stroke-dasharray="5,5"/>',
        # Axes
        f'<line x1="{margin}" y1="{margin}" x2="{margin}" y2="{height-margin}" stroke="black" stroke-width="2"/>',
        f'<line x1="{margin}" y1="{height-margin}" x2="{width-margin}" y2="{height-margin}" stroke="black" stroke-width="2"/>',
    ]
    
    # Paths
    for idx, (color, label) in enumerate(zip(colors, labels)):
        path_str = []
        for i, p in enumerate(points):
            cmd = "M" if i == 0 else "L"
            path_str.append(f"{cmd} {get_x(p[0])} {get_y(p[idx+1])}")
            
        svg.append(f'<path d="{" ".join(path_str)}" fill="none" stroke="{color}" stroke-width="2"/>')
        # Legend
        svg.append(f'<rect x="{width - margin - 150}" y="{margin + idx*20}" width="15" height="15" fill="{color}"/>')
        svg.append(f'<text x="{width - margin - 125}" y="{margin + 12 + idx*20}" font-family="Arial" font-size="12">{label}</text>')
        
    svg.append('</svg>')
    
    with open(output_file, 'w') as f:
        f.write('\n'.join(svg))

def generate_svg_trajectory(data, output_file):
    if not data:
        return
        
    width = 600
    height = 600
    margin = 50
    
    xs = [row[1] for row in data]
    ys = [row[2] for row in data]
    
    x_max, x_min = max(xs), min(xs)
    y_max, y_min = max(ys), min(ys)
    
    # Make square aspect ratio
    x_range = x_max - x_min
    y_range = y_max - y_min
    max_range = max(x_range, y_range) + 1.0
    
    x_center = (x_max + x_min) / 2
    y_center = (y_max + y_min) / 2
    
    def get_x(x):
        return margin + ((x - (x_center - max_range/2)) / max_range) * (width - 2 * margin)
        
    def get_y(y):
        # invert y for screen coordinates
        return height - margin - ((y - (y_center - max_range/2)) / max_range) * (height - 2 * margin)

    svg = [
        f'<svg width="{width}" height="{height}" xmlns="http://www.w3.org/2000/svg">',
        f'<rect width="100%" height="100%" fill="white"/>',
        f'<text x="{width/2}" y="30" font-family="Arial" font-size="20" text-anchor="middle">Robot Trajectory (Odometry)</text>',
        f'<text x="{width/2}" y="{height-10}" font-family="Arial" font-size="14" text-anchor="middle">X (m)</text>',
        f'<text x="20" y="{height/2}" font-family="Arial" font-size="14" text-anchor="middle" transform="rotate(-90 20 {height/2})">Y (m)</text>',
        # Axes
        f'<line x1="{margin}" y1="{margin}" x2="{margin}" y2="{height-margin}" stroke="black" stroke-width="2"/>',
        f'<line x1="{margin}" y1="{height-margin}" x2="{width-margin}" y2="{height-margin}" stroke="black" stroke-width="2"/>',
    ]
    
    path_str = []
    for i, row in enumerate(data):
        cmd = "M" if i == 0 else "L"
        path_str.append(f"{cmd} {get_x(row[1])} {get_y(row[2])}")
        
    svg.append(f'<path d="{" ".join(path_str)}" fill="none" stroke="blue" stroke-width="2"/>')
    svg.append('</svg>')
    
    with open(output_file, 'w') as f:
        f.write('\n'.join(svg))

if __name__ == "__main__":
    db3_path = "/home/chan/Downloads/experiments/bags/mppi_s1_03/mppi_s1_03_0.db3"
    cmd_vel_data, odom_data = extract_bag_data(db3_path)
    
    artifacts_dir = "/home/chan/.gemini/antigravity/brain/bf341c6f-08d2-4891-bdda-a9d0b16ad2b8"
    os.makedirs(artifacts_dir, exist_ok=True)
    
    cmd_vel_svg = os.path.join(artifacts_dir, "mppi_03_cmd_vel.svg")
    odom_svg = os.path.join(artifacts_dir, "mppi_03_trajectory.svg")
    
    generate_svg_line_chart(cmd_vel_data, "Command Velocity Over Time", "Velocity", cmd_vel_svg, ["green", "red"], ["Linear X (m/s)", "Angular Z (rad/s)"])
    generate_svg_trajectory(odom_data, odom_svg)
    
    print(f"Generated {cmd_vel_svg}")
    print(f"Generated {odom_svg}")
