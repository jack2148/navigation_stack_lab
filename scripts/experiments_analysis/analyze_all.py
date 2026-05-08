import os
import glob
import pandas as pd
import yaml

def get_bag_duration(yaml_path):
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
        try:
            return data['rosbag2_bagfile_information']['duration']['nanoseconds'] / 1e9
        except KeyError:
            return None

base_dir = "/home/chan/Downloads/experiments"
bag_dir = os.path.join(base_dir, "bags")
cpu_dir = os.path.join(base_dir, "cpu_logs")

runs = ["mppi_s1_01", "mppi_s1_02", "mppi_s1_03", "mppi_s1_04", "mppi_s1_05"]

results = []

for run in runs:
    bag_meta = os.path.join(bag_dir, run, "metadata.yaml")
    cpu_csv = os.path.join(cpu_dir, f"{run}_cpu.csv")
    
    duration = None
    if os.path.exists(bag_meta):
        duration = get_bag_duration(bag_meta)
        
    avg_cpu = None
    max_cpu = None
    if os.path.exists(cpu_csv):
        df = pd.read_csv(cpu_csv)
        if not df.empty and 'cpu_percent' in df.columns:
            avg_cpu = df['cpu_percent'].mean()
            max_cpu = df['cpu_percent'].max()
            
    results.append({
        "Trial": run.replace("mppi_s1_", ""),
        "Time to Goal (s)": f"{duration:.2f}" if duration else "N/A",
        "Avg CPU (%)": f"{avg_cpu:.2f}" if avg_cpu else "N/A",
        "Max CPU (%)": f"{max_cpu:.2f}" if max_cpu else "N/A"
    })

print("| Trial | Time to Goal (s) | Avg CPU (%) | Max CPU (%) |")
print("|-------|------------------|-------------|-------------|")
for r in results:
    print(f"| {r['Trial']} | {r['Time to Goal (s)']} | {r['Avg CPU (%)']} | {r['Max CPU (%)']} |")
