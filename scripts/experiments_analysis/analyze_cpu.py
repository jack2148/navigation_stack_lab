import pandas as pd
import matplotlib.pyplot as plt
import os
import glob

cpu_log_dir = "/home/chan/Downloads/experiments/cpu_logs"
files = glob.glob(os.path.join(cpu_log_dir, "*.csv"))

plt.figure(figsize=(10, 6))
summary_data = []

for f in sorted(files):
    try:
        df = pd.read_csv(f)
        if df.empty or 'cpu_percent' not in df.columns:
            continue
        df['timestamp'] = pd.to_datetime(df['timestamp'])
        df['time_sec'] = (df['timestamp'] - df['timestamp'].iloc[0]).dt.total_seconds()
        
        run_name = os.path.basename(f).replace("_cpu.csv", "")
        plt.plot(df['time_sec'], df['cpu_percent'], label=run_name, marker='o', markersize=3)
        
        summary_data.append({
            'Run': run_name,
            'Duration (s)': df['time_sec'].iloc[-1],
            'Avg CPU (%)': df['cpu_percent'].mean(),
            'Max CPU (%)': df['cpu_percent'].max(),
            'Min CPU (%)': df['cpu_percent'].min()
        })
    except Exception as e:
        print(f"Error reading {f}: {e}")

plt.xlabel("Time (s)")
plt.ylabel("CPU Usage (%)")
plt.title("CPU Usage over Time for MPPI Trials")
plt.legend()
plt.grid(True)
plt.savefig("/home/chan/Downloads/experiments/cpu_usage.png")
print("Saved CPU usage plot to /home/chan/Downloads/experiments/cpu_usage.png")

summary_df = pd.DataFrame(summary_data)
print("\n--- CPU Usage Summary ---")
print(summary_df.to_markdown(index=False))

