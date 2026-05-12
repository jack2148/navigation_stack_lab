import pandas as pd
import os
import glob

cpu_log_dir = "/home/chan/Downloads/experiments/cpu_logs"
files = glob.glob(os.path.join(cpu_log_dir, "*.csv"))

summary_data = []

for f in sorted(files):
    try:
        df = pd.read_csv(f)
        if df.empty or 'cpu_percent' not in df.columns:
            continue
        df['timestamp'] = pd.to_datetime(df['timestamp'])
        df['time_sec'] = (df['timestamp'] - df['timestamp'].iloc[0]).dt.total_seconds()
        
        run_name = os.path.basename(f).replace("_cpu.csv", "")
        
        summary_data.append({
            'Run': run_name,
            'Duration (s)': round(df['time_sec'].iloc[-1], 2),
            'Avg CPU (%)': round(df['cpu_percent'].mean(), 2),
            'Max CPU (%)': round(df['cpu_percent'].max(), 2),
            'Min CPU (%)': round(df['cpu_percent'].min(), 2)
        })
    except Exception as e:
        print(f"Error reading {f}: {e}")

summary_df = pd.DataFrame(summary_data)
print(summary_df.to_markdown(index=False))
