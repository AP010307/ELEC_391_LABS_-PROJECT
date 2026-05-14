import serial
import time
import pandas as pd
import matplotlib.pyplot as plt

# ── CONFIG ──────────────────────────────────────────────
PORT = 'COM3'
BAUD = 115200
OUTPUT_FILE = 'part2_drift.csv'
DURATION = 22  # 20s + buffer
# ────────────────────────────────────────────────────────

# ── RECORD ──────────────────────────────────────────────
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)

print(f"Recording drift test for {DURATION}s — keep board FLAT and STILL.")

lines = []
start = time.time()
while time.time() - start < DURATION:
    raw = ser.readline().decode('utf-8').strip()
    if raw and not raw.startswith('time'):
        print(raw)
        lines.append(raw)
ser.close()

with open(OUTPUT_FILE, 'w') as f:
    f.write('time_ms,accel_angle,gyro_angle\n')
    for l in lines:
        f.write(l + '\n')
print(f"Saved to {OUTPUT_FILE}")

# ── LOAD ─────────────────────────────────────────────────
df = pd.read_csv(OUTPUT_FILE)
df['time_s'] = df['time_ms'] / 1000.0

# ── PLOT ─────────────────────────────────────────────────
plt.figure(figsize=(10, 5))
plt.plot(df['time_s'], df['gyro_angle'], color='red', linewidth=0.8, label='Gyroscope')
plt.axhline(0, color='gray', linestyle='--', linewidth=0.7, label='0° reference')
plt.xlabel('Time (s)')
plt.ylabel('Angle (°)')
plt.title('Part 2 — Gyroscope: Stationary Drift Test (20s)')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig('part2_drift.png', dpi=150)
plt.show()

# ── DRIFT STATS ───────────────────────────────────────────
# Use 20s window
window = df[(df['time_s'] >= 0.5) & (df['time_s'] <= 20.5)]['gyro_angle']
if not window.empty:
    start_val = window.iloc[0]
    end_val   = window.iloc[-1]
    total_drift = end_val - start_val
    drift_rate  = total_drift / 20.0
    print("\n── Drift Results ────────────────────────────────")
    print(f"  Total Drift after 20s : {total_drift:.4f} °")
    print(f"  Drift Rate            : {drift_rate:.4f} °/s")
