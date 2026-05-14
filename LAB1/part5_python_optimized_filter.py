import serial
import time
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# ── CONFIG ──────────────────────────────────────────────
PORT = 'COM3'
BAUD = 115200
OUTPUT_FILE = 'part5_optimized.csv'
DURATION = 25
K_OPTIMIZED = 0.98  # Change this after tuning
# ────────────────────────────────────────────────────────

# ── RECORD ──────────────────────────────────────────────
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)

print(f"Recording optimized filter (k={K_OPTIMIZED}) for {DURATION}s...")
print("  0s–5s   → Hold at 0°")
print("  ~5s     → Quickly move to 10°")
print("  5s–25s  → Hold at 10° (for drift measurement)")

lines = []
start = time.time()
while time.time() - start < DURATION:
    raw = ser.readline().decode('utf-8').strip()
    if raw and not raw.startswith('time'):
        print(raw)
        lines.append(raw)
ser.close()

with open(OUTPUT_FILE, 'w') as f:
    f.write('time_ms,accel_angle,gyro_angle,comp_angle\n')
    for l in lines:
        f.write(l + '\n')
print(f"Saved to {OUTPUT_FILE}")

# ── LOAD ─────────────────────────────────────────────────
df = pd.read_csv(OUTPUT_FILE)
df['time_s'] = df['time_ms'] / 1000.0

# ── PLOT ─────────────────────────────────────────────────
plt.figure(figsize=(10, 5))
plt.plot(df['time_s'], df['accel_angle'], color='blue',  linewidth=0.8, label='Accelerometer')
plt.plot(df['time_s'], df['gyro_angle'],  color='red',   linewidth=0.8, label='Gyroscope')
plt.plot(df['time_s'], df['comp_angle'],  color='green', linewidth=1.2, label=f'Complementary (k={K_OPTIMIZED})')
plt.axhline(0,  color='gray',  linestyle='--', linewidth=0.7, label='0° target')
plt.axhline(10, color='black', linestyle='--', linewidth=0.7, label='10° target')
plt.xlabel('Time (s)')
plt.ylabel('Angle (°)')
plt.title(f'Part 5 — Optimized Complementary Filter (k={K_OPTIMIZED}): Step Response (0° → 10°)')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig('part5_optimized.png', dpi=150)
plt.show()

# ── STATS ─────────────────────────────────────────────────
# Steady-state window at 10° — last 15 seconds of recording
steady = df[df['time_s'] >= 8.0]['comp_angle']

# Drift window — first vs last value in steady state
drift_window = df[(df['time_s'] >= 5.5) & (df['time_s'] <= 25.0)]['comp_angle']

static_error = steady.mean() - 10.0
drift_20s    = drift_window.iloc[-1] - drift_window.iloc[0]
noise        = steady.std()

# Time constant
tau = (K_OPTIMIZED / (1 - K_OPTIMIZED)) * 0.01  # assumes ~10ms loop

print("\n── Part 5 Results ───────────────────────────────")
print(f"  Optimized k         : {K_OPTIMIZED}")
print(f"  Filter time constant: {tau:.3f} s")
print(f"  Static Error at 10° : {static_error:.4f} °")
print(f"  20s Drift           : {drift_20s:.4f} °")
print(f"  Noise (std dev)     : {noise:.4f} °")
print("─────────────────────────────────────────────────")
