import serial
import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ── CONFIG ──────────────────────────────────────────────
PORT = 'COM3'
BAUD = 115200
OUTPUT_FILE = 'part1_data.csv'
DURATION = 25
# ────────────────────────────────────────────────────────

# ── RECORD ──────────────────────────────────────────────
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)

print(f"Recording for {DURATION}s...")
print("  0s–7s   → Hold at 0°")
print("  7s–14s  → Hold at 10°")
print("  14s–22s → Hold at 30°")

lines = []
start = time.time()

while time.time() - start < DURATION:
    raw = ser.readline().decode('utf-8').strip()
    # Skip all non-data lines from Arduino startup prints
    if raw and not raw.startswith('accel') and not raw.startswith('Started') \
            and not raw.startswith('Gyro') and not raw.startswith('Accel'):
        parts = raw.split(',')
        if len(parts) == 6:  # ensure valid data line
            timestamp = round((time.time() - start) * 1000)  # ms since start
            lines.append(f"{timestamp},{raw}")
            print(f"{timestamp},{raw}")

ser.close()

with open(OUTPUT_FILE, 'w') as f:
    f.write('time_ms,accel_roll,accel_pitch,gyro_roll,gyro_pitch,roll_angle,pitch_angle\n')
    for l in lines:
        f.write(l + '\n')
print(f"\nSaved to {OUTPUT_FILE}")

# ── LOAD ─────────────────────────────────────────────────
df = pd.read_csv(OUTPUT_FILE)
df['time_s'] = df['time_ms'] / 1000.0

# ── PLOT ─────────────────────────────────────────────────
plt.figure(figsize=(10, 5))
plt.plot(df['time_s'], df['accel_roll'], color='blue', linewidth=0.8, label='Accelerometer Roll')
plt.axhline(0,  color='gray',   linestyle='--', linewidth=0.7, label='0° target')
plt.axhline(10, color='orange', linestyle='--', linewidth=0.7, label='10° target')
plt.axhline(30, color='red',    linestyle='--', linewidth=0.7, label='30° target')
plt.xlabel('Time (s)')
plt.ylabel('Angle (°)')
plt.title('Part 1 — Accelerometer: Static Hold Test')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig('part1_static_hold.png', dpi=150)
plt.show()

# ── STATS ─────────────────────────────────────────────────
# Adjust these windows to match when you actually held each angle
windows = {
    '0°':  (2.0,  7.0),
    '10°': (9.0,  14.0),
    '30°': (16.0, 21.0)
}
targets = {'0°': 0, '10°': 10, '30°': 30}

print("\n── Statistical Results ──────────────────────────")
print(f"{'Angle':<8} {'Mean (°)':<12} {'Std Dev (°)':<14} {'Error (°)':<10}")
print("-" * 48)
for label, (t_start, t_end) in windows.items():
    window = df[(df['time_s'] >= t_start) & (df['time_s'] <= t_end)]['accel_roll']
    if window.empty:
        print(f"{label:<8} No data in window {t_start}s–{t_end}s")
        continue
    mean  = window.mean()
    std   = window.std()
    error = mean - targets[label]
    print(f"{label:<8} {mean:<12.3f} {std:<14.4f} {error:<10.3f}")

print("\nUse the 0° mean as angle_offset in your Arduino code.")
