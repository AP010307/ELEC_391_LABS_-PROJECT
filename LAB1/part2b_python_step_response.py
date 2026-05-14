import serial
import time
import pandas as pd
import matplotlib.pyplot as plt

# ── CONFIG ──────────────────────────────────────────────
PORT = 'COM3'
BAUD = 115200
OUTPUT_FILE = 'part2_step.csv'
DURATION = 15  # a few seconds at 0°, then move to 30°, hold
# ────────────────────────────────────────────────────────

# ── RECORD ──────────────────────────────────────────────
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)

print(f"Recording for {DURATION}s...")
print("  0s–3s  → Hold at 0°")
print("  ~3s    → Quickly move to 30°")
print("  3s–15s → Hold at 30°")

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
plt.plot(df['time_s'], df['accel_angle'], color='blue', linewidth=0.8, label='Accelerometer')
plt.plot(df['time_s'], df['gyro_angle'],  color='red',  linewidth=0.8, label='Gyroscope')
plt.axhline(0,  color='gray', linestyle='--', linewidth=0.7, label='0° target')
plt.axhline(30, color='black', linestyle='--', linewidth=0.7, label='30° target')
plt.xlabel('Time (s)')
plt.ylabel('Angle (°)')
plt.title('Part 2 — Step Response: Accelerometer vs Gyroscope (0° → 30°)')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig('part2_step.png', dpi=150)
plt.show()
