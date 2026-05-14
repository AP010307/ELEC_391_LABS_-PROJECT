import serial
import time
import pandas as pd
import matplotlib.pyplot as plt

# ── CONFIG ──────────────────────────────────────────────
PORT = 'COM3'
BAUD = 115200
DURATION = 15
# ────────────────────────────────────────────────────────

def record(filename, k_value):
    input(f"\nSet k = {k_value} in Arduino, upload, then press Enter to record...")
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)
    print(f"Recording k={k_value} for {DURATION}s...")
    print("  0s–3s  → Hold at 0°")
    print("  ~3s    → Quickly move to 30°")
    print("  3s–15s → Hold at 30°")

    lines = []
    start = time.time()
    while time.time() - start < DURATION:
        raw = ser.readline().decode('utf-8').strip()
        if raw and not raw.startswith('accel') and not raw.startswith('Started') \
                and not raw.startswith('Gyro') and not raw.startswith('Accel'):
            parts = raw.split(',')
            if len(parts) == 6:
                timestamp = round((time.time() - start) * 1000)
                lines.append(f"{timestamp},{raw}")
                print(f"{timestamp},{raw}")
    ser.close()

    with open(filename, 'w') as f:
        f.write('time_ms,accel_roll,accel_pitch,gyro_roll,gyro_pitch,roll_angle,pitch_angle\n')
        for l in lines:
            f.write(l + '\n')
    print(f"Saved to {filename}")

def plot_k(filename, k_value, ax):
    df = pd.read_csv(filename)
    df['time_s'] = df['time_ms'] / 1000.0
    ax.plot(df['time_s'], df['accel_roll'], color='blue',  linewidth=0.8, label='Accelerometer')
    ax.plot(df['time_s'], df['gyro_roll'],  color='red',   linewidth=0.8, label='Gyroscope')
    ax.plot(df['time_s'], df['roll_angle'], color='green', linewidth=1.2, label='Complementary')
    ax.axhline(0,  color='gray',  linestyle='--', linewidth=0.6)
    ax.axhline(30, color='black', linestyle='--', linewidth=0.6)
    ax.set_title(f'k = {k_value}')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Angle (°)')
    ax.legend(fontsize=8)
    ax.grid(True)

# ── RECORD ALL THREE ──────────────────────────────────────
k_values = [0.1, 0.5, 0.9]
files = {k: f'part4_k{str(k).replace(".", "")}.csv' for k in k_values}

for k in k_values:
    record(files[k], k)

# ── PLOT ALL THREE ────────────────────────────────────────
fig, axes = plt.subplots(3, 1, figsize=(10, 12), sharex=False)
for i, k in enumerate(k_values):
    plot_k(files[k], k, axes[i])

fig.suptitle('Part 4 — Complementary Filter Step Response (0° → 30°)', fontsize=13)
plt.tight_layout()
plt.savefig('part4_complementary.png', dpi=150)
plt.show()
