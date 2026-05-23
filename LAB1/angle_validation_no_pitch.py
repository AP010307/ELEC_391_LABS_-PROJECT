import csv
import os
import sys
import time
from datetime import datetime

import matplotlib.pyplot as plt
import serial

# ---------------- USER SETTINGS ----------------
PORT = "COM3"          # Change this to your Arduino port, e.g. COM3 or COM4
BAUDRATE = 115200      # Must match Serial.begin(115200) in Arduino
DURATION_S = 120        # Record for about 2 minutes, then plot
BASE_FOLDER = r"C:\Users\anhph\ELEC_391\LAB1_OUTPUT"
SERIAL_TIMEOUT = 0.1
# ------------------------------------------------

os.makedirs(BASE_FOLDER, exist_ok=True)

current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
csv_filename = os.path.join(BASE_FOLDER, f"angle_data_{current_time}, step_response,  k = 0.8, 0-10DEG.csv")
plot_filename = os.path.join(BASE_FOLDER, f"angle_plot_{current_time}, step_response,  k = 0.8, 0-10DEG.png")

CSV_HEADER = [
    "Time (s)",
    "Accel Roll (deg)",
    "Gyro Roll (deg)",
    "Filtered Roll (deg)", 
]


def parse_arduino_line(line):
    """
    Expected Arduino data line:
    accel_roll,accel_pitch,gyro_roll,gyro_pitch,roll_angle,pitch_angle

    This script only uses:
    accel_roll and gyro_roll
    """
    parts = [p.strip() for p in line.split(",")]

    if len(parts) != 6:
        return None

    try:
        return tuple(float(p) for p in parts)
    except ValueError:
        return None


def open_serial():
    try:
        ser = serial.Serial(
            port=PORT,
            baudrate=BAUDRATE,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=SERIAL_TIMEOUT,
        )
        time.sleep(2.0)        
        ser.reset_input_buffer()
        print(f"Connected to {PORT} at {BAUDRATE} baud")
        return ser
    except serial.SerialException as exc:
        print(f"Could not open {PORT}: {exc}")
        print("Check Arduino IDE Tools > Port, and close Serial Monitor/Serial Plotter.")
        sys.exit(1)


def collect_data():
    ser = open_serial()

    times = []
    accel_rolls = []
    gyro_rolls = []
    filtered_rolls = []

    print(f"Saving data to: {csv_filename}")
    print(f"Recording for {DURATION_S} seconds...")

    start_time = time.time()

    try:
        with open(csv_filename, "w", newline="") as csv_file:
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(CSV_HEADER)

            while True:
                t = time.time() - start_time
                if t >= DURATION_S:
                    break

                raw = ser.readline()
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="ignore").strip()
                values = parse_arduino_line(line)

                if values is None:
                    # Ignore lines like "Started" and sample-rate lines.
                    if line:
                        print(f"Skipping: {line}")
                    continue

                accel_roll, accel_pitch, gyro_roll, gyro_pitch, roll, pitch = values

                times.append(t)
                accel_rolls.append(accel_roll)
                gyro_rolls.append(gyro_roll)
                filtered_rolls.append(roll)

                csv_writer.writerow([
                    round(t, 3),
                    accel_roll,
                    gyro_roll,
                    roll,
                ])

                # Print occasionally so you know it is alive.
                if len(times) % 50 == 0:
                    print(
                        f"t={t:6.2f}s | "
                        f"accel_roll={accel_roll:8.3f} deg | "
                        f"gyro_roll={gyro_roll:8.3f} deg | "
                        f"filtered_roll={roll:8.3f} deg"
                    )

    finally:
        if ser.is_open:
            ser.close()

    return {
        "time": times,
        "accel_roll": accel_rolls,
        "gyro_roll": gyro_rolls,
        "roll": filtered_rolls,
    }


def plot_data(data):
    if not data["time"]:
        print("No valid Arduino CSV data was recorded. Check the serial output format.")
        return

    fig, ax_roll = plt.subplots()

    ax_roll.plot(data["time"], data["accel_roll"], label="Accel Roll", linewidth=1.2)
    ax_roll.plot(data["time"], data["gyro_roll"], label="Gyro Roll", linewidth=1.2)
    ax_roll.plot(data["time"], data["roll"], label="Filtered Roll", linewidth=2.0)

    ax_roll.set_title(f"Accel Roll vs Gyro Roll After {DURATION_S} s")
    ax_roll.set_ylabel("Roll Angle (deg)")
    ax_roll.set_xlabel("Time (s)")
    ax_roll.grid(True)
    ax_roll.legend(loc="upper right")

    fig.tight_layout()
    fig.savefig(plot_filename, dpi=200)
    print(f"Saved plot to: {plot_filename}")
    plt.show()


if __name__ == "__main__":
    data = collect_data()
    print(f"Recorded {len(data['time'])} valid samples")
    plot_data(data)
