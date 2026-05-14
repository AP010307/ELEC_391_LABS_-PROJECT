import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys, time, serial, csv, keyboard, os
from datetime import datetime

# Set up serial connection
ser = serial.Serial(
    port='COM4',
    baudrate=115200,   # Make sure this matches Arduino Serial.begin()
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,  # Usually ONE for Arduino
    bytesize=serial.EIGHTBITS,
    timeout=1
)

xsize = 40  # seconds shown in plot window

# Folder for saving CSV files
base_folder = "C:\ELEC_391\LAB1_OUTPUT"
os.makedirs(base_folder, exist_ok=True)

# Generate unique filename based on date/time
current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
csv_filename = os.path.join(base_folder, f"angle_data_{current_time}.csv")

# Open CSV file to save data
csv_file = open(csv_filename, "w", newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(["Time (s)", "Roll (deg)", "Pitch (deg)"])

paused = False

def toggle_pause():
    global paused
    paused = not paused
    print("Paused" if paused else "Resumed")

def toggle_end():
    global paused
    paused = True
    print("Ending program")
    ser.close()
    csv_file.close()
    sys.exit(0)

keyboard.add_hotkey("space", toggle_pause)
keyboard.add_hotkey("esc", toggle_end)

def data_gen():
    start_time = time.time()

    while True:
        while paused:
            time.sleep(0.1)

        line = ser.readline().decode('utf-8', errors='ignore').strip()

        if not line:
            continue

        try:
            # Expected Arduino format:
            # roll,pitch
            roll, pitch = map(float, line.split(","))

            t = time.time() - start_time

            print(f"Time: {t:.2f}s, Roll: {roll:.2f} deg, Pitch: {pitch:.2f} deg")

            csv_writer.writerow([round(t, 3), roll, pitch])
            csv_file.flush()

            yield t, roll, pitch

        except ValueError:
            # Ignore lines like "Started" or "Gyroscope sample rate = ..."
            pass

def run(data):
    t, roll, pitch = data

    xdata.append(t)
    roll_data.append(roll)
    pitch_data.append(pitch)

    if t > xsize:
        ax.set_xlim(t - xsize, t)

    roll_line.set_data(xdata, roll_data)
    pitch_line.set_data(xdata, pitch_data)

    return roll_line, pitch_line

def on_close_figure(event):
    ser.close()
    csv_file.close()
    sys.exit(0)

# Initialize plot
fig = plt.figure()
fig.canvas.mpl_connect('close_event', on_close_figure)

ax = fig.add_subplot(111)

roll_line, = ax.plot([], [], lw=2, label="Roll")
pitch_line, = ax.plot([], [], lw=2, label="Pitch")

ax.set_ylim(-180, 180)
ax.set_xlim(0, xsize)

ax.set_title("Live Complementary Filter Angle")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Angle (deg)")
ax.grid()
ax.legend()

xdata = []
roll_data = []
pitch_data = []

ani = animation.FuncAnimation(
    fig,
    run,
    data_gen,
    blit=False,
    interval=50,
    repeat=False
)

plt.show()