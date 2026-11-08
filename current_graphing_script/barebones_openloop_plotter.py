import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import collections

# --- CONFIGURATION ---
PORT = '/dev/ttyACM0'
BAUD = 115200
MAX_POINTS = 150 # How many data points to show on screen

# Data buffers
angles = collections.deque([0]*MAX_POINTS, maxlen=MAX_POINTS)
ia_data = collections.deque([0]*MAX_POINTS, maxlen=MAX_POINTS)
ib_data = collections.deque([0]*MAX_POINTS, maxlen=MAX_POINTS)
ic_data = collections.deque([0]*MAX_POINTS, maxlen=MAX_POINTS)

# Connect to STM32
try:
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    print(f"Connected to {PORT}. Waiting for data...")
except Exception as e:
    print(f"Failed to open {PORT}: {e}")
    exit()

# Setup the Matplotlib figure
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7))
fig.canvas.manager.set_window_title('FOC Real-Time Telemetry')

# Plot lines
line_angle, = ax1.plot(angles, label="Encoder Angle", color='purple', linewidth=2)
line_ia, = ax2.plot(ia_data, label="Phase A (Ia)", color='red', linewidth=2)
line_ib, = ax2.plot(ib_data, label="Phase B (Ib)", color='green', linewidth=2)
line_ic, = ax2.plot(ic_data, label="Phase C (Ic)", color='blue', linewidth=2)

# Format Top Graph (Angle)
ax1.set_title("Rotor Position (0 to 16383)")
ax1.set_ylim(-1000, 17000)
ax1.legend(loc="upper right")
ax1.grid(True, linestyle='--', alpha=0.6)

# Format Bottom Graph (Currents)
ax2.set_title("3-Phase Currents (Amps)")
ax2.set_ylim(-2.5, 2.5) # Adjust this if your current is higher!
ax2.set_ylabel("Amps")
ax2.legend(loc="upper right")
ax2.grid(True, linestyle='--', alpha=0.6)

def update(frame):
    while ser.in_waiting:
        try:
            line = ser.readline().decode('utf-8').strip()
            
            # Only parse lines that have commas (our CSV data)
            if ',' in line and "Motor is OFF" not in line and "STARTING" not in line:
                parts = line.split(',')
                if len(parts) == 4:
                    angles.append(int(parts[0]))
                    ia_data.append(float(parts[1]))
                    ib_data.append(float(parts[2]))
                    ic_data.append(float(parts[3]))
        except Exception:
            pass # Ignore garbage characters during startup

    # Update the graphs
    line_angle.set_ydata(angles)
    line_ia.set_ydata(ia_data)
    line_ib.set_ydata(ib_data)
    line_ic.set_ydata(ic_data)
    return line_angle, line_ia, line_ib, line_ic

# Run the animation at 50fps (20ms)
ani = animation.FuncAnimation(fig, update, interval=20, blit=False)
plt.tight_layout()
plt.show()
