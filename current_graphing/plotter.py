import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import collections

PORT = '/dev/ttyACM0'
BAUD = 115200
MAX_POINTS = 200 

# Data buffers
angles = collections.deque([0]*MAX_POINTS, maxlen=MAX_POINTS)
ia_data = collections.deque([0]*MAX_POINTS, maxlen=MAX_POINTS)
ib_data = collections.deque([0]*MAX_POINTS, maxlen=MAX_POINTS)
ic_data = collections.deque([0]*MAX_POINTS, maxlen=MAX_POINTS)
id_data = collections.deque([0]*MAX_POINTS, maxlen=MAX_POINTS)
iq_data = collections.deque([0]*MAX_POINTS, maxlen=MAX_POINTS)

try:
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
except Exception as e:
    print(f"Failed to open {PORT}: {e}")
    exit()

fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 9))
fig.canvas.manager.set_window_title('FOC Real-Time Telemetry')

line_angle, = ax1.plot(angles, color='purple', linewidth=2)
line_ia, = ax2.plot(ia_data, label="Ia", color='red')
line_ib, = ax2.plot(ib_data, label="Ib", color='green')
line_ic, = ax2.plot(ic_data, label="Ic", color='blue')
line_id, = ax3.plot(id_data, label="Id (Flux)", color='orange', linewidth=2)
line_iq, = ax3.plot(iq_data, label="Iq (Torque)", color='cyan', linewidth=2)

ax1.set_title("Rotor Position")
ax1.set_ylim(-1000, 17000)

ax2.set_title("3-Phase AC Currents")
ax2.set_ylim(-2.5, 2.5)
ax2.legend(loc="upper right")

ax3.set_title("FOC DC Currents (Id, Iq)")
ax3.set_ylim(-2.5, 2.5)
ax3.legend(loc="upper right")

def update(frame):
    while ser.in_waiting:
        try:
            line = ser.readline().decode('utf-8').strip()
            if ',' in line and "Motor is OFF" not in line and "STARTING" not in line:
                parts = line.split(',')
                if len(parts) == 6:
                    angles.append(int(parts[0]))
                    ia_data.append(float(parts[1]))
                    ib_data.append(float(parts[2]))
                    ic_data.append(float(parts[3]))
                    id_data.append(float(parts[4]))
                    iq_data.append(float(parts[5]))
        except Exception:
            pass 

    line_angle.set_ydata(angles)
    line_ia.set_ydata(ia_data)
    line_ib.set_ydata(ib_data)
    line_ic.set_ydata(ic_data)
    line_id.set_ydata(id_data)
    line_iq.set_ydata(iq_data)
    return line_angle, line_ia, line_ib, line_ic, line_id, line_iq

ani = animation.FuncAnimation(fig, update, interval=20, blit=False)
plt.tight_layout()
plt.show()
