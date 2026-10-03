import serial
import struct
import threading
import collections
import matplotlib.pyplot as plt
import matplotlib.animation as animation

PORT = '/dev/ttyACM0'
BAUD = 2000000
FRAME_SYNC = b'\xAA\x55'
FRAME_LEN = 17
PLOT_WINDOW = 2000

buf_lock = threading.Lock()
idx_buf = collections.deque(maxlen=PLOT_WINDOW)
ia_buf = collections.deque(maxlen=PLOT_WINDOW)
ib_buf = collections.deque(maxlen=PLOT_WINDOW)
ic_buf = collections.deque(maxlen=PLOT_WINDOW)

stop_flag = False
sample_counter = 0

def checksum_ok(payload, cksum):
    c = 0
    for b in payload:
        c ^= b
    return c == cksum

def serial_reader_thread():
    global stop_flag, sample_counter
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    raw = bytearray()

    while not stop_flag:
        chunk = ser.read(4096)
        if chunk:
            raw.extend(chunk)

        while True:
            sync_pos = raw.find(FRAME_SYNC)
            if sync_pos == -1:
                if len(raw) > 1:
                    del raw[:len(raw) - 1]
                break

            if sync_pos > 0:
                del raw[:sync_pos]

            if len(raw) < FRAME_LEN:
                break

            frame = raw[:FRAME_LEN]
            payload = frame[2:16]
            cksum = frame[16]

            if checksum_ok(payload, cksum):
                seq, i_a, i_b, i_c = struct.unpack('<Hfff', payload)
                with buf_lock:
                    sample_counter += 1
                    idx_buf.append(sample_counter)
                    ia_buf.append(i_a)
                    ib_buf.append(i_b)
                    ic_buf.append(i_c)
                del raw[:FRAME_LEN]
            else:
                del raw[:1]

    ser.close()

def main():
    global stop_flag

    reader = threading.Thread(target=serial_reader_thread, daemon=True)
    reader.start()

    fig, ax = plt.subplots()
    line_a, = ax.plot([], [], label='I_a', lw=1)
    line_b, = ax.plot([], [], label='I_b', lw=1)
    line_c, = ax.plot([], [], label='I_c', lw=1)
    ax.set_ylim(-10, 10)
    ax.set_xlabel('Sample #')
    ax.set_ylabel('Current (A)')
    ax.legend(loc='upper right')
    ax.grid(True)

    def update(frame_num):
        with buf_lock:
            if len(idx_buf) == 0:
                return line_a, line_b, line_c
            x = list(idx_buf)
            ya, yb, yc = list(ia_buf), list(ib_buf), list(ic_buf)

        line_a.set_data(x, ya)
        line_b.set_data(x, yb)
        line_c.set_data(x, yc)
        ax.set_xlim(x[0], x[-1] if x[-1] > x[0] else x[0] + 1)
        return line_a, line_b, line_c

    ani = animation.FuncAnimation(fig, update, interval=33, blit=False)
    plt.show()
    stop_flag = True
    reader.join(timeout=1.0)

if __name__ == '__main__':
    main()