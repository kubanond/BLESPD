import socket
import csv
import datetime
import sys
from collections import deque
from threading import Thread, Lock

from PyQt5 import QtWidgets
import pyqtgraph as pg

UDP_IP = "0.0.0.0"
UDP_PORT = 3333
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(True)

csv_filename = "sensor_data.csv"
try:
    with open(csv_filename, 'x', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["timestamp", "X", "Y", "Z"])
        print(f"Created new CSV file: {csv_filename}")
except FileExistsError:
    print(f"Appending to existing CSV file: {csv_filename}")

csvfile = open(csv_filename, 'a', newline='')
writer = csv.writer(csvfile)

max_points = 200
data_x = deque(maxlen=max_points)
data_y = deque(maxlen=max_points)
data_z = deque(maxlen=max_points)
lock = Lock()

app = QtWidgets.QApplication(sys.argv)
win = pg.GraphicsLayoutWidget(show=True, title="Real-time Sensor Data")
plot = win.addPlot(title="X, Y, Z sensor values")
plot.setYRange(-1000, 1000)
curve_x = plot.plot(pen='r', name="X")
curve_y = plot.plot(pen='g', name="Y")
curve_z = plot.plot(pen='b', name="Z")


def udp_receiver():
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            message = data.decode('utf-8').strip()

            parts = message.split(',')
            if len(parts) < 3:
                continue

            x = int(parts[0].split(':')[1].strip())
            y = int(parts[1].split(':')[1].strip())
            z = int(parts[2].split(':')[1].strip())
            timestamp = datetime.datetime.now().isoformat()


            writer.writerow([timestamp, x, y, z])
            csvfile.flush()

            with lock:
                data_x.append(x)
                data_y.append(y)
                data_z.append(z)

        except Exception as e:
            print("Error receiving/parsing UDP message:", e)

receiver_thread = Thread(target=udp_receiver, daemon=True)
receiver_thread.start()

def update_plot():
    with lock:
        curve_x.setData(list(data_x))
        curve_y.setData(list(data_y))
        curve_z.setData(list(data_z))

timer = pg.QtCore.QTimer()
timer.timeout.connect(update_plot)
timer.start(20)

import atexit
atexit.register(lambda: csvfile.close())

if __name__ == '__main__':
    QtWidgets.QApplication.instance().exec_()