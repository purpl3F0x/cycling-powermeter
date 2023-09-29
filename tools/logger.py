import asyncio
from bleak import BleakClient, BleakError
from bleak.backends.characteristic import BleakGATTCharacteristic
from datetime import datetime
import struct
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from PySide6.QtCore import QThread, Signal
from scipy import signal
from collections import deque


strain_data = []
gyro_x_data = []
gyro_y_data = []
gyro_z_data = []
gyro_z_data_filtered = []
power_data = []

class LiveFilter:

    def process(self, x):
        if np.isnan(x):
            return x

        return self._process(x)

    def __call__(self, x):
        return self.process(x)

    def _process(self, x):
        raise NotImplementedError("Derived class must implement _process")


class LiveLFilter(LiveFilter):

    def __init__(self, b, a):

        self.b = b
        self.a = a
        self._xs = deque([0] * len(b), maxlen=len(b))
        self._ys = deque([0] * (len(a) - 1), maxlen=len(a)-1)

    def _process(self, x):
        self._xs.appendleft(x)
        y = np.dot(self.b, self._xs) - np.dot(self.a[1:], self._ys)
        y = y / self.a[0]
        self._ys.appendleft(y)

        return y


b = [-0.0031, -0.0050, -0.0068,  0.0000,  0.0255,  0.0731,  0.1325,  0.1826,
     0.2023,  0.1826,  0.1325,  0.0731,  0.0255,  0.0000, -0.0068, -0.0050, -0.0031]
a = [1.]
filter = LiveLFilter(b, a)


def data_handler(data: bytearray):
    force = struct.unpack_from("<i", data, offset=0)[0]
    gyro_x = struct.unpack_from("f", data, offset=4)[0]
    gyro_y = struct.unpack_from("f", data, offset=8)[0]
    gyro_z = struct.unpack_from("f", data, offset=12)[0]
    angle = struct.unpack_from("<h", data, offset=16)[0] / 10
    tangential_velocity = struct.unpack_from("f", data, offset=20)[0]
    power = struct.unpack_from("<h", data, offset=24)[0]


    if len(gyro_z_data_filtered):
        gyro_z_data_filtered.append(
            # filter.process(gyro_z)
            gyro_z_data_filtered[-1] * 0.75 + gyro_z * 0.25
        )
    else:
        gyro_z_data_filtered.append(gyro_z)

    strain_data.append(force)

    # if len(strain_data):
    #     strain_data.append(
    #         strain_data[-1] * 0.75 + force * 0.25
    #     )
    # else:
    #     strain_data.append(force)

    gyro_x_data.append(gyro_x)
    gyro_y_data.append(gyro_y)
    gyro_z_data.append(gyro_z)
    power_data.append(power)

    p1_curve.setData(strain_data)
    p2_x.setData(gyro_x_data)
    p2_y.setData(gyro_y_data)
    p2_z.setData(gyro_z_data)
    p2_z_f.setData(gyro_z_data_filtered)

    if p1.saveState()['view']['autoRange']:
        p1.setXRange(max(len(strain_data) - 500, 0), max(len(strain_data), 500))
    p2.enableAutoRange(axis='y')
    p2.setAutoVisible(y=True)

    rpm = gyro_z / 6

    angle_rad = np.pi / 180 * angle
    p3_angle.setData(x=[0, np.cos(angle_rad)], y=[0, np.sin(angle_rad)])

    p4_power.setData(power_data)
    p4.setXRange(max(len(strain_data) - 500, 0), max(len(strain_data), 500))


    print(datetime.now(), f"{force: 8d}, {gyro_y: 8.3f}, {gyro_z: 8.3f},\
                            {gyro_z_data_filtered[-1]: 8.3f}\t{rpm: 3.0f}\t{angle: 3.1f} {tangential_velocity: 10.3f} {power: 6d}")


app = pg.mkQApp("PowerMeter Raw Data Logger")
win = pg.GraphicsLayoutWidget(show=True, title="PowerMeter Raw Data Logger")
pg.setConfigOptions(antialias=True)
p1 = win.addPlot(title="Strain (N)",  row=0, col=0, colspan=2)
p1.showGrid(x=True, y=True)
p1_curve = p1.plot(pen={'color': '#9b19f5', 'width': 1.5})
p1_legend = p1.addLegend((1,1))
p1_legend.addItem(p1_curve, 'Strain (N)')

p2 = win.addPlot(title="Gyro (°/s)", row=1, col=0, colspan=2)
p2.showGrid(x=True, y=True)
p2_x = p2.plot(pen={'color': '#0bb4ff', 'width': 1.5}, name="X")
p2_y = p2.plot(pen={'color': '#e60049', 'width': 1.5}, name="Y")
p2_z = p2.plot(pen={'color': '#50e991', 'width': 1.5}, name="Z")
p2_z_f = p2.plot(pen={'color': '#f46a9b', 'width': 1.5}, name="Z-filtered")
p2.setXLink(p1)
p2_legend = p2.addLegend((1, 1))
p2_legend.addItem(p2_x, 'Gyro-X (°/s)')
p2_legend.addItem(p2_y, 'Gyro-Y (°/s)')
p2_legend.addItem(p2_z, 'Gyro-Z (°/s)')
p2_legend.addItem(p2_z_f, 'Gyro-Z Filtered (°/s)')


p3 = win.addPlot(title="Angle", row=2, col=0, colspan=1)
p3.setAspectLocked()
p3.enableAutoRange(axis='y', enable=False)
p3.enableAutoRange(axis='x', enable=False)
p3.setMouseEnabled(False, False)
# Add polar grid lines
p3.addLine(x=0, pen=0.2)
p3.addLine(y=0, pen=0.2)
for r in np.arange(0.2, 1.2, 0.2):
    circle = pg.QtWidgets.QGraphicsEllipseItem(-r, -r, r * 2, r * 2)
    circle.setPen(pg.mkPen(0.5))
    p3.addItem(circle)

p3_angle = p3.plot(pen={'color': '#38E54D', 'width': 5}, name="Angle")
p3.setYRange(-1.05, 1.05, padding=0)
p3.setXRange(-1, 1)

p4 = win.addPlot(title="", row=2, col=1, colspan=1)
p4_power = p4.plot(pen={'color': 'red', 'width': 1.5}, name="Angle")


address = "D6:79:AB:61:76:42"
CustomCharacteristic = "1ef880a7-48fe-4517-8bfa-7063ffb80971"


async def ble_task(address: str):
    print("**************************************")
    while True:
        print("Connecting...")
        async with BleakClient(address) as client:
            print("Connected to:", client.address)
            await client.start_notify(CustomCharacteristic, worker.notification_handler)

            try:
                await client.connect()
            except BleakError as e:
                print(e)
                await client.disconnect()


def ble_th(address: str):
    event_loop = asyncio.new_event_loop()
    future = asyncio.run_coroutine_threadsafe(ble_task(address), event_loop)
    return future.result(timeout=None)


class Worker(QThread):
    signal = data_read_signal = Signal(bytearray)

    def __init__(self, address: str):
        self.address = address
        super().__init__()

    def run(self):
        asyncio.run(ble_task(address))

    def notification_handler(self, characteristic: BleakGATTCharacteristic, data: bytearray):
        self.signal.emit(data)


if __name__ == '__main__':

    worker = Worker(address)
    worker.signal.connect(data_handler)

    worker.start()

    pg.exec()

    worker.terminate()
