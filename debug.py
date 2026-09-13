import serial
import struct
import multiprocessing
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets
import sys
import time
import math

BAUD = 576000
PORT = 'COM3'

def serial_worker(q):
    ser = serial.Serial(PORT, BAUD, timeout=1.0)
    print("Listening to ESP32 Ultra-Fast Radar...")

    sync_pattern = b'\xaa\xbb\xcc\xdd'

    batch = []
    while True:
        try:
            # Sync to header using highly optimized PySerial C-backend
            ser.read_until(sync_pattern)

            payload = ser.read(16)
            if len(payload) == 16:
                scan_angle, target_angle, distance, strength = struct.unpack('<ffff', payload)
                batch.append((scan_angle, target_angle, distance, strength))
                
                # Send data in batches to drastically reduce multiprocessing IPC overhead
                if len(batch) >= 10:
                    q.put(batch)
                    batch = []
                
        except Exception as e:
            print(f"Serial Error: {e}")
            break

if __name__ == '__main__':
    q = multiprocessing.Queue()
    p = multiprocessing.Process(target=serial_worker, args=(q,))
    p.daemon = True
    p.start()

    app = pg.mkQApp("TWS Radar")

    # 1. TACTICAL MAP SETUP
    win = pg.GraphicsLayoutWidget(show=True, title="TWS Fast Sweep")
    win.resize(1000, 800)
    p_radar = win.addPlot(title="2D Tactical Map")
    p_radar.setAspectLocked(True)
    p_radar.setXRange(-2.0, 2.0)
    p_radar.setYRange(0, 2.5)
    p_radar.showGrid(x=True, y=True, alpha=0.3)

    # Draw Range Rings
    for r in [0.5, 1.0, 1.5, 2.0]:
        circle = QtWidgets.QGraphicsEllipseItem(-r, -r, r*2, r*2)
        circle.setPen(pg.mkPen('g', width=1, style=QtCore.Qt.DashLine))
        p_radar.addItem(circle)

    # Radar Sweep Arm
    arm_line = pg.PlotDataItem([0, 0], [0, 2.0], pen=pg.mkPen((0, 255, 0, 150), width=3))
    p_radar.addItem(arm_line)

    # Target Scatter (History/Trail)
    target_history = []
    
    scatter_targets = pg.ScatterPlotItem(
        size=15, 
        pen=pg.mkPen(None), 
        brush=pg.mkBrush(255, 0, 0, 200),
        symbol='o'
    )
    p_radar.addItem(scatter_targets)

    # Fading trail
    scatter_trail = pg.ScatterPlotItem(
        size=10, 
        pen=pg.mkPen(None), 
        brush=pg.mkBrush(255, 100, 100, 80),
        symbol='o'
    )
    p_radar.addItem(scatter_trail)

    def update():
        global target_history
        
        latest_angle = None

        import queue
        while True:
            try:
                batch = q.get_nowait()
                for scan_angle, target_angle, distance, strength in batch:
                    latest_angle = scan_angle
                    
                    # If distance != -1.0, we have a valid target!
                    if distance > 0:
                        rad = math.radians(target_angle)
                        x = distance * math.sin(rad)
                        y = distance * math.cos(rad)
                        
                        # Add to history (x, y, time)
                        target_history.append((x, y, time.time()))
            except queue.Empty:
                break

        # Update Sweep Arm
        if latest_angle is not None:
            rad = math.radians(latest_angle)
            arm_x = 2.5 * math.sin(rad)
            arm_y = 2.5 * math.cos(rad)
            arm_line.setData([0, arm_x], [0, arm_y])

        # Age and prune history
        current_time = time.time()
        # Keep dots that are less than 2.0 seconds old
        target_history = [t for t in target_history if (current_time - t[2]) < 2.0]

        # Split into main targets (very recent) and trail (older)
        main_pts = []
        trail_pts = []
        
        for t in target_history:
            age = current_time - t[2]
            if age < 0.2:
                main_pts.append({'pos': (t[0], t[1])})
            else:
                # Fade alpha based on age (0.2 to 2.0 sec)
                alpha = max(20, int(150 * (1.0 - (age / 2.0))))
                trail_pts.append({
                    'pos': (t[0], t[1]),
                    'brush': pg.mkBrush(255, 100, 100, alpha)
                })

        scatter_targets.setData(main_pts)
        scatter_trail.setData(trail_pts)

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(30) # ~33fps UI update to prevent Qt rendering lag

    pg.exec()