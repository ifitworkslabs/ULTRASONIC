import serial
import numpy as np
from scipy.signal import hilbert
import struct
import multiprocessing
import queue
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

# ==========================================
# I. RADAR PHYSICS & RANGE CALIBRATION
# ==========================================
PORT = 'COM3'  
BAUD = 576000  

CALIB_TEMP = 30.21
CALIB_HUM = 70.84
SPEED_OF_SOUND = 331.4 + (0.606 * CALIB_TEMP) + (0.0124 * CALIB_HUM)
FREQUENCY = 40000.0          
WAVELENGTH = SPEED_OF_SOUND / FREQUENCY
NUM_CHANNELS = 5

MIC_POSITIONS = np.array([-0.025, -0.013, -0.002, 0.009, 0.025])

THETA_DEGREES = np.arange(-90, 91, 1)
THETA_RADIANS = np.radians(THETA_DEGREES)

STEERING_VECTORS = np.zeros((NUM_CHANNELS, len(THETA_RADIANS)), dtype=complex)
for i, theta in enumerate(THETA_RADIANS):
    spatial_phases = (MIC_POSITIONS / WAVELENGTH) * np.sin(theta)
    STEERING_VECTORS[:, i] = np.exp(-1j * 2 * np.pi * spatial_phases)

# TUNE THESE TO YOUR ROOM'S ACOUSTICS
STC_START_GAIN = 1.0
STC_END_GAIN = 1.0     
STC_POWER = 1.0        

# CORRECTED HARDWARE LIMITS
SAMPLE_RATE_PER_CH = 400000.0  # TRUE ESP32 LIMIT: 2MHz total / 5 channels = 400kHz
CAPTURE_OFFSET = 800           
HARDWARE_DELAY_SEC = 0.0008    
MAX_RADAR_RANGE = 2.0          
CROP_OFFSET = 0                

# THE DYNAMIC BLIND SPOT (Mutes the Screaming Transducer)
MIN_RADAR_RANGE = 0.65         

# ==========================================
# II. CORE 1: THE DEDICATED SERIAL WORKER
# ==========================================
def serial_worker(port, baud, data_queue):
    try:
        ser = serial.Serial(port, baud, timeout=2)
        print(">>> CORE 1: High-Speed Serial Worker Online. <<<")
    except Exception as e:
        print(f"CRITICAL ERROR IN WORKER: Failed to open port {port}. {e}")
        return

    while True:
        try:
            if ser.in_waiting > 1000:
                ser.reset_input_buffer()
            
            sync_buffer = b''
            while True:
                byte = ser.read(1)
                if not byte: break
                sync_buffer += byte
                if len(sync_buffer) == 4:
                    if sync_buffer == b'\xaa\xbb\xcc\xdd':
                        break
                    else:
                        sync_buffer = sync_buffer[1:]
                        
            if len(sync_buffer) < 4: continue 

            payload_bytes = ser.read(32)
            if len(payload_bytes) < 32: continue
            scan_angle, temp, hum, target_range, target_x, target_y, confidence, pad = struct.unpack('<ffffffff', payload_bytes)
            
            data_queue.put((scan_angle, temp, hum, target_range, target_x, target_y, confidence, pad))
            
        except Exception as e:
            print(f"Serial Worker Exception: {e}")
            break

# ==========================================
# III. CORE 2: C++ ACCELERATED UI ENGINE
# ==========================================
if __name__ == '__main__':
    print("==================================================")
    print("Professor's Target Tracking Engine Online (2 MHz).")
    print(">>> INSTANT VISUAL THRESHOLD PIN-LOCK ACTIVE <<<")
    print("==================================================\n")
    data_queue = multiprocessing.Queue(maxsize=100) 
    
    worker_process = multiprocessing.Process(
        target=serial_worker, 
        args=(PORT, BAUD, data_queue),
        daemon=True 
    )
    worker_process.start()

    pg.setConfigOptions(antialias=True)
    app = pg.mkQApp("Radar Engine")
    win = pg.GraphicsLayoutWidget(show=True, title="Phased Array 2D Command Center")
    win.resize(1400, 900)
    win.setBackground('#111111')
    
    # --- ENVIRONMENT INFO ---
    title_label = win.addLabel(
        f"Environment Calibration | Temp: {CALIB_TEMP} °C | Hum: {CALIB_HUM} % | SoS: {SPEED_OF_SOUND:.2f} m/s",
        row=0, col=0, colspan=2, size='14pt', color='#00FF00'
    )

    # --- LEFT COLUMN ---
    p_music = win.addPlot(row=1, col=0, title="MUSIC (Spatial Spectrum)")
    p_music.showGrid(x=True, y=True, alpha=0.3)
    p_music.setXRange(-90, 90)
    p_music.setYRange(0, 1.05)
    curve_music = p_music.plot(pen=pg.mkPen('g', width=2))
    scatter_music = pg.ScatterPlotItem(size=12, pen=pg.mkPen('w'), brush=pg.mkBrush('r'))
    p_music.addItem(scatter_music)
    beam_indicator_music = pg.InfiniteLine(angle=90, pen=pg.mkPen((0, 100, 255, 150), width=40))
    p_music.addItem(beam_indicator_music)
    
    # 1. THE VISUAL THRESHOLD LINE
    VISUAL_THRESHOLD = 0.6
    thresh_line = pg.InfiniteLine(angle=0, pos=VISUAL_THRESHOLD, pen=pg.mkPen('y', width=2, style=QtCore.Qt.DashLine))
    p_music.addItem(thresh_line)

    p_raw = win.addPlot(row=2, col=0, title="RAW ADC WAVEFORMS")
    p_raw.showGrid(x=True, y=True, alpha=0.3)
    p_raw.setYRange(-0.1, 0.1)
    channel_colors = [(0, 255, 255), (255, 0, 255), (255, 255, 0), (255, 0, 0), (0, 255, 0)]
    curves_raw = [p_raw.plot(pen=pg.mkPen(color, width=1.5, alpha=200)) for color in channel_colors]

    # --- RIGHT COLUMN ---
    p_radar = win.addPlot(row=1, col=1, rowspan=2, title="2D TACTICAL MAP")
    p_radar.setAspectLocked(True) 
    p_radar.showGrid(x=True, y=True, alpha=0.3)
    p_radar.setXRange(-MAX_RADAR_RANGE, MAX_RADAR_RANGE)
    p_radar.setYRange(0, MAX_RADAR_RANGE)
    p_radar.setLabel('bottom', 'Lateral Distance (m)')
    p_radar.setLabel('left', 'Forward Distance (m)')
    
    theta_ring = np.linspace(0, 2 * np.pi, 100)
    for r in [0.5, 1.0, 1.5, 2.0]:
        x_ring = r * np.sin(theta_ring)
        y_ring = r * np.cos(theta_ring)
        p_radar.plot(x_ring, y_ring, pen=pg.mkPen((255, 255, 255, 75), width=1, style=QtCore.Qt.DashLine))

    curve_radar = p_radar.plot(pen=pg.mkPen('g', width=3))
    scatter_radar = pg.ScatterPlotItem(size=16, pen=pg.mkPen('w'), brush=pg.mkBrush('r'))
    p_radar.addItem(scatter_radar)
    
    beam_cone_radar = p_radar.plot(pen=pg.mkPen((0, 100, 255, 100), width=2))
    
    # 2. THE TACTICAL PIN (Crosshair)
    target_pin = p_radar.plot(pen=pg.mkPen('r', width=3))
    pin_size = 0.15
    
    # 3. COVERAGE TRACE (Breadcrumbs)
    scatter_trace = pg.ScatterPlotItem(size=6, pen=pg.mkPen(None), brush=pg.mkBrush(255, 255, 0, 100))
    p_radar.addItem(scatter_trace)
    trace_memory = []
    
    # 4. PAUSE BUTTON
    trace_active = True
    pause_btn = QtWidgets.QPushButton("PAUSE / RESUME TRACE")
    pause_btn.setStyleSheet("font-size: 14px; font-weight: bold; background-color: #333; color: yellow; padding: 8px;")
    
    def toggle_trace():
        global trace_active
        trace_active = not trace_active
        if trace_active:
            pause_btn.setStyleSheet("font-size: 14px; font-weight: bold; background-color: #333; color: yellow; padding: 8px;")
        else:
            pause_btn.setStyleSheet("font-size: 14px; font-weight: bold; background-color: #500; color: red; padding: 8px;")
            
    pause_btn.clicked.connect(toggle_trace)
    proxy = QtWidgets.QGraphicsProxyWidget()
    proxy.setWidget(pause_btn)
    
    # Add to a new row at the bottom of the right column
    btn_layout = win.addLayout(row=3, col=1)
    btn_layout.addItem(proxy)
    
    active_targets = []
    target_text_items = []

    def update():
        global SPEED_OF_SOUND, active_targets, trace_memory, trace_active
        while not data_queue.empty():
            try:
                current_scan_angle, temp, hum, target_range, target_x, target_y, confidence, raw_energy = data_queue.get_nowait()
            except queue.Empty:
                break
                
            speed_of_sound = 331.4 + (0.606 * temp) + (0.0124 * hum)
            SPEED_OF_SOUND = speed_of_sound
            
            title_label.setText(f"Environment Calibration | Temp: {temp:.2f} °C | Hum: {hum:.2f} % | SoS: {SPEED_OF_SOUND:.2f} m/s")
                
            cone_min = current_scan_angle - 10
            cone_max = current_scan_angle + 10
            
            # =======================================================
            # SPATIAL TARGET CLUSTERING (NMS)
            # =======================================================
            # Decrease TTL for all existing targets
            for t in active_targets:
                t['ttl'] -= 1
            
            # Remove dead targets
            active_targets = [t for t in active_targets if t['ttl'] > 0]
            
            if confidence > VISUAL_THRESHOLD and target_range > 0:
                lock_angle_deg = np.degrees(np.arctan2(target_x, target_y))
                
                # Check if this detection belongs to an existing target (Dynamic radius based on range)
                # At 2 meters, angular error of 10 degrees is ~0.35m physically. 
                dynamic_merge_radius = 0.2 + (0.2 * target_range)
                
                merged = False
                for t in active_targets:
                    dist = np.hypot(t['tx'] - target_x, t['ty'] - target_y)
                    if dist < dynamic_merge_radius:
                        # Confidence-weighted tracking: strong peaks pull the track normally, 
                        # weak ghosts have almost zero effect, completely eliminating drift!
                        weight_ratio = (confidence / max(t['peak'], 0.1)) ** 2
                        alpha = 0.3 * min(weight_ratio, 1.0)
                        
                        t['tx'] = (1.0 - alpha) * t['tx'] + alpha * target_x
                        t['ty'] = (1.0 - alpha) * t['ty'] + alpha * target_y
                        t['range'] = (1.0 - alpha) * t['range'] + alpha * target_range
                        t['deg'] = (1.0 - alpha) * t['deg'] + alpha * lock_angle_deg
                        # EMA the peak confidence so it can slowly adapt down
                        t['peak'] = (0.9 * t['peak']) + (0.1 * confidence)
                        t['raw'] = (0.9 * t.get('raw', 0)) + (0.1 * raw_energy)
                        t['ttl'] = 15 # stay alive for 3 full 5-angle sweeps
                        t['hits'] += 1
                        merged = True
                        break
                
                if not merged:
                    active_targets.append({
                        'deg': lock_angle_deg,
                        'peak': confidence, 'range': target_range,
                        'tx': target_x, 'ty': target_y,
                        'ttl': 15,
                        'hits': 1,
                        'raw': raw_energy
                    })

                pass

            # =======================================================

            # --- RENDER GRAPHICS ---
            beam_indicator_music.setValue(current_scan_angle)

            # We can't plot raw waveforms anymore, just clear them
            for i in range(NUM_CHANNELS):
                curves_raw[i].setData([], [])

            # We can't plot the full spectrum anymore, just show a peak in the cone
            spectrum = np.ones(len(THETA_RADIANS)) * 0.0001
            if confidence > VISUAL_THRESHOLD:
                if target_range > 0:
                    lock_angle = np.degrees(np.arctan2(target_x, target_y))
                    peak_idx = np.argmin(np.abs(THETA_DEGREES - lock_angle))
                    spectrum[peak_idx] = confidence
                
            curve_music.setData(THETA_DEGREES, spectrum)

            x_radar = spectrum * MAX_RADAR_RANGE * np.sin(THETA_RADIANS)
            y_radar = spectrum * MAX_RADAR_RANGE * np.cos(THETA_RADIANS)
            curve_radar.setData(x_radar, y_radar)
            
            cone_x = [0, MAX_RADAR_RANGE * np.sin(np.radians(cone_max)), MAX_RADAR_RANGE * np.sin(np.radians(cone_min)), 0]
            cone_y = [0, MAX_RADAR_RANGE * np.cos(np.radians(cone_max)), MAX_RADAR_RANGE * np.cos(np.radians(cone_min)), 0]
            beam_cone_radar.setData(cone_x, cone_y)

            music_pts, radar_pts = [], []
            pin_x, pin_y = [], [] 
            
            # Make sure we have enough text items
            while len(target_text_items) < len(active_targets):
                t = pg.TextItem(text="", color=(255, 0, 0), anchor=(0.5, -0.5))
                p_radar.addItem(t)
                target_text_items.append(t)
                
            # Hide all text items initially
            for t in target_text_items:
                t.setText("")
            
            for i, data in enumerate(active_targets):
                if data['hits'] < 1: # Draw it immediately! The ESP32 already verified it.
                    continue
                    
                music_pts.append({'pos': (data['deg'], data['peak'])})
                tx, ty = data['tx'], data['ty']
                radar_pts.append({'pos': (tx, ty)})
                
                # --- Trace Breadcrumbs (Paint the Oval) ---
                if trace_active:
                    add_trace = True
                    if len(trace_memory) > 0:
                        last_tx, last_ty = trace_memory[-1]['pos']
                        if np.hypot(last_tx - tx, last_ty - ty) < 0.05:
                            add_trace = False # Only drop a crumb if moved 5cm
                    
                    if add_trace:
                        trace_memory.append({'pos': (tx, ty)})
                        if len(trace_memory) > 1000: # Cap at 1000 dots
                            trace_memory.pop(0)
                
                target_text_items[i].setText(f"{data['range']:.2f}m\nPeak: {data['peak']:.2f}\nRaw: {data.get('raw', 0):.2f}")
                target_text_items[i].setPos(tx, ty)
                
                # BUILD THE TACTICAL PIN CROSSHAIR
                pin_x.extend([
                    tx - pin_size, tx + pin_size, np.nan,
                    tx, tx, np.nan,
                    tx - pin_size, tx - pin_size, tx + pin_size, tx + pin_size, tx - pin_size, np.nan
                ])
                pin_y.extend([
                    ty, ty, np.nan,
                    ty - pin_size, ty + pin_size, np.nan,
                    ty - pin_size, ty + pin_size, ty + pin_size, ty - pin_size, ty - pin_size, np.nan
                ])

            scatter_music.setData(music_pts)
            scatter_radar.setData(radar_pts)
            scatter_trace.setData(trace_memory)
            
            # DRAW THE PIN ON THE MAP
            target_pin.setData(pin_x, pin_y)

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(0) 

    pg.exec()