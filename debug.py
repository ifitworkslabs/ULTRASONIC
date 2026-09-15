import serial
import numpy as np
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

# TRACKING TUNING (Controls how easily new radar pings snap to existing targets)
MERGE_RADIUS_BASE = 0.005
MERGE_RADIUS_SCALE = 0.00
TARGET_TTL_PINGS = 40  # 5 pings = 1 full frame/sweep. 10 pings = lingers for 2 frames when trace is lost

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
    print(">>> MINIMALIST TELEMETRY UI ACTIVE <<<")
    print("==================================================\n")
    data_queue = multiprocessing.Queue(maxsize=100) 
    
    worker_process = multiprocessing.Process(
        target=serial_worker, 
        args=(PORT, BAUD, data_queue),
        daemon=True 
    )
    worker_process.start()

    # 1. GLOBAL TYPOGRAPHY & COLORS
    pg.setConfigOptions(antialias=True, foreground='#8C92AC')
    app = pg.mkQApp("Radar Engine")
    win = pg.GraphicsLayoutWidget(show=True, title="Aero-Acoustic Command Center")
    win.resize(1400, 900)
    win.setBackground('#0B0D14') # Deep matte slate
    
    # --- ENVIRONMENT INFO ---
    title_label = win.addLabel(
        f"SCANNING MODE // Temp: {CALIB_TEMP}°C | Hum: {CALIB_HUM}% | SoS: {SPEED_OF_SOUND:.2f} m/s",
        row=0, col=0, colspan=2, size='12pt', color='#545E75', bold=True
    )

    # --- LEFT COLUMN: SPATIAL SPECTRUM ---
    p_music = win.addPlot(row=1, col=0, title="SPATIAL SPECTRUM")
    p_music.showGrid(x=True, y=True, alpha=0.1) # Barely visible grid
    p_music.setXRange(-90, 90)
    p_music.setYRange(0, 1.05)
    p_music.hideAxis('right')
    p_music.hideAxis('top')
    
    # The pure cyan data line
    curve_music = p_music.plot(pen=pg.mkPen('#00E5FF', width=1.5)) 
    scatter_music = pg.ScatterPlotItem(size=10, pen=pg.mkPen('#FFFFFF', width=2), brush=pg.mkBrush('#00E5FF'))
    p_music.addItem(scatter_music)
    
    # Faint sweeping beam indicator
    beam_indicator_music = pg.InfiniteLine(angle=90, pen=pg.mkPen((0, 229, 255, 20), width=40))
    p_music.addItem(beam_indicator_music)
    
    # Subtle threshold line
    VISUAL_THRESHOLD = 0.7
    thresh_line = pg.InfiniteLine(angle=0, pos=VISUAL_THRESHOLD, pen=pg.mkPen('#2A3143', width=2, style=QtCore.Qt.DashLine))
    p_music.addItem(thresh_line)

    # --- RIGHT COLUMN: TACTICAL MAP ---
    p_radar = win.addPlot(row=1, col=1, title="2D TACTICAL MAP")
    p_radar.setAspectLocked(True) 
    p_radar.showGrid(x=False, y=False) # Turn off square grids!
    p_radar.setXRange(-MAX_RADAR_RANGE, MAX_RADAR_RANGE)
    p_radar.setYRange(0, MAX_RADAR_RANGE)
    p_radar.setLabel('bottom', 'Lateral Distance (m)')
    p_radar.setLabel('left', 'Forward Distance (m)')
    p_radar.hideAxis('right')
    p_radar.hideAxis('top')
    
    # 2. THE MINIMALIST RADAR RINGS
    theta_ring = np.linspace(0, 2 * np.pi, 100)
    for r in [0.5, 1.0, 1.5, 2.0]:
        x_ring = r * np.sin(theta_ring)
        y_ring = r * np.cos(theta_ring)
        p_radar.plot(x_ring, y_ring, pen=pg.mkPen('#1C2233', width=1.5)) # Clean slate rings

    curve_radar = p_radar.plot(pen=pg.mkPen((0, 229, 255, 100), width=2))
    
    # The Verified Target (Bright white core, electric cyan border)
    scatter_radar = pg.ScatterPlotItem(size=12, pen=pg.mkPen('#FFFFFF', width=2), brush=pg.mkBrush('#00E5FF'))
    p_radar.addItem(scatter_radar)
    
    # Ultra-faint beam cone
    beam_cone_radar = p_radar.plot(pen=pg.mkPen((0, 229, 255, 40), width=1, style=QtCore.Qt.DashLine))
    
    active_targets = []
    target_text_items = []

    def update():
        global SPEED_OF_SOUND, active_targets
        
        processed_any = False
        current_scan_angle = 0
        temp = 0
        hum = 0
        target_range = 0
        target_x = 0
        target_y = 0
        confidence = 0
        raw_energy = 0
        
        while not data_queue.empty():
            try:
                current_scan_angle, temp, hum, target_range, target_x, target_y, confidence, raw_energy = data_queue.get_nowait()
            except queue.Empty:
                break
                
            speed_of_sound = 331.4 + (0.606 * temp) + (0.0124 * hum)
            SPEED_OF_SOUND = speed_of_sound
            processed_any = True
                
            cone_min = current_scan_angle - 10
            cone_max = current_scan_angle + 10
            
            # =======================================================
            # SPATIAL TARGET CLUSTERING & TRACKING
            # =======================================================
            # Decrease TTL for all existing targets
            for t in active_targets:
                t['ttl'] -= 1
            
            # Remove dead targets
            active_targets = [t for t in active_targets if t['ttl'] > 0]
            
            if confidence > VISUAL_THRESHOLD and target_range > MIN_RADAR_RANGE:
                lock_angle_deg = np.degrees(np.arctan2(target_x, target_y))
                
                # Check if this detection belongs to an existing target (Dynamic radius based on range)
                # At 2 meters, angular error of 10 degrees is ~0.35m physically. 
                dynamic_merge_radius = MERGE_RADIUS_BASE + (MERGE_RADIUS_SCALE * target_range)
                
                merged = False
                for t in active_targets:
                    dist = np.hypot(t['tx'] - target_x, t['ty'] - target_y)
                    if dist < dynamic_merge_radius:
                        # Confidence-weighted tracking: strong peaks pull the track normally, 
                        # weak ghosts have almost zero effect, completely eliminating drift!
                        weight_ratio = (confidence / max(t['peak'], 0.1)) ** 2
                        alpha = 0.3 * min(weight_ratio, 1.0)
                        
                        # Measurement residual (error)
                        res_x = target_x - t['tx']
                        res_y = target_y - t['ty']
                        
                        # State Update
                        t['tx'] += alpha * res_x
                        t['ty'] += alpha * res_y
                        
                        t['range'] = np.hypot(t['tx'], t['ty'])
                        t['deg'] = np.degrees(np.arctan2(t['tx'], t['ty']))
                        
                        # EMA the peak confidence so it can slowly adapt down
                        t['peak'] = (0.9 * t['peak']) + (0.1 * confidence)
                        t['raw'] = (0.9 * t.get('raw', 0)) + (0.1 * raw_energy)
                        t['ttl'] = TARGET_TTL_PINGS 
                        t['hits'] += 1
                        merged = True
                        break
                
                if not merged:
                    active_targets.append({
                        'deg': lock_angle_deg,
                        'peak': confidence, 'range': target_range,
                        'tx': target_x, 'ty': target_y,
                        'ttl': TARGET_TTL_PINGS,
                        'hits': 1,
                        'raw': raw_energy
                    })

                pass

            # =======================================================

        # =======================================================
        # --- RENDER GRAPHICS (RUNS ONCE PER GUI TICK) ---
        # =======================================================
        if not processed_any:
            return
            
        title_label.setText(f"Environment Calibration | Temp: {temp:.2f} °C | Hum: {hum:.2f} % | SoS: {SPEED_OF_SOUND:.2f} m/s")
            
        cone_min = current_scan_angle - 10
        cone_max = current_scan_angle + 10
        beam_indicator_music.setValue(current_scan_angle)

        # We can't plot the full spectrum anymore, just show a peak in the cone
        spectrum = np.ones(len(THETA_RADIANS)) * 0.0001
        if confidence > VISUAL_THRESHOLD:
            if target_range > MIN_RADAR_RANGE:
                lock_angle = np.degrees(np.arctan2(target_x, target_y))
                peak_idx = int(round(lock_angle)) + 90
                peak_idx = max(0, min(180, peak_idx)) # Safety clamp
                spectrum[peak_idx] = confidence
            
        curve_music.setData(THETA_DEGREES, spectrum)

        x_radar = spectrum * MAX_RADAR_RANGE * np.sin(THETA_RADIANS)
        y_radar = spectrum * MAX_RADAR_RANGE * np.cos(THETA_RADIANS)
        curve_radar.setData(x_radar, y_radar)
        
        cone_x = [0, MAX_RADAR_RANGE * np.sin(np.radians(cone_max)), MAX_RADAR_RANGE * np.sin(np.radians(cone_min)), 0]
        cone_y = [0, MAX_RADAR_RANGE * np.cos(np.radians(cone_max)), MAX_RADAR_RANGE * np.cos(np.radians(cone_min)), 0]
        beam_cone_radar.setData(cone_x, cone_y)

        music_pts, radar_pts = [], []
        
        # Make sure we have enough text items
        while len(target_text_items) < len(active_targets):
            t = pg.TextItem(text="", color='#00E5FF', anchor=(0.5, -0.5))
            p_radar.addItem(t)
            target_text_items.append(t)
            
        # Hide all text items initially
        for t in target_text_items:
            t.setText("")
        
        for i, data in enumerate(active_targets):
            # Dynamic Track Initiation: Stricter at edges (ghost-prone), instant at center
            abs_deg = abs(data['deg'])
            if abs_deg > 30:
                req_hits = 5   # Extreme angles (+/- 40) need heavy verification
            elif abs_deg > 10:
                req_hits = 5   # Mid angles (+/- 20) need some verification
            else:
                req_hits = 3   # Center angle (0) is trusted instantly
            
            # If it hasn't been seen enough times, keep it invisible in the background
            if data['hits'] < req_hits: 
                continue
                
            music_pts.append({'pos': (data['deg'], data['peak'])})
            tx, ty = data['tx'], data['ty']
            radar_pts.append({'pos': (tx, ty)})
            
            target_text_items[i].setText(f"{data['range']:.2f}m\nPeak: {data['peak']:.2f}\nRaw: {data.get('raw', 0):.2f}")
            target_text_items[i].setPos(tx, ty)
            
        scatter_music.setData(music_pts)
        scatter_radar.setData(radar_pts)

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(0) 

    pg.exec()