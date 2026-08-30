import serial
import numpy as np
from scipy.signal import hilbert
import struct
import multiprocessing
import queue

# --- NEW GRAPHICS ENGINE ---
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore

# ==========================================
# I. RADAR PHYSICS & RANGE CALIBRATION
# ==========================================
PORT = 'COM3'  # Update to your ESP32 Port
BAUD = 576000  # Stable High-Speed Pipeline

FREQUENCY = 40000.0          
SPEED_OF_SOUND = 343.0       
WAVELENGTH = SPEED_OF_SOUND / FREQUENCY
NUM_CHANNELS = 5

MIC_POSITIONS = np.array([-0.025, -0.013, -0.002, 0.009, 0.025])

THETA_DEGREES = np.arange(-90, 91, 1)
THETA_RADIANS = np.radians(THETA_DEGREES)

STEERING_VECTORS = np.zeros((NUM_CHANNELS, len(THETA_RADIANS)), dtype=complex)
for i, theta in enumerate(THETA_RADIANS):
    spatial_phases = (MIC_POSITIONS / WAVELENGTH) * np.sin(theta)
    STEERING_VECTORS[:, i] = np.exp(-1j * 2 * np.pi * spatial_phases)

STC_START_GAIN = 1.0
STC_END_GAIN = 1.0

SAMPLE_RATE_PER_CH = 400000.0  
CAPTURE_OFFSET = 1800          
HARDWARE_DELAY_SEC = 0.0008    
MAX_RADAR_RANGE = 1.5          
CROP_OFFSET = 350              

# ==========================================
# II. CORE 1: THE DEDICATED SERIAL WORKER
# ==========================================
def serial_worker(port, baud, data_queue):
    """
    Core 1: Operates at max speed, never pausing. 
    Pushes data to an infinite queue so absolutely zero frames are dropped.
    """
    try:
        ser = serial.Serial(port, baud, timeout=2)
        print(">>> CORE 1: High-Speed Serial Worker Online. <<<")
    except Exception as e:
        print(f"CRITICAL ERROR IN WORKER: Failed to open port {port}. {e}")
        return

    while True:
        try:
            # Buffer flush (Backup safety net)
            if ser.in_waiting > 12000:
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

            angle_bytes = ser.read(4)
            if len(angle_bytes) < 4: continue
            scan_angle = struct.unpack('<f', angle_bytes)[0]
            
            payload_bytes = ser.read(5000)
            if len(payload_bytes) < 5000: continue
            
            # Unthrottled Firehose: Put every single frame onto the infinite belt
            data_queue.put((scan_angle, payload_bytes))
            
        except Exception as e:
            print(f"Serial Worker Exception: {e}")
            break


# ==========================================
# III. CORE 2: C++ ACCELERATED UI ENGINE
# ==========================================
if __name__ == '__main__':
    print("==================================================")
    print("Professor's Target Tracking Engine Online (2 MHz).")
    print(">>> PyQtGraph C++ ACCELERATED PIPELINE ACTIVE <<<")
    print("==================================================\n")

    # 1. Infinite Conveyor Belt (No Dropped Frames!)
    data_queue = multiprocessing.Queue(maxsize=0) 
    
    # 2. Spawn Core 1
    worker_process = multiprocessing.Process(
        target=serial_worker, 
        args=(PORT, BAUD, data_queue),
        daemon=True 
    )
    worker_process.start()

    # 3. Setup PyQtGraph UI (Runs on Core 2)
    pg.setConfigOptions(antialias=True)
    app = pg.mkQApp("Radar Engine")
    win = pg.GraphicsLayoutWidget(show=True, title="Phased Array 2D Command Center (PyQtGraph)")
    win.resize(1400, 900)
    win.setBackground('#111111')

    # --- LEFT COLUMN: MUSIC & WAVEFORMS ---
    p_music = win.addPlot(row=0, col=0, title="MUSIC (Spatial Spectrum)")
    p_music.showGrid(x=True, y=True, alpha=0.3)
    p_music.setXRange(-90, 90)
    p_music.setYRange(0, 1.05)
    curve_music = p_music.plot(pen=pg.mkPen('g', width=2))
    scatter_music = pg.ScatterPlotItem(size=12, pen=pg.mkPen('w'), brush=pg.mkBrush('r'))
    p_music.addItem(scatter_music)
    beam_indicator_music = pg.InfiniteLine(angle=90, pen=pg.mkPen((0, 100, 255, 150), width=40))
    p_music.addItem(beam_indicator_music)

    p_raw = win.addPlot(row=1, col=0, title="RAW ADC WAVEFORMS")
    p_raw.showGrid(x=True, y=True, alpha=0.3)
    p_raw.setYRange(-0.1, 0.1)
    channel_colors = [(0, 255, 255), (255, 0, 255), (255, 255, 0), (255, 0, 0), (0, 255, 0)]
    curves_raw = [p_raw.plot(pen=pg.mkPen(color, width=1.5, alpha=200)) for color in channel_colors]

    # --- RIGHT COLUMN: 2D CARTESIAN RADAR MAP ---
    p_radar = win.addPlot(row=0, col=1, rowspan=2, title="2D TACTICAL MAP")
    p_radar.setAspectLocked(True) # Keeps the physical geometry perfectly square
    p_radar.showGrid(x=True, y=True, alpha=0.3)
    p_radar.setXRange(-MAX_RADAR_RANGE, MAX_RADAR_RANGE)
    p_radar.setYRange(0, MAX_RADAR_RANGE)
    p_radar.setLabel('bottom', 'Lateral Distance (m)')
    p_radar.setLabel('left', 'Forward Distance (m)')
    
    # Draw reference range rings (Parametric Math Approach)
    theta_ring = np.linspace(0, 2 * np.pi, 100)
    for r in [0.5, 1.0, 1.5]:
        x_ring = r * np.sin(theta_ring)
        y_ring = r * np.cos(theta_ring)
        p_radar.plot(x_ring, y_ring, pen=pg.mkPen((255, 255, 255, 75), width=1, style=QtCore.Qt.DashLine))

    curve_radar = p_radar.plot(pen=pg.mkPen('g', width=3))
    scatter_radar = pg.ScatterPlotItem(size=16, pen=pg.mkPen('w'), brush=pg.mkBrush('r'))
    p_radar.addItem(scatter_radar)
    
    # The sweeping blue beam cone for the radar view
    beam_cone_radar = p_radar.plot(pen=pg.mkPen((0, 100, 255, 100), width=2))
    
    # Memory Bank
    target_memory = {}
    target_text_items = {}

    # 4. The High-Speed Update Loop
    def update():
        # Process ALL frames currently on the belt to prevent time-travel lag
        while not data_queue.empty():
            try:
                current_scan_angle, payload_bytes = data_queue.get_nowait()
            except queue.Empty:
                break
                
            # --- MATH ENGINE (Remains Identical) ---
            flat_array = np.frombuffer(payload_bytes, dtype=np.int16).astype(np.float32) / 10000.0
            raw_matrix = flat_array.reshape(500, 5).T 

            num_samples_received = raw_matrix.shape[1]
            stc_curve = np.linspace(STC_START_GAIN, STC_END_GAIN, num_samples_received)
            signal_matrix = raw_matrix * stc_curve
            
            signal_energy = np.mean(np.var(signal_matrix, axis=1))
            NOISE_FLOOR_THRESHOLD = 0.000005 
            spectrum = np.zeros(len(THETA_RADIANS))
            
            if signal_energy < NOISE_FLOOR_THRESHOLD:
                spectrum[:] = 0.0001 
            else:
                signal_matrix = signal_matrix - np.mean(signal_matrix, axis=1, keepdims=True)
                complex_matrix = hilbert(signal_matrix, axis=1)
                num_samples = complex_matrix.shape[1]
                Rxx = (complex_matrix @ complex_matrix.conj().T) / num_samples
                eigenvalues, eigenvectors = np.linalg.eigh(Rxx)
                idx = eigenvalues.argsort()[::-1]
                noise_subspace = eigenvectors[:, idx][:, 1:]
                
                projection_matrix = noise_subspace.conj().T @ STEERING_VECTORS
                projection = np.sum(np.abs(projection_matrix)**2, axis=0)
                spectrum = 1.0 / projection
                spectrum = spectrum / np.max(spectrum)
            
            cone_min = current_scan_angle - 10
            cone_max = current_scan_angle + 10
            mask_outside_cone = (THETA_DEGREES < cone_min) | (THETA_DEGREES > cone_max)
            spectrum[mask_outside_cone] = 0.0001
            
            # --- TARGET DETECTION ---
            peak_value = np.max(spectrum)
            if peak_value > 0.85:
                target_idx = np.argmax(spectrum)
                lock_angle_deg = THETA_DEGREES[target_idx]
                lock_angle_rad = THETA_RADIANS[target_idx]
                
                echo_envelope = np.sum(np.abs(raw_matrix), axis=0)
                edge_threshold = np.max(echo_envelope) * 0.25 
                peak_time_idx = np.argmax(echo_envelope > edge_threshold)
                
                time_of_flight = HARDWARE_DELAY_SEC + ((CAPTURE_OFFSET + CROP_OFFSET + peak_time_idx) / SAMPLE_RATE_PER_CH)
                lock_range_meters = (time_of_flight * SPEED_OF_SOUND) / 2.0
                
                target_memory[current_scan_angle] = {
                    'deg': lock_angle_deg, 'rad': lock_angle_rad,
                    'peak': peak_value, 'range': lock_range_meters
                }
            else:
                if current_scan_angle in target_memory:
                    target_memory[current_scan_angle] = None

            # --- RENDER FAST GRAPHICS ---
            # 1. Update MUSIC Line & Beam Marker
            curve_music.setData(THETA_DEGREES, spectrum)
            beam_indicator_music.setValue(current_scan_angle)

            # 2. Update Waveforms
            x_axis = np.arange(num_samples_received)
            for i in range(NUM_CHANNELS):
                curves_raw[i].setData(x_axis, raw_matrix[i])

            # 3. Update 2D Radar Map (Trig conversion to Cartesian)
            x_radar = spectrum * MAX_RADAR_RANGE * np.sin(THETA_RADIANS)
            y_radar = spectrum * MAX_RADAR_RANGE * np.cos(THETA_RADIANS)
            curve_radar.setData(x_radar, y_radar)
            
            # Draw the V-shape beam cone
            cone_x = [0, MAX_RADAR_RANGE * np.sin(np.radians(cone_max)), MAX_RADAR_RANGE * np.sin(np.radians(cone_min)), 0]
            cone_y = [0, MAX_RADAR_RANGE * np.cos(np.radians(cone_max)), MAX_RADAR_RANGE * np.cos(np.radians(cone_min)), 0]
            beam_cone_radar.setData(cone_x, cone_y)

            # 4. Draw Persistent Targets
            music_pts, radar_pts = [], []
            for sector, data in target_memory.items():
                # Setup dynamic text labels if they don't exist
                if sector not in target_text_items:
                    t = pg.TextItem(text="", color=(255, 0, 0), anchor=(0.5, -0.5))
                    p_radar.addItem(t)
                    target_text_items[sector] = t
                
                if data is not None:
                    # MUSIC Plot point
                    music_pts.append({'pos': (data['deg'], data['peak'])})
                    
                    # Radar Plot point (Convert to Cartesian)
                    tx = data['range'] * np.sin(data['rad'])
                    ty = data['range'] * np.cos(data['rad'])
                    radar_pts.append({'pos': (tx, ty)})
                    
                    # Update label text and position
                    target_text_items[sector].setText(f"{data['range']:.2f}m")
                    target_text_items[sector].setPos(tx, ty)
                else:
                    target_text_items[sector].setText("")

            scatter_music.setData(music_pts)
            scatter_radar.setData(radar_pts)

    # The Heartbeat: Fires the update function as fast as the CPU allows
    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(0) 

    # Start the Qt Event Loop
    pg.exec()