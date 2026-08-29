import sys
import socket
import struct
import time
import queue
import numpy as np
from scipy.signal import hilbert
import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore
from multiprocessing import Process, Queue

# ==========================================
# I. RADAR PHYSICS & RANGE CALIBRATION
# ==========================================
UDP_IP = "0.0.0.0" 
UDP_PORT = 8888

FREQUENCY = 40000.0          
SPEED_OF_SOUND = 343.0       
WAVELENGTH = SPEED_OF_SOUND / FREQUENCY
NUM_CHANNELS = 5

MIC_POSITIONS = np.array([-0.025, -0.013, -0.002, 0.009, 0.025])

THETA_DEGREES = np.arange(-90, 91, 1)
THETA_RADIANS = np.radians(THETA_DEGREES)

SAMPLE_RATE_PER_CH = 400000.0  
CAPTURE_OFFSET = 1800          
CROP_OFFSET = 350              
HARDWARE_DELAY_SEC = 0.0008    
MAX_RADAR_RANGE = 1.5          
WINDOW_SIZE = 500    

# ==========================================
# II. THE MULTIPROCESSING WORKERS (CORES 1, 2, 3)
# ==========================================

def core_1_courier(raw_queue):
    """CORE 1: Dedicated network listener. Zero math, zero delays."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    
    local_buffer = {}
    local_dropped_frames = 0
    
    while True:
        packet, addr = sock.recvfrom(4096)
        
        if len(packet) != 1259 or packet[0:4] != b'\xaa\xbb\xcc\xdd':
            continue
            
        angle, chunk_idx = struct.unpack('<fB', packet[4:9])
        
        if angle not in local_buffer:
            local_buffer[angle] = {}
            
        local_buffer[angle][chunk_idx] = packet[9:]
        
        if len(local_buffer) > 2:
            local_dropped_frames += 1
            local_buffer.clear()
            
        if len(local_buffer.get(angle, {})) == 8:
            full_payload = b''.join([local_buffer[angle][i] for i in range(8)])
            del local_buffer[angle]
            raw_queue.put((local_dropped_frames, angle, full_payload))


def core_physicist(worker_id, raw_queue, gui_queue):
    """CORES 2 & 3: Twin mathematical processors. Pure Physics (No CMR)."""
    
    local_steering = np.zeros((NUM_CHANNELS, len(THETA_RADIANS)), dtype=complex)
    for i, theta in enumerate(THETA_RADIANS):
        spatial_phases = (MIC_POSITIONS / WAVELENGTH) * np.sin(theta)
        local_steering[:, i] = np.exp(-1j * 2 * np.pi * spatial_phases)
        
    while True:
        dropped_count, current_scan_angle, full_payload = raw_queue.get() 
        
        flat_array = np.frombuffer(full_payload, dtype=np.float32)
        if not np.isfinite(flat_array).all() or np.max(np.abs(flat_array)) > 2.0:
            continue 

        raw_matrix = flat_array.reshape(WINDOW_SIZE, 5).T 
        
        # 1. Remove DC Offset to center the waveform
        signal_matrix = raw_matrix - np.mean(raw_matrix, axis=1, keepdims=True)
        
        signal_energy = np.mean(np.var(signal_matrix, axis=1))
        
        spectrum = np.zeros(len(THETA_RADIANS))
        target_data = None
        
        if signal_energy > 0.000005: 
            complex_matrix = hilbert(signal_matrix, axis=1)
            Rxx = (complex_matrix @ complex_matrix.conj().T) / WINDOW_SIZE
            eigenvalues, eigenvectors = np.linalg.eigh(Rxx)
            idx = eigenvalues.argsort()[::-1]
            noise_subspace = eigenvectors[:, idx][:, 1:]
            En = noise_subspace @ noise_subspace.conj().T
            projection = np.sum(local_steering.conj() * (En @ local_steering), axis=0)
            spectrum = 1.0 / np.abs(projection)
            spectrum = spectrum / np.max(spectrum)
            
            cone_min, cone_max = current_scan_angle - 10, current_scan_angle + 10
            mask_outside_cone = (THETA_DEGREES < cone_min) | (THETA_DEGREES > cone_max)
            spectrum[mask_outside_cone] = 0.0001
            
            peak_value = np.max(spectrum)
            if peak_value > 0.80:
                target_idx = np.argmax(spectrum)
                
                echo_envelope = np.sum(np.abs(signal_matrix), axis=0)
                baseline_noise = np.mean(echo_envelope)
                ping_peak = np.max(echo_envelope)
                signal_to_noise_ratio = ping_peak / (baseline_noise + 1e-9) 
                
                # PAR filter remains at 2.5 to ensure we only grab sharp, physical echoes
                if signal_to_noise_ratio > 2.0: 
                    edge_threshold = ping_peak * 0.25 
                    peak_time_idx = np.argmax(echo_envelope > edge_threshold)
                    
                    time_of_flight = HARDWARE_DELAY_SEC + ((CAPTURE_OFFSET + CROP_OFFSET + peak_time_idx) / SAMPLE_RATE_PER_CH)
                    range_m = (time_of_flight * SPEED_OF_SOUND) / 2.0
                    
                    target_data = {
                        'deg': THETA_DEGREES[target_idx],
                        'rad': THETA_RADIANS[target_idx],
                        'peak': peak_value, 
                        'range': range_m
                    }
        
        # Pass the raw_matrix to the GUI so you can visually diagnose the pure hardware signals!
        gui_queue.put((dropped_count, current_scan_angle, raw_matrix, spectrum, target_data))


# ==========================================
# III. IGNITION SEQUENCE & GUI (CORE 4)
# ==========================================
if __name__ == '__main__':
    
    print("==================================================")
    print("Professor's Target Tracking Engine Online.")
    print("UI Upgrade: Y-Axis Locked for Diagnosis")
    print("Filter: 2.5x PAR Exorcism Active (Pure Physics)")
    print("==================================================\n")

    raw_queue = Queue()
    gui_queue = Queue()
    
    courier = Process(target=core_1_courier, args=(raw_queue,), daemon=True)
    physicist_A = Process(target=core_physicist, args=("Alpha", raw_queue, gui_queue), daemon=True)
    physicist_B = Process(target=core_physicist, args=("Beta", raw_queue, gui_queue), daemon=True)
    
    courier.start()
    physicist_A.start()
    physicist_B.start()
    
    app = QtWidgets.QApplication(sys.argv)
    win = pg.GraphicsLayoutWidget(show=True, title="Phased Array 2D Command Center")
    win.resize(1400, 800)
    win.setBackground('#050505')

    p_music = win.addPlot(row=0, col=0, title="Live M.U.S.I.C. Spatial Spectrum")
    p_music.setXRange(-90, 90)
    p_music.setYRange(0, 1.05)
    p_music.showGrid(x=True, y=True, alpha=0.3)
    curve_music = p_music.plot(pen=pg.mkPen('g', width=2))
    scatter_music = p_music.plot(pen=None, symbol='o', symbolBrush='r', symbolSize=10)

    # --- THE CURE FOR THE SHAKY CAMERA ---
    p_raw = win.addPlot(row=1, col=0, title="Raw Acoustic Waveforms (Locked Y-Axis)")
    p_raw.setXRange(0, WINDOW_SIZE)
    p_raw.setYRange(-0.08, 0.08) # <-- Y-Axis is permanently frozen!
    p_raw.setMouseEnabled(x=False, y=False) # Disables accidental mouse dragging
    p_raw.showGrid(x=True, y=True, alpha=0.3)
    # -------------------------------------
    
    channel_colors = ['c', 'm', 'y', 'r', 'g']
    raw_curves = [p_raw.plot(pen=pg.mkPen(color, width=1.5)) for color in channel_colors]

    p_radar = win.addPlot(row=0, col=1, rowspan=2, title="Tactical 2D Radar Map")
    p_radar.setAspectLocked(True)
    p_radar.setXRange(-MAX_RADAR_RANGE, MAX_RADAR_RANGE)
    p_radar.setYRange(0, MAX_RADAR_RANGE)
    p_radar.hideAxis('bottom')
    p_radar.hideAxis('left')

    for r in [0.5, 1.0, 1.5]:
        circle = QtWidgets.QGraphicsEllipseItem(-r, -r, r*2, r*2)
        circle.setPen(pg.mkPen('#333333', width=1, style=QtCore.Qt.DashLine))
        p_radar.addItem(circle)
        text = pg.TextItem(f"{r}m", color='#555555')
        text.setPos(0, r)
        p_radar.addItem(text)

    for deg in [-60, -30, 0, 30, 60]:
        rad = np.radians(deg)
        x = MAX_RADAR_RANGE * np.sin(rad)
        y = MAX_RADAR_RANGE * np.cos(rad)
        line = QtWidgets.QGraphicsLineItem(0, 0, x, y)
        line.setPen(pg.mkPen('#333333', width=1, style=QtCore.Qt.DashLine))
        p_radar.addItem(line)

    curve_radar = p_radar.plot(pen=pg.mkPen('g', width=3))
    scatter_radar = p_radar.plot(pen=None, symbol='o', symbolBrush='r', symbolSize=14)

    target_memory = {}
    target_texts = {}
    angles_tracked = set()
    
    last_sweep_time = time.time()
    sweep_history = []           
    max_sweep_time_ms = 0.0      
    
    previous_total_dropped = 0   
    max_dropped_per_sweep = 0    
    
    telemetry_text = pg.TextItem(text="Initializing Statistical Telemetry...", color='#FFFF00')
    p_radar.addItem(telemetry_text)
    telemetry_text.setPos(-MAX_RADAR_RANGE, MAX_RADAR_RANGE * 0.95)

    def update_radar_gui():
        global last_sweep_time, angles_tracked
        global sweep_history, max_sweep_time_ms
        global previous_total_dropped, max_dropped_per_sweep
        
        try:
            while True:
                total_dropped_frames, angle, raw_matrix, spectrum, target_data = gui_queue.get_nowait()
                
                target_memory[angle] = target_data
                angles_tracked.add(angle)
                
                if len(angles_tracked) >= 5:
                    now = time.time()
                    true_sweep_ms = (now - last_sweep_time) * 1000.0
                    
                    if true_sweep_ms > max_sweep_time_ms:
                        max_sweep_time_ms = true_sweep_ms
                        
                    sweep_history.append(true_sweep_ms)
                    if len(sweep_history) > 50:
                        sweep_history.pop(0)
                    avg_sweep_ms = sum(sweep_history) / len(sweep_history)
                    
                    loss_this_sweep = total_dropped_frames - previous_total_dropped
                    previous_total_dropped = total_dropped_frames
                    
                    if loss_this_sweep > max_dropped_per_sweep:
                        max_dropped_per_sweep = loss_this_sweep
                    
                    telemetry_string = (
                        f"Sweep Speed : {true_sweep_ms:.1f} ms\n"
                        f"Average     : {avg_sweep_ms:.1f} ms\n"
                        f"Worst Sweep : {max_sweep_time_ms:.1f} ms\n"
                        f"---------------------------\n"
                        f"Current Loss: {loss_this_sweep} frames\n"
                        f"Worst Loss  : {max_dropped_per_sweep} frames"
                    )
                    telemetry_text.setText(telemetry_string)
                    
                    last_sweep_time = now
                    angles_tracked.clear()

                active_degs, active_peaks = [], []
                active_xs, active_ys = [], []

                for sector, data in target_memory.items():
                    if sector not in target_texts:
                        target_texts[sector] = pg.TextItem(color='r', anchor=(0.5, 1))
                        p_radar.addItem(target_texts[sector])

                    if data is not None:
                        active_degs.append(data['deg'])
                        active_peaks.append(data['peak'])
                        x_val = data['range'] * np.sin(data['rad'])
                        y_val = data['range'] * np.cos(data['rad'])
                        active_xs.append(x_val)
                        active_ys.append(y_val)
                        target_texts[sector].setText(f"{data['range']:.2f}m")
                        target_texts[sector].setPos(x_val, y_val + 0.08)
                    else:
                        target_texts[sector].setText("")

                curve_music.setData(THETA_DEGREES, spectrum)
                scatter_music.setData(active_degs, active_peaks)
                
                x_axis = np.arange(WINDOW_SIZE)
                for i in range(NUM_CHANNELS):
                    raw_curves[i].setData(x_axis, raw_matrix[i])
                
                sweep_radii = spectrum * MAX_RADAR_RANGE
                radar_x = sweep_radii * np.sin(THETA_RADIANS)
                radar_y = sweep_radii * np.cos(THETA_RADIANS)
                
                curve_radar.setData(radar_x, radar_y)
                scatter_radar.setData(active_xs, active_ys)

        except queue.Empty:
            pass 

    timer = QtCore.QTimer()
    timer.timeout.connect(update_radar_gui)
    timer.start(16) 
    
    sys.exit(app.exec_())