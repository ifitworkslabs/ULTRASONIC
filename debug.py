import serial
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import hilbert

# ==========================================
# I. RADAR PHYSICS & RANGE CALIBRATION
# ==========================================
PORT = 'COM3'  # Update to your ESP32 Port
BAUD = 115200  # Stable High-Speed Pipeline

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

# 2D TARGET MAPPING CONSTANTS 
SAMPLE_RATE_PER_CH = 400000.0  
CAPTURE_OFFSET = 1800          
HARDWARE_DELAY_SEC = 0.0008    
MAX_RADAR_RANGE = 1.5          
CROP_OFFSET = 350              # Compensates for the C++ payload reduction

# ==========================================
# II. INITIALIZE HARDWARE STREAM & TACTICAL UI
# ==========================================
print("==================================================")
print("Professor's Target Tracking Engine Online (2 MHz).")
print(">>> HIGH-SPEED TWS MEMORY & RANGE LOCK ACTIVE <<<")
print("==================================================\n")

try:
    ser = serial.Serial(PORT, BAUD, timeout=2)
except Exception as e:
    print(f"CRITICAL ERROR: Failed to open port {PORT}. {e}")
    exit()

plt.ion()
fig = plt.figure(figsize=(16, 9))
fig.canvas.manager.set_window_title("Phased Array 2D Command Center (High-Speed Edition)")
gs = fig.add_gridspec(2, 2, width_ratios=[1.5, 1], height_ratios=[2, 1])

# --- LEFT COLUMN ---
ax_music = fig.add_subplot(gs[0, 0])
ax_raw = fig.add_subplot(gs[1, 0])

# --- RIGHT COLUMN ---
ax_radar = fig.add_subplot(gs[:, 1], polar=True)

# 1. Setup M.U.S.I.C. Plot
line_music, = ax_music.plot(THETA_DEGREES, np.zeros(len(THETA_DEGREES)), color='lime', linewidth=2, zorder=3)
target_mark_music, = ax_music.plot([], [], 'ro', markersize=10, markeredgecolor='white', zorder=5, label='Target Lock')

ax_music.set_title("Live M.U.S.I.C. Spatial Spectrum", fontsize=14, fontweight='bold', color='white')
ax_music.set_ylabel("Normalized Power", fontsize=10, color='white')
ax_music.set_xlim(-90, 90)
ax_music.set_ylim(0, 1.05)
ax_music.grid(True, linestyle=':', alpha=0.4, color='white')
ax_music.set_facecolor('black')
ax_music.tick_params(colors='white')
ax_music.legend(loc='upper left', facecolor='black', labelcolor='white')

# 2. Setup Raw Waveforms Plot
channel_colors = ['cyan', 'magenta', 'yellow', 'red', 'green']
raw_lines = []
for i in range(NUM_CHANNELS):
    l, = ax_raw.plot([], [], color=channel_colors[i], linewidth=1.5, alpha=0.8, label=f'Ch {i}')
    raw_lines.append(l)

ax_raw.set_title("Raw Acoustic Waveforms", fontsize=12, fontweight='bold', color='white')
ax_raw.set_xlabel("Buffer Index", fontsize=10, color='white')
ax_raw.set_ylabel("Amplitude", fontsize=10, color='white')
ax_raw.grid(True, linestyle=':', alpha=0.4, color='white')
ax_raw.set_facecolor('#111111') 
ax_raw.tick_params(colors='white')
ax_raw.legend(loc='upper right', facecolor='black', labelcolor='white', fontsize=8)

# 3. Setup Tactical 2D Radar Plot
ax_radar.set_title("Tactical 2D Radar Map", fontsize=16, fontweight='bold', color='white', pad=20)
ax_radar.set_facecolor('#050505')
ax_radar.tick_params(colors='white')
ax_radar.grid(True, color='#333333', linestyle='--')
ax_radar.set_theta_zero_location('N')
ax_radar.set_theta_direction(-1) 
ax_radar.set_thetamin(-90)
ax_radar.set_thetamax(90)

ax_radar.set_ylim(0, MAX_RADAR_RANGE)
ax_radar.set_yticks([0.5, 1.0, 1.5])
ax_radar.set_yticklabels(['0.5m', '1.0m', '1.5m'], color='lime', fontsize=9) 

line_radar, = ax_radar.plot(THETA_RADIANS, np.zeros(len(THETA_RADIANS)), color='lime', linewidth=3, zorder=3)
target_mark_radar, = ax_radar.plot([], [], 'ro', markersize=14, markeredgecolor='white', zorder=5)

fig.patch.set_facecolor('#222222')
plt.tight_layout()
plt.show()

recording = False
buffer_data = []
current_scan_angle = 0.0 
scan_patch_music = None
scan_patch_radar = None

target_memory = {}
target_texts = {}

# ==========================================
# III. THE TWS EIGEN-SOLVER LOOP
# ==========================================
while True:
    try:
        raw_line = ser.readline()
        if not raw_line: continue
        
        line_str = raw_line.decode('utf-8', errors='ignore').strip()
        
        if "TWS_FRAME_START" in line_str:
            recording = True
            buffer_data = []
            try:
                parts = line_str.split(',')
                if len(parts) > 1:
                    current_scan_angle = float(parts[1])
            except ValueError:
                pass
            continue
            
        if recording:
            if "TWS_FRAME_END" in line_str:
                recording = False
                
                if len(buffer_data) == 0:
                    continue
                
                raw_matrix = np.array(buffer_data).T
                num_samples_received = raw_matrix.shape[1]
                
                stc_curve = np.linspace(STC_START_GAIN, STC_END_GAIN, num_samples_received)
                raw_matrix = raw_matrix * stc_curve
                
                # Signal matrix is now entirely pre-cropped by C++ hardware logic
                signal_matrix = raw_matrix
                
                # SQUELCH GATE
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
                    
                    for i in range(len(THETA_RADIANS)):
                        a_theta = STEERING_VECTORS[:, i]
                        projection = a_theta.conj().T @ noise_subspace @ noise_subspace.conj().T @ a_theta
                        spectrum[i] = 1.0 / np.abs(projection)
                        
                    spectrum = spectrum / np.max(spectrum)
                
                # SECTOR BLANKING
                cone_min = current_scan_angle - 10
                cone_max = current_scan_angle + 10
                mask_outside_cone = (THETA_DEGREES < cone_min) | (THETA_DEGREES > cone_max)
                spectrum[mask_outside_cone] = 0.0001
                
                # ==========================================================
                # MEMORY BANK UPDATE & RANGE ALGEBRA
                # ==========================================================
                peak_value = np.max(spectrum)
                if peak_value > 0.85:
                    target_idx = np.argmax(spectrum)
                    lock_angle_deg = THETA_DEGREES[target_idx]
                    lock_angle_rad = THETA_RADIANS[target_idx]
                    
                    echo_envelope = np.sum(np.abs(raw_matrix), axis=0)
                    max_echo_val = np.max(echo_envelope)
                    
                    edge_threshold = max_echo_val * 0.25 
                    peak_time_idx = np.argmax(echo_envelope > edge_threshold)
                    
                    # Applying CROP_OFFSET to guarantee perfectly stable range measurement
                    time_of_flight = HARDWARE_DELAY_SEC + ((CAPTURE_OFFSET + CROP_OFFSET + peak_time_idx) / SAMPLE_RATE_PER_CH)
                    lock_range_meters = (time_of_flight * SPEED_OF_SOUND) / 2.0
                    
                    target_memory[current_scan_angle] = {
                        'deg': lock_angle_deg,
                        'rad': lock_angle_rad,
                        'peak': peak_value,
                        'range': lock_range_meters
                    }
                else:
                    if current_scan_angle in target_memory:
                        target_memory[current_scan_angle] = None

                # ==========================================
                # DRAW PERSISTENT TARGETS
                # ==========================================
                active_degs, active_peaks = [], []
                active_rads, active_ranges = [], []

                for sector, data in target_memory.items():
                    if sector not in target_texts:
                        target_texts[sector] = ax_radar.text(0, 0, "", color='red', fontsize=12, fontweight='bold', ha='center', va='bottom', zorder=6)

                    if data is not None:
                        active_degs.append(data['deg'])
                        active_peaks.append(data['peak'])
                        active_rads.append(data['rad'])
                        active_ranges.append(data['range'])
                        
                        target_texts[sector].set_text(f"{data['range']:.2f}m")
                        target_texts[sector].set_position((data['rad'], data['range'] + 0.08))
                    else:
                        target_texts[sector].set_text("")

                target_mark_music.set_data(active_degs, active_peaks)
                target_mark_radar.set_data(active_rads, active_ranges)

                # ==========================================
                # UI UPDATES
                # ==========================================
                line_music.set_ydata(spectrum)
                if scan_patch_music is not None:
                    scan_patch_music.remove()
                scan_patch_music = ax_music.axvspan(cone_min, cone_max, color='blue', alpha=0.15)
                
                x_axis = np.arange(num_samples_received)
                for i in range(NUM_CHANNELS):
                    raw_lines[i].set_data(x_axis, raw_matrix[i])
                
                ax_raw.set_xlim(0, num_samples_received)
                y_min, y_max = np.min(raw_matrix), np.max(raw_matrix)
                if y_max - y_min < 0.01:
                    y_min, y_max = -0.1, 0.1 
                ax_raw.set_ylim(y_min - 0.05, y_max + 0.05)
                
                line_radar.set_data(THETA_RADIANS, spectrum * MAX_RADAR_RANGE)
                if scan_patch_radar is not None:
                    scan_patch_radar.remove()
                
                theta_fill = np.linspace(np.radians(cone_min), np.radians(cone_max), 20)
                scan_patch_radar = ax_radar.fill_between(theta_fill, 0, MAX_RADAR_RANGE, color='blue', alpha=0.2)
                
                fig.canvas.draw()
                fig.canvas.flush_events()
                
            else:
                try:
                    vals = [float(v) for v in line_str.split(",") if v]
                    if len(vals) == 5:
                        buffer_data.append(vals)
                except ValueError:
                    pass 

    except KeyboardInterrupt:
        print("\n>>> TWS Engine Shutting Down.")
        ser.close()
        break
    except serial.SerialException:
        print("\n>>> Hardware Connection Lost.")
        break