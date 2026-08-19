import serial
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import hilbert

# ==========================================
# I. RADAR PHYSICS & GEOMETRY
# ==========================================
PORT = 'COM3'  # Update to your ESP32 Port
BAUD = 115200

FREQUENCY = 40000.0          
SPEED_OF_SOUND = 343.0       
WAVELENGTH = SPEED_OF_SOUND / FREQUENCY
NUM_CHANNELS = 5

# The Non-Uniform Linear Array (NULA) exact physical coordinates in meters
MIC_POSITIONS = np.array([-0.025, -0.013, -0.002, 0.009, 0.025])

THETA_DEGREES = np.arange(-90, 91, 1)
THETA_RADIANS = np.radians(THETA_DEGREES)

STEERING_VECTORS = np.zeros((NUM_CHANNELS, len(THETA_RADIANS)), dtype=complex)
for i, theta in enumerate(THETA_RADIANS):
    spatial_phases = (MIC_POSITIONS / WAVELENGTH) * np.sin(theta)
    STEERING_VECTORS[:, i] = np.exp(-1j * 2 * np.pi * spatial_phases)

# ==========================================
# II. SENSITIVITY TIME CONTROL (STC)
# ==========================================
# We keep the volume ramp to combat distance loss, but squelch is completely removed.
STC_START_GAIN = 1.0
STC_END_GAIN = 3.0

# ==========================================
# III. INITIALIZE HARDWARE STREAM & DUAL-UI
# ==========================================
print("==================================================")
print("Professor's Target Tracking Engine Online (2 MHz).")
print(">>> SQUELCH DISABLED. RAW DATA MODE ENGAGED. <<<")
print("==================================================\n")

try:
    ser = serial.Serial(PORT, BAUD, timeout=2)
except Exception as e:
    print(f"CRITICAL ERROR: Failed to open port {PORT}. {e}")
    exit()

plt.ion()
fig, (ax_music, ax_raw) = plt.subplots(2, 1, figsize=(12, 9), gridspec_kw={'height_ratios': [2, 1]})
fig.canvas.manager.set_window_title("Phased Array TWS Command Center (Unfiltered)")

line_music, = ax_music.plot(THETA_DEGREES, np.zeros(len(THETA_DEGREES)), color='lime', linewidth=2)
ax_music.set_title("Live M.U.S.I.C. Spatial Spectrum", fontsize=16, fontweight='bold', color='white')
ax_music.set_ylabel("Pseudospectrum Power", fontsize=12, color='white')
ax_music.set_xlim(-90, 90)
ax_music.set_ylim(0, 1.05)
ax_music.grid(True, linestyle=':', alpha=0.4, color='white')
ax_music.set_facecolor('black')
ax_music.tick_params(colors='white')

channel_colors = ['cyan', 'magenta', 'yellow', 'red', 'green']
raw_lines = []
for i in range(NUM_CHANNELS):
    l, = ax_raw.plot([], [], color=channel_colors[i], linewidth=1.5, alpha=0.8, label=f'Ch {i}')
    raw_lines.append(l)

ax_raw.set_title("Raw Acoustic Waveforms (STC Amplified & Unfiltered)", fontsize=12, fontweight='bold', color='white')
ax_raw.set_xlabel("Hardware Buffer Index (2MHz)", fontsize=12, color='white')
ax_raw.set_ylabel("Amplitude (Gain Scaled)", fontsize=12, color='white')
ax_raw.grid(True, linestyle=':', alpha=0.4, color='white')
ax_raw.set_facecolor('#111111') 
ax_raw.tick_params(colors='white')
ax_raw.legend(loc='upper right', facecolor='black', labelcolor='white', fontsize=8)

fig.patch.set_facecolor('#222222')
plt.tight_layout()
plt.show()

recording = False
buffer_data = []

# ==========================================
# IV. THE TWS EIGEN-SOLVER LOOP
# ==========================================
while True:
    try:
        raw_line = ser.readline()
        if not raw_line: continue
        
        line_str = raw_line.decode('utf-8', errors='ignore').strip()
        
        if "TWS_FRAME_START" in line_str:
            recording = True
            buffer_data = []
            continue
            
        if recording:
            if "TWS_FRAME_END" in line_str:
                recording = False
                
                if len(buffer_data) == 0:
                    continue
                
                # 1. Convert to NumPy Matrix (Shape: 5 channels x N samples)
                raw_matrix = np.array(buffer_data).T
                num_samples_received = raw_matrix.shape[1]
                
                # 2. Generate the Dynamic STC Curve & Amplification
                stc_curve = np.linspace(STC_START_GAIN, STC_END_GAIN, num_samples_received)
                raw_matrix = raw_matrix * stc_curve
                
                # 3. Extract the Steady-State Burst for Math
                start_idx = 400
                end_idx = 800
                if num_samples_received > end_idx:
                    signal_matrix = raw_matrix[:, start_idx:end_idx] 
                else:
                    signal_matrix = raw_matrix
                
                # ==========================================================
                # 4. UNLEASH M.U.S.I.C. (NO GATES, NO LIMITS)
                # ==========================================================
                signal_matrix = signal_matrix - np.mean(signal_matrix, axis=1, keepdims=True)
                complex_matrix = hilbert(signal_matrix, axis=1)
                
                num_samples = complex_matrix.shape[1]
                Rxx = (complex_matrix @ complex_matrix.conj().T) / num_samples
                
                eigenvalues, eigenvectors = np.linalg.eigh(Rxx)
                idx = eigenvalues.argsort()[::-1]
                eigenvalues = eigenvalues[idx]
                eigenvectors = eigenvectors[:, idx]
                
                noise_subspace = eigenvectors[:, 1:]
                
                spectrum = np.zeros(len(THETA_RADIANS))
                for i in range(len(THETA_RADIANS)):
                    a_theta = STEERING_VECTORS[:, i]
                    projection = a_theta.conj().T @ noise_subspace @ noise_subspace.conj().T @ a_theta
                    spectrum[i] = 1.0 / np.abs(projection)
                
                spectrum = spectrum / np.max(spectrum)
                
                # ==========================================
                # 5. UPDATE THE DUAL-DISPLAY
                # ==========================================
                line_music.set_ydata(spectrum)
                
                x_axis = np.arange(num_samples_received)
                
                # Update the waveforms using the pure, unfiltered matrix
                for i in range(NUM_CHANNELS):
                    raw_lines[i].set_data(x_axis, raw_matrix[i])
                
                # Dynamically scale the raw plot axes 
                ax_raw.set_xlim(0, num_samples_received)
                y_min, y_max = np.min(raw_matrix), np.max(raw_matrix)
                
                if y_max - y_min < 0.01:
                    y_min, y_max = -0.1, 0.1 
                    
                ax_raw.set_ylim(y_min - 0.05, y_max + 0.05)
                
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