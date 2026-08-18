import serial
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import hilbert

# ==========================================
# I. AESA RADAR CONFIGURATION
# ==========================================
PORT = 'COM3'  
BAUD = 115200

FREQUENCY = 40000.0          
SPEED_OF_SOUND = 343.0       
WAVELENGTH = SPEED_OF_SOUND / FREQUENCY
D = WAVELENGTH / 2.0         
NUM_CHANNELS = 5

ECHO_THRESHOLD_VOLTAGE = 0.02 
CAPTURE_OFFSET_INDICES = 1400 # From C++
CHANNEL_SAMPLE_RATE = 250000.0 # 1.25MHz / 5 Channels

THETA_DEGREES = np.arange(-90, 91, 1)
THETA_RADIANS = np.radians(THETA_DEGREES)

STEERING_VECTORS = np.zeros((NUM_CHANNELS, len(THETA_RADIANS)), dtype=complex)
for i, theta in enumerate(THETA_RADIANS):
    spatial_phases = np.arange(NUM_CHANNELS) * (D / WAVELENGTH) * np.sin(theta)
    STEERING_VECTORS[:, i] = np.exp(-1j * 2 * np.pi * spatial_phases)

global_spectrum = np.zeros(len(THETA_DEGREES))

# ==========================================
# II. INITIALIZE HARDWARE STREAM
# ==========================================
print("==================================================")
print("Professor's AESA Wide-Band Engine Online.")
print("==================================================\n")

try:
    ser = serial.Serial(PORT, BAUD, timeout=2)
except Exception as e:
    print(f"CRITICAL ERROR: Failed to open port {PORT}. {e}")
    exit()

plt.ion()
fig, ax = plt.subplots(figsize=(10, 6))
line, = ax.plot(THETA_DEGREES, global_spectrum, color='lime', linewidth=2)
title_text = ax.set_title("Live AESA M.U.S.I.C. | Scanning...", fontsize=16, fontweight='bold')
ax.set_xlabel("Physical Room Angle (Degrees)", fontsize=12)
ax.set_ylabel("Pseudospectrum Power", fontsize=12)
ax.set_xlim(-90, 90)
ax.set_ylim(0, 1.05)
ax.grid(True, linestyle=':', alpha=0.6, color='gray')
ax.set_facecolor('black')

for sec in [-60, -40, -20, 0, 20, 40, 60]:
    ax.axvline(sec, color='white', linestyle='--', alpha=0.2)

plt.show()

recording = False
buffer_data = []
current_sector_angle = 0
last_known_distance = 0.0

# ==========================================
# III. THE AESA STITCHING LOOP
# ==========================================
while True:
    try:
        raw_line = ser.readline()
        if not raw_line: continue
        
        line_str = raw_line.decode('utf-8', errors='ignore').strip()
        
        if line_str.startswith("TWS_FRAME_START:"):
            current_sector_angle = int(line_str.split(":")[1])
            recording = True
            buffer_data = []
            continue
            
        if recording:
            if "TWS_FRAME_END" in line_str:
                recording = False
                if len(buffer_data) == 0: continue
                
                # Full 1.0m to 2.5m window matrix
                signal_matrix = np.array(buffer_data).T
                
                peak_echo = np.max(np.abs(signal_matrix))
                local_spectrum = np.zeros(len(THETA_DEGREES))
                
                if peak_echo >= ECHO_THRESHOLD_VOLTAGE:
                    # 1. Physics: Calculate Live Target Depth
                    peak_index_in_window = np.argmax(np.abs(signal_matrix[2]))
                    true_hardware_index = CAPTURE_OFFSET_INDICES + peak_index_in_window
                    time_seconds = true_hardware_index / CHANNEL_SAMPLE_RATE
                    last_known_distance = (time_seconds * SPEED_OF_SOUND) / 2.0
                    
                    # 2. Math: M.U.S.I.C. Eigenvalue Solver
                    signal_matrix = signal_matrix - np.mean(signal_matrix, axis=1, keepdims=True)
                    complex_matrix = hilbert(signal_matrix, axis=1)
                    
                    num_samples = complex_matrix.shape[1]
                    Rxx = (complex_matrix @ complex_matrix.conj().T) / num_samples
                    
                    eigenvalues, eigenvectors = np.linalg.eigh(Rxx)
                    idx = eigenvalues.argsort()[::-1]
                    eigenvectors = eigenvectors[:, idx]
                    
                    noise_subspace = eigenvectors[:, 1:] 
                    
                    for i in range(len(THETA_RADIANS)):
                        a_theta = STEERING_VECTORS[:, i]
                        projection = a_theta.conj().T @ noise_subspace @ noise_subspace.conj().T @ a_theta
                        local_spectrum[i] = 1.0 / np.abs(projection)
                    
                    local_spectrum = local_spectrum / np.max(local_spectrum)
                
                # SECTOR BLINDER
                mask = (THETA_DEGREES >= current_sector_angle - 20) & (THETA_DEGREES <= current_sector_angle + 20)
                
                # PHOSPHOR FADE
                global_spectrum = global_spectrum * 0.90 
                
                # THE BLEND FIX: Overlap peaks naturally without drawing sheer walls
                global_spectrum[mask] = np.maximum(global_spectrum[mask], local_spectrum[mask])
                
                # Push to display
                line.set_ydata(global_spectrum)
                
                if peak_echo >= ECHO_THRESHOLD_VOLTAGE:
                    title_text.set_text(f"Live AESA M.U.S.I.C. | Target Depth: {last_known_distance:.2f} m")
                else:
                    title_text.set_text(f"Live AESA M.U.S.I.C. | Scanning...")
                    
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
        print("\n>>> AESA Engine Shutting Down.")
        ser.close()
        break
    except serial.SerialException:
        print("\n>>> Hardware Connection Lost.")
        break