import serial
import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import eigh

# ==========================================
# 1. HARDWARE & ACOUSTIC CONSTANTS
# ==========================================
PORT = 'COM3'  
BAUD_RATE = 500000
F_TARGET = 40000.0       
C_SOUND = 343.0          
FS = 500000.0            
N_SAMPLES = 150          

RX_POS = np.array([-0.027, -0.011, 0.000, 0.011, 0.023])
TX_POS = np.array([-0.0084, -0.0042, 0.0, 0.0042, 0.0084])
SCAN_ANGLES = [-40.0, -20.0, 0.0, 20.0, 40.0]
BORESIGHT_OFFSET = 1.0 

print(f"Opening {PORT} for Master TWS Fusion Radar...")
try:
    ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
except Exception as e:
    print(f"\n[FATAL ERROR] Cannot open {PORT}.\nError: {e}")
    exit()

# ==========================================
# 2. UI SETUP: DUAL-SCREEN DASHBOARD
# ==========================================
plt.ion()
fig, (ax_radar, ax_osc) = plt.subplots(2, 1, figsize=(12, 10))

# --- TOP SCREEN ---
ax_radar.set_title("AESA Track-While-Scan (True Volume Scaling)", color='white', fontsize=14, weight='bold')
ax_radar.set_ylabel("Spatial Power (Physical Ratio)", color='white', fontsize=12)
ax_radar.set_xlim(-45, 45)
ax_radar.set_ylim(0, 1.1)
ax_radar.grid(color='#222222', linestyle='--')

theta_scan = np.linspace(-45, 45, 360) 
master_line, = ax_radar.plot(theta_scan, np.zeros(360), color='#00ff00', linewidth=3, label='Unified TWS Tracker')
ax_radar.legend(loc="upper right", facecolor='#222222', edgecolor='white', labelcolor='white')

# --- BOTTOM SCREEN ---
ax_osc.set_title("Live Raw ADC Data", color='white', fontsize=14, weight='bold')
ax_osc.set_xlabel("Buffer Index (Time Gate Window)", color='white', fontsize=12)
ax_osc.set_ylabel("Amplitude (ADC Units)", color='white', fontsize=12)
ax_osc.set_xlim(0, 150)
ax_osc.set_ylim(-800, 800) 
ax_osc.grid(color='#222222', linestyle='--')

colors = ['#ff0000', '#00ff00', '#0000ff', '#ffff00', '#ff00ff']
osc_lines = []
for i in range(5):
    line, = ax_osc.plot([], [], color=colors[i], linewidth=1.5, alpha=0.8)
    osc_lines.append(line)

fig.patch.set_facecolor('#0a0a0a')
ax_radar.set_facecolor('#0a0a0a')
ax_osc.set_facecolor('#0a0a0a')
ax_radar.tick_params(colors='white')
ax_osc.tick_params(colors='white')
plt.tight_layout()

# ==========================================
# 3. PRE-CALCULATIONS & M.U.S.I.C. ENGINE
# ==========================================
tx_patterns = []
for steer_angle in SCAN_ANGLES:
    pattern = np.zeros(len(theta_scan))
    steering_tau = TX_POS * np.sin(np.radians(steer_angle)) / C_SOUND
    for i, angle in enumerate(theta_scan):
        room_tau = TX_POS * np.sin(np.radians(angle)) / C_SOUND
        a_tx = np.exp(-1j * 2 * np.pi * F_TARGET * (room_tau - steering_tau))
        pattern[i] = np.abs(np.sum(a_tx)) ** 2
    pattern = pattern / np.max(pattern)
    tx_patterns.append(pattern)

freqs = np.fft.fftfreq(N_SAMPLES, 1/FS)
target_bin = np.argmin(np.abs(freqs - F_TARGET))

R_matrices = [np.zeros((5, 5), dtype=complex) for _ in range(5)]
ping_counts = [0, 0, 0, 0, 0]
sector_spectra = [np.zeros(len(theta_scan)) for _ in range(5)]

# THE GOLDILOCKS MEMORY ZONE. 
# 60% historical memory provides enough stability to define the Noise Subspace 
# without causing excessive target lag.
alpha = 0.60 

buffer_matrix = [[], [], [], [], []]
recording = False
current_sector = -1
x_data = np.arange(150)

print("\n>>> TWS PHYSICAL VOLUME ENGINE ONLINE. Scanning all sectors... <<<")

try:
    while True:
        raw_line = ser.readline().decode('utf-8', errors='ignore').strip()
        
        if raw_line.startswith("START_PLOT_"):
            try:
                current_sector = int(raw_line.split("_")[-1])
                recording = True
                buffer_matrix = [[], [], [], [], []]
            except ValueError:
                pass
            continue
            
        elif raw_line.startswith("END_PLOT_"):
            recording = False
            
            if len(buffer_matrix[0]) == N_SAMPLES and 0 <= current_sector <= 4:
                data = np.array(buffer_matrix) 
                
                # Update Oscilloscope
                for i in range(5):
                    osc_lines[i].set_data(x_data, data[i])
                
                current_max = np.max(np.abs(data))
                if current_max > ax_osc.get_ylim()[1] * 0.9:
                    ax_osc.set_ylim(-current_max * 1.2, current_max * 1.2)
                elif current_max < 300:
                    ax_osc.set_ylim(-400, 400)
                
                # Baseline Noise Gate (Keeps pure empty static off the screen)
                if current_max < 150.0:
                    sector_spectra[current_sector] = np.zeros(len(theta_scan))
                    # Wipe the historical memory for this sector immediately so ghosts don't linger
                    R_matrices[current_sector] = np.zeros((5, 5), dtype=complex)
                else:
                    # Run Single-Target M.U.S.I.C. for this specific sector
                    X_complex = np.fft.fft(data, axis=1)[:, target_bin].reshape(5, 1)
                    R_matrices[current_sector] = alpha * R_matrices[current_sector] + (1 - alpha) * (X_complex @ X_complex.conj().T)
                    ping_counts[current_sector] += 1
                    
                    if ping_counts[current_sector] > 5:
                        eigenvalues, eigenvectors = eigh(R_matrices[current_sector])
                        idx = eigenvalues.argsort()[::-1]
                        
                        # STRICT SINGLE TARGET ISOLATION
                        # We guarantee 4 dimensions to the Noise Subspace to mathematically crush ghost mirrors.
                        noise_subspace = eigenvectors[:, idx][:, 1:] 
                        
                        rx_spectrum = np.zeros(len(theta_scan))
                        for i, angle in enumerate(theta_scan):
                            corrected_angle = angle + BORESIGHT_OFFSET
                            tau = RX_POS * np.sin(np.radians(corrected_angle)) / C_SOUND
                            a_theta = np.exp(-1j * 2 * np.pi * F_TARGET * tau).reshape(5, 1)
                            denominator = np.abs(a_theta.conj().T @ noise_subspace @ noise_subspace.conj().T @ a_theta)[0, 0]
                            rx_spectrum[i] = 1.0 / denominator if denominator > 1e-10 else 0
                        
                        # 1. Normalize the raw shape 
                        rx_spectrum = rx_spectrum / np.max(rx_spectrum)
                        
                        # 2. Multiply by TX pattern (Electronically isolates the sector)
                        final_spectrum = rx_spectrum * tx_patterns[current_sector]
                        
                        # 3. Scale by absolute physical volume (Prevents noise inflation)
                        physical_volume = min(1.0, current_max / 600.0) 
                        sector_spectra[current_sector] = final_spectrum * physical_volume
                
                # FUSE THE MASTER SCREEN
                unified_display = np.max(np.array(sector_spectra), axis=0)
                master_line.set_ydata(unified_display)
                
                fig.canvas.draw()
                fig.canvas.flush_events()
            continue

        if recording and ',' in raw_line:
            try:
                values = raw_line.split(',')
                if len(values) == 5:
                    for i in range(5):
                        buffer_matrix[i].append(float(values[i]))
            except ValueError:
                pass 

except KeyboardInterrupt:
    print("\nTWS Master Radar offline.")
    ser.close()