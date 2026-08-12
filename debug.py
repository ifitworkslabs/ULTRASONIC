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

# Physical Array Geometries (in meters)
RX_POS = np.array([-0.027, -0.011, 0.000, 0.011, 0.023])
TX_POS = np.array([-0.0084, -0.0042, 0.0, 0.0042, 0.0084])

print(f"Opening {PORT} for Two-Way M.U.S.I.C. Algorithm...")
try:
    ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
except Exception as e:
    print(f"\n[FATAL ERROR] Cannot open {PORT}.\nError: {e}")
    exit()

# ==========================================
# 2. UI SETUP & TX BEAM PRE-CALCULATION
# ==========================================
plt.ion()
fig, ax = plt.subplots(figsize=(10, 6))

ax.set_title("AESA Two-Way Pattern Multiplication", color='white', fontsize=14, weight='bold')
ax.set_xlabel("Room Angle (Degrees)", color='white', fontsize=12)
ax.set_ylabel("Spatial Power (Normalized)", color='white', fontsize=12)
ax.set_xlim(-45, 45)
ax.set_ylim(0, 1.1)

fig.patch.set_facecolor('#0a0a0a')
ax.set_facecolor('#0a0a0a')
ax.tick_params(colors='white')
ax.grid(color='#222222', linestyle='--')

theta_scan = np.linspace(-45, 45, 360) 

# --- PRE-CALCULATE THE TX ARRAY FACTOR (0-Degree Beam) ---
tx_power = np.zeros(len(theta_scan))
for i, angle in enumerate(theta_scan):
    tau_tx = TX_POS * np.sin(np.radians(angle)) / C_SOUND
    # Sum the geometric phases to find the raw transmit directivity
    a_tx = np.exp(-1j * 2 * np.pi * F_TARGET * tau_tx)
    tx_power[i] = np.abs(np.sum(a_tx)) ** 2

tx_power = tx_power / np.max(tx_power) # Normalize to 1.0

# Plot the theoretical TX Beam in faint red so you can see its shape
ax.plot(theta_scan, tx_power, color='#ff0000', linestyle='--', alpha=0.4, label='TX Array Factor (0°)')
line_final, = ax.plot(theta_scan, np.zeros(360), color='#00ff00', linewidth=3, label='Multiplied M.U.S.I.C. Spectrum')
ax.legend(loc="upper right", facecolor='#222222', edgecolor='white', labelcolor='white')

# ==========================================
# 3. M.U.S.I.C. MATH ENGINE
# ==========================================
freqs = np.fft.fftfreq(N_SAMPLES, 1/FS)
target_bin = np.argmin(np.abs(freqs - F_TARGET))

R_matrix = np.zeros((5, 5), dtype=complex)
alpha = 0.8 
ping_count = 0

buffer_matrix = [[], [], [], [], []]
recording = False

print("\n>>> TWO-WAY MATRIX ONLINE. Tracking Target... <<<")

try:
    while True:
        raw_line = ser.readline().decode('utf-8', errors='ignore').strip()
        
        if raw_line == "START_PLOT":
            recording = True
            buffer_matrix = [[], [], [], [], []]
            continue
            
        elif raw_line == "END_PLOT":
            recording = False
            
            if len(buffer_matrix[0]) == N_SAMPLES:
                data = np.array(buffer_matrix) 
                
                # STEP 1: FFT Extraction
                X_complex = np.fft.fft(data, axis=1)[:, target_bin].reshape(5, 1)
                
                # STEP 2: Covariance
                R_matrix = alpha * R_matrix + (1 - alpha) * (X_complex @ X_complex.conj().T)
                ping_count += 1
                
                if ping_count > 5:
                    # STEP 3: Eigen Decomposition
                    eigenvalues, eigenvectors = eigh(R_matrix)
                    idx = eigenvalues.argsort()[::-1]
                    noise_subspace = eigenvectors[:, idx][:, 1:] 
                    
                    # ==========================================
                    # STEP 4: M.U.S.I.C. Pseudospectrum
                    # ==========================================
                    rx_spectrum = np.zeros(len(theta_scan))
                    
                    # MECHANICAL CALIBRATION OFFSET
                    # A positive number nudges the graph LEFT. 
                    # A negative number nudges the graph RIGHT.
                    BORESIGHT_OFFSET = 1.0 
                    
                    for i, angle in enumerate(theta_scan):
                        # Apply the mechanical nudge to the math
                        corrected_angle = angle + BORESIGHT_OFFSET
                        
                        tau = RX_POS * np.sin(np.radians(corrected_angle)) / C_SOUND
                        a_theta = np.exp(-1j * 2 * np.pi * F_TARGET * tau).reshape(5, 1)
                        
                        denominator = np.abs(a_theta.conj().T @ noise_subspace @ noise_subspace.conj().T @ a_theta)[0, 0]
                        rx_spectrum[i] = 1.0 / denominator if denominator > 1e-10 else 0
                    
                    rx_spectrum = rx_spectrum / np.max(rx_spectrum)
                    
                    # ==========================================
                    # STEP 5: PATTERN MULTIPLICATION
                    # ==========================================
                    final_spectrum = rx_spectrum * tx_power
                    final_spectrum = final_spectrum / np.max(final_spectrum)
                    
                    line_final.set_ydata(final_spectrum)
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
    print("\nTwo-Way Tracker offline.")
    ser.close()