import serial
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import hilbert
import time

# ==========================================
# I. RADAR PHYSICS CONFIGURATION
# ==========================================
PORT = 'COM3'  # Update to your ESP32 Port
BAUD = 115200

# Physical constants of your array
FREQUENCY = 40000.0          # 40 kHz Ultrasonic
SPEED_OF_SOUND = 343.0       # m/s at standard room temperature
WAVELENGTH = SPEED_OF_SOUND / FREQUENCY
D = WAVELENGTH / 2.0         # Assumes standard half-wavelength microphone spacing
NUM_CHANNELS = 5

# Define the search grid (-90 degrees to +90 degrees)
THETA_DEGREES = np.arange(-90, 91, 1)
THETA_RADIANS = np.radians(THETA_DEGREES)

# Pre-calculate the Steering Matrix for all possible angles
# Formula: a(theta) = exp(-j * 2 * pi * (d/lambda) * sin(theta) * channel_index)
STEERING_VECTORS = np.zeros((NUM_CHANNELS, len(THETA_RADIANS)), dtype=complex)
for i, theta in enumerate(THETA_RADIANS):
    spatial_phases = np.arange(NUM_CHANNELS) * (D / WAVELENGTH) * np.sin(theta)
    STEERING_VECTORS[:, i] = np.exp(-1j * 2 * np.pi * spatial_phases)

# ==========================================
# II. INITIALIZE HARDWARE STREAM
# ==========================================
print("==================================================")
print("Professor's Target Tracking Engine Online.")
print("==================================================\n")

try:
    ser = serial.Serial(PORT, BAUD, timeout=2)
except Exception as e:
    print(f"CRITICAL ERROR: Failed to open port {PORT}. {e}")
    exit()

# Initialize real-time plotting
plt.ion()
fig, ax = plt.subplots(figsize=(10, 6))
line, = ax.plot(THETA_DEGREES, np.zeros(len(THETA_DEGREES)), color='lime', linewidth=2)
ax.set_title("Live M.U.S.I.C. Spatial Spectrum", fontsize=16, fontweight='bold')
ax.set_xlabel("Physical Room Angle (Degrees)", fontsize=12)
ax.set_ylabel("Pseudospectrum Power (Normalized)", fontsize=12)
ax.set_xlim(-90, 90)
ax.set_ylim(0, 1.05)
ax.grid(True, linestyle=':', alpha=0.6)
ax.set_facecolor('black')
plt.show()

recording = False
buffer_data = []

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
            continue
            
        if recording:
            if "TWS_FRAME_END" in line_str:
                recording = False
                
                if len(buffer_data) == 0:
                    continue
                
                # 1. Convert to NumPy Matrix (Shape: 5 channels x N samples)
                raw_matrix = np.array(buffer_data).T
                
                # We only want to run math on the steady-state acoustic burst
                # Ignoring the silence and the messy ring-in
                if raw_matrix.shape[1] > 300:
                    signal_matrix = raw_matrix[:, 200:400] 
                else:
                    signal_matrix = raw_matrix
                    
                # 2. The Hilbert Transform (Convert Real Voltage to Complex Vectors)
                # Subtract mean to remove DC bias, then apply Hilbert
                signal_matrix = signal_matrix - np.mean(signal_matrix, axis=1, keepdims=True)
                complex_matrix = hilbert(signal_matrix, axis=1)
                
                # 3. Calculate the Spatial Covariance Matrix (Rxx)
                # Formula: Rxx = (X * X_Hermitian) / N
                num_samples = complex_matrix.shape[1]
                Rxx = (complex_matrix @ complex_matrix.conj().T) / num_samples
                
                # 4. Eigenvalue Decomposition
                eigenvalues, eigenvectors = np.linalg.eigh(Rxx)
                
                # Sort eigenvectors by eigenvalues in descending order
                idx = eigenvalues.argsort()[::-1]
                eigenvalues = eigenvalues[idx]
                eigenvectors = eigenvectors[:, idx]
                
                # 5. Isolate the Noise Subspace
                # Assuming 1 primary target, the lowest 4 eigenvectors form the noise floor
                NUM_TARGETS = 1
                noise_subspace = eigenvectors[:, NUM_TARGETS:]
                
                # 6. Calculate the Pseudospectrum
                # Formula: P(theta) = 1 / (a(theta)_Hermitian * E_noise * E_noise_Hermitian * a(theta))
                spectrum = np.zeros(len(THETA_RADIANS))
                for i in range(len(THETA_RADIANS)):
                    a_theta = STEERING_VECTORS[:, i]
                    projection = a_theta.conj().T @ noise_subspace @ noise_subspace.conj().T @ a_theta
                    spectrum[i] = 1.0 / np.abs(projection)
                
                # Normalize the spectrum for clean plotting
                spectrum = spectrum / np.max(spectrum)
                
                # 7. Update the Radar Display
                line.set_ydata(spectrum)
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