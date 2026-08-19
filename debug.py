import serial
import numpy as np
import matplotlib.pyplot as plt

# ==========================================
# I. HARDWARE CONFIGURATION
# ==========================================
PORT = 'COM3'  # Update this to match your ESP32's COM Port
BAUD = 115200

print("==================================================")
print("Professor's Python Terminal: Listening to the ESP32...")
print("==================================================\n")

try:
    ser = serial.Serial(PORT, BAUD, timeout=10)
except Exception as e:
    print(f"CRITICAL ERROR: Failed to open port {PORT}. Is the ESP32 plugged in?")
    exit()

recording = False
indices = []
buffer_data = []

# ==========================================
# II. LISTEN AND PARSE MATRIX
# ==========================================
while True:
    try:
        raw_line = ser.readline()
        if not raw_line:
            continue
            
        line = raw_line.decode('utf-8', errors='ignore').strip()
        
        # Print the ESP32's auto-tuning progress to the terminal
        if not recording and line:
            print(line) 
            
        if "START_FINAL_PLOT" in line:
            print("\n>>> PYTHON: Target locked. Receiving Pre-Aligned Master Matrix...")
            recording = True
            continue
            
        if recording:
            if "END_FINAL_PLOT" in line:
                print(">>> PYTHON: Matrix transfer complete. Initiating Plot Protocol.")
                break
            else:
                # Capture the comma-separated values
                try:
                    vals = [float(v) for v in line.split(",") if v]
                    
                    # WE NOW EXPECT 6 VALUES: (1 Index + 5 Voltages)
                    if len(vals) == 6: 
                        indices.append(vals[0])      # Save the X-axis index
                        buffer_data.append(vals[1:]) # Save the 5 Y-axis voltages
                except ValueError:
                    pass # Ignore random serial garbage
                    
    except serial.SerialException as e:
        print(f"\nUSB CRASH: The ESP32 disconnected. {e}")
        break

ser.close()

# ==========================================
# III. MATHEMATICAL CORRECTION & PLOTTING
# ==========================================
if len(buffer_data) > 0:
    # Convert to NumPy array and transpose to get 5 distinct channels
    aligned_matrix = np.array(buffer_data).T 
        
    # Generate the Proof Graph (Dark Mode)
    fig = plt.figure(figsize=(14, 7))
    fig.canvas.manager.set_window_title("Sub-Sample Calibration Proof")
    fig.patch.set_facecolor('#222222')
    
    ax = plt.gca()
    ax.set_facecolor('#111111')
    
    plt.title("Hardware-Corrected Phase Alignment (2MHz DMA & Sub-Sample Locked)", fontsize=16, fontweight='bold', color='white')
    
    # High-contrast colors representing your left-to-right physical layout
    colors = ['cyan', 'magenta', 'yellow', 'red', 'green']
    for i in range(5):
        plt.plot(indices, aligned_matrix[i], label=f"RX Channel {i}", color=colors[i], alpha=0.8, linewidth=1.5)
        
    # Zoom into the exact window where the acoustic echo strikes
    plt.xlim(2000, 2500) 
    
    plt.xlabel("Hardware Buffer Index (2MHz)", fontsize=12, color='white')
    plt.ylabel("Voltage Amplitude", fontsize=12, color='white')
    
    ax.grid(True, linestyle=':', alpha=0.4, color='white')
    ax.tick_params(colors='white')
    
    legend = plt.legend(loc='upper right', facecolor='black', edgecolor='white')
    for text in legend.get_texts():
        text.set_color("white")
        
    plt.tight_layout()
    plt.show()
else:
    print("\nFailure: No matrix data was received. Check the wiring and restart.")