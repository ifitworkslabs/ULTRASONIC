import serial
import numpy as np
import matplotlib.pyplot as plt

# --- HARDWARE CONFIGURATION ---
PORT = 'COM3'  # You must change this to match your ESP32's COM Port
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

# --- LISTEN AND PARSE ---
while True:
    try:
        raw_line = ser.readline()
        if not raw_line:
            continue
            
        line = raw_line.decode('utf-8', errors='ignore').strip()
        
        # Print the ESP32's progress to the terminal
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
        print(f"\nUSB CRASH: The ESP32 pulled too much power and disconnected. {e}")
        break

ser.close()

# --- MATHEMATICAL CORRECTION & PLOTTING ---
if len(buffer_data) > 0:
    # Convert to NumPy array and transpose to get 5 distinct channels
    aligned_matrix = np.array(buffer_data).T 
        
    # Generate the Proof Graph
    plt.figure(figsize=(14, 7))
    plt.title("Hardware-Corrected Phase Alignment (2MHz DMA & Plateau Locked)", fontsize=18, fontweight='bold')
    
    colors = ['blue', 'orange', 'green', 'red', 'purple']
    for i in range(5):
        plt.plot(indices, aligned_matrix[i], label=f"RX Channel {i}", color=colors[i], alpha=0.8)
        
    plt.xlim(2000, 2500) # Zoom into the exact window where the echo strikes
    plt.xlabel("Hardware Buffer Index (2MHz)", fontsize=12)
    plt.ylabel("Voltage", fontsize=12)
    plt.grid(True, linestyle=':', alpha=0.7)
    plt.legend(loc='upper right')
    plt.tight_layout()
    plt.show()
else:
    print("\nFailure: No matrix data was received. Check the wiring and restart.")