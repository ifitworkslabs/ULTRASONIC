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

calib_tx_str = ""
calib_rx_str = ""
bme_temp_str = "0.0"
bme_hum_str = "0.0"

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
            if line.startswith("const int CALIB_TX_HW_TICKS"):
                calib_tx_str = line
            elif line.startswith("const float CALIB_RX_HW_ERROR"):
                calib_rx_str = line
            elif line.startswith("BME280_TEMP:"):
                bme_temp_str = line.split(":")[1].strip()
            elif line.startswith("BME280_HUM:"):
                bme_hum_str = line.split(":")[1].strip()
            
        if "START_FINAL_PLOT" in line:
            print("\n>>> PYTHON: Target locked. Receiving Pre-Aligned Master Matrix...")
            recording = True
            continue
            
        if recording:
            if "END_FINAL_PLOT" in line:
                print(">>> PYTHON: Matrix transfer complete. Initiating Plot Protocol.")
                
                print(">>> PYTHON: Saving auto-calibration report to 'calibrated data.txt'")
                try:
                    with open("calibrated data.txt", "w") as f:
                        f.write("1. The Transmitter Array (TX)\n")
                        f.write(f"{calib_tx_str}\n\n")
                        f.write("The Unit: 240MHz CPU Ticks\n")
                        f.write("The Physical Scale: 1 Unit = 4.16 nanoseconds\n")
                        f.write("The Mechanics: This array operates in pure processor clock cycles. When the array dictates that TX1 is 2015, it means the ESP32 silicon must physically count exactly 2,015 clock cycles (which takes roughly 8,382 nanoseconds) before it allows the electrical spark to travel to that specific piezoelectric disc. It forces the faster transmitters to wait on the starting line so the sluggish ones can catch up.\n\n")
                        f.write("2. The Receiver Array (RX)\n")
                        f.write(f"{calib_rx_str}\n\n")
                        f.write("The Unit: 400kHz DMA Buffer Indices (Array Slots)\n")
                        f.write("The Physical Scale: 1 Unit = 2.5 microseconds (2,500 nanoseconds)\n")
                        f.write("The Mechanics: This array operates in memory addresses. Because your 2 MHz Analog-to-Digital Converter is multiplexing across 5 microphones, each microphone gets sampled exactly once every 2.5 microseconds. When the array says RX0 is 24.0000, it means the physical acoustic wave crashed into that microphone's memory buffer exactly 24 slots (or 60 microseconds) later than the center anchor.\n\n")
                        f.write("3. Environmental Conditions (BME280)\n")
                        f.write(f"Temperature: {bme_temp_str} C\n")
                        f.write(f"Humidity: {bme_hum_str} %\n\n")
                        f.write("=========================================\n")
                        f.write("Calibration Variables Summary:\n")
                        f.write(f"{calib_tx_str}\n")
                        f.write(f"{calib_rx_str}\n")
                        f.write(f"const float CALIB_TEMP = {bme_temp_str};\n")
                        f.write(f"const float CALIB_HUM = {bme_hum_str};\n")
                except Exception as ex:
                    print(f"Warning: Failed to save calibrated data.txt: {ex}")
                    
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
        
    plt.xlim(1900, 2300) # Perfectly frame the 109cm echo burst
    plt.xlabel("Hardware Buffer Index (2MHz)", fontsize=12)
    plt.ylabel("Voltage", fontsize=12)
    plt.grid(True, linestyle=':', alpha=0.7)
    plt.legend(loc='upper right')
    plt.tight_layout()
    plt.show()
else:
    print("\nFailure: No matrix data was received. Check the wiring and restart.")