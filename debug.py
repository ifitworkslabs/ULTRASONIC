import serial
import matplotlib.pyplot as plt
import sys

COM_PORT = 'COM3'  # Verify your exact ESP32 Port
BAUD_RATE = 115200

try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
    print(f"Connected to ESP32 on {COM_PORT}. Listening to 4.16ns Auto-Tuner...")
except Exception as e:
    print(f"Connection Failed: {e}")
    sys.exit()

indices = []
rx_data = {0: [], 1: [], 2: [], 3: [], 4: []}
recording = False

while True:
    try:
        raw_line = ser.readline()
        if not raw_line: continue
            
        line = raw_line.decode('utf-8', errors='ignore').strip()
        
        if not recording and not line.startswith("START_FINAL_PLOT"):
            print(line)
        
        if line == "START_FINAL_PLOT":
            print("\nCatching ultra-high resolution waveform data...")
            recording = True
            continue
            
        elif line == "END_FINAL_PLOT":
            print("\nSuccess! Rendering 2MHz Proof Graph...")
            break
            
        elif recording:
            vals = line.split(',')
            if len(vals) == 6:
                indices.append(int(vals[0]))
                for i in range(5):
                    rx_data[i].append(float(vals[i+1]))
                    
    except KeyboardInterrupt:
        print("\nAborted.")
        break

ser.close()

if len(indices) > 0:
    plt.figure(figsize=(12, 6))
    plt.title("Post-Calibration Alignment (2MHz Sample Rate / 4.16ns TX Precision)", fontsize=16, fontweight='bold')
    
    colors = ['blue', 'orange', 'green', 'red', 'purple']
    for i in range(5):
        plt.plot(indices, rx_data[i], label=f'RX Channel {i}', color=colors[i], alpha=0.7)
    
    # Render the new Time Gate bounds
    plt.axvline(x=1900, color='black', linestyle='--', linewidth=2, label='Gate Start (1900)')
    plt.axvline(x=2600, color='black', linestyle='--', linewidth=2, label='Gate End (2600)')
    
    plt.xlabel("Hardware Buffer Index (High Resolution)", fontsize=12)
    plt.ylabel("Voltage", fontsize=12)
    plt.grid(True, linestyle=':', alpha=0.7)
    plt.legend()
    
    # Zoom deeply into the target
    plt.xlim(1600, 2800) 
    plt.tight_layout()
    plt.show()