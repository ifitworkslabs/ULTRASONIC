import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

# --- CONFIGURATION ---
COM_PORT = 'COM3'     # Verify your port
BAUD_RATE = 115200    # The rock-solid safe speed

print(f"Connecting to {COM_PORT}...")
try:
    ser = serial.Serial()
    ser.port = COM_PORT
    ser.baudrate = BAUD_RATE
    ser.timeout = 0.5 
    ser.setDTR(False) 
    ser.setRTS(False) 
    ser.open()
    time.sleep(1) 
    print("Connection established. Reading ESP32 Calibration sequence...\n")
except Exception as e:
    print(f"Hardware Error: {e}")
    exit()

# --- GRAPH SETUP ---
fig, ax = plt.subplots(figsize=(14, 7))
colors = ['red', 'blue', 'green', 'orange', 'purple']
lines = [ax.plot([], [], color=colors[i], alpha=0.9, linewidth=1.5)[0] for i in range(5)]

# Open the X-axis back up to see the entire 1000-index landscape
ax.set_xlim(0, 1000)
ax.set_ylim(-0.4, 0.4) # Keep it zoomed on the pure AC amplitude

ax.set_title("M.U.S.I.C. AI Pipeline: Perfectly Phase-Aligned Acoustic Core")
ax.set_xlabel("Buffer Index (Time Gate Window)")
ax.set_ylabel("Pure AC Amplitude (DC Bias Removed)")
ax.grid(True)

# --- REAL-TIME LOOP ---
def update_graph(frame):
    ser.reset_input_buffer()
    
    found_start = False
    attempts = 0
    
    while attempts < 100:
        raw_line = ser.readline()
        if not raw_line: break 
            
        line = raw_line.decode('utf-8', errors='ignore').strip()
        
        if line == "START_PLOT":
            found_start = True
            break
        elif line and line != "END_PLOT":
            # Terminal Bridge: Print ESP32 text output to the PC console
            if not (',' in line and any(char.isdigit() for char in line)):
                print(f"[ESP32] {line}")
                
        attempts += 1

    if not found_start:
        return lines

    channels = [[], [], [], [], []]
    
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line == "END_PLOT" or not line:
            break
            
        try:
            vals = line.split(',')
            if len(vals) == 5:
                for i in range(5): 
                    channels[i].append(float(vals[i]))
        except:
            pass

    # Render the perfectly aligned frame directly into the 200-350 window
    if len(channels[0]) > 0:
        for i in range(5):
            x_data = range(len(channels[i])) 
            lines[i].set_data(x_data, channels[i])
            
    return lines

# Execute the live animation
ani = animation.FuncAnimation(fig, update_graph, interval=100, blit=True, cache_frame_data=False)
plt.tight_layout()
plt.show()
ser.close()