#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>

// --- SYSTEM GEOMETRY (Meters) ---
const float TARGET_DISTANCE_Y = 0.54; // Cylinder at 54 cm
const float RX_Z_OFFSET = 0.01517;    // Vertical Vernier Z-axis offset
const float SAMPLE_RATE = 100000.0;   // 100 kHz I2S DMA sample rate

// --- SPATIAL COORDINATES ---
extern const float TX_X[5];
extern const float RX_X[5];

// --- THE CALIBRATION RECORD ---
struct CalibrationData {
    float rx_error[5]; // Mechanical delays of the receivers (seconds)
    float tx_error[5]; // Mechanical delays of the transmitters (seconds)
};

// --- FUNCTION PROTOTYPES ---
float calculateSubSamplePeak(float y_left, float y_center, float y_right, int peak_index);
void calibrateReceivers(float** raw_rx_buffers, int buffer_length, CalibrationData& calData, float speed_of_sound);
void calibrateTransmitters(float** raw_rx_buffers, int buffer_length, CalibrationData& calData, float speed_of_sound);
void applyPhaseCorrection(float** raw_buffers, float** aligned_buffers, CalibrationData& cal, int length);

#endif