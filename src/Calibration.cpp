#include "Calibration.h"
#include <math.h>

// The TX Array (The Mouths) - Centered at Z = 0
const float TX_X[5] = {-0.0084, -0.0042, 0.0, 0.0042, 0.0084};

// The RX Array (The Ears) - Offset Z = 15.17 mm
const float RX_X[5] = {-0.025, -0.009, 0.002, 0.013, 0.025}; 

float calculateSubSamplePeak(float y_left, float y_center, float y_right, int peak_index) {
    float denominator = (y_left - 2.0f * y_center + y_right);
    if (denominator == 0.0f) return (float)peak_index; 
    float delta_x = (y_left - y_right) / (2.0f * denominator);
    return (float)peak_index + delta_x;
}

// ---------------------------------------------------------
// PHASE 1: CALIBRATE THE EARS (Listen to Center TX3)
// ---------------------------------------------------------
void calibrateReceivers(float** raw_rx_buffers, int buffer_length, CalibrationData& calData, float speed_of_sound) {
    float tx_center_x = TX_X[2]; 
    float d_tx_to_target = sqrt(pow(0.0f - tx_center_x, 2) + pow(TARGET_DISTANCE_Y, 2));

    for(int i = 0; i < 5; i++) {
        float d_target_to_rx = sqrt(pow(0.0f - RX_X[i], 2) + pow(TARGET_DISTANCE_Y, 2) + pow(0.0f - RX_Z_OFFSET, 2));
        float theoretical_tof = (d_tx_to_target + d_target_to_rx) / speed_of_sound;

        // SOFTWARE TIME GATE: Define a strict search window (+/- 50 samples = +/- 500 us)
        int expected_index = round(theoretical_tof * SAMPLE_RATE);
        int search_start = expected_index - 50;
        int search_end = expected_index + 50;
        
        // Safety bounds
        if (search_start < 1) search_start = 1;
        if (search_end > buffer_length - 2) search_end = buffer_length - 2;

        int max_idx = search_start; 
        float max_val = 0.0f;
        
        // Search ONLY within the time gate, ignoring the wall completely
        for(int j = search_start; j <= search_end; j++) {
            if(raw_rx_buffers[i][j] > max_val) { 
                max_val = raw_rx_buffers[i][j]; 
                max_idx = j; 
            }
        }

        float true_peak_idx = calculateSubSamplePeak(raw_rx_buffers[i][max_idx-1], raw_rx_buffers[i][max_idx], raw_rx_buffers[i][max_idx+1], max_idx);
        float measured_tof = true_peak_idx / SAMPLE_RATE;
        
        calData.rx_error[i] = measured_tof - theoretical_tof;
    }
}

// ---------------------------------------------------------
// PHASE 2: CALIBRATE THE MOUTHS (Speak to Center RX3)
// ---------------------------------------------------------
void calibrateTransmitters(float** raw_rx_buffers, int buffer_length, CalibrationData& calData, float speed_of_sound) {
    float rx_center_x = RX_X[2];
    float d_target_to_rx = sqrt(pow(0.0f - rx_center_x, 2) + pow(TARGET_DISTANCE_Y, 2) + pow(0.0f - RX_Z_OFFSET, 2));

    for(int i = 0; i < 5; i++) {
        float d_tx_to_target = sqrt(pow(0.0f - TX_X[i], 2) + pow(TARGET_DISTANCE_Y, 2));
        float theoretical_tof = (d_tx_to_target + d_target_to_rx) / speed_of_sound;

        // SOFTWARE TIME GATE
        int expected_index = round(theoretical_tof * SAMPLE_RATE);
        int search_start = expected_index - 50;
        int search_end = expected_index + 50;
        if (search_start < 1) search_start = 1;
        if (search_end > buffer_length - 2) search_end = buffer_length - 2;

        int max_idx = search_start; 
        float max_val = 0.0f;
        for(int j = search_start; j <= search_end; j++) {
            if(raw_rx_buffers[i][j] > max_val) { max_val = raw_rx_buffers[i][j]; max_idx = j; }
        }

        float true_peak_idx = calculateSubSamplePeak(raw_rx_buffers[i][max_idx-1], raw_rx_buffers[i][max_idx], raw_rx_buffers[i][max_idx+1], max_idx);
        float measured_tof = true_peak_idx / SAMPLE_RATE;
        
        calData.tx_error[i] = measured_tof - theoretical_tof;
    }
}

// ---------------------------------------------------------
// ARRAY SLIDE: Phase Correction
// ---------------------------------------------------------
void applyPhaseCorrection(float** raw_buffers, float** aligned_buffers, CalibrationData& cal, int length) {
    for(int i = 0; i < 5; i++) {
        int shift_amount = round(cal.rx_error[i] * SAMPLE_RATE);
        for(int j = 0; j < length; j++) aligned_buffers[i][j] = 0.0f;
        
        for(int j = 0; j < length; j++) {
            int new_index = j - shift_amount; 
            if(new_index >= 0 && new_index < length) {
                aligned_buffers[i][new_index] = raw_buffers[i][j];
            }
        }
    }
}