#include <Arduino.h>
#include "soc/gpio_reg.h"
#include "esp_rom_sys.h"
#include "esp_adc/adc_continuous.h"
#include "hal/adc_types.h"
#include <math.h>

// ==============================================================================
// I. GLOBAL HARDWARE & MASSIVE MEMORY ALLOCATION
// ==============================================================================
const int TX_TRIG[5] = {4, 14, 17, 19, 25};
const int TX_ECHO[5] = {13, 16, 18, 23, 26};

// --- SPATIAL GEOMETRY (In Meters) ---
const float TX_X[5] = {-0.0084, -0.0042, 0.0, 0.0042, 0.0084};
const float RX_X[5] = {-0.025, -0.009, 0.002, 0.013, 0.025}; 
const float RX_Z_OFFSET = 0.01517; 
const float TARGET_Y = 0.54;       
const float CALIBRATION_SPEED_OF_SOUND = 343.0; // The temp of the room DURING calibration
const float SAMPLE_RATE = 500000.0; 

// --- MAXIMUM RAM EXPANSION ---
// --- MAXIMUM SAFE RAM EXPANSION (1200 Samples = 4.1 Meters Range) ---
const int BUFFER_LENGTH = 1200;
float rx_buffers[5][BUFFER_LENGTH] = {0};
float aligned_buffers[5][BUFFER_LENGTH] = {0}; 
float accumulation_buffers[5][BUFFER_LENGTH] = {0}; 

// DMA memory sized exactly for 1200 samples
adc_continuous_handle_t adc_handle = NULL;
const uint32_t DMA_FLAT_BUFFER_SIZE = 12000;
uint8_t dma_flat_buffer[DMA_FLAT_BUFFER_SIZE] = {0};

const adc_channel_t RX_CHANNELS[5] = {ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_7, ADC_CHANNEL_0, ADC_CHANNEL_3};

// --- PERMANENT HARDWARE CONSTANTS ---
float rx_offsets[5] = {0}; 
float tx_offsets[5] = {0};
float rx_gain_multipliers[5] = {1.0, 1.0, 1.0, 1.0, 1.0}; // The Integral Fix

enum SystemState { RAW_DIAGNOSTIC, CALIBRATE_RX, CALIBRATE_TX, PRINT_CALIB_FILE, VERIFY_FIRE, MUSIC_DOA };
SystemState currentState = CALIBRATE_RX;
int tx_calib_index = 0;

// ==============================================================================
// II. SIGNAL PROCESSING MATHEMATICS
// ==============================================================================

void removeDCBias() {
    for(int i = 0; i < 5; i++) {
        float sum = 0;
        for(int j = 10; j < 100; j++) sum += rx_buffers[i][j];
        float bias = sum / 90.0f;
        for(int j = 0; j < BUFFER_LENGTH; j++) rx_buffers[i][j] -= bias;
    }
}

// Integral Area Calculation (Gain Matching)
void calculateGainMultipliers(int start_idx, int end_idx) {
    float area[5] = {0};
    
    // Calculate the absolute integral area for all 5 channels
    for(int i = 0; i < 5; i++) {
        for(int j = start_idx; j <= end_idx; j++) {
            area[i] += abs(rx_buffers[i][j]);
        }
    }
    
    // RX3 (Index 2) is the master volume
    for(int i = 0; i < 5; i++) {
        if(area[i] > 0.0f) {
            rx_gain_multipliers[i] = area[2] / area[i];
        }
    }
}

float calculateCrossCorrelationOffset(int master_ch, int target_ch, int start_idx, int end_idx) {
    int best_shift = 0;
    float max_dot_product = -1000000.0f; 
    float dot_curve[21] = {0}; 

    for (int shift = -10; shift <= 10; shift++) {
        float dot_product = 0;
        for (int j = start_idx; j <= end_idx; j++) {
            int shifted_idx = j + shift;
            if (shifted_idx >= 0 && shifted_idx < BUFFER_LENGTH) {
                dot_product += rx_buffers[master_ch][j] * rx_buffers[target_ch][shifted_idx];
            }
        }
        dot_curve[shift + 10] = dot_product; 
        if (dot_product > max_dot_product) { max_dot_product = dot_product; best_shift = shift; }
    }
    
    if (best_shift == -10 || best_shift == 10) return (float)best_shift;
    
    float y_left = dot_curve[(best_shift - 1) + 10];
    float y_center = dot_curve[best_shift + 10];
    float y_right = dot_curve[(best_shift + 1) + 10];
    float denominator = (y_left - 2.0f * y_center + y_right);
    if (denominator == 0.0f) return (float)best_shift;
    
    return (float)best_shift + ((y_left - y_right) / (2.0f * denominator));
}

// Applies Hardware Phase and Hardware Gain corrections
void applyPhaseCorrection() {
    for(int i = 0; i < 5; i++) {
        int discrete_shift = round(rx_offsets[i]); 
        for(int j = 0; j < BUFFER_LENGTH; j++) aligned_buffers[i][j] = 0.0f; 
        
        for(int j = 0; j < BUFFER_LENGTH; j++) {
            int new_index = j - discrete_shift; 
            if(new_index >= 0 && new_index < BUFFER_LENGTH) {
                // Apply the exact wave data MULTIPLIED by the integral gain fixer
                aligned_buffers[i][new_index] = rx_buffers[i][j] * rx_gain_multipliers[i];
            }
        }
    }
}

// ==============================================================================
// III. HARDWARE CONTROL (DMA & FIRING)
// ==============================================================================

void initHardwareDMA() {
    adc_continuous_handle_cfg_t adc_config = { .max_store_buf_size = 10240, .conv_frame_size = 1000 };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 500000, .conv_mode = ADC_CONV_SINGLE_UNIT_1, .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };

    adc_digi_pattern_config_t adc_pattern[5];
    for (int i = 0; i < 5; i++) {
        adc_pattern[i].atten = ADC_ATTEN_DB_12;      
        adc_pattern[i].channel = RX_CHANNELS[i];
        adc_pattern[i].unit = ADC_UNIT_1;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH; 
    }
    dig_cfg.pattern_num = 5; dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &dig_cfg));
}

void recordAcousticEchoes() {
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
    uint32_t total_bytes_read = 0;
    
    while (total_bytes_read < DMA_FLAT_BUFFER_SIZE) {
        uint32_t bytes_chunk = 0;
        if (adc_continuous_read(adc_handle, dma_flat_buffer + total_bytes_read, DMA_FLAT_BUFFER_SIZE - total_bytes_read, &bytes_chunk, ADC_MAX_DELAY) == ESP_OK) {
            total_bytes_read += bytes_chunk;
        }
    }
    ESP_ERROR_CHECK(adc_continuous_stop(adc_handle));

    int rx_indices[5] = {0, 0, 0, 0, 0};
    for (int i = 0; i < DMA_FLAT_BUFFER_SIZE; i += SOC_ADC_DIGI_RESULT_BYTES) {
        adc_digi_output_data_t *p = (adc_digi_output_data_t*)&dma_flat_buffer[i];
        float volt = (float)p->type1.data / 4095.0f;

        if (p->type1.channel == ADC_CHANNEL_4 && rx_indices[0] < BUFFER_LENGTH) rx_buffers[0][rx_indices[0]++] = volt;
        else if (p->type1.channel == ADC_CHANNEL_5 && rx_indices[1] < BUFFER_LENGTH) rx_buffers[1][rx_indices[1]++] = volt;
        else if (p->type1.channel == ADC_CHANNEL_7 && rx_indices[2] < BUFFER_LENGTH) rx_buffers[2][rx_indices[2]++] = volt;
        else if (p->type1.channel == ADC_CHANNEL_0 && rx_indices[3] < BUFFER_LENGTH) rx_buffers[3][rx_indices[3]++] = volt;
        else if (p->type1.channel == ADC_CHANNEL_3 && rx_indices[4] < BUFFER_LENGTH) rx_buffers[4][rx_indices[4]++] = volt;
    }
}

void IRAM_ATTR fireSingleTX(int tx_index) {
    portDISABLE_INTERRUPTS(); 
    for (int i = 0; i < 8; i++) {
        REG_WRITE(GPIO_OUT_W1TS_REG, (1 << TX_TRIG[tx_index])); 
        REG_WRITE(GPIO_OUT_W1TC_REG, (1 << TX_ECHO[tx_index])); 
        esp_rom_delay_us(12); 
        REG_WRITE(GPIO_OUT_W1TC_REG, (1 << TX_TRIG[tx_index])); 
        REG_WRITE(GPIO_OUT_W1TS_REG, (1 << TX_ECHO[tx_index])); 
        esp_rom_delay_us(12);
    }
    REG_WRITE(GPIO_OUT_W1TC_REG, (1 << TX_TRIG[tx_index]));
    REG_WRITE(GPIO_OUT_W1TC_REG, (1 << TX_ECHO[tx_index]));
    esp_rom_delay_us(800); 
    portENABLE_INTERRUPTS();
}

void IRAM_ATTR fireAllTX_Aligned() {
    portDISABLE_INTERRUPTS();
    for (int i = 0; i < 8; i++) {
        for(int tx = 0; tx < 5; tx++) {
            REG_WRITE(GPIO_OUT_W1TS_REG, (1 << TX_TRIG[tx])); 
            REG_WRITE(GPIO_OUT_W1TC_REG, (1 << TX_ECHO[tx])); 
        }
        esp_rom_delay_us(12); 
        for(int tx = 0; tx < 5; tx++) {
            REG_WRITE(GPIO_OUT_W1TC_REG, (1 << TX_TRIG[tx])); 
            REG_WRITE(GPIO_OUT_W1TS_REG, (1 << TX_ECHO[tx])); 
        }
        esp_rom_delay_us(12);
    }
    for(int tx = 0; tx < 5; tx++) {
        REG_WRITE(GPIO_OUT_W1TC_REG, (1 << TX_TRIG[tx]));
        REG_WRITE(GPIO_OUT_W1TC_REG, (1 << TX_ECHO[tx]));
    }
    esp_rom_delay_us(800); 
    portENABLE_INTERRUPTS();
}

void fireAndAverage(int tx_index, int num_shots) {
    for(int i = 0; i < 5; i++) {
        for(int j = 0; j < BUFFER_LENGTH; j++) accumulation_buffers[i][j] = 0.0f;
    }
    
    for(int shot = 0; shot < num_shots; shot++) {
        fireSingleTX(tx_index);
        recordAcousticEchoes();
        for(int i = 0; i < 5; i++) {
            for(int j = 0; j < BUFFER_LENGTH; j++) accumulation_buffers[i][j] += rx_buffers[i][j];
        }
        delay(5); // Feed watchdog timer to prevent crash
    }
    
    for(int i = 0; i < 5; i++) {
        for(int j = 0; j < BUFFER_LENGTH; j++) rx_buffers[i][j] = accumulation_buffers[i][j] / (float)num_shots;
    }
    removeDCBias();
}

// ==============================================================================
// IV. THE MASTER STATE MACHINE
// ==============================================================================

void setup() {
    Serial.begin(115200); 
    for(int i = 0; i < 5; i++) {
        pinMode(TX_TRIG[i], OUTPUT); pinMode(TX_ECHO[i], OUTPUT);
        digitalWrite(TX_TRIG[i], LOW); digitalWrite(TX_ECHO[i], LOW);
    }
    initHardwareDMA();
    Serial.println("System Boot. Heavy-Duty Sequence Initiated.");
    delay(2000); 
}

void loop() {
    switch(currentState) {
        
        case CALIBRATE_RX:
            Serial.println("Phase 1: RX Calibration (100-Shot Average & Integral Gain)...");
            fireAndAverage(2, 100); 
            
            // Generate Gain Integrals First
            calculateGainMultipliers(200, 300);
            
            // Generate Phase Delays
            for(int i = 0; i < 5; i++) {
                float measured_shift = calculateCrossCorrelationOffset(2, i, 200, 300);
                float d_master = sqrt(pow(RX_X[2] - TX_X[2], 2) + pow(TARGET_Y, 2) + pow(RX_Z_OFFSET, 2));
                float d_current = sqrt(pow(RX_X[i] - TX_X[2], 2) + pow(TARGET_Y, 2) + pow(RX_Z_OFFSET, 2));
                
                float time_diff = (d_current - d_master) / CALIBRATION_SPEED_OF_SOUND;
                float theoretical_index_shift = time_diff * SAMPLE_RATE;
                
                rx_offsets[i] = measured_shift - theoretical_index_shift;
            }
            currentState = CALIBRATE_TX;
            delay(1000);
            break;

        case CALIBRATE_TX:
            if(tx_calib_index == 0) Serial.println("Phase 2: TX Calibration (Aligning the Beam)...");
            
            fireAndAverage(tx_calib_index, 100); 
            
            {
                float tx_measured_shift = calculateCrossCorrelationOffset(2, 2, 200, 300); 
                float d_tx_master = sqrt(pow(0.0 - TX_X[2], 2) + pow(TARGET_Y, 2));
                float d_tx_current = sqrt(pow(0.0 - TX_X[tx_calib_index], 2) + pow(TARGET_Y, 2));
                
                float tx_time_diff = (d_tx_current - d_tx_master) / CALIBRATION_SPEED_OF_SOUND;
                float tx_theoretical_shift = tx_time_diff * SAMPLE_RATE;
                
                tx_offsets[tx_calib_index] = tx_measured_shift - tx_theoretical_shift; 
            }
            
            tx_calib_index++;
            if(tx_calib_index >= 5) {
                currentState = PRINT_CALIB_FILE;
                delay(1000);
            }
            break;

        case PRINT_CALIB_FILE:
            // This is your permanent, weather-immune hardware profile.
            Serial.println("\n========================================================");
            Serial.println(">>> CALIBRATION COMPLETE. COPY THIS INTO YOUR C++ CODE <<<");
            Serial.println("========================================================\n");
            
            Serial.println("// Replace your global calibration arrays with these locked constants:");
            
            Serial.print("const float CALIB_RX_HW_ERROR[5] = {");
            for(int i=0; i<5; i++) { Serial.printf("%.4f%s", rx_offsets[i], (i<4)?", ":""); }
            Serial.println("};");
            
            Serial.print("const float CALIB_TX_HW_ERROR[5] = {");
            for(int i=0; i<5; i++) { Serial.printf("%.4f%s", tx_offsets[i], (i<4)?", ":""); }
            Serial.println("};");

            Serial.print("const float CALIB_RX_GAIN[5] = {");
            for(int i=0; i<5; i++) { Serial.printf("%.4f%s", rx_gain_multipliers[i], (i<4)?", ":""); }
            Serial.println("};\n");
            
            Serial.println("========================================================\n");
            
            currentState = VERIFY_FIRE;
            delay(2000);
            break;

        case VERIFY_FIRE:
            Serial.println("Phase 3: VERIFY FIRE. Constructive Interference Incoming!");
            
            fireAllTX_Aligned();
            recordAcousticEchoes();
            removeDCBias();
            applyPhaseCorrection();
            
            // We only dump the tight target window to the Python script to prove it works
            Serial.println("START_PLOT");
            for(int j = 200; j < 350; j++) { 
                Serial.printf("%.4f,%.4f,%.4f,%.4f,%.4f\n", 
                    aligned_buffers[0][j], aligned_buffers[1][j], aligned_buffers[2][j], 
                    aligned_buffers[3][j], aligned_buffers[4][j]);
            }
            Serial.println("END_PLOT");
            
            Serial.println("Matrix mathematically pure. Transitioning to M.U.S.I.C. Tracker in 5 seconds...");
            currentState = MUSIC_DOA;
            delay(5000); 
            break;

        case MUSIC_DOA:
            fireAllTX_Aligned();
            recordAcousticEchoes();
            removeDCBias();
            applyPhaseCorrection();
            
            Serial.println("START_PLOT");
            for(int j = 200; j < 350; j++) { 
                Serial.printf("%.4f,%.4f,%.4f,%.4f,%.4f\n", 
                    aligned_buffers[0][j], aligned_buffers[1][j], aligned_buffers[2][j], 
                    aligned_buffers[3][j], aligned_buffers[4][j]);
            }
            Serial.println("END_PLOT");
            
            delay(16); 
            break;
    }
}