#include <Arduino.h>
#include "soc/gpio_reg.h"
#include "esp_rom_sys.h"
#include "esp_adc/adc_continuous.h"
#include "hal/adc_types.h"
#include <math.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

Adafruit_BME280 bme;


// ==============================================================================
// I. GLOBAL HARDWARE & MASSIVE MEMORY ALLOCATION
// ==============================================================================
const int TX_TRIG[5] = {4, 14, 17, 19, 25};
const int TX_ECHO[5] = {13, 16, 18, 23, 26};

const float TX_X[5] = {-0.0084, -0.0042, 0.0, 0.0042, 0.0084};
const float RX_X[5] = {-0.025, -0.009, 0.002, 0.013, 0.025}; 
const float RX_Z_OFFSET = 0.01517; 
const float TARGET_Y = 0.54;       
volatile float live_speed_of_sound = 343.0;
const float SAMPLE_RATE = 500000.0; 

const int BUFFER_LENGTH = 1200;
float rx_buffers[5][BUFFER_LENGTH] = {0};
float aligned_buffers[5][BUFFER_LENGTH] = {0}; 
float accumulation_buffers[5][BUFFER_LENGTH] = {0}; 

// Your Mathematical Baselines
const float CALIB_RX_HW_ERROR[5] = {0.8878, -2.3402, 0.0011, 0.2319, -2.7091};
const float CALIB_RX_GAIN[5] = {0.8324, 1.0094, 1.0000, 1.0918, 0.9189};

// THE MUTABLE ARRAY: Starts with your baseline, but will be overwritten by the Brute-Force Tuner
float dynamic_tx_error[5] = {3.0758, -0.9833, 0.0014, -0.0937, 4.0451};

adc_continuous_handle_t adc_handle = NULL;
const uint32_t DMA_FLAT_BUFFER_SIZE = 12000;
uint8_t dma_flat_buffer[DMA_FLAT_BUFFER_SIZE] = {0};

const adc_channel_t RX_CHANNELS[5] = {ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_7, ADC_CHANNEL_0, ADC_CHANNEL_3};

// We skip Phase 1 and 2, and boot directly into the Brute Force Auto-Tuner
enum SystemState { EMPIRICAL_AUTO_TUNE, GAIN_COMPARISON, MUSIC_DOA };
SystemState currentState = EMPIRICAL_AUTO_TUNE;
int tune_tx_idx = 0;

// ==============================================================================
// II. SIGNAL PROCESSING MATHEMATICS
// ==============================================================================
// Thermodynamics (Decoupled Background Task on Core 0)
void bme280_task(void *pvParameters) {
    Wire.begin(21, 22); // Initialize I2C on SDA=21, SCL=22
    
    if (!bme.begin(0x76)) { // 0x76 or 0x77 is the standard BME280 I2C address
        Serial.println("BME280 Sensor missing! Defaulting to 343.0 m/s");
        vTaskDelete(NULL); // Terminate task if hardware is missing
    }
    
    while (true) {
        float tempC = bme.readTemperature();
        // Calculate live speed of sound down to the decimal
        live_speed_of_sound = 331.3f + (0.606f * tempC);
        
        // Pause this task for exactly 1000 milliseconds (1 Hz tick rate)
        vTaskDelay(1000 / portTICK_PERIOD_MS); 
    }
}
void removeDCBias() {
    for(int i = 0; i < 5; i++) {
        float sum = 0;
        for(int j = 10; j < 100; j++) sum += rx_buffers[i][j];
        float bias = sum / 90.0f;
        for(int j = 0; j < BUFFER_LENGTH; j++) rx_buffers[i][j] -= bias;
    }
}

void applyPhaseCorrection() {
    for(int i = 0; i < 5; i++) {
        int discrete_shift = round(CALIB_RX_HW_ERROR[i]); 
        for(int j = 0; j < BUFFER_LENGTH; j++) aligned_buffers[i][j] = 0.0f; 
        for(int j = 0; j < BUFFER_LENGTH; j++) {
            int new_index = j - discrete_shift; 
            if(new_index >= 0 && new_index < BUFFER_LENGTH) {
                aligned_buffers[i][new_index] = rx_buffers[i][j] * CALIB_RX_GAIN[i];
            }
        }
    }
}

// Measures the pure acoustic energy bouncing off the cylinder into the center microphone
float getCenterIntegral() {
    float area = 0;
    // Window locked strictly onto your 54cm target echo
    for(int j = 200; j < 300; j++) {
        area += abs(rx_buffers[2][j]);
    }
    return area;
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

void IRAM_ATTR fireZeroDegreeBeam() {
    float max_err = dynamic_tx_error[0];
    for(int i = 1; i < 5; i++) {
        if(dynamic_tx_error[i] > max_err) max_err = dynamic_tx_error[i];
    }

    uint32_t start_delays[5];
    for(int i = 0; i < 5; i++) {
        float index_diff = max_err - dynamic_tx_error[i];
        start_delays[i] = (uint32_t)(index_diff * 480.0f); 
    }

    uint32_t half_period = 3000; 
    uint32_t full_period = 6000; 
    
    uint32_t transitions[5][16];
    for(int i = 0; i < 5; i++) {
        for(int p = 0; p < 8; p++) {
            transitions[i][p*2]     = start_delays[i] + (p * full_period);               
            transitions[i][p*2 + 1] = start_delays[i] + (p * full_period) + half_period; 
        }
    }

    uint8_t state_index[5] = {0, 0, 0, 0, 0};
    bool active = true;

    portDISABLE_INTERRUPTS(); 
    uint32_t start_time = xthal_get_ccount(); 

    while(active) {
        uint32_t current_time = xthal_get_ccount() - start_time;
        active = false;
        for(int i = 0; i < 5; i++) {
            if(state_index[i] < 16) {
                active = true; 
                if(current_time >= transitions[i][state_index[i]]) {
                    if(state_index[i] % 2 == 0) {
                        REG_WRITE(GPIO_OUT_W1TS_REG, (1 << TX_TRIG[i]));
                        REG_WRITE(GPIO_OUT_W1TC_REG, (1 << TX_ECHO[i]));
                    } else {
                        REG_WRITE(GPIO_OUT_W1TC_REG, (1 << TX_TRIG[i]));
                        REG_WRITE(GPIO_OUT_W1TS_REG, (1 << TX_ECHO[i]));
                    }
                    state_index[i]++; 
                }
            }
        }
    }

    for(int i = 0; i < 5; i++) {
        REG_WRITE(GPIO_OUT_W1TC_REG, (1 << TX_TRIG[i]));
        REG_WRITE(GPIO_OUT_W1TC_REG, (1 << TX_ECHO[i]));
    }
    esp_rom_delay_us(800); 
    portENABLE_INTERRUPTS();
}

// Averages 100 shots of the AESA BEAM to perfectly measure Constructive Interference
void fireBeamAndAverage(int num_shots) {
    for(int i = 0; i < 5; i++) {
        for(int j = 0; j < BUFFER_LENGTH; j++) accumulation_buffers[i][j] = 0.0f;
    }
    for(int shot = 0; shot < num_shots; shot++) {
        fireZeroDegreeBeam();
        recordAcousticEchoes();
        for(int i = 0; i < 5; i++) {
            for(int j = 0; j < BUFFER_LENGTH; j++) accumulation_buffers[i][j] += rx_buffers[i][j];
        }
        delay(3); 
    }
    for(int i = 0; i < 5; i++) {
        for(int j = 0; j < BUFFER_LENGTH; j++) rx_buffers[i][j] = accumulation_buffers[i][j] / (float)num_shots;
    }
    removeDCBias();
}

// Averages 100 shots of a SINGLE transmitter (For Gain Comparison)
void fireSingleAndAverage(int tx_index, int num_shots) {
    for(int i = 0; i < 5; i++) {
        for(int j = 0; j < BUFFER_LENGTH; j++) accumulation_buffers[i][j] = 0.0f;
    }
    for(int shot = 0; shot < num_shots; shot++) {
        fireSingleTX(tx_index);
        recordAcousticEchoes();
        for(int i = 0; i < 5; i++) {
            for(int j = 0; j < BUFFER_LENGTH; j++) accumulation_buffers[i][j] += rx_buffers[i][j];
        }
        delay(3); 
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
    
    // Launch the Thermodynamics task on Core 0 (Priority 1)
    xTaskCreatePinnedToCore(bme280_task, "BME_Task", 4096, NULL, 1, NULL, 0);
    
    Serial.println("System Boot. Automated Brute-Force Tuning Initiated.");
    delay(2000); 
}

void loop() {
    switch(currentState) {
        
        case EMPIRICAL_AUTO_TUNE:
            // We skip the center TX (Index 2), it is our static anchor
            if(tune_tx_idx == 2) {
                tune_tx_idx++;
            }
            
            if(tune_tx_idx < 5) {
                Serial.printf("\n--- Brute-Forcing TX%d ---\n", tune_tx_idx + 1);
                
                float baseline = dynamic_tx_error[tune_tx_idx];
                float best_offset = baseline;
                float max_integral = 0;
                
                // Sweep 1000 nudges: -5.0 to +5.0 indices from baseline (in steps of 0.01)
                for(int step = -500; step <= 500; step++) {
                    float test_offset = baseline + (step * 0.01f);
                    dynamic_tx_error[tune_tx_idx] = test_offset;
                    
                    // Fire 100 times to get a perfectly sterile wave, calculate AUC
                    fireBeamAndAverage(100);
                    float current_energy = getCenterIntegral();
                    
                    if(current_energy > max_integral) {
                        max_integral = current_energy;
                        best_offset = test_offset;
                    }
                    
                    if(step % 100 == 0) {
                        Serial.printf("Step %d/500 (Offset: %.2f) | Max Energy so far: %.2f\n", step, test_offset, max_integral);
                    }
                }
                
                // Lock the absolute best offset into memory
                dynamic_tx_error[tune_tx_idx] = best_offset;
                Serial.printf(">>> TX%d LOCKED AT OFFSET: %.4f <<<\n", tune_tx_idx + 1, best_offset);
                
                tune_tx_idx++;
            } else {
                // Done Tuning
                Serial.println("\n========================================================");
                Serial.println(">>> BRUTE-FORCE OPTIMIZATION COMPLETE. COPY THIS ARRAY <<<");
                Serial.println("========================================================\n");
                Serial.print("const float OPTIMIZED_TX_HW_ERROR[5] = {");
                for(int i=0; i<5; i++) { Serial.printf("%.4f%s", dynamic_tx_error[i], (i<4)?", ":""); }
                Serial.println("};\n");
                
                currentState = GAIN_COMPARISON;
                delay(2000);
            }
            break;

        case GAIN_COMPARISON:
            Serial.println("\n--- RUNNING GAIN COMPARISON TEST ---");
            
            // 1. Fire strictly the center transmitter (No Beamforming)
            fireSingleAndAverage(2, 100);
            float single_tx_energy;
            single_tx_energy = getCenterIntegral();
            
            // 2. Fire the fully optimized AESA Beam
            fireBeamAndAverage(100);
            float aesa_tx_energy;
            aesa_tx_energy = getCenterIntegral();
            
            Serial.printf("Single TX Center Energy: %.2f\n", single_tx_energy);
            Serial.printf("AESA Beam Center Energy: %.2f\n", aesa_tx_energy);
            Serial.printf(">>> ACOUSTIC MULTIPLIER: %.2fx GAIN <<<\n", aesa_tx_energy / single_tx_energy);
            
            Serial.println("\nTransitioning to Live M.U.S.I.C. Radar Stream...");
            currentState = MUSIC_DOA;
            delay(5000);
            break;

        case MUSIC_DOA:
            // Continuous AI radar streaming
            fireZeroDegreeBeam();
            recordAcousticEchoes();
            removeDCBias();
            applyPhaseCorrection();
            
            // Serial.println("START_PLOT");
            // for(int j = 200; j < 350; j++) { 
            //     Serial.printf("%.4f,%.4f,%.4f,%.4f,%.4f\n", 
            //         aligned_buffers[0][j], aligned_buffers[1][j], aligned_buffers[2][j], 
            //         aligned_buffers[3][j], aligned_buffers[4][j]);
            // }
            // Serial.println("END_PLOT");
            
            delay(16); 
            break;
    }
}