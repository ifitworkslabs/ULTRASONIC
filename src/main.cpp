#include <Arduino.h>
#include "soc/gpio_reg.h"
#include "esp_rom_sys.h"
#include "esp_adc/adc_continuous.h"
#include "hal/adc_types.h"
#include <math.h>

// ==============================================================================
// I. HARDWARE GEOMETRY & WINDOWED MEMORY ARCHITECTURE
// ==============================================================================
const int TX_TRIG[5] = {4, 14, 17, 19, 25};
const int TX_ECHO[5] = {13, 16, 18, 23, 26};

const int CAPTURE_OFFSET = 1800; 
const int WINDOW_SIZE = 1000;    

const int GATE_START = 100;
const int GATE_END   = 800; 

float rx_buffers[5][WINDOW_SIZE] = {0};
float accumulation_buffers[5][WINDOW_SIZE] = {0}; 

int dynamic_tx_delay_ticks[5] = {12000, 12000, 12000, 12000, 12000};
float calib_rx_hw_error[5] = {0, 0, 0, 0, 0}; 

adc_continuous_handle_t adc_handle = NULL;
const uint32_t DMA_FLAT_BUFFER_SIZE = 30000; 
uint8_t dma_flat_buffer[DMA_FLAT_BUFFER_SIZE] = {0};

const adc_channel_t RX_CHANNELS[5] = {ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_7, ADC_CHANNEL_0, ADC_CHANNEL_3};

enum SystemState { TX_TICK_CLIMB, RX_CALIBRATE, DONE };
SystemState currentState = TX_TICK_CLIMB;

int current_step_ticks = 480; 
bool array_changed = false;
int current_tx = 0;
int sweep_counter = 0; 

// ==============================================================================
// II. MAXIMUM FREQUENCY DMA CORE (2 MHz)
// ==============================================================================
void initHardwareDMA() {
    adc_continuous_handle_cfg_t adc_config = { .max_store_buf_size = 20480, .conv_frame_size = 2000 };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 2000000, .conv_mode = ADC_CONV_SINGLE_UNIT_1, .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
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
        
        int ch = -1;
        if (p->type1.channel == ADC_CHANNEL_5) ch = 0;      
        else if (p->type1.channel == ADC_CHANNEL_4) ch = 1; 
        else if (p->type1.channel == ADC_CHANNEL_7) ch = 2; 
        else if (p->type1.channel == ADC_CHANNEL_3) ch = 3; 
        else if (p->type1.channel == ADC_CHANNEL_0) ch = 4; 

        if (ch != -1) {
            if (rx_indices[ch] >= CAPTURE_OFFSET && rx_indices[ch] < CAPTURE_OFFSET + WINDOW_SIZE) {
                rx_buffers[ch][rx_indices[ch] - CAPTURE_OFFSET] = volt;
            }
            rx_indices[ch]++;
        }
    }
}

void removeDCBias() {
    for(int i = 0; i < 5; i++) {
        float sum = 0;
        for(int j = 10; j < 80; j++) sum += rx_buffers[i][j];
        float bias = sum / 70.0f;
        for(int j = 0; j < WINDOW_SIZE; j++) rx_buffers[i][j] -= bias;
    }
}

float getGatedCenterIntegral() {
    float area = 0;
    for(int j = GATE_START; j <= GATE_END; j++) {
        area += fabs(rx_buffers[2][j]); 
    }
    return area;
}

// ==============================================================================
// III. CLOCK-ACCURATE FIRING CONTROL
// ==============================================================================
void IRAM_ATTR fireZeroDegreeBeam() {
    uint32_t half_period = 3000; 
    uint32_t full_period = 6000; 
    uint32_t transitions[5][16];
    
    for(int i = 0; i < 5; i++) {
        for(int p = 0; p < 8; p++) {
            transitions[i][p*2]     = dynamic_tx_delay_ticks[i] + (p * full_period);               
            transitions[i][p*2 + 1] = dynamic_tx_delay_ticks[i] + (p * full_period) + half_period; 
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

void fireBeamAndAverage(int num_shots) {
    for(int i = 0; i < 5; i++) {
        for(int j = 0; j < WINDOW_SIZE; j++) accumulation_buffers[i][j] = 0.0f;
    }
    for(int shot = 0; shot < num_shots; shot++) {
        fireZeroDegreeBeam();
        recordAcousticEchoes();
        for(int i = 0; i < 5; i++) {
            for(int j = 0; j < WINDOW_SIZE; j++) accumulation_buffers[i][j] += rx_buffers[i][j];
        }
        delay(3); 
    }
    for(int i = 0; i < 5; i++) {
        for(int j = 0; j < WINDOW_SIZE; j++) rx_buffers[i][j] = accumulation_buffers[i][j] / (float)num_shots;
    }
    removeDCBias();
}

// ==============================================================================
// IV. THE PRISTINE ENGINE
// ==============================================================================
void setup() {
    Serial.begin(115200); 
    for(int i = 0; i < 5; i++) {
        pinMode(TX_TRIG[i], OUTPUT); pinMode(TX_ECHO[i], OUTPUT);
        digitalWrite(TX_TRIG[i], LOW); digitalWrite(TX_ECHO[i], LOW);
    }
    initHardwareDMA();
    
    Serial.println("System Boot. Anti-Aliasing Auto-Tuner Initiated.");
    delay(2000); 
}

void loop() {
    switch(currentState) {
        
        case TX_TICK_CLIMB: {
            if(current_tx == 2) {
                current_tx++; 
                break;
            }

            int center_val = dynamic_tx_delay_ticks[current_tx];
            int test_vals[3] = {center_val - current_step_ticks, center_val, center_val + current_step_ticks};
            float energies[3] = {0, 0, 0};

            for(int i=0; i<3; i++) {
                dynamic_tx_delay_ticks[current_tx] = test_vals[i];
                fireBeamAndAverage(100);
                energies[i] = getGatedCenterIntegral();
            }

            int best_idx = 1; 
            if(energies[0] > energies[1] && energies[0] > energies[2]) best_idx = 0;
            if(energies[2] > energies[1] && energies[2] > energies[0]) best_idx = 2;

            if(best_idx != 1) {
                dynamic_tx_delay_ticks[current_tx] = test_vals[best_idx]; 
                array_changed = true;
                Serial.printf("TX%d STEPPED to %d ticks (Energy: %.2f)\n", current_tx, test_vals[best_idx], energies[best_idx]);
            } else {
                dynamic_tx_delay_ticks[current_tx] = center_val; 
            }

            current_tx++;

            if(current_tx >= 5) {
                current_tx = 0;
                sweep_counter++; 
                
                if(!array_changed || sweep_counter >= 3) { 
                    current_step_ticks /= 2; 
                    sweep_counter = 0; 
                    Serial.printf("\n>>> Temporal bounds tightened. New Step: %d CPU Ticks <<<\n\n", current_step_ticks);
                    
                    if(current_step_ticks <= 60) { 
                        Serial.println("===============================================================");
                        Serial.println(">>> TX ARRAY ALIGNED TO <250 NANOSECONDS. AVOIDING NOISE PLATEAU. <<<");
                        Serial.println("===============================================================");
                        currentState = RX_CALIBRATE;
                    }
                }
                array_changed = false; 
            }
            break;
        }

        case RX_CALIBRATE: {
            Serial.println("\n--- COMMENCING DSP CROSS-CORRELATION PHASE ALIGNMENT ---");
            fireBeamAndAverage(100); 

            // 1. Find the gross anchor in the steady-state burst
            int steady_start = 2240 - CAPTURE_OFFSET; 
            int steady_end   = 2350 - CAPTURE_OFFSET; 
            
            float center_max = -1000.0f; 
            int center_peak = steady_start;
            
            for(int j = steady_start; j <= steady_end; j++) {
                if(rx_buffers[2][j] > center_max) {
                    center_max = rx_buffers[2][j];
                    center_peak = j;
                }
            }
            Serial.printf("ANCHOR (RX2) Found at: %d\n", center_peak + CAPTURE_OFFSET);

            // ==============================================================================
            // UPGRADE: Sliding Cross-Correlation Engine
            // ==============================================================================
            int search_range = 15; // Unlocks +/- 15 indices of freedom
            int template_start = center_peak - 25; 
            int template_end = center_peak + 25;   
            
            for(int ch = 0; ch < 5; ch++) {
                if (ch == 2) {
                    calib_rx_hw_error[ch] = 0.0f;
                    continue;
                }
                
                float max_corr = -1000000.0f;
                int best_shift = 0;
                float corr_curve[31] = {0}; 
                
                for(int shift = -search_range; shift <= search_range; shift++) {
                    float current_corr = 0.0f;
                    // Slide the waveforms against each other and calculate the dot-product
                    for(int j = template_start; j <= template_end; j++) {
                        int ch_idx = j + shift;
                        if(ch_idx >= 0 && ch_idx < WINDOW_SIZE) {
                            current_corr += rx_buffers[2][j] * rx_buffers[ch][ch_idx];
                        }
                    }
                    corr_curve[shift + search_range] = current_corr;
                    
                    if(current_corr > max_corr) {
                        max_corr = current_corr;
                        best_shift = shift;
                    }
                }
                
                // Extract Sub-Sample Precision directly from the Correlation Curve
                int b_idx = best_shift + search_range;
                float y_m1 = (b_idx > 0) ? corr_curve[b_idx - 1] : corr_curve[b_idx];
                float y_0  = corr_curve[b_idx];
                float y_p1 = (b_idx < 2 * search_range) ? corr_curve[b_idx + 1] : corr_curve[b_idx];
                
                float denom = (y_m1 - 2*y_0 + y_p1);
                float delta = (denom != 0) ? 0.5f * (y_m1 - y_p1) / denom : 0;
                
                calib_rx_hw_error[ch] = (float)best_shift + delta;

                Serial.printf("RX%d True DSP Lock at Shift: %.2f\n", ch, calib_rx_hw_error[ch]);
            }

            Serial.println("\n=========================================================================");
            Serial.println(">>> PRISTINE CALIBRATION ARRAYS COMPUTED. COPY THESE INTO MEMORY. <<<");
            Serial.println("=========================================================================\n");

            int min_delay = dynamic_tx_delay_ticks[0];
            for(int i=1; i<5; i++) if(dynamic_tx_delay_ticks[i] < min_delay) min_delay = dynamic_tx_delay_ticks[i];

            Serial.print("const int CALIB_TX_HW_TICKS[5] = {");
            for(int i=0; i<5; i++) { Serial.printf("%d%s", dynamic_tx_delay_ticks[i] - min_delay, (i<4)?", ":""); }
            Serial.println("};\n");

            Serial.print("const float CALIB_RX_HW_ERROR[5] = {");
            for(int i=0; i<5; i++) { Serial.printf("%.4f%s", calib_rx_hw_error[i], (i<4)?", ":""); }
            Serial.println("};\n");

            Serial.println("Transmitting ALIGNED high-resolution proof graph to Python...");
            Serial.println("START_FINAL_PLOT");
            
            for(int j = 0; j < WINDOW_SIZE; j++) { 
                Serial.printf("%d,", j + CAPTURE_OFFSET);
                for(int ch = 0; ch < 5; ch++) {
                    int shift = round(calib_rx_hw_error[ch]);
                    int original_idx = j + shift; 
                    float val = 0.0f;
                    if(original_idx >= 0 && original_idx < WINDOW_SIZE) {
                        val = rx_buffers[ch][original_idx];
                    }
                    Serial.printf("%.4f%s", val, (ch==4) ? "\n" : ",");
                }
            }
            Serial.println("END_FINAL_PLOT");
            
            currentState = DONE;
            break;
        }

        case DONE:
            delay(1000); 
            break;
    }
}