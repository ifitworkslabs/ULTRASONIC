#include <Arduino.h>
#include "soc/gpio_reg.h"
#include "esp_rom_sys.h"
#include "esp_adc/adc_continuous.h"
#include "hal/adc_types.h"
#include <math.h>

// ==============================================================================
// I. THE GOLDEN CALIBRATION KEYS 
// ==============================================================================
const int TX_TRIG[5] = {4, 14, 17, 19, 25};
const int TX_ECHO[5] = {13, 16, 18, 23, 26};

const int CALIB_TX_HW_TICKS[5] = {0, 1800, 1200, 1920, 120};
const float CALIB_RX_HW_ERROR[5] = {8.0000, -8.0000, 0.0000, 0.0000, -8.0000}; 

const int CAPTURE_OFFSET = 1400; 
const int WINDOW_SIZE = 2200;    

// THE MEMORY FIX: Switched from 32-bit Floats to 16-bit Integers (Saves 44 KB RAM)
int16_t rx_buffers[5][WINDOW_SIZE] = {0};
int16_t accumulation_buffers[5][WINDOW_SIZE] = {0}; 

adc_continuous_handle_t adc_handle = NULL;
const uint32_t DMA_FLAT_BUFFER_SIZE = 40000; 
uint8_t dma_flat_buffer[DMA_FLAT_BUFFER_SIZE] = {0};
const adc_channel_t RX_CHANNELS[5] = {ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_7, ADC_CHANNEL_0, ADC_CHANNEL_3};

const float SCAN_ANGLES[7] = {-60.0, -40.0, -20.0, 0.0, 20.0, 40.0, 60.0};
int current_scan_idx = 0;

// ==============================================================================
// II. 1.25 MHz DMA CORE
// ==============================================================================
void initHardwareDMA() {
    adc_continuous_handle_cfg_t adc_config = { .max_store_buf_size = 20480, .conv_frame_size = 2000 };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 1250000, 
        .conv_mode = ADC_CONV_SINGLE_UNIT_1, .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
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
        
        // Grab the raw 12-bit integer from the silicon
        int16_t raw_val = (int16_t)p->type1.data;
        
        int ch = -1;
        if (p->type1.channel == ADC_CHANNEL_4) ch = 0;
        else if (p->type1.channel == ADC_CHANNEL_5) ch = 1;
        else if (p->type1.channel == ADC_CHANNEL_7) ch = 2;
        else if (p->type1.channel == ADC_CHANNEL_0) ch = 3;
        else if (p->type1.channel == ADC_CHANNEL_3) ch = 4;

        if (ch != -1 && rx_indices[ch] >= CAPTURE_OFFSET && rx_indices[ch] < CAPTURE_OFFSET + WINDOW_SIZE) {
            rx_buffers[ch][rx_indices[ch] - CAPTURE_OFFSET] = raw_val;
        }
        if(ch != -1) rx_indices[ch]++;
    }
}

void removeDCBias() {
    for(int i = 0; i < 5; i++) {
        // Use a 32-bit integer for the sum to safely absorb 9,000,000 without overflowing
        int32_t sum = 0; 
        for(int j = 0; j < WINDOW_SIZE; j++) sum += rx_buffers[i][j];
        
        int16_t bias = sum / WINDOW_SIZE;
        for(int j = 0; j < WINDOW_SIZE; j++) rx_buffers[i][j] -= bias;
    }
}

// ==============================================================================
// III. DYNAMIC STEERING ENGINE
// ==============================================================================
void IRAM_ATTR fireSteeredBeam(float angle_degrees) {
    uint32_t half_period = 3000; 
    uint32_t full_period = 6000; 
    uint32_t transitions[5][16];
    
    float angle_rad = angle_degrees * (M_PI / 180.0);
    int tick_step = round(3000.0 * sin(angle_rad));

    int min_tick = 0;
    for(int i = 0; i < 5; i++) {
        int raw_delay = CALIB_TX_HW_TICKS[i] + (i * tick_step);
        if(raw_delay < min_tick) min_tick = raw_delay;
    }
    
    for(int i = 0; i < 5; i++) {
        int final_delay = CALIB_TX_HW_TICKS[i] + (i * tick_step) - min_tick;
        for(int p = 0; p < 8; p++) {
            transitions[i][p*2]     = final_delay + (p * full_period);               
            transitions[i][p*2 + 1] = final_delay + (p * full_period) + half_period; 
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

void fireBeamAndAverage(float angle, int num_shots) {
    for(int i=0; i<5; i++) {
        for(int j=0; j<WINDOW_SIZE; j++) accumulation_buffers[i][j] = 0;
    }
    for(int shot=0; shot<num_shots; shot++) {
        fireSteeredBeam(angle);
        recordAcousticEchoes();
        for(int i=0; i<5; i++) {
            for(int j=0; j<WINDOW_SIZE; j++) accumulation_buffers[i][j] += rx_buffers[i][j];
        }
        delay(3); 
    }
    for(int i=0; i<5; i++) {
        for(int j=0; j<WINDOW_SIZE; j++) rx_buffers[i][j] = accumulation_buffers[i][j] / num_shots;
    }
    removeDCBias();
}

// ==============================================================================
// IV. AESA LIVE LOOP
// ==============================================================================
void setup() {
    Serial.begin(115200); 
    for(int i = 0; i < 5; i++) {
        pinMode(TX_TRIG[i], OUTPUT); pinMode(TX_ECHO[i], OUTPUT);
        digitalWrite(TX_TRIG[i], LOW); digitalWrite(TX_ECHO[i], LOW);
    }
    initHardwareDMA();
    Serial.println("System Boot. RAM-Optimized AESA Engine Online.");
    delay(1000); 
}

void loop() {
    float current_angle = SCAN_ANGLES[current_scan_idx];
    
    fireBeamAndAverage(current_angle, 3); 
    Serial.printf("TWS_FRAME_START:%d\n", (int)current_angle);
    
    for(int j = 0; j < WINDOW_SIZE; j++) { 
        for(int ch = 0; ch < 5; ch++) {
            float speed_scale = 1250000.0f / 2000000.0f;
            int shift = round(CALIB_RX_HW_ERROR[ch] * speed_scale); 
            int original_idx = j + shift; 
            
            // Translate the fast memory integers back into decimal voltage for Python
            float val = (original_idx >= 0 && original_idx < WINDOW_SIZE) ? 
                        ((float)rx_buffers[ch][original_idx] / 4095.0f) : 0.0f;
                        
            Serial.printf("%.4f%s", val, (ch==4) ? "\n" : ",");
        }
    }
    Serial.println("TWS_FRAME_END");
    
    current_scan_idx++;
    if(current_scan_idx >= 7) current_scan_idx = 0;
    
    delay(20); 
}