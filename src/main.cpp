#include <Arduino.h>
#include "soc/gpio_reg.h"
#include "esp_rom_sys.h"
#include "esp_adc/adc_continuous.h"
#include "hal/adc_types.h"
#include <math.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

const int TX_TRIG[5] = {4, 14, 17, 19, 25};
const int TX_ECHO[5] = {13, 16, 18, 23, 26};
const adc_channel_t RX_CHANNELS[5] = {ADC_CHANNEL_5, ADC_CHANNEL_4, ADC_CHANNEL_7, ADC_CHANNEL_3, ADC_CHANNEL_0};

// === 1. CALIBRATED HARDWARE CONSTANTS ===
const int CALIB_TX_HW_TICKS[5] = {120, 1680, 1200, 2280, 0};
const int CALIB_RX_HW_ERROR[5] = {-16, 16, 0, -16, 8};

// === 2. ENVIRONMENTAL PHYSICS ===
// Based on T=30.21C, H=70.84%
const float SPEED_OF_SOUND = 350.48; 
const float TX_POSITIONS[5] = {-0.0084, -0.0042, 0.0000, 0.0042, 0.0084}; // in meters (4.2mm apart)

// === 3. RADAR ENGINE CONFIG ===
const int CAPTURE_OFFSET = 800; // Match debug.py
const int WINDOW_SIZE = 3600;   // Match debug.py (3600 samples)

int16_t rx_buffers[5][WINDOW_SIZE + 40] = {0}; // Extra padding for shifts
int16_t tx_buffer[WINDOW_SIZE * 5]; 

adc_continuous_handle_t adc_handle = NULL;
uint8_t dma_chunk_buffer[4000] = {0}; // Tiny 4KB streaming buffer instead of 90KB!

void initHardwareDMA() {
    adc_continuous_handle_cfg_t adc_config = { .max_store_buf_size = 30000, .conv_frame_size = 2000 };
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

void IRAM_ATTR fireSteeredBeam(float angle_deg) {
    float angle_rad = angle_deg * PI / 180.0f;
    
    int delays[5];
    for(int i=0; i<5; i++) {
        float delta_t = (TX_POSITIONS[i] * sin(angle_rad)) / SPEED_OF_SOUND;
        delays[i] = (int)(delta_t * 240000000.0f);
    }
    
    int min_d = delays[0];
    for(int i=1; i<5; i++) if(delays[i] < min_d) min_d = delays[i];
    
    for(int i=0; i<5; i++) {
        delays[i] = delays[i] - min_d + CALIB_TX_HW_TICKS[i];
    }
    
    uint32_t half_period = 3000; 
    uint32_t full_period = 6000; 
    uint32_t transitions[5][16];
    
    for(int i = 0; i < 5; i++) {
        for(int p = 0; p < 8; p++) {
            transitions[i][p*2]     = delays[i] + (p * full_period);               
            transitions[i][p*2 + 1] = delays[i] + (p * full_period) + half_period; 
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

void recordAcousticEchoes() {
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));

    int rx_indices[5] = {0, 0, 0, 0, 0};
    bool done = false;
    
    while (!done) {
        uint32_t bytes_chunk = 0;
        if (adc_continuous_read(adc_handle, dma_chunk_buffer, sizeof(dma_chunk_buffer), &bytes_chunk, ADC_MAX_DELAY) == ESP_OK) {
            for (int i = 0; i < bytes_chunk; i += SOC_ADC_DIGI_RESULT_BYTES) {
                adc_digi_output_data_t *p = (adc_digi_output_data_t*)&dma_chunk_buffer[i];
                int16_t val = (int16_t)p->type1.data;
                
                int ch = -1;
                if (p->type1.channel == ADC_CHANNEL_5) ch = 0;
                else if (p->type1.channel == ADC_CHANNEL_4) ch = 1;
                else if (p->type1.channel == ADC_CHANNEL_7) ch = 2;
                else if (p->type1.channel == ADC_CHANNEL_3) ch = 3;
                else if (p->type1.channel == ADC_CHANNEL_0) ch = 4;

                if (ch != -1) {
                    if (rx_indices[ch] >= CAPTURE_OFFSET && rx_indices[ch] < CAPTURE_OFFSET + WINDOW_SIZE + 40) {
                        rx_buffers[ch][rx_indices[ch] - CAPTURE_OFFSET] = val;
                    }
                    rx_indices[ch]++;
                }
            }
        }
        
        done = true;
        for(int ch=0; ch<5; ch++) {
            if(rx_indices[ch] < CAPTURE_OFFSET + WINDOW_SIZE + 40) {
                done = false;
                break;
            }
        }
    }
    ESP_ERROR_CHECK(adc_continuous_stop(adc_handle));
}

void setup() {
    Serial.begin(576000); 
    for(int i = 0; i < 5; i++) {
        pinMode(TX_TRIG[i], OUTPUT); pinMode(TX_ECHO[i], OUTPUT);
        digitalWrite(TX_TRIG[i], LOW); digitalWrite(TX_ECHO[i], LOW);
    }
    initHardwareDMA();
}

float scan_angle = -45.0f;
float scan_dir = 5.0f;

void loop() {
    fireSteeredBeam(scan_angle);
    recordAcousticEchoes();
    
    // Hardware RX Error Correction & Matrix Interleaving
    int idx = 0;
    for(int j = 0; j < WINDOW_SIZE; j++) {
        for(int ch = 0; ch < 5; ch++) {
            int shift = CALIB_RX_HW_ERROR[ch];
            int read_idx = j + shift + 20; 
            if(read_idx < 0) read_idx = 0;
            if(read_idx >= WINDOW_SIZE + 40) read_idx = WINDOW_SIZE + 39;
            tx_buffer[idx++] = rx_buffers[ch][read_idx];
        }
    }
    
    // High-Speed Binary Payload
    uint8_t sync[4] = {0xAA, 0xBB, 0xCC, 0xDD};
    Serial.write(sync, 4);
    Serial.write((uint8_t*)&scan_angle, 4);
    Serial.write((uint8_t*)tx_buffer, WINDOW_SIZE * 5 * 2);
    
    // Sweep the Radar
    scan_angle += scan_dir;
    if(scan_angle > 45.0f) {
        scan_angle = 45.0f;
        scan_dir = -5.0f;
    } else if (scan_angle < -45.0f) {
        scan_angle = -45.0f;
        scan_dir = 5.0f;
    }
    
    delay(5); // Small breather for serial buffer
}