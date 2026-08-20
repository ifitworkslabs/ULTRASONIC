#include <Arduino.h>
#include "soc/gpio_reg.h"
#include "esp_rom_sys.h"
#include "esp_adc/adc_continuous.h"
#include "hal/adc_types.h"
#include <math.h>

// ==============================================================================
// I. THE GOLDEN CALIBRATION KEYS (Locked at 2 MHz)
// ==============================================================================
const int TX_TRIG[5] = {4, 14, 17, 19, 25};
const int TX_ECHO[5] = {13, 16, 18, 23, 26};

// Dynamically extracted hardware delays
const int CALIB_TX_HW_TICKS[5] = {600, 1680, 1320, 2040, 0};
const float CALIB_RX_HW_ERROR[5] = {-8.2302, 7.6886, 0.0000, -7.8158, 7.6084};

// High-Resolution 2 MHz Windowing
const int CAPTURE_OFFSET = 1800; 
const int WINDOW_SIZE = 1000;    

float rx_buffers[5][WINDOW_SIZE] = {0};
float accumulation_buffers[5][WINDOW_SIZE] = {0}; 

adc_continuous_handle_t adc_handle = NULL;
const uint32_t DMA_FLAT_BUFFER_SIZE = 30000; 
uint8_t dma_flat_buffer[DMA_FLAT_BUFFER_SIZE] = {0};

// Hardware Polling Sequence
const adc_channel_t RX_CHANNELS[5] = {ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_7, ADC_CHANNEL_0, ADC_CHANNEL_3};

// ==============================================================================
// II. HIGH-RESOLUTION DMA CORE (2 MHz)
// ==============================================================================
void initHardwareDMA() {
    adc_continuous_handle_cfg_t adc_config = { .max_store_buf_size = 20480, .conv_frame_size = 2000 };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 2000000, 
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
        float volt = (float)p->type1.data / 4095.0f;
        
        int ch = -1;
        // The True Physical Geometric Parser (Left to Right)
        if (p->type1.channel == ADC_CHANNEL_5) ch = 0;      // GPIO 33 (Leftmost)
        else if (p->type1.channel == ADC_CHANNEL_4) ch = 1; // GPIO 32 (Mid-Left)
        else if (p->type1.channel == ADC_CHANNEL_7) ch = 2; // GPIO 35 (Center)
        else if (p->type1.channel == ADC_CHANNEL_3) ch = 3; // GPIO 39 (Mid-Right)
        else if (p->type1.channel == ADC_CHANNEL_0) ch = 4; // GPIO 36 (Rightmost)

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

// ==============================================================================
// III. CONTINUOUS FIRING ENGINE
// ==============================================================================
void IRAM_ATTR fireSteeredBeam(float angle_degrees) {
    uint32_t half_period = 3000; 
    uint32_t full_period = 6000; 
    uint32_t transitions[5][16];
    
    // Convert angle to radians and calculate progressive hardware delay
    float angle_rad = angle_degrees * (M_PI / 180.0);
    int tick_step = round(3000.0 * sin(angle_rad));

    int min_tick = 0;
    for(int i = 0; i < 5; i++) {
        int raw_delay = CALIB_TX_HW_TICKS[i] + (i * tick_step);
        if(raw_delay < min_tick) {
            min_tick = raw_delay;
        }
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

void fireBeamAndAverage(int num_shots, float target_angle) {
    for(int i = 0; i < 5; i++) {
        for(int j = 0; j < WINDOW_SIZE; j++) accumulation_buffers[i][j] = 0.0f;
    }
    for(int shot = 0; shot < num_shots; shot++) {
        fireSteeredBeam(target_angle); 
        recordAcousticEchoes();
        for(int i = 0; i < 5; i++) {
            for(int j = 0; j < WINDOW_SIZE; j++) accumulation_buffers[i][j] += rx_buffers[i][j];
        }
        
        // =======================================================
        // The Acoustic Clearing Delay
        // 40 ms allows sound to fully decay off distant walls
        // =======================================================
        delay(40); 
    }
    for(int i = 0; i < 5; i++) {
        for(int j = 0; j < WINDOW_SIZE; j++) rx_buffers[i][j] = accumulation_buffers[i][j] / (float)num_shots;
    }
    removeDCBias();
}

// ==============================================================================
// IV. TRACK-WHILE-SCAN (TWS) LIVE LOOP
// ==============================================================================
void setup() {
    Serial.begin(115200); 
    for(int i = 0; i < 5; i++) {
        pinMode(TX_TRIG[i], OUTPUT); pinMode(TX_ECHO[i], OUTPUT);
        digitalWrite(TX_TRIG[i], LOW); digitalWrite(TX_ECHO[i], LOW);
    }
    initHardwareDMA();
    
    Serial.println("System Boot. 2 MHz Target Tracking Engine Online.");
    delay(1000); 
}

void loop() {
    // 1. Array constrained strictly to the hardware's efficient beamwidth
    static const float scan_angles[5] = {-40.0, -20.0, 0.0, 20.0, 40.0};
    static int angle_index = 0;
    
    float current_angle = scan_angles[angle_index];
    
    // 3 shots to cut room noise while preserving high framerate
    fireBeamAndAverage(3, current_angle); 
    
    // Transmit the current angle to Python
    Serial.printf("TWS_FRAME_START,%.1f\n", current_angle);
    
    // Hardware Mathematical Eraser
    for(int j = 0; j < WINDOW_SIZE; j++) { 
        for(int ch = 0; ch < 5; ch++) {
            int shift = round(CALIB_RX_HW_ERROR[ch]); 
            int original_idx = j + shift; 
            float val = 0.0f;
            
            if(original_idx >= 0 && original_idx < WINDOW_SIZE) {
                val = rx_buffers[ch][original_idx];
            }
            Serial.printf("%.4f%s", val, (ch==4) ? "\n" : ",");
        }
    }
    
    Serial.println("TWS_FRAME_END");
    
    // 2. Modulo 5 ensures the array perfectly loops through the 5 targets
    angle_index = (angle_index + 1) % 5;
    delay(100); 
}