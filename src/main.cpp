#include <Arduino.h>
#include "soc/gpio_reg.h"
#include "esp_rom_sys.h"
#include "esp_adc/adc_continuous.h"
#include "hal/adc_types.h"
#include <math.h>

const int TX_TRIG[5] = {4, 14, 17, 19, 25};
const int TX_ECHO[5] = {13, 16, 18, 23, 26};
const adc_channel_t RX_CHANNELS[5] = {ADC_CHANNEL_5, ADC_CHANNEL_4, ADC_CHANNEL_7, ADC_CHANNEL_3, ADC_CHANNEL_0};

// === 1. CALIBRATED HARDWARE CONSTANTS ===
const int CALIB_TX_HW_TICKS[5] = {120, 1680, 1200, 2280, 0};
const int CALIB_RX_HW_ERROR[5] = {-16, 16, 0, -16, 8};

// === 2. ENVIRONMENTAL PHYSICS ===
const float SPEED_OF_SOUND = 350.48; 
const float TX_POSITIONS[5] = {-0.0084, -0.0042, 0.0000, 0.0042, 0.0084}; // in meters
const float RX_POSITIONS[5] = {-0.025, -0.013, -0.002, 0.009, 0.025}; // in meters

// === 3. RADAR ENGINE CONFIG ===
const int CAPTURE_OFFSET = 800; // 800 samples = 2ms delay
const int WINDOW_SIZE = 3600;   
const float SAMPLE_RATE = 400000.0; 

int16_t rx_buffers_0[5][WINDOW_SIZE + 40] = {0};
int16_t rx_buffers_1[5][WINDOW_SIZE + 40] = {0};
bool use_buffer_0 = true;

struct RadarScanData {
    int16_t (*buffers)[WINDOW_SIZE + 40];
    float scan_angle;
};

QueueHandle_t dspQueue;

adc_continuous_handle_t adc_handle = NULL;
uint8_t dma_chunk_buffer[4000] = {0}; 

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

void recordAcousticEchoes(int16_t (*target_rx_buffers)[WINDOW_SIZE + 40]) {
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
                        target_rx_buffers[ch][rx_indices[ch] - CAPTURE_OFFSET] = val;
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

void radarEngineTask(void *pvParameters) {
    float scan_angle = -45.0;
    float scan_dir = 2.0; // Fast 2-degree step!

    while (1) {
        fireSteeredBeam(scan_angle);

        int16_t (*active_buffer)[WINDOW_SIZE + 40] = use_buffer_0 ? rx_buffers_0 : rx_buffers_1;
        recordAcousticEchoes(active_buffer);

        RadarScanData data;
        data.buffers = active_buffer;
        data.scan_angle = scan_angle;
        // Push to DSP queue, don't block if DSP is slow (though it won't be)
        xQueueSend(dspQueue, &data, (TickType_t)0); 

        use_buffer_0 = !use_buffer_0;

        scan_angle += scan_dir;
        if (scan_angle >= 45.0) scan_dir = -2.0;
        if (scan_angle <= -45.0) scan_dir = 2.0;
        
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
}

#pragma pack(push, 1)
struct SerialPacket {
    uint8_t sync[4];
    float scan_angle;
    float target_angle;
    float distance;
    float strength;
};
#pragma pack(pop)

void dspEngineTask(void *pvParameters) {
    RadarScanData data;
    
    // Tuning parameters
    float STC_START_GAIN = 1.0;
    float STC_END_GAIN = 5.0;
    float STC_POWER = 2.5;
    const float WAVELENGTH = SPEED_OF_SOUND / 40000.0f;
    
    while (1) {
        if (xQueueReceive(dspQueue, &data, portMAX_DELAY) == pdTRUE) {
            
            // 1. Find the Echo Peak (using Center Channel 2)
            int peak_idx = -1;
            float max_env = 0;
            
            // Remove DC offset for Center Channel
            long dc_sum = 0;
            for(int j=0; j<100; j++) dc_sum += data.buffers[2][j+20];
            float dc_offset = (float)dc_sum / 100.0f;

            for (int j = 0; j < WINDOW_SIZE; j++) {
                float val = (float)data.buffers[2][j + 20] - dc_offset;
                float env = abs(val);
                float t_ratio = (float)j / (float)WINDOW_SIZE;
                float stc = STC_START_GAIN + (STC_END_GAIN - STC_START_GAIN) * pow(t_ratio, STC_POWER);
                env *= stc;
                if (env > max_env) {
                    max_env = env;
                    peak_idx = j;
                }
            }
            
            // If nothing loud enough, ignore
            if (max_env < 300.0 || peak_idx < 50) {
                SerialPacket pkt;
                pkt.sync[0] = 0xAA; pkt.sync[1] = 0xBB; pkt.sync[2] = 0xCC; pkt.sync[3] = 0xDD;
                pkt.scan_angle = data.scan_angle;
                pkt.target_angle = 0; pkt.distance = -1.0; pkt.strength = 0;
                Serial.write((uint8_t*)&pkt, sizeof(SerialPacket));
                continue;
            }

            // 2. Extract 100-sample window and IQ Demodulate
            int start_idx = peak_idx - 30;
            if(start_idx < 0) start_idx = 0;
            if(start_idx > WINDOW_SIZE - 100) start_idx = WINDOW_SIZE - 100;

            struct Complex { float r; float i; };
            Complex X[5][100];
            
            for(int ch = 0; ch < 5; ch++) {
                // Find DC for this channel
                long sum = 0;
                for(int j=0; j<100; j++) sum += data.buffers[ch][j+20];
                float ch_dc = (float)sum / 100.0f;

                for(int k = 0; k < 100; k++) {
                    int idx = start_idx + k + CALIB_RX_HW_ERROR[ch] + 20;
                    if(idx < 0) idx = 0;
                    if(idx >= WINDOW_SIZE + 40) idx = WINDOW_SIZE + 39;
                    
                    float val = (float)data.buffers[ch][idx] - ch_dc;
                    // 40kHz at 400kHz SR = 10 samples per period
                    float angle = 2.0 * PI * (float)(k % 10) / 10.0;
                    X[ch][k].r = val * cos(angle);
                    X[ch][k].i = -val * sin(angle);
                }
            }

            // Low-pass filter (Moving Average over 10 samples)
            Complex X_lpf[5][90];
            for(int ch = 0; ch < 5; ch++) {
                for(int k = 0; k < 90; k++) {
                    float sum_r = 0, sum_i = 0;
                    for(int m = 0; m < 10; m++) {
                        sum_r += X[ch][k+m].r;
                        sum_i += X[ch][k+m].i;
                    }
                    X_lpf[ch][k].r = sum_r / 10.0f;
                    X_lpf[ch][k].i = sum_i / 10.0f;
                }
            }

            // 3. Covariance Matrix (5x5)
            Complex R[5][5];
            for(int i=0; i<5; i++) {
                for(int j=0; j<5; j++) {
                    R[i][j].r = 0; R[i][j].i = 0;
                    for(int k=0; k<90; k++) {
                        R[i][j].r += X_lpf[i][k].r * X_lpf[j][k].r + X_lpf[i][k].i * X_lpf[j][k].i;
                        R[i][j].i += X_lpf[i][k].i * X_lpf[j][k].r - X_lpf[i][k].r * X_lpf[j][k].i;
                    }
                }
            }

            // 4. Power Iteration for Principal Eigenvector
            Complex v[5] = {{1,0}, {1,0}, {1,0}, {1,0}, {1,0}};
            for(int iter=0; iter<10; iter++) {
                Complex v_new[5] = {0};
                for(int i=0; i<5; i++) {
                    for(int j=0; j<5; j++) {
                        v_new[i].r += R[i][j].r * v[j].r - R[i][j].i * v[j].i;
                        v_new[i].i += R[i][j].r * v[j].i + R[i][j].i * v[j].r;
                    }
                }
                float norm_sq = 0;
                for(int i=0; i<5; i++) norm_sq += v_new[i].r*v_new[i].r + v_new[i].i*v_new[i].i;
                float norm = sqrt(norm_sq);
                for(int i=0; i<5; i++) {
                    v[i].r = v_new[i].r / norm;
                    v[i].i = v_new[i].i / norm;
                }
            }

            // 5. MUSIC Spectrum Search
            float best_target_angle = data.scan_angle;
            float max_music_val = 0;

            for(float theta = data.scan_angle - 15.0f; theta <= data.scan_angle + 15.0f; theta += 1.0f) {
                float angle_rad = theta * PI / 180.0f;
                
                Complex a[5];
                for(int i=0; i<5; i++) {
                    float phase = 2.0f * PI * (RX_POSITIONS[i] * sin(angle_rad)) / WAVELENGTH;
                    a[i].r = cos(phase);
                    a[i].i = sin(phase);
                }
                
                float proj_r = 0; float proj_i = 0;
                for(int i=0; i<5; i++) {
                    proj_r += a[i].r * v[i].r + a[i].i * v[i].i;
                    proj_i += a[i].r * v[i].i - a[i].i * v[i].r;
                }
                
                float proj_mag_sq = proj_r*proj_r + proj_i*proj_i;
                float denom = 5.0f - proj_mag_sq;
                if(denom < 0.0001f) denom = 0.0001f;
                float music_val = 1.0f / denom;
                
                if(music_val > max_music_val) {
                    max_music_val = music_val;
                    best_target_angle = theta;
                }
            }

            // 6. Calculate Distance and Threshold
            float time_of_flight = 0.0008 + ((CAPTURE_OFFSET + peak_idx) / SAMPLE_RATE);
            float distance = (time_of_flight * SPEED_OF_SOUND) / 2.0;

            // MUSIC Peak thresholding (If proj_mag_sq is close to 5, denom is tiny, music_val is HUGE)
            // music_val > 5.0 means very sharp lock
            if (max_music_val < 5.0 || distance < 0.65 || distance > 2.0) {
                distance = -1.0;
            }

            // 7. Output Packet
            SerialPacket pkt;
            pkt.sync[0] = 0xAA; pkt.sync[1] = 0xBB; pkt.sync[2] = 0xCC; pkt.sync[3] = 0xDD;
            pkt.scan_angle = data.scan_angle;
            pkt.target_angle = best_target_angle;
            pkt.distance = distance;
            pkt.strength = max_music_val;

            Serial.write((uint8_t*)&pkt, sizeof(SerialPacket));
        }
    }
}

void setup() {
    Serial.begin(576000); 
    for(int i = 0; i < 5; i++) {
        pinMode(TX_TRIG[i], OUTPUT); pinMode(TX_ECHO[i], OUTPUT);
        digitalWrite(TX_TRIG[i], LOW); digitalWrite(TX_ECHO[i], LOW);
    }
    initHardwareDMA();

    dspQueue = xQueueCreate(2, sizeof(RadarScanData));

    xTaskCreatePinnedToCore(radarEngineTask, "RadarTask", 8192, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(dspEngineTask, "DSPTask", 32768, NULL, 1, NULL, 0);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}