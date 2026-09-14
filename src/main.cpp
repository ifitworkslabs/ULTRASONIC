#include <Arduino.h>
#include "soc/gpio_reg.h"
#include "esp_rom_sys.h"
#include "esp_adc/adc_continuous.h"
#include "hal/adc_types.h"
#include <math.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ArduinoEigenDense.h>
#include <complex>

Adafruit_BME280 bme; 
bool bme_status = false;
float current_temp = 30.21;
float current_hum = 70.84;
// ==============================================================================
// I. THE GOLDEN CALIBRATION KEYS (Locked at 2 MHz)
// ==============================================================================
// Maps hardware ADC channel directly to our array index. -1 means ignore.
const int HARDWARE_CH_MAP[8] = {4, -1, -1, 3, 1, 0, -1, 2}; 

const int TX_TRIG[5] = {4, 14, 17, 19, 25};
const int TX_ECHO[5] = {13, 16, 18, 23, 26};

const int CALIB_TX_HW_TICKS[5] = {120, 1680, 1200, 2280, 0};
const int CALIB_RX_HW_SHIFT[5] = {-16, 16, 0, -16, 8};

const int CAPTURE_OFFSET = 800;  
const int WINDOW_SIZE = 3600; // Wide enough for a >2.0m Return Range

// THE MEMORY DIET (UPDATED): 
// Max ADC is 4095. 3 shots max out at 12285. 
// This easily fits inside a 16-bit integer (max 32767).
// Safe up to 15 shots before integer overflow (Max 65,535)
uint16_t accumulation_buffers[5][WINDOW_SIZE] = {0}; 
uint16_t (*processing_buffers)[WINDOW_SIZE] = nullptr;

float processing_angle = 0;
float processing_temp = 30.21;
float processing_hum = 70.84;

SemaphoreHandle_t dsp_ready_sem;
SemaphoreHandle_t dsp_done_sem;
TaskHandle_t dspTaskHandle;

adc_continuous_handle_t adc_handle = NULL;
const uint32_t DMA_FLAT_BUFFER_SIZE = 48000; 
uint8_t dma_flat_buffer[DMA_FLAT_BUFFER_SIZE] = {0};

const adc_channel_t RX_CHANNELS[5] = {ADC_CHANNEL_5, ADC_CHANNEL_4, ADC_CHANNEL_7, ADC_CHANNEL_3, ADC_CHANNEL_0};

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
        
        int raw_ch = p->type1.channel;
        int ch = (raw_ch < 8) ? HARDWARE_CH_MAP[raw_ch] : -1;

        if (ch != -1) {
            if (rx_indices[ch] >= CAPTURE_OFFSET && rx_indices[ch] < CAPTURE_OFFSET + WINDOW_SIZE) {
                // Safely add raw integer data directly into our 16-bit array
                accumulation_buffers[ch][rx_indices[ch] - CAPTURE_OFFSET] += p->type1.data;
            }
            rx_indices[ch]++;
        }
    }
}

// ==============================================================================
// III. CONTINUOUS FIRING ENGINE
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
    memset(accumulation_buffers, 0, sizeof(accumulation_buffers));
    
    for(int shot = 0; shot < num_shots; shot++) {
        fireSteeredBeam(target_angle); 
        recordAcousticEchoes();
        // Jitter the Pulse Repetition Interval (PRI) to randomize the time-of-flight of multi-path
        // echoes. This causes ghost reflections to jump wildly in distance, allowing the UI's
        // tracking filter to easily reject them as noise.
        int jitter = 15 + (rand() % 15); // Random delay between 15ms and 30ms
        delay(jitter); 
    }
}

// ==============================================================================
// IV. CORE 0: DSP MATH & COMMUNICATION TASK
// ==============================================================================
#pragma pack(push, 1)
struct TargetDataPayload {
    uint8_t sync[4];       
    float scan_angle;      
    float temp;
    float hum;
    float target_range;
    float target_x;
    float target_y;
    float confidence;
    float pad;             
};
#pragma pack(pop)

void dspTask(void *pvParameters) {
    while(1) {
        // Wait until Core 1 gives us new data
        xSemaphoreTake(dsp_ready_sem, portMAX_DELAY);
        
        TargetDataPayload payload;
        payload.sync[0] = 0xAA; payload.sync[1] = 0xBB; payload.sync[2] = 0xCC; payload.sync[3] = 0xDD;
        payload.scan_angle = processing_angle;
        payload.temp = processing_temp;
        payload.hum = processing_hum;
        payload.target_range = 0;
        payload.target_x = 0;
        payload.target_y = 0;
        payload.confidence = 0;
        payload.pad = 0;

        float speed_of_sound = 331.4f + (0.606f * processing_temp) + (0.0124f * processing_hum);
        const int start_idx = 50;
        const int end_idx = WINDOW_SIZE - 50;
        const int num_samples = end_idx - start_idx;
        
        float channel_bias[5] = {0};
        float var_sum = 0;
        for (int ch = 0; ch < 5; ch++) {
            float sum = 0;
            int count = 0;
            for (int j = start_idx; j < end_idx; j++) {
                int original_idx = j + CALIB_RX_HW_SHIFT[ch];
                if (original_idx >= 0 && original_idx < WINDOW_SIZE) {
                    sum += (float)processing_buffers[ch][original_idx] / 12285.0f;
                    count++;
                }
            }
            channel_bias[ch] = (count > 0) ? (sum / count) : 0.0f;
            
            float var = 0;
            for (int j = start_idx; j < end_idx; j++) {
                int original_idx = j + CALIB_RX_HW_SHIFT[ch];
                if (original_idx >= 0 && original_idx < WINDOW_SIZE) {
                    float val = ((float)processing_buffers[ch][original_idx] / 12285.0f) - channel_bias[ch];
                    var += val * val;
                }
            }
            var_sum += var / num_samples;
        }
        float signal_energy = var_sum / 5.0f;
        float noise_floor_threshold = 0.00002f; // Reject absolute silence
        
        typedef Eigen::Matrix<std::complex<float>, 5, 5> Matrix5cf;
        typedef Eigen::Matrix<std::complex<float>, 5, 1> Vector5cf;
        
        if (signal_energy > noise_floor_threshold) {
            Matrix5cf Rxx = Matrix5cf::Zero();
            
            float stc_start = 1.0f;
            float stc_end = 5.0f; 
            int delay_idx = 2;
            
            float max_env_sq = 0;
            int peak_time_idx = 0;

            // --- PASS 1: Find the Peak (Time of Flight) ---
            // GENIUS FIX: Only listen to the center microphone (Channel 2) to find the time peak.
            for (int j = start_idx + delay_idx + 1; j < end_idx; j += 4) {
                int orig_j = j + CALIB_RX_HW_SHIFT[2]; // Only look at Channel 2
                if (orig_j >= 0 && orig_j < WINDOW_SIZE) {
                    
                    // Skip I/Q demodulation. Just find the raw squared amplitude of the real signal
                    // Note: Make sure to use the 12285.0f fix we discussed earlier!
                    float val = ((float)processing_buffers[2][orig_j] / 12285.0f) - channel_bias[2];
                    float inst_env_sq = val * val; 
                    
                    float min_time = (0.15f * 2.0f) / speed_of_sound;
                    int min_idx = (int)((min_time - 0.0008f) * 400000.0f - CAPTURE_OFFSET);
                    
                    if (j > min_idx && inst_env_sq > max_env_sq) {
                        max_env_sq = inst_env_sq;
                        peak_time_idx = j;
                    }
                }
            }

            // --- PASS 2: Calculate Spatial Covariance (Rxx) ONLY around the peak ---
            // This prevents multipath ghosts at different distances from corrupting the MUSIC matrix!
            int rxx_count = 0;
            int window_half_width = 75; // Cover 3 full wave cycles to stabilize the matrix
            int rxx_start = max((int)(start_idx + delay_idx + 1), peak_time_idx - window_half_width);
            int rxx_end = min((int)end_idx, peak_time_idx + window_half_width);
            
            for (int j = rxx_start; j < rxx_end; j++) {
                // GENIUS FIX: STC mathematically deleted. Eigenvectors are scale-invariant!
                
                Vector5cf X;
                for (int ch = 0; ch < 5; ch++) {
                    int orig_j = j + CALIB_RX_HW_SHIFT[ch];
                    float real_part = 0, imag_part = 0;
                    if(orig_j >= 0 && orig_j < WINDOW_SIZE) {
                        real_part = ((float)processing_buffers[ch][orig_j] / 12285.0f) - channel_bias[ch];
                    }
                    // EXACT 2.5 SAMPLE INTERPOLATION FOR 90-DEGREE I/Q SHIFT AT 400kHz
                    int d2 = (j - 2) + CALIB_RX_HW_SHIFT[ch];
                    int d3 = (j - 3) + CALIB_RX_HW_SHIFT[ch];
                    if (d2 >= 0 && d2 < WINDOW_SIZE && d3 >= 0 && d3 < WINDOW_SIZE) {
                        // Combine integers first (instantaneous), divide ONCE, subtract bias ONCE.
                        float raw_sum = (float)(processing_buffers[ch][d2] + processing_buffers[ch][d3]);
                        imag_part = (raw_sum / 24570.0f) - channel_bias[ch]; 
                    } else {
                        imag_part = 0.0f;
                    }
                    

                    X(ch) = std::complex<float>(real_part, imag_part);
                }
                // Rank Update strictly calculates ONLY the lower triangular half of the matrix!
                Rxx.selfadjointView<Eigen::Lower>().rankUpdate(X, 1.0f);
                rxx_count++;
            }
            if (rxx_count > 0) {
                Rxx /= (float)(rxx_count);
            }
            
            // Diagonal Loading: inject artificial noise to prevent matrix singularity
            for (int i = 0; i < 5; i++) {
                Rxx(i, i) += std::complex<float>(0.00005f, 0.0f);
            }
            
            Eigen::SelfAdjointEigenSolver<Matrix5cf> eigensolver(Rxx);
            Matrix5cf eigenvectors = eigensolver.eigenvectors(); 
            Eigen::Matrix<std::complex<float>, 5, 4> noise_subspace = eigenvectors.leftCols<4>();
            // Matrix5cf P_noise = noise_subspace * noise_subspace.adjoint(); 
            Eigen::Matrix<std::complex<float>, 4, 5> Un_H = noise_subspace.adjoint();
            
            float mic_positions[5] = {-0.025f, -0.013f, -0.002f, 0.009f, 0.025f};
            float wavelength = speed_of_sound / 40000.0f;

            float global_music_max = 0.0f;
            float best_two_way_peak = 0.0f;
            int best_angle = 0;
            
            // Standard deviation of 12.0 creates a much wider, forgiving beam
            const float TX_SIGMA = 12.0f; 
            
            // 1. Search the ENTIRE room
            float k_constant = 2.0f * M_PI / wavelength;
            
            // Create a static look-up table (LUT) that persists in memory
            static float sin_lut[181];
            static bool lut_init = false;
            if (!lut_init) {
                for (int i = 0; i <= 180; i++) {
                    sin_lut[i] = sinf((i - 90) * (M_PI / 180.0f));
                }
                lut_init = true;
            }

            for (int theta = -90; theta <= 90; theta++) {
                // Just grab the pre-calculated answer from the array!
                float k_x = k_constant * sin_lut[theta + 90]; 
                
                Vector5cf a;
                for (int ch = 0; ch < 5; ch++) {
                    // A single bare multiplication. No redundant sines!
                    float phase = k_x * mic_positions[ch]; 
                    a(ch) = std::complex<float>(cosf(phase), -sinf(phase));
                }
                // --- RX GAIN (The raw MUSIC spatial spectrum) ---
                Eigen::Vector<std::complex<float>, 4> projection = Un_H * a;
                float denom = projection.squaredNorm();
                
                float p_music = 1.0f / denom; 
                
                // Track the absolute loudest echo in the room (used for the final Python score)
                if (p_music > global_music_max) {
                    global_music_max = p_music; 
                }
                
                // --- TX GAIN (The mathematical representation of your firing cone) ---
                float angle_diff = (float)theta - processing_angle;
                // Gaussian formula: Drops off rapidly outside the +-10 degree threshold
                float tx_gain = expf(-(angle_diff * angle_diff) / (2.0f * TX_SIGMA * TX_SIGMA)); 
                
                // --- THE TWO-WAY FILTER (RX * TX) ---
                float p_two_way = p_music * tx_gain;
                
                // Find the best target after the Two-Way multiplication
                if (p_two_way > best_two_way_peak) {
                    best_two_way_peak = p_two_way;
                    best_angle = theta;
                }
            }
            
            // 2. THE ULTIMATE CONFIDENCE SCORE
            // We divide our best Two-Way target by the absolute loudest raw echo in the room.
            payload.confidence = (global_music_max > 0) ? (best_two_way_peak / global_music_max) : 0.0f;
            
            // 3. ALWAYS calculate and send the physical coordinates
            float lock_angle_rad = best_angle * (M_PI / 180.0f);
            float time_of_flight = 0.0008f + ((CAPTURE_OFFSET + peak_time_idx) / 400000.0f);
            payload.target_range = (time_of_flight * speed_of_sound) / 2.0f;
            payload.target_x = payload.target_range * sinf(lock_angle_rad);
            payload.target_y = payload.target_range * cosf(lock_angle_rad);
            payload.pad = max_env_sq;
        } else {
            // If it's pure silence, output zero confidence!
            payload.confidence = 0.0f;
            payload.target_range = 0.0f;
        }
        
        Serial.write((uint8_t*)&payload, sizeof(TargetDataPayload));
        
        // Tell Core 1 we are done
        xSemaphoreGive(dsp_done_sem);
    }
}

// ==============================================================================
// V. CORE 1: TRACK-WHILE-SCAN (TWS) ACQUISITION LOOP
// ==============================================================================
void setup() {
    Serial.begin(576000); 
    
    Wire.begin();
    bme_status = bme.begin(0x76);
    if (!bme_status) {
        bme_status = bme.begin(0x77);
    }
    
    for(int i = 0; i < 5; i++) {
        pinMode(TX_TRIG[i], OUTPUT); pinMode(TX_ECHO[i], OUTPUT);
        digitalWrite(TX_TRIG[i], LOW); digitalWrite(TX_ECHO[i], LOW);
    }
    initHardwareDMA();
    
    // Dynamically allocate to avoid .bss overflow
    processing_buffers = (uint16_t (*)[WINDOW_SIZE]) malloc(5 * WINDOW_SIZE * sizeof(uint16_t));
    if (processing_buffers == NULL) {
        Serial.println("FATAL: Failed to allocate processing_buffers");
        while (1) {
            delay(100);
        }
    }
    
    dsp_ready_sem = xSemaphoreCreateBinary();
    dsp_done_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(dsp_done_sem); // Start free
    
    // Launch DSP task on Core 0
    xTaskCreatePinnedToCore(dspTask, "DSPTask", 16384, NULL, 1, &dspTaskHandle, 0);
    
    Serial.println("System Boot. Dual-Core Radar Engine Online.");
    delay(1000); 
}

void loop() {
    static unsigned long last_bme_read = 0;
    if (bme_status && (millis() - last_bme_read > 2000)) {
        current_temp = bme.readTemperature();
        current_hum = bme.readHumidity();
        last_bme_read = millis();
    }

    // Interleaved sweep pattern to maximize spatial distance between consecutive pings.
    // This forces echoes from the previous sweep to arrive at an off-axis angle in the current sweep,
    // where they are heavily suppressed by the MUSIC algorithm's spatial cone and Gaussian weighting,
    // effectively eliminating "ghosts" without slowing down the radar.
    static const float scan_angles[5] = {-40.0, 20.0, -20.0, 40.0, 0.0};
    static int angle_index = 0;
    
    float current_angle = scan_angles[angle_index];
    
    // 1. Acquire Data (Takes ~30ms total)
    // Change num_shots from 1 to 3 (or 4) to let the PRI jitter destroy ghosts
    fireBeamAndAverage(3, current_angle); 
    
    // 2. Wait for Core 0 to finish processing the previous angle's data
    xSemaphoreTake(dsp_done_sem, portMAX_DELAY);
    
    // 3. Copy new data to processing buffer for Core 0
    memcpy(processing_buffers, accumulation_buffers, sizeof(accumulation_buffers));
    processing_angle = current_angle;
    processing_temp = current_temp;
    processing_hum = current_hum;
    
    // 4. Trigger Core 0 to start processing the new data
    xSemaphoreGive(dsp_ready_sem);
    
    // 5. Advance angle for the next acquisition loop
    angle_index = (angle_index + 1) % 5;
}