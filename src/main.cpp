#include <Arduino.h>
#include "soc/gpio_reg.h"
#include "esp_rom_sys.h"
#include "esp_adc/adc_continuous.h"
#include "hal/adc_types.h"
#include <math.h>

// ==============================================================================
// 1. HARDWARE PINS
// ==============================================================================
const int TX_TRIG[5] = {4, 14, 17, 19, 25};
const int TX_ECHO[5] = {13, 16, 18, 23, 26};
const adc_channel_t RX_CHANNELS[5] = {ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_7, ADC_CHANNEL_0, ADC_CHANNEL_3};

// ==============================================================================
// 2. THE HYBRID CALIBRATION MATRIX
// ==============================================================================
// Brute-Forced Transmit Offsets
const float OPTIMIZED_TX_HW_ERROR[5] = {3.0758, -0.9833, 0.0014, -0.0937, 4.0451};

// Extracted Receiver Phase and Gain Offsets
const float CALIB_RX_HW_ERROR[5] = {0.8830, -2.3373, 0.0001, 0.2231, -2.7072};
const float CALIB_RX_GAIN[5] = {0.7992, 0.9419, 1.0000, 1.1074, 0.9922};

uint32_t active_tx_cycles[5]; 

// ==============================================================================
// 3. MASSIVE MEMORY ALLOCATION & DMA
// ==============================================================================
const int BUFFER_LENGTH = 1200; 
float rx_buffers[5][BUFFER_LENGTH] = {0};
float aligned_buffers[5][BUFFER_LENGTH] = {0};

adc_continuous_handle_t tws_adc_handle = NULL;
const uint32_t DMA_FLAT_BUFFER_SIZE = 24000;
uint8_t dma_flat_buffer[DMA_FLAT_BUFFER_SIZE] = {0};

// FreeRTOS Synchronization Locks
SemaphoreHandle_t dma_data_ready_sem;
SemaphoreHandle_t serial_print_done_sem;

// ==============================================================================
// 4. SIGNAL PROCESSING (CORE 0)
// ==============================================================================
void removeDCBias() {
    for(int i = 0; i < 5; i++) {
        float sum = 0;
        // Deep silence baseline to avoid electrical crosstalk
        for(int j = 100; j < 180; j++) sum += rx_buffers[i][j];
        float bias = sum / 80.0f;
        for(int j = 0; j < BUFFER_LENGTH; j++) rx_buffers[i][j] -= bias;
    }
}

void applyPhaseAndGainCorrection() {
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

// ==============================================================================
// 5. HARDWARE INITIALIZATION
// ==============================================================================
void initHardwareDMA() {
    adc_continuous_handle_cfg_t adc_config = { .max_store_buf_size = 24000, .conv_frame_size = 1000 };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &tws_adc_handle));

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
    ESP_ERROR_CHECK(adc_continuous_config(tws_adc_handle, &dig_cfg));
}

// ==============================================================================
// 6. CORE 1 TASK (THE PHYSICS ENGINE)
// ==============================================================================
void physicsTask(void *pvParameters) {
    while (true) {
        xSemaphoreTake(serial_print_done_sem, portMAX_DELAY);

        // A. 8-PULSE PUSH-PULL BEAMFORMING (0 Degrees)
        uint32_t edge_times[5][16];
        for (int i = 0; i < 5; i++) {
            uint32_t base_delay = active_tx_cycles[i];
            for (int e = 0; e < 16; e++) {
                edge_times[i][e] = base_delay + (e * 3000); 
            }
        }

        uint32_t next_edge[5] = {0, 0, 0, 0, 0};
        uint32_t start_cycle = ESP.getCycleCount();
        bool bursting = true;

        portDISABLE_INTERRUPTS(); 
        
        while (bursting) {
            uint32_t current = ESP.getCycleCount() - start_cycle;
            bursting = false;
            
            for (int i = 0; i < 5; i++) {
                if (next_edge[i] < 16) {
                    bursting = true; 
                    if (current >= edge_times[i][next_edge[i]]) {
                        if (next_edge[i] % 2 == 0) {
                            REG_WRITE(GPIO_OUT_W1TS_REG, 1UL << TX_TRIG[i]); 
                            REG_WRITE(GPIO_OUT_W1TC_REG, 1UL << TX_ECHO[i]); 
                        } else {
                            REG_WRITE(GPIO_OUT_W1TC_REG, 1UL << TX_TRIG[i]); 
                            REG_WRITE(GPIO_OUT_W1TS_REG, 1UL << TX_ECHO[i]); 
                        }
                        next_edge[i]++;
                    }
                }
            }
        }
        
        for (int i = 0; i < 5; i++) {
            REG_WRITE(GPIO_OUT_W1TC_REG, 1UL << TX_TRIG[i]);
            REG_WRITE(GPIO_OUT_W1TC_REG, 1UL << TX_ECHO[i]);
        }
        portENABLE_INTERRUPTS();

        // B. TIME GATING (250 microseconds)
        delayMicroseconds(250);

        // C. DMA HIGH-SPEED RECORDING
        ESP_ERROR_CHECK(adc_continuous_start(tws_adc_handle));
        uint32_t total_bytes_read = 0;
        
        while (total_bytes_read < DMA_FLAT_BUFFER_SIZE) {
            uint32_t bytes_chunk = 0;
            if (adc_continuous_read(tws_adc_handle, dma_flat_buffer + total_bytes_read, DMA_FLAT_BUFFER_SIZE - total_bytes_read, &bytes_chunk, ADC_MAX_DELAY) == ESP_OK) {
                total_bytes_read += bytes_chunk;
            }
        }
        ESP_ERROR_CHECK(adc_continuous_stop(tws_adc_handle));

        // D. PARSE RAW DMA BYTES
        int rx_indices[5] = {0, 0, 0, 0, 0};
        for (int i = 0; i < DMA_FLAT_BUFFER_SIZE; i += SOC_ADC_DIGI_RESULT_BYTES) {
            adc_digi_output_data_t *p = (adc_digi_output_data_t*)&dma_flat_buffer[i];
            float volt = (float)p->type1.data;

            if (p->type1.channel == ADC_CHANNEL_4 && rx_indices[0] < BUFFER_LENGTH) rx_buffers[0][rx_indices[0]++] = volt;
            else if (p->type1.channel == ADC_CHANNEL_5 && rx_indices[1] < BUFFER_LENGTH) rx_buffers[1][rx_indices[1]++] = volt;
            else if (p->type1.channel == ADC_CHANNEL_7 && rx_indices[2] < BUFFER_LENGTH) rx_buffers[2][rx_indices[2]++] = volt;
            else if (p->type1.channel == ADC_CHANNEL_0 && rx_indices[3] < BUFFER_LENGTH) rx_buffers[3][rx_indices[3]++] = volt;
            else if (p->type1.channel == ADC_CHANNEL_3 && rx_indices[4] < BUFFER_LENGTH) rx_buffers[4][rx_indices[4]++] = volt;
        }

        xSemaphoreGive(dma_data_ready_sem); 
    }
}

// ==============================================================================
// 7. CORE 0 TASK (THE COMMUNICATIONS ENGINE)
// ==============================================================================
void commsTask(void *pvParameters) {
    while (true) {
        xSemaphoreTake(dma_data_ready_sem, portMAX_DELAY);

        removeDCBias();
        applyPhaseAndGainCorrection();

        Serial.println("START_PLOT");
        // Outputting exactly 150 indices for the Python script
        for(int j = 200; j < 350; j++) { 
            Serial.printf("%.1f,%.1f,%.1f,%.1f,%.1f\n", 
                aligned_buffers[0][j], aligned_buffers[1][j], aligned_buffers[2][j], 
                aligned_buffers[3][j], aligned_buffers[4][j]);
        }
        Serial.println("END_PLOT");

        xSemaphoreGive(serial_print_done_sem); 
    }
}

// ==============================================================================
// 8. SYSTEM BOOTSTRAP
// ==============================================================================
void setup() {
    Serial.begin(500000); 
    setCpuFrequencyMhz(240); 
    
    for (int i = 0; i < 5; i++) {
        pinMode(TX_TRIG[i], OUTPUT); 
        pinMode(TX_ECHO[i], OUTPUT);
        digitalWrite(TX_TRIG[i], LOW); 
        digitalWrite(TX_ECHO[i], LOW);
    }
    
    float min_offset = OPTIMIZED_TX_HW_ERROR[0];
    for (int i = 1; i < 5; i++) {
        if (OPTIMIZED_TX_HW_ERROR[i] < min_offset) min_offset = OPTIMIZED_TX_HW_ERROR[i];
    }
    for (int i = 0; i < 5; i++) {
        float precise_hw_us = (OPTIMIZED_TX_HW_ERROR[i] - min_offset) * 2.0;
        active_tx_cycles[i] = (uint32_t)(precise_hw_us * 240.0); 
    }

    initHardwareDMA();

    dma_data_ready_sem = xSemaphoreCreateBinary();
    serial_print_done_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(serial_print_done_sem);

    Serial.println("\n========================================================");
    Serial.println(">>> 0-DEGREE DUAL-CORE BORESIGHT ENGINE ONLINE       <<<");
    Serial.println("========================================================\n");

    xTaskCreatePinnedToCore(physicsTask, "PhysicsCore1", 8192, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(commsTask, "CommsCore0", 8192, NULL, 1, NULL, 0);
}

void loop() {
    vTaskDelete(NULL);
}