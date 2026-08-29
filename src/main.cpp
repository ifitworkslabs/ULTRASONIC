#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "soc/gpio_reg.h"
#include "esp_rom_sys.h"
#include "esp_adc/adc_continuous.h"
#include "hal/adc_types.h"
#include <math.h>

// ==============================================================================
// I. NETWORK CREDENTIALS & ZERO-CONFIG UDP
// ==============================================================================
const char* ssid = "LAN";         // <-- UPDATE THIS
const char* password = "T0ixuatsac2011"; // <-- UPDATE THIS

const uint16_t UDP_PORT = 8888;
WiFiUDP udp;

// Shared memory for Dual-Core Handoff
volatile bool new_data_ready = false;
float shared_angle = 0;

// ==============================================================================
// II. THE GOLDEN CALIBRATION KEYS
// ==============================================================================
const int TX_TRIG[5] = {4, 14, 17, 19, 25};
const int TX_ECHO[5] = {13, 16, 18, 23, 26};

const int CALIB_TX_HW_TICKS[5] = {0, 2160, 1560, 1920, 120};
const float CALIB_RX_HW_ERROR[5] = {-8.2058, 6.9029, 0.0000, -7.7438, 7.8892};

const int CAPTURE_OFFSET = 1800; 
const int WINDOW_SIZE = 1000;    

float rx_buffers[5][WINDOW_SIZE] = {0};
float accumulation_buffers[5][WINDOW_SIZE] = {0}; 

// THE MEMORY OVERLAY TRICK: 
// By pointing shared_payload to the memory address of accumulation_buffers,
// we recycle 10,000 bytes of "dead" RAM and prevent the compilation overflow!
float* shared_payload = (float*)accumulation_buffers; 

adc_continuous_handle_t adc_handle = NULL;
const uint32_t DMA_FLAT_BUFFER_SIZE = 30000; 
uint8_t dma_flat_buffer[DMA_FLAT_BUFFER_SIZE] = {0};

const adc_channel_t RX_CHANNELS[5] = {ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_7, ADC_CHANNEL_0, ADC_CHANNEL_3};

// ==============================================================================
// III. HIGH-RESOLUTION DMA CORE (CORE 1)
// ==============================================================================
void initHardwareDMA() {
    // Reduced max_store_buf_size to 10240 to save an extra 10KB of heap RAM for Wi-Fi stability
    adc_continuous_handle_cfg_t adc_config = { .max_store_buf_size = 10240, .conv_frame_size = 2000 };
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
        delay(20); 
    }
    for(int i = 0; i < 5; i++) {
        for(int j = 0; j < WINDOW_SIZE; j++) rx_buffers[i][j] = accumulation_buffers[i][j] / (float)num_shots;
    }
    removeDCBias();
}

// ==============================================================================
// IV. THE WI-FI RADIO TASK (PINNED TO CORE 0)
// ==============================================================================
void udpRadioTask(void *pvParameters) {
    while(true) {
        if(new_data_ready) {
            // Split the 10,000-byte HD payload into 8 chunks of 1250 bytes
            for(uint8_t chunk = 0; chunk < 8; chunk++) {
                udp.beginPacket("255.255.255.255", UDP_PORT);
                
                // 9-Byte Header: Sync(4) + Angle(4) + ChunkIndex(1)
                uint8_t header[9];
                header[0] = 0xAA; header[1] = 0xBB; header[2] = 0xCC; header[3] = 0xDD;
                memcpy(&header[4], (void*)&shared_angle, 4);
                header[8] = chunk; // 0 to 7
                
                udp.write(header, 9);
                // Send exactly 1/8th of the floating-point array bytes
                udp.write(((uint8_t*)shared_payload) + (chunk * 1250), 1250);
                udp.endPacket();
            }
            new_data_ready = false;
        }
        vTaskDelay(pdMS_TO_TICKS(5)); 
    }
}

// ==============================================================================
// V. TRACK-WHILE-SCAN (TWS) LIVE LOOP
// ==============================================================================
void setup() {
    Serial.begin(115200); 
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.print("Connecting to Wi-Fi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWi-Fi Connected! Broadcasting on Port " + String(UDP_PORT));
    udp.begin(UDP_PORT);

    for(int i = 0; i < 5; i++) {
        pinMode(TX_TRIG[i], OUTPUT); pinMode(TX_ECHO[i], OUTPUT);
        digitalWrite(TX_TRIG[i], LOW); digitalWrite(TX_ECHO[i], LOW);
    }
    initHardwareDMA();
    
    // Launch the Wi-Fi Radio on Core 0 (Leaves Core 1 strictly for Radar Math)
    xTaskCreatePinnedToCore(udpRadioTask, "UDP_Task", 4096, NULL, 1, NULL, 0);
    
    Serial.println("System Boot. Dual-Core HD Wi-Fi Engine Online.");
}

void loop() {
    // Thread Safety: Wait until Core 0 finishes sending the last packet
    while(new_data_ready) {
        delay(1); 
    }

    static const float scan_angles[5] = {-40.0, -20.0, 0.0, 20.0, 40.0};
    static int angle_index = 0;
    float current_angle = scan_angles[angle_index];
    
    fireBeamAndAverage(3, current_angle); 
    
    shared_angle = current_angle;
    
    int ptr = 0;
    for(int j = 350; j < 850; j++) { 
        for(int ch = 0; ch < 5; ch++) {
            int shift = round(CALIB_RX_HW_ERROR[ch]); 
            int original_idx = j + shift; 
            float val = 0.0f;
            
            if(original_idx >= 0 && original_idx < WINDOW_SIZE) {
                val = rx_buffers[ch][original_idx];
            }
            // Pack into the recycled memory space!
            shared_payload[ptr++] = val; 
        }
    }
    
    // Flag Core 0 to transmit the 8 chunks
    new_data_ready = true;
    angle_index = (angle_index + 1) % 5;
}