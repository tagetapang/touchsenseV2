#include <stdio.h>
#include "esp_spiffs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/touch_pad.h"
#include "driver/i2s.h"
#include <string.h>
#include "esp_err.h"
#include <inttypes.h>
#include "esp_timer.h"  // For time tracking
#include "driver/ledc.h"

//______________________________________________________________________________________________________
//______________________________________________________________________________________________________
//______________________________________________________________________________________________________


#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#define SERVO_GPIO 18
#define SERVO1_GPIO 19
#define SERVO_MIN_PULSEWIDTH 600 // 0-degree
#define SERVO_MAX_PULSEWIDTH 2400 // 180-degree
#define SERVO_FREQ 50  // Servo PWM frequency (50Hz)
#define SERVO_CHANNEL LEDC_CHANNEL_0
#define SERVO1_CHANNEL LEDC_CHANNEL_1
#define SERVO_TIMER LEDC_TIMER_0
#define intro 800
#define words 1000

//______________________________________________________________________________________________________
//______________________________________________________________________________________________________
//______________________________________________________________________________________________________

#define LED_GPIO_PIN 23

//______________________________________________________________________________________________________
//______________________________________________________________________________________________________
//______________________________________________________________________________________________________

#define TOUCH_THRESHOLD 300
#define TOUCHPAD_NOISE_FILTER 20
#define SAMPLE_RATE 16000  // 8 kHz
#define I2S_NUM I2S_NUM_0
#define AMP_SD_PIN 33  // Assuming GPIO 33 is used for shutdown

//______________________________________________________________________________________________________
//______________________________________________________________________________________________________
//______________________________________________________________________________________________________

#define WAV_FILE_SENSOR6 "/storage/left_hand.wav"
#define WAV_FILE_SENSOR7 "/storage/left_leg.wav"
#define WAV_FILE_SENSOR3 "/storage/right_hand.wav"
#define WAV_FILE_SENSOR4 "/storage/right_leg.wav"
#define WAV_FILE_SENSOR5 "/storage/song1.wav"

//______________________________________________________________________________________________________
//______________________________________________________________________________________________________
//______________________________________________________________________________________________________

static const char *TAG0 = "TouchSensorDemo";
static const char *TAG = "FileSystem";
static volatile bool is_playing = false;
bool is_dancing = false;

//______________________________________________________________________________________________________
//______________________________________________________________________________________________________
//______________________________________________________________________________________________________

void led_init() {
    gpio_set_direction(LED_GPIO_PIN, GPIO_MODE_OUTPUT);
}

//______________________________________________________________________________________________________
//______________________________________________________________________________________________________
//______________________________________________________________________________________________________

void calibrate_touch_sensors() {
    uint16_t touch_value;
    for (int i = 0; i < TOUCH_PAD_MAX; i++) {
        touch_pad_read(i, &touch_value);
        ESP_LOGI(TAG0, "Calibrating Sensor %d: Baseline %d", i, touch_value);
    }
}

//______________________________________________________________________________________________________
//______________________________________________________________________________________________________
//______________________________________________________________________________________________________

// Function to set servo angle based on the given angle (0 to 180 degrees)
static uint32_t pulsewidth_us_to_duty(uint32_t pulsewidth_us) {
    const uint32_t max_duty = (1 << 13) - 1; // 8191 for 13-bit
    const uint32_t period_us = 20000; // 20ms = 50Hz
    return (pulsewidth_us * max_duty) / period_us;
}

//______________________________________________________________________________________________________
//______________________________________________________________________________________________________
//______________________________________________________________________________________________________

void set_servo_angle(ledc_channel_t channel,int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    uint32_t pulsewidth = SERVO_MIN_PULSEWIDTH + 
        ((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * angle) / 180;

    uint32_t duty = pulsewidth_us_to_duty(pulsewidth);

    

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, channel);
}

//______________________________________________________________________________________________________
//______________________________________________________________________________________________________
//______________________________________________________________________________________________________

i2s_config_t i2s_config = {
    .mode = I2S_MODE_MASTER | I2S_MODE_TX,  // Master mode & Transmit
    .sample_rate = 16000,                   // Try 16000 or 44100 Hz
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // 16-bit depth
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, 
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .dma_buf_count = 4,
    .dma_buf_len = 1024,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1
};

i2s_pin_config_t pin_config = {
    .bck_io_num = 26,   // BCLK
    .ws_io_num = 25,    // LRC
    .data_out_num = 22, // DIN
    .data_in_num = I2S_PIN_NO_CHANGE
};

//______________________________________________________________________________________________________
//______________________________________________________________________________________________________
//______________________________________________________________________________________________________

void configure_i2s() {
    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM, &pin_config);
    i2s_zero_dma_buffer(I2S_NUM);
}

//______________________________________________________________________________________________________
//______________________________________________________________________________________________________
//______________________________________________________________________________________________________

void play_audio(const char* file_name) {
    if (is_playing) {
        return;  
    }

    is_playing = true;
    gpio_set_level(AMP_SD_PIN, 1);  // Turn ON amplifier

    FILE *f = fopen(file_name, "r");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open WAV file: %s", file_name);
        is_playing = false;
        return;
    }
    if(strcmp(file_name,WAV_FILE_SENSOR3) == 0){
        set_servo_angle(SERVO1_CHANNEL,180);
    }
    if(strcmp(file_name,WAV_FILE_SENSOR6) == 0){
        set_servo_angle(SERVO_CHANNEL,0);
    }

    uint8_t header[44];

    if (fread(header, 1, 44, f) != 44) {
        ESP_LOGE(TAG, "Failed to read WAV header: %s", file_name);
        fclose(f);
        is_playing = false;
        return;
    }

    int16_t audio_buffer[512];
    size_t bytes_written;
    size_t samples_read;


    while ((samples_read = fread(audio_buffer, sizeof(int16_t), 512, f)) > 0) {
        gpio_set_level(LED_GPIO_PIN,1);

        i2s_write(I2S_NUM, audio_buffer, samples_read * sizeof(int16_t), &bytes_written, portMAX_DELAY);
        gpio_set_level(LED_GPIO_PIN,0);
    }

    // // Silence output after finishing or stopping playback
    // int16_t silence[64] = {0};
    // i2s_write(I2S_NUM, silence, sizeof(silence), &bytes_written, portMAX_DELAY);
    gpio_set_level(AMP_SD_PIN, 0);  // Turn OFF amplifier

    vTaskDelay(pdMS_TO_TICKS(500));

    if(strcmp(file_name,WAV_FILE_SENSOR3) == 0){
        set_servo_angle(SERVO1_CHANNEL,0);
    }
    if(strcmp(file_name,WAV_FILE_SENSOR6) == 0){
        set_servo_angle(SERVO_CHANNEL,180);
    }

    fclose(f);
    is_playing = false;
    gpio_set_level(LED_GPIO_PIN,0);
    
    
    ESP_LOGI(TAG0, "Finished playing: %s", file_name);
}

//______________________________________________________________________________________________________
//______________________________________________________________________________________________________
//______________________________________________________________________________________________________
void servo_dance_task(void *param) {
    
    while(is_dancing) {
            // initial first 5 second task
            set_servo_angle(SERVO1_CHANNEL,30);//right
            vTaskDelay(pdMS_TO_TICKS(intro));
            set_servo_angle(SERVO_CHANNEL,150);
            vTaskDelay(pdMS_TO_TICKS(intro));
            set_servo_angle(SERVO1_CHANNEL,0);//right
            vTaskDelay(pdMS_TO_TICKS(intro));
            set_servo_angle(SERVO_CHANNEL,180);
            vTaskDelay(pdMS_TO_TICKS(intro-200));
            set_servo_angle(SERVO1_CHANNEL,30);//right
            vTaskDelay(pdMS_TO_TICKS(intro));
            set_servo_angle(SERVO_CHANNEL,150);
            vTaskDelay(pdMS_TO_TICKS(intro));
            set_servo_angle(SERVO1_CHANNEL,0);
            set_servo_angle(SERVO_CHANNEL,180);
            vTaskDelay(pdMS_TO_TICKS(words));
            //intial 5 second
            //words in the song
            set_servo_angle(SERVO1_CHANNEL,90);
            vTaskDelay(pdMS_TO_TICKS(words + 500));
            set_servo_angle(SERVO1_CHANNEL,180);
            vTaskDelay(pdMS_TO_TICKS(words));
            set_servo_angle(SERVO_CHANNEL,90);
            vTaskDelay(pdMS_TO_TICKS(words + 500));
            set_servo_angle(SERVO_CHANNEL,0);
            vTaskDelay(pdMS_TO_TICKS(words));
            set_servo_angle(SERVO1_CHANNEL,90);
            vTaskDelay(pdMS_TO_TICKS(words));
            set_servo_angle(SERVO_CHANNEL,90);
            vTaskDelay(pdMS_TO_TICKS(words));
            set_servo_angle(SERVO1_CHANNEL,180);
            set_servo_angle(SERVO_CHANNEL,0);
            vTaskDelay(pdMS_TO_TICKS(words));
            set_servo_angle(SERVO_CHANNEL,90);
            vTaskDelay(pdMS_TO_TICKS(words));
            set_servo_angle(SERVO1_CHANNEL,90);
            vTaskDelay(pdMS_TO_TICKS(words));
            set_servo_angle(SERVO1_CHANNEL,0);
            set_servo_angle(SERVO_CHANNEL,180);
            vTaskDelay(pdMS_TO_TICKS(intro));
            //words in the song
             // initial first 5 second task
            set_servo_angle(SERVO1_CHANNEL,30);//right
            vTaskDelay(pdMS_TO_TICKS(intro));
            set_servo_angle(SERVO_CHANNEL,150);
            vTaskDelay(pdMS_TO_TICKS(intro));
            set_servo_angle(SERVO1_CHANNEL,0);//right
            vTaskDelay(pdMS_TO_TICKS(intro));
            set_servo_angle(SERVO_CHANNEL,180);
            vTaskDelay(pdMS_TO_TICKS(intro-200));
            set_servo_angle(SERVO1_CHANNEL,30);//right
            vTaskDelay(pdMS_TO_TICKS(intro));
            set_servo_angle(SERVO_CHANNEL,150);
            vTaskDelay(pdMS_TO_TICKS(intro));
            set_servo_angle(SERVO1_CHANNEL,0);
            set_servo_angle(SERVO_CHANNEL,180);
            vTaskDelay(pdMS_TO_TICKS(intro));
            //intial 5 second
             // initial first 5 second task
            set_servo_angle(SERVO1_CHANNEL,30);//right
            vTaskDelay(pdMS_TO_TICKS(intro));
            set_servo_angle(SERVO_CHANNEL,150);
            vTaskDelay(pdMS_TO_TICKS(intro));
            set_servo_angle(SERVO1_CHANNEL,0);//right
            vTaskDelay(pdMS_TO_TICKS(intro));
            set_servo_angle(SERVO_CHANNEL,180);
            vTaskDelay(pdMS_TO_TICKS(intro-200));
            set_servo_angle(SERVO1_CHANNEL,30);//right
            vTaskDelay(pdMS_TO_TICKS(intro));
            set_servo_angle(SERVO_CHANNEL,150);
            vTaskDelay(pdMS_TO_TICKS(intro));
            set_servo_angle(SERVO1_CHANNEL,0);
            set_servo_angle(SERVO_CHANNEL,180);
            vTaskDelay(pdMS_TO_TICKS(intro));
            //intial 5 second
             // initial first 5 second task
            set_servo_angle(SERVO1_CHANNEL,30);//right
            vTaskDelay(pdMS_TO_TICKS(intro));
            set_servo_angle(SERVO_CHANNEL,150);
            vTaskDelay(pdMS_TO_TICKS(intro));
            set_servo_angle(SERVO1_CHANNEL,0);//right
            vTaskDelay(pdMS_TO_TICKS(intro));
            set_servo_angle(SERVO_CHANNEL,180);
            vTaskDelay(pdMS_TO_TICKS(intro-200));
            set_servo_angle(SERVO1_CHANNEL,30);//right
            vTaskDelay(pdMS_TO_TICKS(intro));
            set_servo_angle(SERVO_CHANNEL,150);
            vTaskDelay(pdMS_TO_TICKS(intro));
            set_servo_angle(SERVO1_CHANNEL,0);
            set_servo_angle(SERVO_CHANNEL,180);
            vTaskDelay(pdMS_TO_TICKS(intro));
            //intial 5 second
            break;
            //inital 5 seconds task
    } 
    set_servo_angle(SERVO1_CHANNEL,0);
    set_servo_angle(SERVO_CHANNEL,180);
    vTaskDelay(pdMS_TO_TICKS(200));
    vTaskDelete(NULL);
    
}

//______________________________________________________________________________________________________
//______________________________________________________________________________________________________
//______________________________________________________________________________________________________


void play_song() {
    if (is_playing) {
        return;  
    }
    is_playing = true;
    is_dancing = true;
    gpio_set_level(AMP_SD_PIN, 1);  // Turn ON amplifier
    FILE *f = fopen(WAV_FILE_SENSOR5, "r");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open WAV file of song");
        is_playing = false;
        return;
    }
    uint8_t header[44];
    if (fread(header, 1, 44, f) != 44) {
        ESP_LOGE(TAG, "Failed to read WAV header of song");
        fclose(f);
        is_playing = false;
        return;
    }
    int16_t audio_buffer[512];
    size_t bytes_written;
    size_t samples_read;
    uint16_t touch_value6,touch_value7, touch_value3, touch_value4, touch_value5;
    int iteration_counter = 0;  
    xTaskCreate(servo_dance_task, "servo_dance_task", 2048, NULL, 5, NULL);
    while ((samples_read = fread(audio_buffer, sizeof(int16_t), 512, f)) > 0) {
        iteration_counter++;
        // Check the touch sensor only after 94 iterations (approximately 3 seconds)
        gpio_set_level(LED_GPIO_PIN,1);
        i2s_write(I2S_NUM, audio_buffer, samples_read * sizeof(int16_t), &bytes_written, portMAX_DELAY);
        if (iteration_counter >= 95 && iteration_counter % 5 == 0) {
            touch_pad_read(TOUCH_PAD_NUM6, &touch_value6);
            touch_pad_read(TOUCH_PAD_NUM7, &touch_value7);
            touch_pad_read(TOUCH_PAD_NUM3, &touch_value3); 
            touch_pad_read(TOUCH_PAD_NUM4, &touch_value4);
            touch_pad_read(TOUCH_PAD_NUM5, &touch_value5);
            if (touch_value5 < TOUCH_THRESHOLD) {
                ESP_LOGI(TAG0, "Breaking the audio playback due to touch sensor input");
                is_dancing = false;
                vTaskDelay(pdMS_TO_TICKS(500));
                break;  // If touched, stop playing the audio
            }
            if (touch_value6 < (TOUCH_THRESHOLD - 200)) {
                ESP_LOGI(TAG0, "inside play_sound value6");
                fclose(f);
                is_playing = false;
                is_dancing = false;
                play_audio(WAV_FILE_SENSOR6);
                break;           
            }
            if (touch_value7 < TOUCH_THRESHOLD) {
                ESP_LOGI(TAG0, "inside play_sound value7");
                fclose(f);
                is_playing = false;
                is_dancing = false;
                play_audio(WAV_FILE_SENSOR7);
                break;
            }
            if (touch_value3 < TOUCH_THRESHOLD) {
                ESP_LOGI(TAG0, "inside play_sound value3");
                fclose(f);
                is_playing = false;
                is_dancing = false;
                play_audio(WAV_FILE_SENSOR3);
                break;
            }
            if (touch_value4 < TOUCH_THRESHOLD) {
                ESP_LOGI(TAG0, "inside play_sound value4");
                fclose(f);
                is_playing = false;
                is_dancing = false;
                play_audio(WAV_FILE_SENSOR4);
                break;
    
            }
          
        }
        gpio_set_level(LED_GPIO_PIN,0); 
       
    }
    gpio_set_level(AMP_SD_PIN, 0);
    fclose(f);
    is_playing = false;
    is_dancing = false;
    ESP_LOGI(TAG0, "Finished playing song");
    gpio_set_level(LED_GPIO_PIN,0); 

    vTaskDelay(pdMS_TO_TICKS(200));
}


//______________________________________________________________________________________________________
//______________________________________________________________________________________________________
//______________________________________________________________________________________________________

void touch_sensor_task(void *pvParameters) {
    uint16_t touch_value6,touch_value7, touch_value3, touch_value4, touch_value5;

    while (1) {
        touch_pad_read(TOUCH_PAD_NUM6, &touch_value6);
        touch_pad_read(TOUCH_PAD_NUM7, &touch_value7);
        touch_pad_read(TOUCH_PAD_NUM3, &touch_value3);
        touch_pad_read(TOUCH_PAD_NUM4, &touch_value4);
        touch_pad_read(TOUCH_PAD_NUM5, &touch_value5);

        if (touch_value6 < (TOUCH_THRESHOLD - 200)&& !is_playing) {
            ESP_LOGI(TAG0, "Sensor 6 touched");
            play_audio(WAV_FILE_SENSOR6);
        }
        else if (touch_value7 < TOUCH_THRESHOLD && !is_playing) {
            ESP_LOGI(TAG0, "Sensor 7 (GPIO 27) touched:");
            play_audio(WAV_FILE_SENSOR7);
             
        }
        else if (touch_value3 < TOUCH_THRESHOLD && !is_playing) {
            ESP_LOGI(TAG0, "Sensor 3 touched");
            play_audio(WAV_FILE_SENSOR3);
        }
        else if (touch_value4 < TOUCH_THRESHOLD && !is_playing) {
            ESP_LOGI(TAG0, "Sensor 4 (GPIO 13) touched:");
            play_audio(WAV_FILE_SENSOR4);
        }
        else if (touch_value5 < TOUCH_THRESHOLD && !is_playing) {
            ESP_LOGI(TAG0, "Sensor 5 (GPIO 12) touched");
            play_song();

        }
        

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

//______________________________________________________________________________________________________
//______________________________________________________________________________________________________
//______________________________________________________________________________________________________

void app_main(void) {
    led_init();
    touch_pad_init();
    touch_pad_filter_start(TOUCHPAD_NOISE_FILTER);
    calibrate_touch_sensors();
    gpio_set_direction(AMP_SD_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(AMP_SD_PIN, 0);  // Start with AMP off
    gpio_set_level(LED_GPIO_PIN,0);


    esp_vfs_spiffs_conf_t config = {
        .base_path = "/storage",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };
    esp_err_t result = esp_vfs_spiffs_register(&config);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount or format SPIFFS");
        return;
    }
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // Resolution (8192 levels)
        .freq_hz = SERVO_FREQ,               // 50 Hz for servos
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = SERVO_TIMER
    };
    ledc_timer_config(&ledc_timer);
  // Configure the LEDC channel for servo control
    ledc_channel_config_t ledc_channel = {
        .channel = SERVO_CHANNEL,
        .duty = 0,
        .gpio_num = SERVO_GPIO,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint = 0,
        .timer_sel = SERVO_TIMER
    };
    ledc_channel_config(&ledc_channel);
    ledc_channel_config_t ledc1_channel = {
        .channel = SERVO1_CHANNEL,
        .duty = 0,
        .gpio_num = SERVO1_GPIO,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint = 0,
        .timer_sel = SERVO_TIMER
    };
    ledc_channel_config(&ledc1_channel);
    ESP_LOGI(TAG, "SPIFFS mounted successfully");
        
    configure_i2s();
    set_servo_angle(SERVO_CHANNEL,180);
    set_servo_angle(SERVO1_CHANNEL,0);

    touch_pad_config(TOUCH_PAD_NUM6, TOUCH_THRESHOLD); // GPIO 4
    touch_pad_config(TOUCH_PAD_NUM7, TOUCH_THRESHOLD); // GPIO 2
    touch_pad_config(TOUCH_PAD_NUM3, TOUCH_THRESHOLD); // GPIO 15
    touch_pad_config(TOUCH_PAD_NUM4, TOUCH_THRESHOLD); // GPIO 13
    touch_pad_config(TOUCH_PAD_NUM5, TOUCH_THRESHOLD); // GPIO 12


    xTaskCreate(touch_sensor_task, "touch_sensor_task", 4096, NULL, 5, NULL);
}
