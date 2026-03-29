#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"

#define LED1_GPIO   4
#define LED2_GPIO   5
#define LED3_GPIO   6
#define LED4_GPIO   7
#define BUZZER_GPIO 8

#define LEDC_MODE   LEDC_LOW_SPEED_MODE
#define LED_TIMER   LEDC_TIMER_0
#define BUZ_TIMER   LEDC_TIMER_1
#define DUTY_MAX    1023   // 10 bits

#define DELAY_MS    50

static void init_pwm(void);
static void set_led(int channel, int duty);
static void buzzer_tone(int freq);
static void buzzer_off(void);

static void phase1_sync_fade(void);
static void phase2_sequential_fade(void);
static void phase3_buzzer_sweep(void);

static void init_pwm(void) {
    // Timer LEDs
    ledc_timer_config_t timer_led = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LED_TIMER,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_led));

    // Timer buzzer
    ledc_timer_config_t timer_buz = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = BUZ_TIMER,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_buz));

    int pins[4] = {LED1_GPIO, LED2_GPIO, LED3_GPIO, LED4_GPIO};

    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t ch = {
            .gpio_num = pins[i],
            .speed_mode = LEDC_MODE,
            .channel = (ledc_channel_t)i,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LED_TIMER,
            .duty = 0,
            .hpoint = 0
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch));
    }

    ledc_channel_config_t buzzer = {
        .gpio_num = BUZZER_GPIO,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_4,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = BUZ_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&buzzer));
}

static void set_led(int channel, int duty) {
    ledc_set_duty(LEDC_MODE, channel, duty);
    ledc_update_duty(LEDC_MODE, channel);
}

static void buzzer_tone(int freq) {
    ledc_set_freq(LEDC_MODE, BUZ_TIMER, freq);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_4, DUTY_MAX / 2);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_4);
}

static void buzzer_off(void) {
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_4, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_4);
}

static void phase1_sync_fade(void) {
    printf("FASE 1 - Fade sincronizado\n");

    for (int duty = 0; duty <= DUTY_MAX; duty += 50) {
        for (int ch = 0; ch < 4; ch++) {
            set_led(ch, duty);
        }
        vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
    }

    for (int duty = DUTY_MAX; duty >= 0; duty -= 50) {
        for (int ch = 0; ch < 4; ch++) {
            set_led(ch, duty);
        }
        vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
    }

    vTaskDelay(pdMS_TO_TICKS(300));
}

static void phase2_sequential_fade(void) {
    printf("FASE 2 - Fade sequencial\n");

    int sequence[] = {0, 1, 2, 3, 2, 1, 0};
    int len = sizeof(sequence) / sizeof(sequence[0]);

    for (int s = 0; s < len; s++) {
        int ch = sequence[s];

        for (int duty = 0; duty <= DUTY_MAX; duty += 100) {
            set_led(ch, duty);
            vTaskDelay(pdMS_TO_TICKS(30));
        }

        for (int duty = DUTY_MAX; duty >= 0; duty -= 100) {
            set_led(ch, duty);
            vTaskDelay(pdMS_TO_TICKS(30));
        }
    }

    vTaskDelay(pdMS_TO_TICKS(300));
}

static void phase3_buzzer_sweep(void) {
    printf("FASE 3 - Buzzer sweep\n");

    for (int freq = 500; freq <= 2000; freq += 100) {
        buzzer_tone(freq);
        vTaskDelay(pdMS_TO_TICKS(80));
    }

    for (int freq = 2000; freq >= 500; freq -= 100) {
        buzzer_tone(freq);
        vTaskDelay(pdMS_TO_TICKS(80));
    }

    buzzer_off();

    vTaskDelay(pdMS_TO_TICKS(500));
}

void app_main(void) {
    init_pwm();

    while (1) {
        phase1_sync_fade();
        phase2_sequential_fade();
        phase3_buzzer_sweep();
    }
}