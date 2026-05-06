#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h" // Biblioteca para ler as tensões

#define POT_PIN        ADC_CHANNEL_6   // GPIO7]





#define BUTTON_PIN     47
#define LED_PIN        39

#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL        LEDC_CHANNEL_0
#define LEDC_DUTY_RES       LEDC_TIMER_12_BIT // 12 bits, vai de 0 a 4095
#define LEDC_FREQUENCY      5000

bool hold_mode = false;
int frozen_value = 0;

void pwm_init()
{
    ledc_timer_config_t timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };

    ledc_timer_config(&timer);

    ledc_channel_config_t channel = {
        .gpio_num   = LED_PIN,
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CHANNEL,
        .timer_sel  = LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0
    };

    ledc_channel_config(&channel);
}

void app_main(void)
{
    // BOTÃO
    gpio_config_t btn = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };

    gpio_config(&btn);

    // PWM
    pwm_init();

    // ADC
    adc_oneshot_unit_handle_t adc_handle;

    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1
    };

    adc_oneshot_new_unit(&init_config, &adc_handle);

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12
    };

    adc_oneshot_config_channel(adc_handle, POT_PIN, &config);

    int adc_raw;
    int voltage;

    bool last_button = false;

    while (1)
    {
        bool current_button = gpio_get_level(BUTTON_PIN);

        // Detecção de borda
        if (current_button && !last_button)
        {
            hold_mode = !hold_mode;

            if (hold_mode)
            {
                frozen_value = adc_raw;
            }

            vTaskDelay(pdMS_TO_TICKS(200));
        }

        last_button = current_button;

        if (!hold_mode)
        {
            adc_oneshot_read(adc_handle, POT_PIN, &adc_raw);
        }
        else
        {
            adc_raw = frozen_value;
        }

        // Conversão para mV
        voltage = (adc_raw * 3300) / 4095;

        // PWM proporcional
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, adc_raw);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

        printf("ADC: %d | Tensao: %d mV | Estado: %s\n",
               adc_raw,
               voltage,
               hold_mode ? "HOLD" : "LIVE");

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}