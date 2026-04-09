#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#define LED_PIN 4
#define BUTTON_PIN 17

#define DEBOUNCE_TIME 50

void app_main() {
gpio_config_t io_conf = {};

// LED
io_conf.pin_bit_mask = (1ULL << LED_PIN);
io_conf.mode = GPIO_MODE_OUTPUT;
io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
gpio_config(&io_conf);

// BOTÃO
io_conf.pin_bit_mask = (1ULL << BUTTON_PIN);
io_conf.mode = GPIO_MODE_INPUT;
io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
gpio_config(&io_conf);

int led_state = 0;
int last_button = 0;

int64_t led_on_time = 0;

while (1) {
  int button = gpio_get_level(BUTTON_PIN);

  if (button == 1 && last_button == 0) {
    vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_TIME));

    if (gpio_get_level(BUTTON_PIN) == 1) {
      led_state = !led_state;

      gpio_set_level(LED_PIN, led_state);

      if (led_state)
      led_on_time = esp_timer_get_time();
    }
  }

  last_button = button;

  if (led_state) {
    int64_t now = esp_timer_get_time();

    if ((now - led_on_time) >= 10000000) {
      led_state = 0;
      gpio_set_level(LED_PIN, 0);
    }
  }

  vTaskDelay(pdMS_TO_TICKS(10));
  }
}