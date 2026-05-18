#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

/* ------------------------------------------------------------------------- */
/* Pinos e configuracoes de perifericos                                       */
/* ------------------------------------------------------------------------- */
#define POT_PIN              ADC_CHANNEL_1   /* GPIO2 -> ADC1_CH1 */
#define BUTTON_PIN           47
#define LED_PIN              35

/* PWM (LEDC) - resolucao de 13 bits e 5kHz, conforme enunciado */
#define LEDC_TIMER           LEDC_TIMER_0
#define LEDC_MODE            LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL         LEDC_CHANNEL_0
#define LEDC_DUTY_RES        LEDC_TIMER_13_BIT     /* 13 bits = 0..8191 */
#define LEDC_FREQUENCY       5000                  /* 5 kHz */
#define LEDC_DUTY_MAX        ((1 << 13) - 1)       /* 8191 */

/* ADC - 12 bits (0..4095), atenuacao 12 dB (~3.3V no pino) */
#define ADC_BITWIDTH         ADC_BITWIDTH_12
#define ADC_MAX              4095
#define ADC_ATTEN            ADC_ATTEN_DB_12

/* I2C - 100 kHz Standard Mode */
#define I2C_PORT             I2C_NUM_0
#define I2C_SDA_IO           18
#define I2C_SCL_IO           918
#define I2C_FREQ_HZ          100000

/* MPU6050 */
#define MPU6050_ADDR         0x68
#define MPU6050_REG_PWR_MGMT 0x6B
#define MPU6050_REG_GYRO_CFG 0x1B
#define MPU6050_REG_ACCEL_CFG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_SENS_2G    16384.0f  /* LSB/g em escala +/- 2g */


// Recursos compartilhados do RTOS    

static QueueHandle_t     xQueuePotLed   = NULL;  /* Pot -> LED  (uint32_t duty) */
static SemaphoreHandle_t xSemHold       = NULL;  /* Button -> sistema           */
static SemaphoreHandle_t xMutexImu      = NULL;  /* protege struct IMU          */
static SemaphoreHandle_t xMutexStatus   = NULL;  /* protege variaveis de status */

/* Estado global do sistema (modo LIVE/HOLD) */
typedef enum { MODE_LIVE = 0, MODE_HOLD = 1 } sys_mode_t;
static volatile sys_mode_t g_mode = MODE_LIVE;

/* Snapshot para o console (atualizado pelas tarefas) */
typedef struct {
    int     adc_raw;      /* 0..4095 */
    int     voltage_mv;   /* 0..3300 */
    uint32_t duty;        /* 0..8191 */
} status_t;

static status_t g_status = {0};

typedef struct {
    float ax_g;
    float ay_g;
    float az_g;
} imu_data_t;

static imu_data_t g_imu = {0};

static const char *TAG = "ATV09";


// Inicializacao dos perifericos

static void pwm_init(void)
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

static void button_init(void)
{
    gpio_config_t btn = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE, 
        .pull_up_en   = GPIO_PULLUP_DISABLE
    };
    gpio_config(&btn);
}

static adc_oneshot_unit_handle_t adc_handle;

static void adc_init(void)
{
    adc_oneshot_unit_init_cfg_t init_config = { .unit_id = ADC_UNIT_1 };
    adc_oneshot_new_unit(&init_config, &adc_handle);

    adc_oneshot_chan_cfg_t cfg = {
        .bitwidth = ADC_BITWIDTH,
        .atten    = ADC_ATTEN
    };
    adc_oneshot_config_channel(adc_handle, POT_PIN, &cfg);
}

static esp_err_t i2c_init(void)
{
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_SDA_IO,
        .scl_io_num       = I2C_SCL_IO,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ
    };
    esp_err_t err = i2c_param_config(I2C_PORT, &conf);
    if (err != ESP_OK) return err;
    return i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
}


// Funcoes de baixo nivel para o MPU6050

static esp_err_t mpu_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    return i2c_master_write_to_device(I2C_PORT, MPU6050_ADDR,
                                      buf, sizeof(buf),
                                      pdMS_TO_TICKS(100));
}

static esp_err_t mpu_read_bytes(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_PORT, MPU6050_ADDR,
                                        &reg, 1,
                                        data, len,
                                        pdMS_TO_TICKS(100));
}

static esp_err_t mpu6050_init(void)
{
    esp_err_t err = mpu_write_reg(MPU6050_REG_PWR_MGMT, 0x00);
    if (err != ESP_OK) return err;

    err = mpu_write_reg(MPU6050_REG_ACCEL_CFG, 0x00);
    if (err != ESP_OK) return err;

    return mpu_write_reg(MPU6050_REG_GYRO_CFG, 0x00);
}


/* TAREFA 1: Potenciometro (Produtora)                                        */
/*   - Le ADC, escala para 13 bits (duty) e envia para a Queue                */
static void task_pot(void *arg)
{
    int adc_raw = 0;
    while (1) {
        adc_oneshot_read(adc_handle, POT_PIN, &adc_raw);

        uint32_t duty = ((uint32_t)adc_raw * LEDC_DUTY_MAX) / ADC_MAX;
        int voltage_mv = (adc_raw * 3300) / ADC_MAX;

        if (xSemaphoreTake(xMutexStatus, pdMS_TO_TICKS(50)) == pdTRUE) {
            g_status.adc_raw    = adc_raw;
            g_status.voltage_mv = voltage_mv;
            g_status.duty       = duty;
            xSemaphoreGive(xMutexStatus);
        }

        xQueueOverwrite(xQueuePotLed, &duty);

        vTaskDelay(pdMS_TO_TICKS(50));   /* 20 Hz */
    }
}

/* TAREFA 2: LED (Consumidora)                                                */
/*   - Recebe duty da Queue e atualiza PWM                                    */
/*   - Em modo HOLD, ignora novos dados e mantem o ultimo duty                */
static void task_led(void *arg)
{
    uint32_t duty = 0;
    uint32_t last_duty = 0;

    while (1) {
        if (g_mode == MODE_LIVE) {
            if (xQueueReceive(xQueuePotLed, &duty, pdMS_TO_TICKS(100)) == pdTRUE) {
                last_duty = duty;
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            }
        } else {
            uint32_t trash;
            while (xQueueReceive(xQueuePotLed, &trash, 0) == pdTRUE) {
            }

            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, last_duty);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

/* TAREFA 3: Button (Produtora)                                               */
/*   - Detecta borda de subida com debounce simples                           */
/*   - Libera o semaforo binario para alternar o estado                       */
static void task_button(void *arg)
{
    bool last = false;
    while (1) {
        bool current = gpio_get_level(BUTTON_PIN);

        if (current && !last) {
            xSemaphoreGive(xSemHold);
            vTaskDelay(pdMS_TO_TICKS(200));  
        }
        last = current;
        vTaskDelay(pdMS_TO_TICKS(20));       
    }
}

/* TAREFA auxiliar: Controlador de Estado                                     */
/*   - Pega o semaforo do botao e alterna entre LIVE e HOLD                   */
/*   - Mantida separada para evitar acoplar logica no task_led                */
static void task_state(void *arg)
{
    while (1) {
        if (xSemaphoreTake(xSemHold, portMAX_DELAY) == pdTRUE) {
            g_mode = (g_mode == MODE_LIVE) ? MODE_HOLD : MODE_LIVE;
            ESP_LOGI(TAG, "Botao -> modo %s",
                     (g_mode == MODE_LIVE) ? "LIVE" : "HOLD");
        }
    }
}

/* TAREFA 4: IMU (Produtora)                                                  */
/*   - Le 6 bytes do acelerometro do MPU6050                                  */
/*   - Converte para "g" usando sensibilidade de 16384 LSB/g (escala +/-2g)   */
/*   - Atualiza struct global protegida pelo Mutex                            */
static void task_imu(void *arg)
{
    uint8_t buf[6];
    while (1) {
        if (mpu_read_bytes(MPU6050_REG_ACCEL_XOUT_H, buf, 6) == ESP_OK) {
            int16_t ax_raw = (int16_t)((buf[0] << 8) | buf[1]);
            int16_t ay_raw = (int16_t)((buf[2] << 8) | buf[3]);
            int16_t az_raw = (int16_t)((buf[4] << 8) | buf[5]);

            float ax_g = ax_raw / MPU6050_ACCEL_SENS_2G;
            float ay_g = ay_raw / MPU6050_ACCEL_SENS_2G;
            float az_g = az_raw / MPU6050_ACCEL_SENS_2G;

            if (xSemaphoreTake(xMutexImu, pdMS_TO_TICKS(50)) == pdTRUE) {
                g_imu.ax_g = ax_g;
                g_imu.ay_g = ay_g;
                g_imu.az_g = az_g;
                xSemaphoreGive(xMutexImu);
            }
        } else {
            ESP_LOGW(TAG, "Falha lendo MPU6050");
        }
        vTaskDelay(pdMS_TO_TICKS(100));  /* 10 Hz */
    }
}

/* TAREFA 5: Console (Consumidora)                                            */
/*   - Coleta status e IMU (sob mutex) e imprime no formato pedido            */
static void task_console(void *arg)
{
    status_t   st_local;
    imu_data_t imu_local;

    while (1) {
        /* Le status do POT/LED sob protecao */
        if (xSemaphoreTake(xMutexStatus, pdMS_TO_TICKS(100)) == pdTRUE) {
            st_local = g_status;
            xSemaphoreGive(xMutexStatus);
        }

        /* Le IMU sob protecao - garante leitura coerente dos 3 eixos */
        if (xSemaphoreTake(xMutexImu, pdMS_TO_TICKS(100)) == pdTRUE) {
            imu_local = g_imu;
            xSemaphoreGive(xMutexImu);
        }

        int led_pct = (st_local.duty * 100) / LEDC_DUTY_MAX;

        printf("=====================================================\n");
        printf("STATUS: [%s] | POT: %4d (%4d mV) | LED: %3d%%\n",
               (g_mode == MODE_LIVE) ? "LIVE" : "HOLD",
               st_local.adc_raw,
               st_local.voltage_mv,
               led_pct);
        printf("IMU ACCEL (g): X: %+0.2f | Y: %+0.2f | Z: %+0.2f\n",
               imu_local.ax_g, imu_local.ay_g, imu_local.az_g);
        printf("=====================================================\n");

        vTaskDelay(pdMS_TO_TICKS(500));   /* 2 Hz */
    }
}

void app_main(void)
{
    button_init();
    pwm_init();
    adc_init();

    if (i2c_init() != ESP_OK) {
        ESP_LOGE(TAG, "Falha inicializando I2C");
    }
    if (mpu6050_init() != ESP_OK) {
        ESP_LOGE(TAG, "Falha inicializando MPU6050");
    }

    // Recursos do RTOS 
    xQueuePotLed = xQueueCreate(1, sizeof(uint32_t));    
    xSemHold     = xSemaphoreCreateBinary();
    xMutexImu    = xSemaphoreCreateMutex();
    xMutexStatus = xSemaphoreCreateMutex();

    if (!xQueuePotLed || !xSemHold || !xMutexImu || !xMutexStatus) {
        ESP_LOGE(TAG, "Falha criando objetos do RTOS");
        return;
    }

    // Tarefas
    xTaskCreate(task_button,  "task_button",  2048, NULL, 4, NULL);
    xTaskCreate(task_state,   "task_state",   2048, NULL, 3, NULL);
    xTaskCreate(task_led,     "task_led",     2048, NULL, 3, NULL);
    xTaskCreate(task_pot,     "task_pot",     2048, NULL, 2, NULL);
    xTaskCreate(task_imu,     "task_imu",     3072, NULL, 2, NULL);
    xTaskCreate(task_console, "task_console", 3072, NULL, 1, NULL);
}