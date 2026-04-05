#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"   /* Para obter o tempo em microsegundos (debounce) */

/* ============================================================
 * DEFINIÇÕES DE PINOS
 * ============================================================ */

/* Pinos dos LEDs */
#define LED_BIT0  GPIO_NUM_16   
#define LED_BIT1  GPIO_NUM_15   
#define LED_BIT2  GPIO_NUM_5   
#define LED_BIT3  GPIO_NUM_4   

/* Pinos dos botões */
#define BTN_A  GPIO_NUM_42
#define BTN_B  GPIO_NUM_41

/* ============================================================
 * DEFINIÇÕES DO SISTEMA
 * ============================================================ */

/* Tempo mínimo entre dois acionamentos válidos. */
#define DEBOUNCE_US  50000

/* Valor máximo do contador de 4 bits */
#define CONTADOR_MAX  15

/* ============================================================
 * VARIÁVEIS GLOBAIS
 * ============================================================ */

/* Valor atual do contador */
static uint8_t contador = 0;

/* Passo de incremento atual */
static uint8_t passo = 1;

/* Timestamps do último acionamento válido de cada botão. */
static int64_t ultimo_tempo_btn_a = 0;
static int64_t ultimo_tempo_btn_b = 0;

/* ============================================================
 * FUNÇÕES AUXILIARES
 * ============================================================ */

static void atualiza_leds(void)
{
    /* Extrai cada bit do contador usando operação AND com máscara,
     * depois converte para nível lógico (0 ou 1) */
    gpio_set_level(LED_BIT0, !!(contador & 0x01)); /* bit 0: máscara 0001 */
    gpio_set_level(LED_BIT1, !!(contador & 0x02)); /* bit 1: máscara 0010 */
    gpio_set_level(LED_BIT2, !!(contador & 0x04)); /* bit 2: máscara 0100 */
    gpio_set_level(LED_BIT3, !!(contador & 0x08)); /* bit 3: máscara 1000 */
}

static void incrementa_contador(void)
{
    contador = (contador + passo) % (CONTADOR_MAX + 1);

    printf("Contador: %2d (0x%X) | Passo: +%d\n", contador, contador, passo);

    atualiza_leds();
}

static void alterna_passo(void)
{
    passo = (passo == 1) ? 2 : 1;
    printf("Passo alterado para: +%d\n", passo);
}

static int debounce_ok(int64_t *ultimo_tempo)
{
    int64_t agora = esp_timer_get_time(); /* Pegando tempo em µs  */
    int64_t decorrido = agora - *ultimo_tempo; /* Diferença em µs */

    if (decorrido >= DEBOUNCE_US) {
        *ultimo_tempo = agora;
        return 1;
    }

    return 0;
}

/* ============================================================
 * CONFIGURAÇÃO DOS GPIOs
 * ============================================================ */

static void configura_gpios(void)
{
    gpio_config_t cfg;

    /* Configuração dos LEDs */
    cfg.pin_bit_mask = (1ULL << LED_BIT0) |  
                       (1ULL << LED_BIT1) |
                       (1ULL << LED_BIT2) |
                       (1ULL << LED_BIT3);
    cfg.mode         = GPIO_MODE_OUTPUT;      
    cfg.pull_up_en   = GPIO_PULLUP_DISABLE;   
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE; 
    cfg.intr_type    = GPIO_INTR_DISABLE;     
    gpio_config(&cfg);

    /* Configuração dos Botões */
    cfg.pin_bit_mask = (1ULL << BTN_A) |      
                       (1ULL << BTN_B);
    cfg.mode         = GPIO_MODE_INPUT;        
    cfg.pull_up_en   = GPIO_PULLUP_DISABLE;    
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;  
    cfg.intr_type    = GPIO_INTR_DISABLE;      
    gpio_config(&cfg);

    /* Setando todos os leds para iniciarem apagados */
    gpio_set_level(LED_BIT0, 0);
    gpio_set_level(LED_BIT1, 0);
    gpio_set_level(LED_BIT2, 0);
    gpio_set_level(LED_BIT3, 0);
}

/* ============================================================
 * LEITURA DOS BOTÕES
 * ============================================================ */

static void tarefa_botoes(void *arg)
{
    int estado_anterior_a = 0;
    int estado_anterior_b = 0;

    while (1) {
        int estado_a = gpio_get_level(BTN_A);
        int estado_b = gpio_get_level(BTN_B);

        if (estado_a == 1 && estado_anterior_a == 0) {
            if (debounce_ok(&ultimo_tempo_btn_a)) {
                incrementa_contador();
            }
        }

        if (estado_b == 1 && estado_anterior_b == 0) {
            if (debounce_ok(&ultimo_tempo_btn_b)) {
                alterna_passo();
            }
        }

        estado_anterior_a = estado_a;
        estado_anterior_b = estado_b;

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    printf("=== Contador Binário de 4 bits ===\n");
    printf("Botão A: incrementa | Botão B: alterna passo (+1/+2)\n");
    printf("==================================\n");

    configura_gpios();

    atualiza_leds();
    xTaskCreate(tarefa_botoes, "botoes", 2048, NULL, 5, NULL);
}