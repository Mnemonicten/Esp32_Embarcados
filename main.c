#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "esp_err.h"

// Definições de GPIO e TAGs para log
#define GPIO_OUTPUT_LED    2  // LED azul no GPIO2
#define GPIO_INPUT_BTN_0   21 // Botão 0 no GPIO21
#define GPIO_INPUT_BTN_1   22 // Botão 1 no GPIO22
#define GPIO_INPUT_BTN_2   23 // Botão 2 no GPIO23
#define ESP_INTR_FLAG_DEFAULT 0
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO_PWM0     (32) // Define the output GPIO
#define LEDC_OUTPUT_IO_PWM1     (17) // Define the output GPIO
#define LEDC_CHANNEL_PWM0       LEDC_CHANNEL_0
#define LEDC_CHANNEL_PWM1       LEDC_CHANNEL_1  
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4096) // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY          (4000) // Frequency in Hertz. Set frequency at 4 kHz

static const char *TAG1 = "Configuração";
static const char *TAG2 = "GPIO";
static const char *TAG3 = "Relogio";
static const char *TAG4 = "PWM";
static QueueHandle_t filagpio = NULL;
static QueueHandle_t filatimer = NULL;

//////////////////////////// ATIVIDADE 4 ///////////////////////////////

static void configure_and_start_led(void) {
    // PWM1
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer_0 = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_0));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel_0 = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_PWM0 ,
        .timer_sel      = LEDC_TIMER, 
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO_PWM0,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_0));

    // PWM2
    // Prepare and then apply the LEDC PWM timer configuration 
    ledc_timer_config_t ledc_timer_1 = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_1));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel_1 = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_PWM1 ,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO_PWM1,
        // não precisa setar o valor do duty na main como no PWM0
        .duty           = 4000, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_1));

    
    // Set duty to 50%
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, LEDC_DUTY));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0));
}

//////////////////////////// ATIVIDADE 3 ///////////////////////////////

typedef struct {
    uint64_t event_count;
} queue_element;

typedef struct {
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
} real_time_clock_t;


static bool IRAM_ATTR timer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
    QueueHandle_t queue = (QueueHandle_t)user_data;

// Calculate time in seconds
    uint64_t total_seconds = edata->count_value / 1000000;
    real_time_clock_t rtc = {
        .hours = (total_seconds / 3600) % 24,
        .minutes = (total_seconds / 60) % 60,
        .seconds = total_seconds % 60
    };

    xQueueSendFromISR(queue, &rtc, &high_task_awoken);

    // reconfigure alarm value
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = edata->alarm_value + 1000000, // alarm in next 1s
    };
    gptimer_set_alarm_action(timer, &alarm_config);

    // return whether we need to yield at the end of ISR
    return (high_task_awoken == pdTRUE);
}

static void timer_task(void* arg) {

    filatimer = xQueueCreate(10, sizeof(real_time_clock_t));

    real_time_clock_t rtc;

        // Configurando timer
    ESP_LOGI(TAG3, "Definindo configurações do timer");
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_callback,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, filatimer));

    // Inicializando timer
    ESP_LOGI(TAG3, "Enable timer");
    ESP_ERROR_CHECK(gptimer_enable(gptimer));

    ESP_LOGI(TAG3, "Start timer, stop it at alarm event");
    gptimer_alarm_config_t alarm_config1 = {
        .alarm_count = 100000, // period = 100ms
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config1));
    ESP_ERROR_CHECK(gptimer_start(gptimer));


    for (;;) {
        if (xQueueReceive(filatimer, &rtc, portMAX_DELAY)) {
            ESP_LOGI(TAG3, "Clock: %02u:%02u:%02u", rtc.hours, rtc.minutes, rtc.seconds);
        }
    }
}

void config_and_start_timer(void){
    
    xTaskCreate(timer_task, "timer_task", 2048, NULL, 10, NULL);

}

//////////////////////////// ATIVIDADE 2 ///////////////////////////////

// Função de interrupção para o GPIO
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(filagpio, &gpio_num, NULL);
}

// Tarefa para manipulação de eventos de GPIO
static void gpio_task_example(void* arg) {
    uint32_t io_num;
    int led_state = 0;

     // Configuração do LED como saída
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_OUTPUT_LED);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // Configuração dos botões como entrada com PULL UP e interrupção
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = ((1ULL << GPIO_INPUT_BTN_0) | (1ULL << GPIO_INPUT_BTN_1) | (1ULL << GPIO_INPUT_BTN_2));
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

        // Instalar o serviço de interrupção GPIO
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_INPUT_BTN_0, gpio_isr_handler, (void*) GPIO_INPUT_BTN_0);
    gpio_isr_handler_add(GPIO_INPUT_BTN_1, gpio_isr_handler, (void*) GPIO_INPUT_BTN_1);
    gpio_isr_handler_add(GPIO_INPUT_BTN_2, gpio_isr_handler, (void*) GPIO_INPUT_BTN_2);

    for(;;) {
        if (xQueueReceive(filagpio, &io_num, portMAX_DELAY)) {
            switch (io_num) {
                case GPIO_INPUT_BTN_0:
                    ESP_LOGI(TAG2, "Botão 0 pressionado: LED ACESO");
                    gpio_set_level(GPIO_OUTPUT_LED, 1);
                    break;
                case GPIO_INPUT_BTN_1:
                    ESP_LOGI(TAG2, "Botão 1 pressionado: LED APAGADO");
                    gpio_set_level(GPIO_OUTPUT_LED, 0);
                    break;
                case GPIO_INPUT_BTN_2:
                    led_state = !led_state;
                    ESP_LOGI(TAG2, "Botão 2 pressionado: LED estado invertido para %d", led_state);
                    gpio_set_level(GPIO_OUTPUT_LED, led_state);
                    break;
                default:
                    break;
            }
        }
    }
}

//////////////////////////// ATIVIDADE 1 ///////////////////////////////

// Função para coleta e exibição de informações do chip
void display_chip_info() {

    // Criar uma fila para gerenciar eventos GPIO e iniciar a tarefa de controle de GPIO
    queue_element ele;
    filagpio = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);
   

    ESP_LOGI(TAG1, "Iniciando o sistema e coletando informações do chip...");

    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);

    ESP_LOGI(TAG1, "Chip: %s com %d núcleo(s) de CPU e recursos Wi-Fi%s%s%s",
             CONFIG_IDF_TARGET,
             chip_info.cores,
             (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
             (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "",
             (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    ESP_LOGI(TAG1, "Revisão do silício: v%d.%d", major_rev, minor_rev);

    if (esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        ESP_LOGE(TAG1, "Erro ao obter o tamanho da memória flash.");
        return;
    }

    ESP_LOGI(TAG1, "Memória Flash: %" PRIu32 "MB (%s)",
             flash_size / (1024 * 1024),
             (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "interna" : "externa");

    ESP_LOGI(TAG1, "Versão mínima disponível de heap: %" PRIu32 " bytes", 
             esp_get_minimum_free_heap_size());

    ESP_LOGI(TAG1, "Configuração inicializada. Aguardando pressionamento de botões...");
}

//////////////////////////// FUNÇÃO MAIN ///////////////////////////////

void app_main(void) {
    // ------------------------------ Atividade 1 ------------------------------
    display_chip_info();

    // ------------------------------ Atividade 2 ------------------------------
    config_and_start_timer();

    // ------------------------------ Atividade 4 ------------------------------
    configure_and_start_led();
}