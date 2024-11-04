#include <stdio.h>  // Biblioteca padrão para entrada e saída de dados
#include <inttypes.h>  // Define tipos de dados com tamanhos específicos, como uint32_t
#include "sdkconfig.h"  // Configurações do SDK usadas no projeto
#include "freertos/FreeRTOS.h"  // Inclui a API do FreeRTOS, como gerenciamento de tarefas
#include "freertos/task.h"  // API para manipulação de tarefas específicas do FreeRTOS
#include "esp_chip_info.h"  // Fornece informações sobre o chip ESP32
#include "esp_flash.h"  // API para acesso e manipulação da memória Flash
#include "esp_log.h"  // Biblioteca para o sistema de logs do ESP-IDF

// Declaração de uma tag global para identificar o módulo em mensagens de log
static const char *TAG = "Pratica1";  

// Função principal do programa, executada ao iniciar o ESP32
void app_main(void) {
    // Exibe uma mensagem informando o início da coleta de informações do chip
    ESP_LOGI(TAG, "Iniciando o sistema e coletando informações do chip...");

    /* Obter informações sobre o chip e suas características */
    esp_chip_info_t chip_info;  // Estrutura que armazena dados sobre o chip
    uint32_t flash_size;  // Variável para armazenar o tamanho da memória flash
    esp_chip_info(&chip_info);  // Preenche a estrutura com dados do chip

    // Exibe informações básicas sobre o chip, como núcleos e recursos disponíveis
    ESP_LOGI(TAG, "Chip: %s com %d núcleo(s) de CPU e recursos Wi-Fi%s%s%s",
             CONFIG_IDF_TARGET,  // Tipo de chip (ESP32, ESP32-S3, etc.)
             chip_info.cores,  // Número de núcleos da CPU
             (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",  // Bluetooth
             (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "",  // Bluetooth Low Energy (BLE)
             (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");  // Zigbee/Thread

    // Divide a revisão do silício em duas partes: maior e menor
    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    // Exibe a revisão do chip
    ESP_LOGI(TAG, "Revisão do silício: v%d.%d", major_rev, minor_rev);

    // Tenta obter o tamanho da memória flash e exibe um erro caso falhe
    if (esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        ESP_LOGE(TAG, "Erro ao obter o tamanho da memória flash.");  // Mensagem de erro
        return;  // Interrompe a execução se falhar
    }

    // Exibe o tamanho da memória flash e informa se é interna ou externa
    ESP_LOGI(TAG, "Memória Flash: %" PRIu32 "MB (%s)",
             flash_size / (1024 * 1024),  // Converte bytes para megabytes
             (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "interna" : "externa");  // Verifica tipo da memória

    // Exibe a quantidade mínima de memória heap disponível durante a execução
    ESP_LOGI(TAG, "Versão mínima disponível de heap: %" PRIu32 " bytes", 
             esp_get_minimum_free_heap_size());

}
