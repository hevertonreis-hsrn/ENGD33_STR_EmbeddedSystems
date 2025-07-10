#include "main.h"
#include "cmsis_os.h"

typedef struct {
    float velocidade;
    float corrente_motor;
    float pos_x;
    float pos_y;
    float orientacao;
    float corrente_ref[3];
    float velocidade_ref[3];
    int8_t sinal_motor[3];
} DadosRobo_t;

typedef enum {
    CMD_RESET,
    CMD_MODO_AUTONOMO,
    CMD_MODO_MANUAL,
    // Possibilidade de expansão
} ComandoRecebido_t;

// UART real (declarada externamente em main.c)
extern UART_HandleTypeDef huart6;

// Buffer global para recepção de 1 byte via interrupção
volatile uint8_t comando_recebido;

// Fila de dados do robô
QueueHandle_t xFilaDadosRobo;

// Buffer para comandos recebidos
QueueHandle_t xFilaComandos;

// Notificação para dados recebidos via UART
TaskHandle_t xHandleRecepcao = NULL;

void vTaskGeraDados(void *pvParameters) {
    DadosRobo_t dados;

    while (1) {
        dados.velocidade = 1.0 + rand() % 10;
        dados.corrente_motor = 0.5 + rand() % 5;
        dados.pos_x += 0.1f;
        dados.pos_y += 0.1f;
        dados.orientacao += 0.05f;

        for (int i = 0; i < 3; i++) {
            dados.velocidade_ref[i] = 2.0f;
            dados.corrente_ref[i] = 0.5f;
            dados.sinal_motor[i] = 1;
        }

        // Envia a struct dados para a fila. 
        // A portMAX_DELAY garante que a tarefa espere 
        // indefinidamente se a fila estiver cheia 
        // (não trava o sistema).
        xQueueSend(xFilaDadosRobo, &dados, portMAX_DELAY);

        // Espera 100ms antes de gerar o próximo pacote, 
        // simulando uma frequência de atualização de 10 Hz
        vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz
    }
}

void vTaskTransmissao(void *pvParameters) {
    DadosRobo_t dados;

    while (1) {
        // Espera indefinidamente até receber um item da fila de dados simulados.
        if (xQueueReceive(xFilaDadosRobo, &dados, portMAX_DELAY)) {
            HAL_UART_Transmit(&huart6, (uint8_t*)&dados, sizeof(dados), HAL_MAX_DELAY);
        }
    }
}

// Função de callback chamada quando um byte é recebido via UART
// Esta função é chamada automaticamente pela HAL quando a recepção é concluída
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart6) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        ComandoRecebido_t cmd = (ComandoRecebido_t)comando_recebido;
        xQueueSendFromISR(xFilaComandos, &cmd, &xHigherPriorityTaskWoken);
        HAL_UART_Receive_IT(&huart6, &comando_recebido, 1);

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void vTaskRecepcao(void *pvParameters) {
    ComandoRecebido_t comando;

    while (1) {
        if (xQueueReceive(xFilaComandos, &comando, portMAX_DELAY)) {
            switch (comando) {
                case CMD_RESET:
                    // Lógica de reset
                    break;
                case CMD_MODO_AUTONOMO:
                    // Lógica de modo autônomo
                    break;
                case CMD_MODO_MANUAL:
                    // Lógica de modo manual
                    break;
                default:
                    break;
            }
        }
    }
}

void SetupTarefasPortaCOMM(void) {
    // Inicializa as filas
    xFilaDadosRobo = xQueueCreate(10, sizeof(DadosRobo_t));
    xFilaComandos  = xQueueCreate(5, sizeof(ComandoRecebido_t));

    // Verifica se a criação das filas foi bem-sucedida (opcional, mas recomendado)
    configASSERT(xFilaDadosRobo != NULL);
    configASSERT(xFilaComandos  != NULL);

    // Inicia recepção UART com interrupção
    HAL_UART_Receive_IT(&huart6, &comando_recebido, 1);

    // Cria as tarefas
    xTaskCreate(vTaskGeraDados, "GeraDados",      256, NULL, 2, NULL);
    xTaskCreate(vTaskTransmissao, "Tx",           256, NULL, 2, NULL);
    xTaskCreate(vTaskRecepcao, "Rx",              256, NULL, 2, &xHandleRecepcao);
}
