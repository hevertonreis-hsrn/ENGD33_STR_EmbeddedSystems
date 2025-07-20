#include "main.h"
#include "cmsis_os.h"
#include "queue.h"
#include <stdlib.h>
#include <string.h>

#define TAM_FILA_DADOS_MOTOR       120
#define TAM_FILA_DADOS_VELOCIDADE  12
#define TAM_FILA_DADOS_GPS         2

#define TIPO_MOTOR   0x01
#define TIPO_VELOC   0x02
#define TIPO_GPS     0x03

#define TAMANHO_BUFFER_TRANSMISSAO (3 + sizeof(DadosMotor_t) + 3 + sizeof(DadosVelocidade_t) + 3 + sizeof(DadosGPS_t))

extern UART_HandleTypeDef huart6;

typedef struct {
    TickType_t timestamp;
    int16_t corrente[3];
    int16_t controle_tracao[3];
    float ganho_tracao[9];
} DadosMotor_t;

typedef struct {
    TickType_t timestamp;
    int16_t velocidade_angular[3];
    int16_t aceleracao_linear[3];
    int16_t giroscopio[3];
    int16_t campo_magnetico[3];
    int16_t controle_velocidade[3];
    int16_t setpoint_velocidade[3];
    float ganho_velocidade[9];
    int16_t angulo_rotacao[3];
} DadosVelocidade_t;

typedef struct {
    TickType_t timestamp;
    int32_t x, y, z;
    int16_t controle_posicao[3];
    float ganho_posicao[3];
    int16_t roll, pitch, yaw;
} DadosGPS_t;

typedef enum {
    CMD_RESET,
    CMD_MODO_AUTONOMO,
    CMD_MODO_MANUAL,
} ComandoRecebido_t;

QueueHandle_t xFilaDadosMotor;
QueueHandle_t xFilaDadosVelocidade;
QueueHandle_t xFilaDadosGPS;
QueueHandle_t xFilaComandos;

TaskHandle_t xHandleRecepcao = NULL;

uint8_t comando_recebido;

void vTaskGeraDadosMotor(void *pvParameters) {
    DadosMotor_t dados;
    const TickType_t xPeriod = pdMS_TO_TICKS(1);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        dados.timestamp = xTaskGetTickCount();

        for (int i = 0; i < 3; i++) {
            dados.corrente[i] = rand() % 1000;
            dados.controle_tracao[i] = rand() % 200 - 100;
            for (int j = 0; j < 3; j++) {
                dados.ganho_tracao[3 * i + j] = 0.1f * ((rand() % 100) / 10.0f);
            }
        }

        xQueueSend(xFilaDadosMotor, &dados, portMAX_DELAY);
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

void vTaskGeraDadosVel(void *pvParameters) {
    DadosVelocidade_t dados;
    const TickType_t xPeriod = pdMS_TO_TICKS(10);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        dados.timestamp = xTaskGetTickCount();

        for (int i = 0; i < 3; i++) {
            dados.velocidade_angular[i] = rand() % 500;
            dados.aceleracao_linear[i] = rand() % 100;
            dados.giroscopio[i] = rand() % 360;
            dados.campo_magnetico[i] = rand() % 100;
            dados.controle_velocidade[i] = rand() % 200 - 100;
            dados.setpoint_velocidade[i] = 100 + rand() % 200;
            dados.angulo_rotacao[i] = rand() % 360;
            for (int j = 0; j < 3; j++) {
                dados.ganho_velocidade[3 * i + j] = 0.05f * ((rand() % 100) / 10.0f);
            }
        }

        xQueueSend(xFilaDadosVelocidade, &dados, portMAX_DELAY);
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

void vTaskGeraDadosGPS(void *pvParameters) {
    DadosGPS_t dados;
    const TickType_t xPeriod = pdMS_TO_TICKS(100);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        dados.timestamp = xTaskGetTickCount();
        dados.x = rand() % 10000;
        dados.y = rand() % 10000;
        dados.z = rand() % 1000;

        for (int i = 0; i < 3; i++) {
            dados.controle_posicao[i] = rand() % 200 - 100;
            dados.ganho_posicao[i] = 0.01f * ((rand() % 100) / 10.0f);
        }

        dados.roll = rand() % 360;
        dados.pitch = rand() % 360;
        dados.yaw = rand() % 360;

        xQueueSend(xFilaDadosGPS, &dados, portMAX_DELAY);
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

size_t serializar_bloco(uint8_t tipo, const void *dados, uint16_t tamanho, uint8_t *destino) {
    destino[0] = tipo;
    destino[1] = (uint8_t)(tamanho & 0xFF);
    destino[2] = (uint8_t)((tamanho >> 8) & 0xFF);
    memcpy(&destino[3], dados, tamanho);
    return 3 + tamanho;
}

void vTaskTransmissao(void *pvParameters) {
    uint8_t buffer[TAMANHO_BUFFER_TRANSMISSAO];
    size_t offset;
    DadosMotor_t motor;
    DadosVelocidade_t velocidade;
    DadosGPS_t gps;

    const TickType_t xPeriod = pdMS_TO_TICKS(100);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        offset = 0;

        if (xQueueReceive(xFilaDadosMotor, &motor, 0)) {
            offset += serializar_bloco(TIPO_MOTOR, &motor, sizeof(DadosMotor_t), &buffer[offset]);
        }

        if (xQueueReceive(xFilaDadosVelocidade, &velocidade, 0)) {
            offset += serializar_bloco(TIPO_VELOC, &velocidade, sizeof(DadosVelocidade_t), &buffer[offset]);
        }

        if (xQueueReceive(xFilaDadosGPS, &gps, 0)) {
            offset += serializar_bloco(TIPO_GPS, &gps, sizeof(DadosGPS_t), &buffer[offset]);
        }

        if (offset > 0) {
            HAL_UART_Transmit(&huart6, buffer, offset, HAL_MAX_DELAY);
        }

        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

void vTaskRecepcao(void *pvParameters) {
    ComandoRecebido_t comando;
    while (1) {
        if (xQueueReceive(xFilaComandos, &comando, portMAX_DELAY)) {
            switch (comando) {
                case CMD_RESET: break;
                case CMD_MODO_AUTONOMO: break;
                case CMD_MODO_MANUAL: break;
                default: break;
            }
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart6) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        ComandoRecebido_t cmd = (ComandoRecebido_t)comando_recebido;
        xQueueSendFromISR(xFilaComandos, &cmd, &xHigherPriorityTaskWoken);
        HAL_UART_Receive_IT(&huart6, &comando_recebido, 1);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void SetupTarefasPortaCOMM(void) {
    xFilaDadosMotor       = xQueueCreate(TAM_FILA_DADOS_MOTOR, sizeof(DadosMotor_t));
    xFilaDadosVelocidade  = xQueueCreate(TAM_FILA_DADOS_VELOCIDADE, sizeof(DadosVelocidade_t));
    xFilaDadosGPS         = xQueueCreate(TAM_FILA_DADOS_GPS, sizeof(DadosGPS_t));
    xFilaComandos         = xQueueCreate(5, sizeof(ComandoRecebido_t));

    configASSERT(xFilaDadosMotor != NULL);
    configASSERT(xFilaDadosVelocidade != NULL);
    configASSERT(xFilaDadosGPS != NULL);
    configASSERT(xFilaComandos != NULL);

    HAL_UART_Receive_IT(&huart6, &comando_recebido, 1);

    xTaskCreate(vTaskTransmissao,  "Transmite", 256, NULL, 2, NULL);
    xTaskCreate(vTaskRecepcao,    "Recepcao",  256, NULL, 2, &xHandleRecepcao);
    xTaskCreate(vTaskGeraDadosMotor, "Gera Dados Motor", 256, NULL, 2, NULL);
    xTaskCreate(vTaskGeraDadosVel, "Gera Dados Velocidade", 256, NULL, 2, NULL);
    xTaskCreate(vTaskGeraDadosGPS, "Gera Dados GPS", 256, NULL, 2, NULL);
}
