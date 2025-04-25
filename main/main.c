#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/uart.h"
#include "hardware/i2c.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "hc06.h"
#include "mpu6050.h"
#include "Fusion.h"

// ---------------- Constantes e Definições ---------------- //
#define AVG_SAMPLES        10
#define ADC_MAX            4095
#define ADC_CENTER         2048
#define JOYSTICK_MAX       255
#define DEAD_ZONE          30
#define SAMPLE_PERIOD      (0.01f)   // 10 ms

// Pinos de hardware
const int BUZZER_PIN       = 14;
const int LED              = 15;
const int START_BTN        = 16;
const int CLUTCH_BTN       = 17;
const int UPSHIFT_BTN      = 18;
const int DOWNSHIFT_BTN    = 19;
const int BREAK_BTN        = 20;
const int ACCELERATE_BTN   = 21;
const int WHEEL_PIN        = 28;
const int GPy              = 27;
const int GPx              = 26;

// I2C para MPU6050
#define MPU_ADDRESS     0x68
#define I2C_SDA_GPIO    12
#define I2C_SCL_GPIO    13
#define MPU_CLICK_BTN   100   // Valor virtual para identificar o botão de click do MPU

// UART "fila USB"
#define UART_ID            uart0
#define UART_TX_PIN        0
#define UART_RX_PIN        1
#define BAUD_RATE          115200

// UART Bluetooth HC-06
#define BT_UART_ID         uart1
#define BT_TX_PIN          5   // ajuste conforme wiring
#define BT_RX_PIN          4   // ajuste conforme wiring
#define HC06_ENABLE_PIN    6   // pino para AT-Mode do HC-06
#define HC06_BAUD_RATE     9600

// Protocolo
#define PKT_HEADER         0xAA
#define PKT_FOOTER         0xFF
#define MSG_ANALOG         0x01
#define MSG_BUTTON         0x02

#define ANALOG_PAYLOAD_SIZE   3
#define BUTTON_PAYLOAD_SIZE   2

// ---------------- Variáveis Globais ---------------- //
SemaphoreHandle_t xCommSemaphore;
SemaphoreHandle_t xAccelerateSemaphore;
QueueHandle_t xQueueADC;
QueueHandle_t xQueueInput;

typedef struct {
    int axis;
    int val;
} adc_t;

typedef struct {
    uint32_t gpio;
    uint32_t state;
} button_event_t;

// ---------------- Funções de Utilidade ---------------- //
static int process_adc_value(uint16_t raw_adc) {
    int centered = (int)raw_adc - ADC_CENTER;
    float scale = (float)JOYSTICK_MAX / (ADC_CENTER - 1);
    int scaled = (int)(centered * scale);
    if (scaled > -DEAD_ZONE && scaled < DEAD_ZONE) scaled = 0;
    return scaled;
}

uint8_t calc_checksum(const uint8_t *data, int len) {
    uint8_t cs = 0;
    for (int i = 0; i < len; i++) cs ^= data[i];
    return cs;
}

void send_packet_uart(uint8_t msg_type, const uint8_t *payload, uint8_t payload_size) {
    uint8_t packet[10];
    int idx = 0;
    packet[idx++] = PKT_HEADER;
    packet[idx++] = msg_type;
    packet[idx++] = payload_size;
    for (int i = 0; i < payload_size; i++) packet[idx++] = payload[i];
    packet[idx++] = calc_checksum(&packet[1], 2 + payload_size);
    packet[idx++] = PKT_FOOTER;

    // envia pela USB
    //uart_write_blocking(UART_ID, packet, idx);
    // envia pelo Bluetooth
    uart_write_blocking(BT_UART_ID, packet, idx);
}

// ---------------- Funções do MPU6050 ---------------- //
static void mpu6050_reset() {
    uint8_t buf[] = {0x6B, 0x00};  // PWR_MGMT_1 = 0
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[6];
    uint8_t reg;

    // Leitura acelerômetro
    reg = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);
    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[2*i] << 8) | buffer[2*i+1];
    }

    // Leitura giroscópio
    reg = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);
    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[2*i] << 8) | buffer[2*i+1];
    }

    // Leitura temperatura
    reg = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 2, false);
    *temp = (buffer[0] << 8) | buffer[1];
}

// ---------------- Tasks de Aquisição ---------------- //
void x_task(void *p) {
    adc_gpio_init(GPx);
    uint16_t samples[AVG_SAMPLES] = {0};
    int idx = 0, count = 0, sum = 0;
    while (1) {
        adc_select_input(0);
        uint16_t raw = adc_read();
        if (count < AVG_SAMPLES) { samples[idx] = raw; sum += raw; count++; }
        else { sum -= samples[idx]; samples[idx] = raw; sum += raw; }
        idx = (idx + 1) % AVG_SAMPLES;
        int scaled = process_adc_value(sum / count);
        if (scaled) {
            adc_t d = { .axis = 0, .val = scaled };
            xQueueSend(xQueueADC, &d, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void y_task(void *p) {
    adc_gpio_init(GPy);
    uint16_t samples[AVG_SAMPLES] = {0};
    int idx = 0, count = 0, sum = 0;
    while (1) {
        adc_select_input(1);
        uint16_t raw = adc_read();
        if (count < AVG_SAMPLES) { samples[idx] = raw; sum += raw; count++; }
        else { sum -= samples[idx]; samples[idx] = raw; sum += raw; }
        idx = (idx + 1) % AVG_SAMPLES;
        int scaled = process_adc_value(sum / count);
        if (scaled) {
            adc_t d = { .axis = 1, .val = scaled };
            xQueueSend(xQueueADC, &d, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void pot_task(void *p) {
    adc_gpio_init(WHEEL_PIN);
    uint16_t samples[AVG_SAMPLES] = {0};
    int idx = 0, count = 0, sum = 0;
    while (1) {
        adc_select_input(2);
        uint16_t raw = adc_read();
        if (count < AVG_SAMPLES) { samples[idx] = raw; sum += raw; count++; }
        else { sum -= samples[idx]; samples[idx] = raw; sum += raw; }
        idx = (idx + 1) % AVG_SAMPLES;
        int scaled = process_adc_value(sum / count);
        if (scaled) {
            adc_t d = { .axis = 2, .val = scaled };
            xQueueSend(xQueueADC, &d, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ---------------- Task do MPU6050 ---------------- //
void mpu6050_task(void *p) {
    // Inicializa e configura o MPU6050
    int16_t rawAccel[3], rawGyro[3], rawTemp;
    
    // Variáveis para detecção de click
    uint8_t lastClickState = 0;
    button_event_t btn_evt;

    while (1) {
        mpu6050_read_raw(rawAccel, rawGyro, &rawTemp);

        FusionVector accelerometer = {
            .axis.x = rawAccel[0] / 16384.0f,
            .axis.y = rawAccel[1] / 16384.0f,
            .axis.z = rawAccel[2] / 16384.0f,
        };

        // Determina se houve um click com base na aceleração em Z
        uint8_t clickState = (accelerometer.axis.z > 0.5f) ? 1 : 0;
        
        // Se houve mudança no estado do click, envia para a fila de botões
        if (clickState != lastClickState) {
            btn_evt.gpio = MPU_CLICK_BTN;  // Identificador virtual para o botão de click do MPU
            btn_evt.state = clickState;
            xQueueSend(xQueueInput, &btn_evt, 0);
            lastClickState = clickState;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));  // 10 ms
    }
}

// ---------------- Configuração UART e HC-06 ---------------- //
void setup_uart() {
//    // 1) USB
//    uart_init(UART_ID, BAUD_RATE);
//    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
//    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

   // 2) Bluetooth HC-06: UART1
   uart_init(BT_UART_ID, HC06_BAUD_RATE);
   gpio_set_function(BT_RX_PIN, GPIO_FUNC_UART);
   gpio_set_function(BT_TX_PIN, GPIO_FUNC_UART);
}

// ---------------- Task de Comunicação UART ---------------- //
void uart_task(void *p) {
    adc_t adc_data;
    button_event_t btn;

    hc06_init("Volante BT", "1234");

    //sleep_ms(100);
    //printf("Sistema iniciado e pronto\n");
    while (1) {
        while (xQueueReceive(xQueueInput, &btn, 0) == pdPASS) {
            if (btn.gpio == START_BTN && uxSemaphoreGetCount(xCommSemaphore) == 0) {
                xSemaphoreGive(xCommSemaphore);
                gpio_put(LED, 1);
                //printf("Comunicação iniciada via START!\n");
            }
            if (btn.gpio == ACCELERATE_BTN) {
                if (btn.state && uxSemaphoreGetCount(xAccelerateSemaphore) == 0)
                    xSemaphoreGive(xAccelerateSemaphore);
                else if (!btn.state && uxSemaphoreGetCount(xAccelerateSemaphore) > 0)
                    xSemaphoreTake(xAccelerateSemaphore, 0);
            }
            uint8_t payload[BUTTON_PAYLOAD_SIZE] = { (uint8_t)btn.gpio, (uint8_t)btn.state };
            send_packet_uart(MSG_BUTTON, payload, BUTTON_PAYLOAD_SIZE);
        }
        
        // Se a comunicação estiver habilitada
        if (uxSemaphoreGetCount(xCommSemaphore) > 0) {
            // Processa dados do ADC
            while (xQueueReceive(xQueueADC, &adc_data, 0) == pdPASS) {
                uint8_t payload[ANALOG_PAYLOAD_SIZE] = {
                    (uint8_t)adc_data.axis,
                    (uint8_t)((adc_data.val >> 8) & 0xFF),
                    (uint8_t)(adc_data.val & 0xFF)
                };
                send_packet_uart(MSG_ANALOG, payload, ANALOG_PAYLOAD_SIZE);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ---------------- Task do Buzzer ---------------- //
void buzzer_task(void *p) {
    const uint32_t half_period = 1000000 / (2 * 440);
    while (1) {
        if (uxSemaphoreGetCount(xAccelerateSemaphore) > 0) {
            gpio_put(BUZZER_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(half_period / 1000));
            gpio_put(BUZZER_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(half_period / 1000));
        } else {
            gpio_put(BUZZER_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

// ---------------- Callback dos Botões ---------------- //
void btn_callback(uint gpio, uint32_t events) {
    button_event_t evt = { .gpio = gpio };
    if (events & GPIO_IRQ_EDGE_FALL) evt.state = 1;
    else if (events & GPIO_IRQ_EDGE_RISE) evt.state = 0;
    else return;
    xQueueSendFromISR(xQueueInput, &evt, 0);
}

// ---------------- Função Principal ---------------- //
int main() {
    stdio_init_all();

    // Configura UARTs e HC-06
    setup_uart();

    // Inicializa I2C para o MPU6050
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);
    
    // Reset do sensor MPU6050
    mpu6050_reset();

    // ADC
    adc_init();

    // LED
    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);
    gpio_put(LED, 0);

    // Botões
    int btns[] = {START_BTN, CLUTCH_BTN, UPSHIFT_BTN, DOWNSHIFT_BTN, BREAK_BTN, ACCELERATE_BTN};
    for (int i = 0; i < 6; i++) {
        gpio_init(btns[i]);
        gpio_set_dir(btns[i], GPIO_IN);
        gpio_pull_up(btns[i]);
    }
    gpio_set_irq_enabled_with_callback(START_BTN, GPIO_IRQ_EDGE_FALL|GPIO_IRQ_EDGE_RISE, true, btn_callback);
    for (int i = 1; i < 6; i++) {
        gpio_set_irq_enabled(btns[i], GPIO_IRQ_EDGE_FALL|GPIO_IRQ_EDGE_RISE, true);
    }

    // Buzzer
    gpio_init(BUZZER_PIN);
    gpio_set_dir(BUZZER_PIN, GPIO_OUT);
    gpio_put(BUZZER_PIN, 0);

    // Filas e semáforos
    xQueueADC            = xQueueCreate(10, sizeof(adc_t));
    xQueueInput          = xQueueCreate(10, sizeof(button_event_t));
    xCommSemaphore       = xSemaphoreCreateBinary();
    xAccelerateSemaphore = xSemaphoreCreateBinary();

    // Criação das tasks
    xTaskCreate(x_task,        "X_Axis",        256, NULL, 1, NULL);
    xTaskCreate(y_task,        "Y_Axis",        256, NULL, 1, NULL);
    xTaskCreate(pot_task,      "Potentiometer", 256, NULL, 1, NULL);
    xTaskCreate(mpu6050_task,  "MPU6050",       512, NULL, 1, NULL);  // Task para MPU6050
    xTaskCreate(uart_task,     "UART_Task",     512, NULL, 1, NULL);
    xTaskCreate(buzzer_task,   "Buzzer_Task",   256, NULL, 1, NULL);

    // Inicia scheduler
    vTaskStartScheduler();

    while (1) {}
    return 0;
}