/*
 * Comunicação UART melhorada com FreeRTOS, utilizando semáforo para o START.
 * Agora os eventos dos botões são transmitidos com o estado:
 *   - 1: botão pressionado (FALL)
 *   - 0: botão liberado (RISE)
 * 
 * Protocolo:
 *   Header     : 0xAA
 *   Tipo       : 0x01 = sinal analógico, 0x02 = evento de botão
 *   Tamanho    : tamanho do payload (1 byte para identificação + 1 byte para estado ou 3 bytes para analógico)
 *   Payload    : 
 *                 - Para analógico: 1 byte (eixo: 0=X, 1=Y, 2=pot) + 2 bytes (valor int16_t em MSB/LSB)
 *                 - Para botão  : 1 byte (identificador do botão) + 1 byte (estado: 1 = pressionado, 0 = liberado)
 *   Checksum   : XOR de (Tipo, Tamanho e Payload)
 *   Footer     : 0xFF
 *
 * Obs.: Pode-se implementar "byte stuffing" se os dados puderem conter os valores do header/footer.
 */

 #include <FreeRTOS.h>
 #include <task.h>
 #include <queue.h>
 #include <semphr.h>
 #include "pico/stdlib.h"
 #include "hardware/gpio.h"
 #include "hardware/adc.h"
 #include "hardware/uart.h"
 #include <stdio.h>
 #include <stdint.h>
 
 // ---------------- Constantes e Definições ---------------- //
 
 #define AVG_SAMPLES        10
 
 #define ADC_MAX            4095
 #define ADC_CENTER         2048
 #define JOYSTICK_MAX       255
 #define DEAD_ZONE          30
 
 // Pinos de hardware
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
 
 // Configuração da UART
 const int UART_TX          = 0;
 const int UART_RX          = 1;
 #define BAUD_RATE          115200
 #define UART_ID            uart0
 
 // Definições do protocolo
 #define PKT_HEADER         0xAA
 #define PKT_FOOTER         0xFF
 
 #define MSG_ANALOG         0x01
 #define MSG_BUTTON         0x02
 
 // Tamanhos dos payloads
 #define ANALOG_PAYLOAD_SIZE   3   // 1 byte para o eixo + 2 bytes para o valor
 #define BUTTON_PAYLOAD_SIZE   2   // 1 byte para o identificador + 1 byte para o estado
 
 // ---------------- Estruturas e Variáveis Globais ---------------- //
 
 // Semáforo para indicar que a comunicação foi iniciada via botão START
 SemaphoreHandle_t xCommSemaphore;
 
 // Estrutura para dados analógicos
 typedef struct {
     int axis;  // 0 = eixo X, 1 = eixo Y, 2 = potenciômetro
     int val;   // Valor processado (-255 a +255)
 } adc_t;
 
 // Estrutura para eventos de botão
 typedef struct {
     uint32_t gpio;
     uint32_t state;  // 1 = pressionado (FALL), 0 = liberado (RISE)
 } button_event_t;
 
 QueueHandle_t xQueueADC;
 QueueHandle_t xQueueInput;  // Alterado para receber objetos do tipo button_event_t
 
 // ---------------- Funções de Utilidade ---------------- //
 
 // Processa o valor bruto do ADC, aplicando escala e zona morta.
 static int process_adc_value(uint16_t raw_adc) {
     int centered = (int)raw_adc - ADC_CENTER;
     float scale_factor = (float)JOYSTICK_MAX / (ADC_CENTER - 1);
     int scaled = (int)(centered * scale_factor);
     
     if (scaled > -DEAD_ZONE && scaled < DEAD_ZONE) {
         scaled = 0;
     }
     return scaled;
 }
 
 // Calcula um checksum simples (XOR dos bytes)
 uint8_t calc_checksum(uint8_t *data, int len) {
     uint8_t checksum = 0;
     for (int i = 0; i < len; i++) {
         checksum ^= data[i];
     }
     return checksum;
 }
 
 // Monta e envia um pacote via UART conforme o protocolo definido.
 void send_packet_uart(uint8_t msg_type, uint8_t *payload, uint8_t payload_size) {
     uint8_t packet[10];  // Ajuste se necessário
     int idx = 0;
     
     packet[idx++] = PKT_HEADER;
     packet[idx++] = msg_type;
     packet[idx++] = payload_size;
     
     for (int i = 0; i < payload_size; i++) {
         packet[idx++] = payload[i];
     }
     
     uint8_t checksum = calc_checksum(&packet[1], 2 + payload_size);
     packet[idx++] = checksum;
     packet[idx++] = PKT_FOOTER;
     
     uart_write_blocking(UART_ID, packet, idx);
 }
 
 // ---------------- Tasks de Aquisição ---------------- //
 
 // Task para aquisição dos dados do eixo X.
 void x_task(void *p) {
     adc_gpio_init(GPx);
     uint16_t samples[AVG_SAMPLES] = {0};
     int index = 0, count = 0, sum = 0;
     
     while (1) {
         adc_select_input(0);  // Canal 0 para eixo X.
         uint16_t raw = adc_read();
         
         if (count < AVG_SAMPLES) {
             samples[index] = raw;
             sum += raw;
             count++;
         } else {
             sum -= samples[index];
             samples[index] = raw;
             sum += raw;
         }
         index = (index + 1) % AVG_SAMPLES;
         int avg_raw = sum / count;
         int scaled_val = process_adc_value(avg_raw);
         
         if (scaled_val != 0) {
             adc_t data;
             data.axis = 0;
             data.val  = scaled_val;
             xQueueSend(xQueueADC, &data, 0);
         }
         vTaskDelay(pdMS_TO_TICKS(10));
     }
 }
 
 // Task para aquisição dos dados do eixo Y.
 void y_task(void *p) {
     adc_gpio_init(GPy);
     uint16_t samples[AVG_SAMPLES] = {0};
     int index = 0, count = 0, sum = 0;
     
     while (1) {
         adc_select_input(1);  // Canal 1 para eixo Y.
         uint16_t raw = adc_read();
         
         if (count < AVG_SAMPLES) {
             samples[index] = raw;
             sum += raw;
             count++;
         } else {
             sum -= samples[index];
             samples[index] = raw;
             sum += raw;
         }
         index = (index + 1) % AVG_SAMPLES;
         int avg_raw = sum / count;
         int scaled_val = process_adc_value(avg_raw);
         
         if (scaled_val != 0) {
             adc_t data;
             data.axis = 1;
             data.val  = scaled_val;
             xQueueSend(xQueueADC, &data, 0);
         }
         vTaskDelay(pdMS_TO_TICKS(10));
     }
 }
 
 // Task para aquisição do potenciômetro (por exemplo, pedal ou volante).
 void pot_task(void *p) {
     adc_gpio_init(WHEEL_PIN);
     uint16_t samples[AVG_SAMPLES] = {0};
     int index = 0, count = 0, sum = 0;
     
     while (1) {
         adc_select_input(2);  // Canal 2 para o potenciômetro.
         uint16_t raw = adc_read();
         
         if (count < AVG_SAMPLES) {
             samples[index] = raw;
             sum += raw;
             count++;
         } else {
             sum -= samples[index];
             samples[index] = raw;
             sum += raw;
         }
         index = (index + 1) % AVG_SAMPLES;
         int avg_raw = sum / count;
         int scaled_val = process_adc_value(avg_raw);
         
         if (scaled_val != 0) {
             adc_t data;
             data.axis = 2;
             data.val  = scaled_val;
             xQueueSend(xQueueADC, &data, 0);
         }
         vTaskDelay(pdMS_TO_TICKS(10));
     }
 }
 
 // ---------------- Task de Comunicação UART ---------------- //
 
 // A task envia dados analógicos e eventos de botão via UART.
 void uart_task(void *p) {
     adc_t adc_data;
     button_event_t btn_event;
     
     while (1) {
         // Processa eventos de botão recebidos na fila.
         while (xQueueReceive(xQueueInput, &btn_event, 0) == pdPASS) {
             if (btn_event.gpio == START_BTN) {
                 // Lógica de START.
                 if (uxSemaphoreGetCount(xCommSemaphore) == 0) {
                     xSemaphoreGive(xCommSemaphore);
                     gpio_put(LED, 1);  // Acende o LED.
                     printf("Comunicação iniciada via botão START!\n");
                     // (Opcional) Enviar pacote especial de START.
                 }
             } else {
                 uint8_t payload[BUTTON_PAYLOAD_SIZE];
                 payload[0] = (uint8_t)btn_event.gpio;  // ID do botão.
                 payload[1] = (uint8_t)btn_event.state; // Estado (1 ou 0).
                 send_packet_uart(MSG_BUTTON, payload, BUTTON_PAYLOAD_SIZE);
             }
         }
         
         // Processa dados analógicos se a comunicação tiver sido iniciada.
         if (uxSemaphoreGetCount(xCommSemaphore) > 0) {
             while (xQueueReceive(xQueueADC, &adc_data, 0) == pdPASS) {
                 uint8_t payload[ANALOG_PAYLOAD_SIZE];
                 payload[0] = (uint8_t)adc_data.axis;
                 int16_t movement = (int16_t)adc_data.val;
                 payload[1] = (uint8_t)((movement >> 8) & 0xFF);  // MSB.
                 payload[2] = (uint8_t)(movement & 0xFF);         // LSB.
                 send_packet_uart(MSG_ANALOG, payload, ANALOG_PAYLOAD_SIZE);
             }
         }
         vTaskDelay(pdMS_TO_TICKS(10));
     }
 }
 
 // ---------------- Callback para Botões ---------------- //
 
 // A função de callback é chamada em interrupção quando ocorre uma mudança no botão.
 // Ela determina o estado com base nos eventos (FALL para pressionado, RISE para liberado)
 // e envia um objeto button_event_t para a fila.
 void btn_callback(uint gpio, uint32_t events) {
     button_event_t btn_evt;
     btn_evt.gpio = gpio;
     
     // Se o evento tiver a flag FALL, o botão foi pressionado; se tiver RISE, foi liberado.
     if (events & GPIO_IRQ_EDGE_FALL) {
         btn_evt.state = 1;  // Pressionado.
     } else if (events & GPIO_IRQ_EDGE_RISE) {
         btn_evt.state = 0;  // Liberado.
     } else {
         // Caso não seja reconhecido, ignore.
         return;
     }
     xQueueSendFromISR(xQueueInput, &btn_evt, 0);
 }
 
 // ---------------- Configurações de Hardware ---------------- //
 
 void setup_uart() {
     uart_init(UART_ID, BAUD_RATE);
     gpio_set_function(UART_TX, GPIO_FUNC_UART);
     gpio_set_function(UART_RX, GPIO_FUNC_UART);
 }
 
 // ---------------- Função Principal ---------------- //
 
 int main() {
     stdio_init_all();
     setup_uart();
     adc_init();
     
     // Configuração do LED
     gpio_init(LED);
     gpio_set_dir(LED, GPIO_OUT);
     gpio_put(LED, 0);
     
     // Configuração dos botões com pull-up
     gpio_init(START_BTN);
     gpio_set_dir(START_BTN, GPIO_IN);
     gpio_pull_up(START_BTN);
     
     gpio_init(CLUTCH_BTN);
     gpio_set_dir(CLUTCH_BTN, GPIO_IN);
     gpio_pull_up(CLUTCH_BTN);
     
     gpio_init(UPSHIFT_BTN);
     gpio_set_dir(UPSHIFT_BTN, GPIO_IN);
     gpio_pull_up(UPSHIFT_BTN);
     
     gpio_init(DOWNSHIFT_BTN);
     gpio_set_dir(DOWNSHIFT_BTN, GPIO_IN);
     gpio_pull_up(DOWNSHIFT_BTN);
     
     gpio_init(BREAK_BTN);
     gpio_set_dir(BREAK_BTN, GPIO_IN);
     gpio_pull_up(BREAK_BTN);
     
     gpio_init(ACCELERATE_BTN);
     gpio_set_dir(ACCELERATE_BTN, GPIO_IN);
     gpio_pull_up(ACCELERATE_BTN);
     
     // Configura interrupção para o botão START com ambas as bordas e uma callback.
     gpio_set_irq_enabled_with_callback(START_BTN, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, btn_callback);
     // Habilita interrupção para os demais botões (usando a mesma callback)
     gpio_set_irq_enabled(CLUTCH_BTN, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
     gpio_set_irq_enabled(UPSHIFT_BTN, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
     gpio_set_irq_enabled(DOWNSHIFT_BTN, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
     gpio_set_irq_enabled(BREAK_BTN, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
     gpio_set_irq_enabled(ACCELERATE_BTN, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
     
     // Cria as filas
     xQueueADC   = xQueueCreate(10, sizeof(adc_t));
     xQueueInput = xQueueCreate(10, sizeof(button_event_t));
     
     // Cria o semáforo que sinaliza o início da comunicação (inicialmente vazio)
     xCommSemaphore = xSemaphoreCreateBinary();
     
     // Criação das tasks
     xTaskCreate(x_task,    "X_Axis",        256, NULL, 1, NULL);
     xTaskCreate(y_task,    "Y_Axis",        256, NULL, 1, NULL);
     xTaskCreate(pot_task,  "Potentiometer", 256, NULL, 1, NULL);
     xTaskCreate(uart_task, "UART_Task",     256, NULL, 1, NULL);
     
     vTaskStartScheduler();
     
     while (true) { }
 }
 