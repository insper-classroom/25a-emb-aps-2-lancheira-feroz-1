# Controle Customizado para Need for Speed

## Jogo
**Need for Speed** – Série de jogos de corrida onde o jogador participa de disputas em alta velocidade, utilizando carros customizados em diferentes pistas (urbanas ou circuitos fechados).

---

## Ideia do Controle
Este projeto consiste em um **volante personalizado** para uso em *Need for Speed*, porém **sem pedais analógicos**. Em vez disso, **aceleração** e **freio** serão acionados por **botões digitais**. Haverá:
- **1 eixo analógico** (potenciômetro) para o ângulo de direção do volante.
- **5 entradas digitais** (GPIO) para as principais funções de corrida (acelerar, frear, trocar marcha para cima/baixo, nitro).

Além disso, teremos **feedback** por meio de um **LED indicador** e um **buzzer** para alertas ou efeitos sonoros simples.

---

## Entradas (Inputs)

1. **Volante (Eixo Analógico)**  
   - **Potenciômetro** acoplado ao eixo do volante para detectar o ângulo de rotação.  
   - Valores podem variar de 0 a 1023 (caso de ADC de 10 bits), mapeados para esquerda (0) até direita (máximo).
  
2. **Joystick Analógico (Para Menu)**
   - Um pequeno joystick com ao menos **2 eixos analógicos** (eixo X e eixo Y).  
   - Serve para navegar pelos menus do jogo (selecionar opções, voltar, trocar configurações, etc.).  
   - Cada eixo envia valores de 0 a 1023 (ADC de 10 bits), podendo ser mapeados para cima/baixo/esquerda/direita dentro do menu.

3. **5 Entradas Digitais (Botões/GPIO):**  
   1. **Acelerar (Botão)** – em vez de pedal analógico.  
   2. **Frear (Botão)** – em vez de pedal analógico.  
   3. **Shift Up** (troca de marcha para cima).  
   4. **Shift Down** (troca de marcha para baixo).  
   5. **Nitro** (impulso extra de velocidade).

---

## Saídas (Outputs)

1. **LED Indicador**  
   - Sinalizar status de conexão com o PC.

2. **Buzzer** (opcional)  
   - Para alertas sonoros (ex.: ativar nitro) ou confirmação de troca de marcha.  

---

## Protocolo Utilizado

- **UART (Universal Asynchronous Receiver-Transmitter)** para comunicação entre o volante e o computador.
- **GPIO Interrupts** para os botões e entradas digitais.

---

## Diagrama de Blocos Explicativo do Firmware
### Estrutura Geral
---

![Estrutura](diagrama-white.png)

---

#### **Principais Componentes do RTOS**

- **Tasks:**
  - Task de leitura de entradas (botões e potenciômetro)
  - Task de envio de comandos via UART
  - Task de controle do buzzer para feedback sonoro
  - Task de atualização do LED indicador

- **Filas:**
  - Fila de eventos de entrada
  - Fila de comandos para o jogo
  - Fila de feedback sonoro para o buzzer

- **Semáforos:**
  - Verificação do estado de conexão

- **Interrupts:**
  - Callbacks para os botões e potenciômetro

## Imagens do Volante
### Proposta Inicial
---

![Proposta](esboco.jpg)

---
