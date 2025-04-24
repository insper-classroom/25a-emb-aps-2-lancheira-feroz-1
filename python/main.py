import serial
import time
from pynput.keyboard import Key, Controller

# Inicializa o controller de teclado
keyboard = Controller()

# CONFIGURAÇÃO DA PORTA SERIAL (ajuste conforme seu sistema)
SERIAL_PORT = '/dev/ttyACM0'  # Ex.: '/dev/ttyUSB0' no Linux ou 'COM3' no Windows
BAUD_RATE = 115200

# Constantes do protocolo
PKT_HEADER = 0xAA
PKT_FOOTER = 0xFF

MSG_ANALOG = 0x01   # Sinais analógicos (joystick ou potenciômetro)
MSG_BUTTON = 0x02   # Eventos de botão

# Mapeamento dos botões digitais para teclas e modos
# IDs (do microcontrolador) mapeados para teclas:
button_key_map = {
    21: 'w',      # Acelera
    20: 's',      # Freia
    18: 'e',      # Sobe marcha
    19: 'q',      # Desce marcha
    17: Key.space # Freio de mão
}

# Mapeamento dos modos (click ou hold) para cada botão
# "hold" indica que, enquanto o botão estiver pressionado, a tecla deve permanecer pressionada.
# "click" indica que o evento é momentâneo (pressiona e solta imediatamente).
button_mode = {
    21: "hold",   # Acelera: modo hold (você poderá mantê-lo pressionado)
    20: "hold",   # Freia: modo hold
    18: "hold",   # Sobe marcha: clique (disparo único)
    19: "hold",   # Desce marcha: clique
    17: "hold"    # Freio de mão: modo hold
}

# Dicionário para manter o estado dos botões digitais em modo "hold"
current_hold = {}  # Ex: {21: True} se o botão 21 estiver pressionado

# Para os sinais analógicos do joystick, mantemos o estado atual e timestamps.
current_keys = {
    'joystick_x': None,  # Eixo horizontal: Key.left ou Key.right
    'joystick_y': None   # Eixo vertical: Key.up ou Key.down
}

last_update = {
    'joystick_x': 0,
    'joystick_y': 0
}

# Variável para controlar o instante do último clique simulado do potenciômetro (não será usada na nova lógica de hold)
last_wheel_click = 0

# Constantes para o comportamento do potenciômetro (axis 2)
MIN_THRESHOLD = 60   # Zona morta: valores abaixo de 30 (ou acima de -30) não acionam nada
MIN_HOLD = 0.25       # Tempo mínimo de hold (segundos)
MAX_HOLD = 0.5       # Tempo máximo de hold para giros intensos (segundos)
CONTINUOUS_THRESHOLD = 0.95  # Se o valor normalizado (ratio) atingir esse valor, mantém pressionado continuamente

# Variáveis globais para controlar o estado do potenciômetro (axis 2)
wheel_holding = False     # Indica se a tecla do potenciômetro está atualmente pressionada
wheel_current_key = None  # Armazena a tecla atualmente pressionada ('a' ou 'd')
wheel_hold_start = 0.0    # Timestamp de quando a tecla foi pressionada
wheel_hold_duration = 0.0 # Duração calculada para manter o hold

# Timeout para liberar as teclas do joystick caso não haja atualização
JOYSTICK_TIMEOUT = 0.1  # segundos

def handle_analog(axis, value):
    """
    Processa sinais analógicos para gerar eventos de teclado.
    
    - Para o potenciômetro (axis 2):
      Se value > MIN_THRESHOLD, mapeia para tecla 'a';
      se value < -MIN_THRESHOLD, mapeia para tecla 'd';
      caso contrário, não aciona nada.
      
      Quando o sinal estiver presente, calcula a magnitude efetiva (valor absoluto - MIN_THRESHOLD)
      e o valor normalizado (ratio). Se ratio >= CONTINUOUS_THRESHOLD, a tecla é mantida pressionada
      continuamente; caso contrário, simula um "click hold" em que a tecla é pressionada e mantida por
      um tempo T_hold proporcional à intensidade do giro, e após esse período é liberada (para reiniciar o ciclo).
    
    - Para o joystick (axis 0 e 1):
      * Eixo 0 (horizontal): valor negativo → Key.left, positivo → Key.right.
      * Eixo 1 (vertical): valor negativo → Key.up, positivo → Key.down.
      Usa timeout para liberar a tecla se não houver atualização.
    """
    global wheel_holding, wheel_current_key, wheel_hold_start, wheel_hold_duration
    now = time.time()
    
    if axis == 2:
        # Determina a tecla desejada a partir do sinal do potenciômetro
        if value > MIN_THRESHOLD:
            desired = 'a'
        elif value < -MIN_THRESHOLD:
            desired = 'd'
        else:
            desired = None
        
        # Se o sinal não ultrapassar a zona morta, libera tecla se estiver pressionada
        if desired is None:
            if wheel_holding:
                keyboard.release(wheel_current_key)
                wheel_holding = False
                wheel_current_key = None
            return
        
        # Calcula a magnitude efetiva e o ratio normalizado (0 a 1)
        magnitude = abs(value) - MIN_THRESHOLD  # varia de 0 até ~225 (se o valor máximo for 255)
        ratio = magnitude / 225.0
        if ratio > 1.0:
            ratio = 1.0
        
        # Se o giro estiver próximo do máximo, mantém a tecla pressionada continuamente
        if ratio >= CONTINUOUS_THRESHOLD:
            if (not wheel_holding) or (wheel_current_key != desired):
                if wheel_holding and wheel_current_key is not None and wheel_current_key != desired:
                    keyboard.release(wheel_current_key)
                keyboard.press(desired)
                wheel_holding = True
                wheel_current_key = desired
            # Em hold contínuo, não realizamos liberação
        else:
            # Para giros não máximos, calcula o tempo de hold proporcional à intensidade do giro.
            # Quanto maior o giro (maior ratio), maior o tempo que a tecla será mantida pressionada.
            # Aqui usamos uma relação linear entre MIN_HOLD e MAX_HOLD.
            T_hold = MIN_HOLD + (MAX_HOLD - MIN_HOLD) * ratio
            if not wheel_holding:
                # Se a tecla ainda não estiver pressionada, inicia o hold
                keyboard.press(desired)
                wheel_holding = True
                wheel_current_key = desired
                wheel_hold_start = now
                wheel_hold_duration = T_hold
            else:
                # Se a tecla já está pressionada e o mesmo desired, verifica se o período de hold expirou.
                if now - wheel_hold_start >= wheel_hold_duration:
                    keyboard.release(desired)
                    wheel_holding = False
                    wheel_current_key = None
                    # Se o sinal ainda estiver presente, na próxima iteração um novo ciclo de hold será iniciado.
    
    elif axis == 0:
        # Joystick horizontal
        desired = Key.left if value < 0 else (Key.right if value > 0 else None)
        last_update['joystick_x'] = now
        if current_keys['joystick_x'] != desired:
            if current_keys['joystick_x'] is not None:
                keyboard.release(current_keys['joystick_x'])
            current_keys['joystick_x'] = desired
            if desired is not None:
                keyboard.press(desired)
    
    elif axis == 1:
        # Joystick vertical
        desired = Key.up if value < 0 else (Key.down if value > 0 else None)
        last_update['joystick_y'] = now
        if current_keys['joystick_y'] != desired:
            if current_keys['joystick_y'] is not None:
                keyboard.release(current_keys['joystick_y'])
            current_keys['joystick_y'] = desired
            if desired is not None:
                keyboard.press(desired)

def handle_button(button, state):
    """
    Processa eventos digitais de botão.
    Recebe o ID do botão e seu estado (1 para pressionado, 0 para solto).
    Dependendo do modo configurado para o botão:
      - "click": se state == 1, simula um click (pressiona e solta imediatamente).
      - "hold": se state == 1, pressiona a tecla (se ainda não estiver pressionada);
                se state == 0, libera a tecla.
    """
    key = button_key_map.get(button)
    mode = button_mode.get(button, "click")
    if key is None:
        print(f"Botão {button} sem mapeamento definido.")
        return

    if mode == "click":
        if state == 1:
            keyboard.press(key)
            keyboard.release(key)
            print(f"Botão {button} (click) acionado: tecla {key}")
    elif mode == "hold":
        if state == 1:
            if not current_hold.get(button, False):
                current_hold[button] = True
                keyboard.press(key)
                print(f"Botão {button} (hold) pressionado: tecla {key}")
        elif state == 0:
            if current_hold.get(button, False):
                keyboard.release(key)
                current_hold[button] = False
                print(f"Botão {button} (hold) liberado: tecla {key}")

def process_packet(packet):
    """
    Processa um pacote completo validando o checksum e interpretando os dados.
    
    Estrutura do pacote:
      [0] Header | [1] msg_type | [2] payload_size | [3...payload] | [checksum] | [Footer]
    """
    msg_type = packet[1]
    payload_size = packet[2]
    expected_checksum = 0
    for i in range(1, 3 + payload_size):
        expected_checksum ^= packet[i]

    if expected_checksum != packet[3 + payload_size]:
        print("Checksum inválido. Pacote descartado.")
        return

    if msg_type == MSG_ANALOG:
        if payload_size != 3:
            print("Tamanho de payload inválido para sinal analógico.")
            return
        axis = packet[3]
        msb = packet[4]
        lsb = packet[5]
        value = (msb << 8) | lsb
        if value & 0x8000:
            value = value - 0x10000
        handle_analog(axis, value)

    elif msg_type == MSG_BUTTON:
        if payload_size != 2:
            print("Tamanho de payload inválido para botão.")
            return
        button = packet[3]
        state = packet[4]  # Espera 1 (pressionado) ou 0 (solto)
        handle_button(button, state)
    else:
        print("Tipo de mensagem desconhecido:", msg_type)

def check_timeouts():
    """
    Verifica os timeouts para liberar as teclas dos dispositivos analógicos (joystick) caso não haja atualização.
    Para o volante (potenciômetro), o comportamento é baseado na lógica de hold e não necessita de timeout.
    """
    now = time.time()
    if current_keys['joystick_x'] is not None and now - last_update['joystick_x'] > JOYSTICK_TIMEOUT:
        keyboard.release(current_keys['joystick_x'])
        current_keys['joystick_x'] = None
    if current_keys['joystick_y'] is not None and now - last_update['joystick_y'] > JOYSTICK_TIMEOUT:
        keyboard.release(current_keys['joystick_y'])
        current_keys['joystick_y'] = None

def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    except serial.SerialException as e:
        print("Erro ao abrir a porta serial:", e)
        return

    buffer = bytearray()
    print("Iniciando leitura da UART...")
    try:
        while True:
            data = ser.read(ser.in_waiting or 1)
            if data:
                buffer.extend(data)

            while len(buffer) >= 6:
                if buffer[0] != PKT_HEADER:
                    buffer.pop(0)
                    continue
                if len(buffer) < 3:
                    break
                payload_size = buffer[2]
                packet_length = 1 + 1 + 1 + payload_size + 1 + 1
                if len(buffer) < packet_length:
                    break
                packet = buffer[:packet_length]
                if packet[-1] != PKT_FOOTER:
                    buffer.pop(0)
                    continue
                process_packet(packet)
                buffer = buffer[packet_length:]
            
            check_timeouts()
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("Encerrando leitura...")
    finally:
        ser.close()
        for key in current_keys.values():
            if key is not None:
                keyboard.release(key)
        for key in current_hold.values():
            if key:
                # Se necessário, liberar as teclas em hold
                pass

if __name__ == '__main__':
    main()
