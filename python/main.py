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
    16: Key.space # Freio de mão
}

# Mapeamento dos modos (click ou hold) para cada botão
# "hold" indica que, enquanto o botão estiver pressionado, a tecla deve permanecer pressionada.
# "click" indica que o evento é momentâneo (pressiona e solta imediatamente).
button_mode = {
    21: "hold",   # Acelera: modo hold (você poderá mantê-lo pressionado)
    20: "hold",   # Freia: modo hold
    18: "hold",  # Sobe marcha: clique (disparo único)
    19: "hold",  # Desce marcha: clique
    16: "hold"    # Freio de mão: modo hold
}

# Dicionário para manter o estado dos botões digitais em modo "hold"
current_hold = {}  # Ex: {21: True} se o botão 21 estiver pressionado

# Para os sinais analógicos (joystick e potenciômetro), mantemos o estado atual e timestamps.
current_keys = {
    'joystick_x': None,  # Eixo horizontal: Key.left ou Key.right
    'joystick_y': None,  # Eixo vertical: Key.up ou Key.down
    'wheel': None        # Potenciômetro (volante): 'a' ou 'd'
}

last_update = {
    'joystick_x': 0,
    'joystick_y': 0,
    'wheel': 0
}

# Timeout para considerar que o dispositivo (joystick ou volante) voltou à posição neutra
JOYSTICK_TIMEOUT = 0.1  # segundos

def handle_analog(axis, value):
    """
    Processa sinais analógicos para gerar eventos de teclado.
    
    - Para o potenciômetro (axis 2):
      Se value < -30, mapeia para tecla 'd';
      se value > 30, mapeia para tecla 'a';
      se value == 0 (ou dentro da zona morta), a tecla é liberada.
      A tecla é mantida pressionada enquanto houver sinal (modo hold contínuo).
    
    - Para o joystick:
      * Eixo 0 (horizontal): valor negativo → Key.left, positivo → Key.right.
      * Eixo 1 (vertical): valor negativo → Key.up, positivo → Key.down.
      Usa timeout para liberar a tecla se não houver atualização.
    """
    global current_keys, last_update
    now = time.time()

    if axis == 2:
        # Potenciômetro: mapeia para 'd' se value negativo (fora da zona morta) ou 'a' se positivo.
        desired = 'd' if value < -30 else ('a' if value > 30 else None)
        last_update['wheel'] = now
        if current_keys['wheel'] != desired:
            if current_keys['wheel'] is not None:
                keyboard.release(current_keys['wheel'])
            current_keys['wheel'] = desired
            if desired is not None:
                keyboard.press(desired)
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
        # Apenas no rising edge, simula um click
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
    Verifica os timeouts para liberar as teclas dos dispositivos analógicos caso não haja atualização.
    Para joystick e volante, se não houver atualização por JOYSTICK_TIMEOUT, libera a tecla.
    """
    now = time.time()
    if current_keys['joystick_x'] is not None and now - last_update['joystick_x'] > JOYSTICK_TIMEOUT:
        keyboard.release(current_keys['joystick_x'])
        current_keys['joystick_x'] = None
    if current_keys['joystick_y'] is not None and now - last_update['joystick_y'] > JOYSTICK_TIMEOUT:
        keyboard.release(current_keys['joystick_y'])
        current_keys['joystick_y'] = None
    if current_keys['wheel'] is not None and now - last_update['wheel'] > JOYSTICK_TIMEOUT:
        keyboard.release(current_keys['wheel'])
        current_keys['wheel'] = None

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
            # Lê os dados disponíveis da serial
            data = ser.read(ser.in_waiting or 1)
            if data:
                buffer.extend(data)

            # Processa pacotes enquanto houver bytes suficientes
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
            
            # Verifica timeouts para liberar as teclas, se necessário
            check_timeouts()

            time.sleep(0.01)
    except KeyboardInterrupt:
        print("Encerrando leitura...")
    finally:
        ser.close()
        # Libera todas as teclas atualmente pressionadas
        for key in current_keys.values():
            if key is not None:
                keyboard.release(key)
        # Libera teclas em hold, se houver
        for key in current_hold.values():
            if key:
                # Se necessário, pode liberar as teclas que estão em hold
                pass

if __name__ == '__main__':
    main()
