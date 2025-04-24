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
button_key_map = {
    21: 'w',         # Acelera
    20: 's',         # Freia
    18: 'e',         # Sobe marcha
    19: 'q',         # Desce marcha
    17: Key.space,   # Freio de mão
    100: Key.enter   # Enter especial
}

button_mode = {
    21: "hold",      # Acelera: hold
    20: "hold",      # Freia: hold
    18: "hold",      # Sobe marcha: hold
    19: "hold",      # Desce marcha: hold
    17: "hold",      # Freio de mão: hold
    100: "special"   # Enter: modo especial (apenas na borda de subida)
}

# Estado para botões em hold
current_hold = {}

# Para sinais analógicos do joystick
current_keys = {'joystick_x': None, 'joystick_y': None}
last_update = {'joystick_x': 0, 'joystick_y': 0}

# Estado anterior de cada botão (para debouncing de clicks e special)
last_button_state = {}

# Variáveis para o potenciômetro (axis 2)
wheel_holding = False
wheel_current_key = None
wheel_hold_start = 0.0
wheel_hold_duration = 0.0

MIN_THRESHOLD = 60
MIN_HOLD = 0.25
MAX_HOLD = 0.5
CONTINUOUS_THRESHOLD = 0.95

JOYSTICK_TIMEOUT = 0.1  # segundos

def handle_analog(axis, value):
    global wheel_holding, wheel_current_key, wheel_hold_start, wheel_hold_duration
    now = time.time()

    if axis == 2:
        value -= 120
        if value > MIN_THRESHOLD:
            desired = 'a'
        elif value < -MIN_THRESHOLD:
            desired = 'd'
        else:
            desired = None

        if desired is None:
            if wheel_holding:
                keyboard.release(wheel_current_key)
                wheel_holding = False
                wheel_current_key = None
            return

        magnitude = abs(value) - MIN_THRESHOLD
        ratio = min(magnitude / 225.0, 1.0)

        if ratio >= CONTINUOUS_THRESHOLD:
            if (not wheel_holding) or (wheel_current_key != desired):
                if wheel_holding and wheel_current_key and wheel_current_key != desired:
                    keyboard.release(wheel_current_key)
                keyboard.press(desired)
                wheel_holding = True
                wheel_current_key = desired
        else:
            T_hold = MIN_HOLD + (MAX_HOLD - MIN_HOLD) * ratio
            if not wheel_holding:
                keyboard.press(desired)
                wheel_holding = True
                wheel_current_key = desired
                wheel_hold_start = now
                wheel_hold_duration = T_hold
            else:
                if now - wheel_hold_start >= wheel_hold_duration:
                    keyboard.release(desired)
                    wheel_holding = False
                    wheel_current_key = None

    elif axis == 0:
        desired = Key.left if value < 0 else (Key.right if value > 0 else None)
        last_update['joystick_x'] = now
        if current_keys['joystick_x'] != desired:
            if current_keys['joystick_x'] is not None:
                keyboard.release(current_keys['joystick_x'])
            current_keys['joystick_x'] = desired
            if desired:
                keyboard.press(desired)

    elif axis == 1:
        desired = Key.up if value < 0 else (Key.down if value > 0 else None)
        last_update['joystick_y'] = now
        if current_keys['joystick_y'] != desired:
            if current_keys['joystick_y'] is not None:
                keyboard.release(current_keys['joystick_y'])
            current_keys['joystick_y'] = desired
            if desired:
                keyboard.press(desired)

def handle_button(button, state):
    """
    Modo click: dispara apenas se state==1 e veio de 0.
    Modo hold: pressiona em 1 e libera em 0.
    Modo special: igual ao click, mas exclusivo para botões como o 100.
    """
    key = button_key_map.get(button)
    mode = button_mode.get(button, "click")
    if key is None:
        print(f"Botão {button} sem mapeamento definido.")
        return

    prev = last_button_state.get(button, 0)

    if mode == "click":
        if state == 1 and prev == 0:
            keyboard.press(key)
            keyboard.release(key)
            print(f"Botão {button} (click) acionado: tecla {key}")

    elif mode == "hold":
        if state == 1 and not current_hold.get(button, False):
            current_hold[button] = True
            keyboard.press(key)
            print(f"Botão {button} (hold) pressionado: tecla {key}")
        elif state == 0 and current_hold.get(button, False):
            keyboard.release(key)
            current_hold[button] = False
            print(f"Botão {button} (hold) liberado: tecla {key}")

    elif mode == "special":
        # dispara apenas uma vez por pressão
        if state == 1 and prev == 0:
            keyboard.press(key)
            keyboard.release(key)
            print(f"Botão {button} (special) acionado: tecla {key}")

    else:
        print(f"Modo {mode} para o botão {button} não suportado.")

    last_button_state[button] = state

def process_packet(packet):
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
            value -= 0x10000
        handle_analog(axis, value)

    elif msg_type == MSG_BUTTON:
        if payload_size != 2:
            print("Tamanho de payload inválido para botão.")
            return
        button = packet[3]
        state = packet[4]
        handle_button(button, state)

    else:
        print("Tipo de mensagem desconhecido:", msg_type)

def check_timeouts():
    now = time.time()
    if current_keys['joystick_x'] and now - last_update['joystick_x'] > JOYSTICK_TIMEOUT:
        keyboard.release(current_keys['joystick_x'])
        current_keys['joystick_x'] = None
    if current_keys['joystick_y'] and now - last_update['joystick_y'] > JOYSTICK_TIMEOUT:
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
        # Libera todas as teclas pressionadas em analog y/x
        for key in current_keys.values():
            if key:
                keyboard.release(key)
        # Libera botões em hold, se houver
        for btn, held in current_hold.items():
            if held:
                keyboard.release(button_key_map[btn])

if __name__ == '__main__':
    main()
